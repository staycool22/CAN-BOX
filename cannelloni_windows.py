import socket
import struct
import threading
import time
import argparse
import random

# Cannelloni Protocol Constants
CANNELLONI_VERSION = 2
CANNELLONI_OP_DATA = 0
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000

# CAN-FD Flags
CANFD_BRS = 0x01
CANFD_ESI = 0x02

class CannelloniFrame:
    def __init__(self, can_id, data, is_fd=False, brs=False):
        self.can_id = can_id
        self.data = data
        self.is_fd = is_fd
        self.brs = brs

def pack_cannelloni_packet(seq_no, frames):
    # Header: Version(1) + OpCode(1) + SeqNo(1) + Count(2)
    # Note: Count is big-endian (network byte order)
    header = struct.pack('!BBBH', CANNELLONI_VERSION, CANNELLONI_OP_DATA, seq_no, len(frames))
    
    payload = bytearray(header)
    
    for frame in frames:
        # ID (4 bytes, Network Order)
        payload.extend(struct.pack('!I', frame.can_id))
        
        # Length Byte
        # Bit 7: CANFD flag
        # Bits 0-6: Data Length (0-64)
        len_byte = len(frame.data) & 0x7F
        if frame.is_fd:
            len_byte |= 0x80
        payload.append(len_byte)
        
        # Flags (only if CANFD)
        if frame.is_fd:
            flags = 0
            if frame.brs:
                flags |= CANFD_BRS
            payload.append(flags)
            
        # Data
        payload.extend(frame.data)
        
    return payload

def unpack_cannelloni_packet(data):
    if len(data) < 5:
        return []
        
    version, opcode, seq, count = struct.unpack('!BBBH', data[:5])
    
    if version != CANNELLONI_VERSION:
        return []
        
    frames = []
    offset = 5
    
    for _ in range(count):
        if offset + 5 > len(data):
            break
            
        can_id = struct.unpack('!I', data[offset:offset+4])[0]
        offset += 4
        
        len_byte = data[offset]
        offset += 1
        
        is_fd = (len_byte & 0x80) != 0
        data_len = len_byte & 0x7F
        
        brs = False
        if is_fd:
            if offset + 1 > len(data):
                break
            flags = data[offset]
            offset += 1
            brs = (flags & CANFD_BRS) != 0
            
        if offset + data_len > len(data):
            break
            
        frame_data = data[offset:offset+data_len]
        offset += data_len
        
        frames.append(CannelloniFrame(can_id, frame_data, is_fd, brs))
        
    return frames

class ChannelStats:
    def __init__(self):
        self.tx_frames = 0
        self.rx_frames = 0
        self.lock = threading.Lock()

class ChannelTester:
    def __init__(self, ch_index, target_ip, base_port, frequency, duration):
        self.ch_index = ch_index
        self.target_ip = target_ip
        self.port = base_port + ch_index
        self.frequency = frequency
        self.duration = duration
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0) # 1 second timeout
        self.stop_event = threading.Event()
        self.stats = ChannelStats()
        self.seq = 0
        
    def receiver_loop(self):
        print(f"Channel {self.ch_index}: Receiver started on port {self.port}")
        while not self.stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(2048)
                frames = unpack_cannelloni_packet(data)
                with self.stats.lock:
                    self.stats.rx_frames += len(frames)
            except socket.timeout:
                continue
            except Exception as e:
                if not self.stop_event.is_set():
                    print(f"Channel {self.ch_index} Rx Error: {e}")

    def sender_loop(self):
        print(f"Channel {self.ch_index}: Sender started ({self.frequency} Hz)")
        
        # Calculate interval and batch size
        # If freq > 1000, we might need to batch frames per packet to keep up
        # Standard Ethernet MTU ~1500. Each frame ~70-80 bytes.
        # Can fit ~15-20 frames per packet.
        
        # Let's target 1ms sleep interval for stability
        batch_size = max(1, int(self.frequency / 1000))
        interval = 1.0 / (self.frequency / batch_size)
        
        next_time = time.time()
        
        while not self.stop_event.is_set():
            frames = []
            for _ in range(batch_size):
                # Create random CAN-FD frame
                can_id = 0x123 + self.ch_index
                dlc_len = 64
                data = bytearray([random.randint(0, 255) for _ in range(dlc_len)])
                frames.append(CannelloniFrame(can_id, data, is_fd=True, brs=True))
            
            packet = pack_cannelloni_packet(self.seq, frames)
            self.seq = (self.seq + 1) % 256
            
            self.sock.sendto(packet, (self.target_ip, self.port))
            
            with self.stats.lock:
                self.stats.tx_frames += len(frames)
            
            next_time += interval
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

    def run(self):
        rx_thread = threading.Thread(target=self.receiver_loop)
        tx_thread = threading.Thread(target=self.sender_loop)
        
        rx_thread.start()
        tx_thread.start()
        
        start_time = time.time()
        while time.time() - start_time < self.duration:
            time.sleep(1)
            with self.stats.lock:
                print(f"CH{self.ch_index} Stats: Tx {self.stats.tx_frames} | Rx {self.stats.rx_frames}")
        
        self.stop_event.set()
        rx_thread.join()
        tx_thread.join()
        self.sock.close()

def main():
    parser = argparse.ArgumentParser(description='Cannelloni Windows Tester')
    parser.add_argument('--ip', type=str, required=True, help='Target MCU IP Address')
    parser.add_argument('--channels', type=int, default=1, help='Number of channels (1-4)')
    parser.add_argument('--freq', type=int, default=4000, help='Tx Frequency in Hz')
    parser.add_argument('--duration', type=int, default=10, help='Test duration in seconds')
    
    args = parser.parse_args()
    
    testers = []
    threads = []
    
    print(f"Starting test: Target={args.ip}, Channels={args.channels}, Freq={args.freq}Hz")
    
    for i in range(args.channels):
        tester = ChannelTester(i, args.ip, 20000, args.freq, args.duration)
        testers.append(tester)
        t = threading.Thread(target=tester.run)
        threads.append(t)
        t.start()
        
    for t in threads:
        t.join()
        
    print("Test Complete")

if __name__ == "__main__":
    main()
