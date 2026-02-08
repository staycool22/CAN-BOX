import can
import time
import threading
import argparse
import sys
import os

class ChannelStats:
    def __init__(self):
        self.tx_count = 0
        self.rx_count = 0
        self.last_tx_count = 0
        self.last_rx_count = 0
        self.last_time = time.perf_counter()

class ChannelHandler:
    def __init__(self, interface, frequency, duration, data_len, existing_bus=None, tzethcan_transmitter=None, tx_count_limit=None, rx_count_limit=None):
        self.interface = interface
        self.frequency = frequency
        self.duration = duration
        self.data_len = data_len
        self.stop_event = threading.Event()
        self.stats = ChannelStats()
        self.bus = existing_bus
        self.tx_thread = None
        self.rx_thread = None
        self.tzethcan_transmitter = tzethcan_transmitter
        self.tx_count_limit = tx_count_limit
        self.rx_count_limit = rx_count_limit
        self.rx_finished = False

    def start(self):
        if self.bus is None:
            try:
                # Create bus
                self.bus = can.interface.Bus(channel=self.interface, bustype='socketcan', fd=True)
                print(f"[{self.interface}] Bus opened.")
            except Exception as e:
                print(f"[{self.interface}] Error opening bus: {e}")
                return False
        else:
             print(f"[{self.interface}] Using existing bus.")

        # Create wrapper instance if provided class but not instance
        if self.tzethcan_transmitter:
            # We already have the bus, so we just wrap it if needed.
            # But wait, the user wants to use _send_can_data and _receive_can_data
            # We need an instance of TZETHCANTransmitter for THIS channel.
            # The previous init_can_device returned a device instance, we should use that.
            pass

        self.rx_thread = threading.Thread(target=self.rx_loop, name=f"{self.interface}-RX")
        self.tx_thread = threading.Thread(target=self.tx_loop, name=f"{self.interface}-TX")
        
        self.rx_thread.start()
        self.tx_thread.start()
        return True

    def stop(self):
        self.stop_event.set()
        if self.tx_thread and self.tx_thread.is_alive():
            self.tx_thread.join()
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join()
        if self.bus:
            self.bus.shutdown()
        print(f"[{self.interface}] Stopped.")

    def rx_loop(self):
        while not self.stop_event.is_set():
            try:
                # Use a small timeout to allow checking stop_event
                # Check limit
                if self.rx_count_limit is not None and self.stats.rx_count >= self.rx_count_limit:
                    self.rx_finished = True
                    # If we only care about RX limit and it's reached, we could stop?
                    # But maybe we still need to send?
                    # The requirement says "wait for rx count to match"
                    time.sleep(0.1)
                    continue

                if self.tzethcan_transmitter:
                    msg_tuple = self.tzethcan_transmitter._receive_can_data(timeout=0.1)
                    if msg_tuple:
                        self.stats.rx_count += 1
                else:
                    msg = self.bus.recv(timeout=0.1)
                    if msg:
                        self.stats.rx_count += 1
            except can.CanError:
                pass
            except Exception as e:
                print(f"[{self.interface}] RX Error: {e}")
                break

    def tx_loop(self):
        target_period = 1.0 / self.frequency
        batch_size = 1
        
        # Batching strategy for high frequency
        if self.frequency > 1000:
            # Aim for ~2ms per batch to reduce system call overhead and sleep jitter
            batch_size = int(self.frequency * 0.002) 
            if batch_size < 1: batch_size = 1
            target_period = batch_size / self.frequency
        
        print(f"[{self.interface}] Starting TX: {self.frequency}Hz, Batch: {batch_size}")

        # Prepare data for TZETHCANTransmitter if used
        data_list = [i % 255 for i in range(self.data_len)]
        
        msg = can.Message(
            arbitration_id=0x123,
            data=bytes(data_list),
            is_extended_id=False,
            is_fd=True,
            bitrate_switch=True
        )
        
        start_time = time.time()
        next_batch_time = time.perf_counter()
        
        while not self.stop_event.is_set():
            if self.duration > 0 and (time.time() - start_time > self.duration):
                break
            
            # Check TX limit
            if self.tx_count_limit is not None and self.stats.tx_count >= self.tx_count_limit:
                time.sleep(0.1)
                continue

            # Send batch
            for _ in range(batch_size):
                if self.tx_count_limit is not None and self.stats.tx_count >= self.tx_count_limit:
                    break
                    
                try:
                    if self.tzethcan_transmitter:
                        self.tzethcan_transmitter._send_can_data(0x123, data_list, is_ext_frame=False, canfd_mode=True, brs=1)
                    else:
                        self.bus.send(msg)
                    self.stats.tx_count += 1
                except can.CanError as e:
                    # Buffer full or other error
                    # print(f"[{self.interface}] TX Error: {e}")
                    time.sleep(0.0001) 
                except Exception as e:
                    print(f"[{self.interface}] TX Exception: {e}")
                    break
            
            # Timing control
            next_batch_time += target_period
            now = time.perf_counter()
            remaining = next_batch_time - now
            
            if remaining > 0.001:
                time.sleep(remaining)
            else:
                # Busy wait for high precision
                while time.perf_counter() < next_batch_time:
                    pass

def main():
    parser = argparse.ArgumentParser(description="Multi-channel CAN FD Send/Receive Test")
    parser.add_argument('-c', '--channels', default="vcan0,vcan1,vcan2,vcan3", help="Comma separated list of interfaces")
    parser.add_argument('-f', '--freq', default="4000", help='Frequency per channel in Hz. Can be single value or comma separated list matching channels (e.g. 2000,4000)')
    parser.add_argument('-t', '--time', type=float, default=10.0, help='Duration in seconds (default: 10)')
    parser.add_argument('-l', '--len', type=int, default=64, help='Data length in bytes (default: 64)')
    parser.add_argument('--tx-count', type=int, default=None, help='Number of packets to send (default: unlimited/time-based)')
    parser.add_argument('--rx-count', type=int, default=None, help='Number of packets to receive before exit (default: unlimited)')
    parser.add_argument('--tzethcan', action='store_true', help="Use TZETHCANTransmitter for variable baud rates (Configures Ch0=500k/2M, Ch2=1M/4M)")
    
    args = parser.parse_args()
    
    interfaces = [x.strip() for x in args.channels.split(',')]
    
    # Parse frequencies
    freq_list = []
    try:
        raw_freqs = [float(x.strip()) for x in args.freq.split(',')]
        if len(raw_freqs) == 1:
            freq_list = raw_freqs * len(interfaces)
        elif len(raw_freqs) == len(interfaces):
            freq_list = raw_freqs
        else:
            print(f"Error: Frequency count ({len(raw_freqs)}) must match channel count ({len(interfaces)}) or be 1.")
            sys.exit(1)
    except ValueError:
        print("Error: Invalid frequency format")
        sys.exit(1)

    handlers = []
    bus_map = {}
    tz_transmitters = {} # Map interface -> TZETHCANTransmitter instance

    if args.tzethcan:
        # Add path to find TZETHCANTransmitter
        sys.path.append(os.path.join(os.path.dirname(__file__), 'CAN-BOX', 'CAN'))
        try:
            from TZETHCANTransmitter import TZETHCANTransmitter
            print("Initializing TZETHCAN Channels with custom baud rates...")
            
            # Setup Ch0: 500k/2M
            if 'vcan0' in interfaces:
                print("Configuring Ch0: 500k/2M")
                # init_can_device returns (None, dev_instance)
                # Force can_type=1 (CANFD) for high speed test
                ret = TZETHCANTransmitter.init_can_device(baud_rate=500000, dbit_baud_rate=2000000, channels=[0], can_type=1)
                bus_map['vcan0'] = ret[1].bus
                tz_transmitters['vcan0'] = ret[1]

            # Setup Ch2: 1M/4M
            if 'vcan2' in interfaces:
                print("Configuring Ch2: 1M/4M")
                ret = TZETHCANTransmitter.init_can_device(baud_rate=1000000, dbit_baud_rate=4000000, channels=[2], can_type=1)
                bus_map['vcan2'] = ret[1].bus
                tz_transmitters['vcan2'] = ret[1]
                
            # Note: Other channels will default to standard SocketCAN open if not specially configured here
            
        except ImportError as e:
            print(f"Error: Could not import TZETHCANTransmitter: {e}")
            sys.exit(1)
        except Exception as e:
            print(f"Error initializing TZETHCAN: {e}")
            sys.exit(1)

    print(f"Preparing to test on {len(interfaces)} channels: {interfaces}")
    print(f"Targets: {freq_list} Hz, Payload: {args.len} bytes")
    if args.tx_count: print(f"TX Limit: {args.tx_count} packets")
    if args.rx_count: print(f"RX Target: {args.rx_count} packets")
    
    # Start all handlers
    for i, iface in enumerate(interfaces):
        existing_bus = bus_map.get(iface)
        # If using TZETHCAN mode but channel wasn't specially configured above, 
        # we might still want to use TZETHCANTransmitter wrapper if available?
        # But for now, only use it if we have an instance in tz_transmitters.
        tz_inst = tz_transmitters.get(iface)
        
        h = ChannelHandler(iface, freq_list[i], args.time, args.len, 
                           existing_bus=existing_bus, 
                           tzethcan_transmitter=tz_inst,
                           tx_count_limit=args.tx_count,
                           rx_count_limit=args.rx_count)
        if h.start():
            handlers.append(h)
        else:
            print(f"Failed to start {iface}, aborting.")
            for h in handlers: h.stop()
            return

    try:
        start_time = time.time()
        while True:
            # Check exit conditions
            # 1. Time limit (only if no RX count limit is set, OR if we want a hard timeout)
            # The user said: "if both set, wait for RX count to match"
            # Let's interpret: 
            # - If rx_count is set, run until ALL handlers finish RX (or user interrupt)
            # - If rx_count NOT set, run until time limit.
            
            all_rx_finished = True
            if args.rx_count:
                for h in handlers:
                    if not h.rx_finished:
                        all_rx_finished = False
                        break
                if all_rx_finished:
                    print("\nAll channels reached RX target. Exiting.")
                    break
            else:
                # Time based exit
                if time.time() - start_time > args.time:
                    print("\nTime limit reached.")
                    break

            time.sleep(1.0)
            
            # Print stats
            print("\n--- Status Report (1s) ---")
            total_tx_rate = 0
            total_rx_rate = 0
            now = time.perf_counter()
            
            for h in handlers:
                dt = now - h.stats.last_time
                if dt == 0: dt = 1.0 # Avoid div by zero on startup
                
                curr_tx = h.stats.tx_count
                curr_rx = h.stats.rx_count
                
                tx_rate = (curr_tx - h.stats.last_tx_count) / dt
                rx_rate = (curr_rx - h.stats.last_rx_count) / dt
                
                h.stats.last_tx_count = curr_tx
                h.stats.last_rx_count = curr_rx
                h.stats.last_time = now
                
                total_tx_rate += tx_rate
                total_rx_rate += rx_rate
                
                print(f"{h.interface}: TX {tx_rate:.1f} fps | RX {rx_rate:.1f} fps | Total TX {curr_tx} | Total RX {curr_rx}")
            
            print(f"TOTAL: TX {total_tx_rate:.1f} fps | RX {total_rx_rate:.1f} fps")
            
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("\nStopping threads...")
        for h in handlers:
            h.stop()
        print("Done.")

if __name__ == "__main__":
    main()
