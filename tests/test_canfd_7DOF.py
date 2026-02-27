import sys
import os
import time
import signal
import threading

# 将父目录加入路径以便导入 CAN 模块
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

try:
    import can
except ImportError:
    print("请先安装 python-can: pip install python-can")
    sys.exit(1)

try:
    from tzcan import CANMessageTransmitter, BasicConfig
except ImportError:
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    from tzcan import CANMessageTransmitter, BasicConfig
TZUSB2CANTransmitter = CANMessageTransmitter.choose_can_device("TZUSB2CAN")

def main():
    # 配置参数：接收CANFD数据（扩展帧、1M/4M、采样率都是75%）
    BAUD_RATE = 1000000       # 1M Nominal
    DATA_BAUD_RATE = 4000000  # 4M Data
    SAMPLE_POINT = 75.0       # 75%
    DATA_SAMPLE_POINT = 75.0  # 75%
    CHANNEL = 0               # 默认使用通道 0

    print("="*60)
    print(f"CANFD 接收测试工具 - 7DOF")
    print(f"配置要求: 扩展帧兼容, 1M/4M, 采样点 75%")
    print("="*60)
    
    print(f"正在初始化 CANFD 设备...")
    print(f" -> 仲裁域波特率: {BAUD_RATE/1000000}M")
    print(f" -> 数据域波特率: {DATA_BAUD_RATE/1000000}M")
    print(f" -> 采样点: {SAMPLE_POINT}%")

    # 初始化设备
    # init_can_device 会自动检测 candle 设备
    try:
        dev_config, ch0, ch1 = TZUSB2CANTransmitter.init_can_device(
            baud_rate=BAUD_RATE,
            dbit_baud_rate=DATA_BAUD_RATE,
            channels=[CHANNEL],
            fd=True,
            sp=SAMPLE_POINT,
            dsp=DATA_SAMPLE_POINT,
            backend='candle' # 显式指定 candle 后端，也可设为 None 自动检测
        )
    except Exception as e:
        print(f"❌ 初始化失败: {e}")
        return

    # 获取对应的 Bus 对象
    bus = None
    if CHANNEL == 0:
        bus = ch0
    elif CHANNEL == 1:
        bus = ch1
    else:
        # 如果是其他通道，从 dev_config['buses'] 获取
        bus = dev_config.get('buses', {}).get(CHANNEL)

    if not bus:
        print(f"❌ 未找到通道 {CHANNEL} 或初始化失败")
        TZUSB2CANTransmitter.close_can_device(dev_config)
        return

    # 实例化 Transmitter
    transmitter = TZUSB2CANTransmitter(bus, channel_id=CHANNEL)

    # --- 发送配置 (优化版：单线程轮询发送) ---
    ENABLE_SEND = True  # ✅ 启用发送功能
    SEND_IDS = [0x32C]
    SEND_DATA = [0x00, 0x00, 0x13, 0x88]
    SEND_FREQ = 200.0  # Hz (每帧的频率)
    
    # 轮询模式下，每个 ID 之间的发送间隔
    # 如果每个 ID 都要达到 SEND_FREQ，那么总线上的总帧率是 SEND_FREQ * len(SEND_IDS)
    # 两个 ID 发送之间的时间间隔 = 1 / (SEND_FREQ * len(SEND_IDS))
    TOTAL_MSG_COUNT = len(SEND_IDS)
    GLOBAL_SEND_FREQ = SEND_FREQ * TOTAL_MSG_COUNT
    INTER_FRAME_INTERVAL = 1.0 / GLOBAL_SEND_FREQ
    
    if ENABLE_SEND:
        print(f"\n🚀 启动 {TOTAL_MSG_COUNT} 组 ID 的轮询发送 (Polling)...")
        print(f"   目标单帧频率: {SEND_FREQ}Hz")
        print(f"   总线总帧率:   {GLOBAL_SEND_FREQ:.1f}Hz (间隔 {INTER_FRAME_INTERVAL*1000:.3f}ms)")
        print(f"   内容: {' '.join([f'{b:02X}' for b in SEND_DATA])}")
        print(f"   IDs:  {', '.join([hex(i) for i in SEND_IDS])}")
    else:
        print(f"\nℹ️ 发送功能已禁用。")
    
    # 发送控制标志
    sending_active = ENABLE_SEND
    
    def send_thread_func():
        """
        单线程轮询发送所有消息，使用自旋等待 (Busy Wait) 确保高精度。
        """
        next_time = time.perf_counter()
        
        while sending_active:
            # 轮询发送每个 ID
            for sid in SEND_IDS:
                if not sending_active:
                    break
                    
                try:
                    # 发送单个 ID
                    transmitter._send_can_data(
                        send_id=sid,
                        data_list=SEND_DATA,
                        is_ext_frame=True,
                        canfd_mode=True,
                        brs=1,
                        esi=0
                    )
                    
                    # 计算下一帧的目标时间
                    next_time += INTER_FRAME_INTERVAL
                    
                    # 自旋等待
                    while time.perf_counter() < next_time:
                        time.sleep(0.001)
                        
                except Exception as e:
                    print(f"发送错误: {e}")
                    time.sleep(1) # 出错后避让
                    # 重置时间基准
                    next_time = time.perf_counter()

    send_thread = threading.Thread(target=send_thread_func, daemon=True)
    
    try:
        if ENABLE_SEND:
            send_thread.start()
    except Exception as e:
        print(f"❌ 启动发送任务失败: {e}")
        TZUSB2CANTransmitter.close_can_device(dev_config)
        return

    total_count = 0
    start_time = time.time()
    
    # 用于统计每个 ID 的数据: {can_id: {'count': 0, 'last_ts': 0.0, 'freq': 0.0}}
    id_stats = {}
    stats_lock = threading.Lock()
    stop_print_event = threading.Event()
    
    # 监控配置
    # 设备 ID 映射：设备名 -> {该设备包含的 CAN ID 集合}
    DEVICE_MAP = {
        "0x21": {0x921, 0xE21},
        "0x25": {0x925, 0xE25},
        "0x29": {0x929, 0xE29},
        "0x2C": {0x92C, 0xE2C},
        "0x2D": {0x92D, 0xE2D},
        "0x2E": {0x92E, 0xE2E},
        "0x2F": {0x92F, 0xE2F},
    }
    
    # 生成所有需要监控的 ID 集合
    TARGET_IDS = set()
    for ids in DEVICE_MAP.values():
        TARGET_IDS.update(ids)
        
    EXPECTED_DEVICE_COUNT = len(DEVICE_MAP)
    TIMEOUT_THRESHOLD = 1.0  # 1秒
    monitor_started = False
    
    PRINT_INTERVAL = 0.5  # 每 0.5 秒刷新一次摘要

    print("\n✅ 开始接收数据 (按 Ctrl+C 退出)...")
    print(f"等待收集 {EXPECTED_DEVICE_COUNT} 个设备 ({len(TARGET_IDS)} 种 ID) 的数据...")
    print(f"监控设备: {', '.join(sorted(DEVICE_MAP.keys()))}")
    print("正在收集数据，将每 0.5s 刷新一次统计摘要...")

    # 最后一次收到有效数据的时间
    last_valid_data_time = time.time()
    # 判定设备掉线的超时时间（秒）
    # 如果超过这个时间没有收到任何有效数据（不包括错误帧），则认为设备掉线
    DEVICE_DEAD_TIMEOUT = 1.0 

    def print_loop():
        while not stop_print_event.is_set():
            time.sleep(PRINT_INTERVAL)
            with stats_lock:
                id_stats_snapshot = {cid: stats.copy() for cid, stats in id_stats.items()}
                total_count_snapshot = total_count
                monitor_started_snapshot = monitor_started
            
            current_time = time.time()
            os.system('cls' if os.name == 'nt' else 'clear')
            
            print(f"CANFD 7DOF 测试工具 - 统计摘要 (刷新率 {PRINT_INTERVAL}s)")
            print(f"全局总帧数: {total_count_snapshot}")
            online_devices = 0
            
            for cid, stats in id_stats_snapshot.items():
                period_dur = current_time - stats['period_start']
                if period_dur > 0.001:
                    stats['freq'] = stats['period_count'] / period_dur
                stats['period_count'] = 0
                stats['period_start'] = current_time

            for dev_name, ids in DEVICE_MAP.items():
                if any(cid in id_stats_snapshot for cid in ids):
                    online_devices += 1
            
            print(f"已上线设备: {online_devices} / {EXPECTED_DEVICE_COUNT}")
            
            status_header = "MONITORING" if monitor_started_snapshot else "WAITING"
            print(f"系统状态: [{status_header}]  (超时阈值: {TIMEOUT_THRESHOLD}s)")
            print("-" * 95)
            print(f"{'Device':<8} {'ID':<12} {'Type':<10} {'DLC':<5} {'Count':<10} {'Freq(Hz)':<12} {'Last Rx Time':<20} {'Status':<10}")
            print("-" * 95)
            
            for dev_name in sorted(DEVICE_MAP.keys()):
                ids = sorted(DEVICE_MAP[dev_name])
                for i, cid in enumerate(ids):
                    dev_str = dev_name if i == 0 else ""
                    
                    if cid in id_stats_snapshot:
                        stats = id_stats_snapshot[cid]
                        
                        type_flags = []
                        if stats['is_ext']: type_flags.append("EXT")
                        else: type_flags.append("STD")
                        if stats['is_fd']: type_flags.append("FD")
                        type_str = "/".join(type_flags)
                        
                        ago = current_time - stats['last_ts']
                        
                        id_status = "OK"
                        if ago > TIMEOUT_THRESHOLD:
                            id_status = "TIMEOUT"
                        
                        last_rx_str = f"{time.ctime(stats['last_ts'])[11:19]} ({ago:.1f}s)"
                        
                        print(f"{dev_str:<8} {hex(cid):<12} {type_str:<10} {stats['dlc']:<5} {stats['count']:<10} {stats['freq']:<12.1f} {last_rx_str:<20} {id_status:<10}")
                    else:
                        print(f"{dev_str:<8} {hex(cid):<12} {'-':<10} {'-':<5} {'0':<10} {'-':<12} {'NEVER':<20} {'WAIT':<10}")

            other_ids = sorted([cid for cid in id_stats_snapshot.keys() if cid not in TARGET_IDS])
            if other_ids:
                print("-" * 95)
                print(f"其他 ID ({len(other_ids)} 个):")
                for cid in other_ids:
                    stats = id_stats_snapshot[cid]
                    type_flags = []
                    if stats['is_ext']: type_flags.append("EXT")
                    else: type_flags.append("STD")
                    if stats['is_fd']: type_flags.append("FD")
                    type_str = "/".join(type_flags)
                    ago = current_time - stats['last_ts']
                    last_rx_str = f"{time.ctime(stats['last_ts'])[11:19]} ({ago:.1f}s)"
                    
                    print(f"{'Other':<8} {hex(cid):<12} {type_str:<10} {stats['dlc']:<5} {stats['count']:<10} {stats['freq']:<12.1f} {last_rx_str:<20} {'-':<10}")

            print("-" * 95)

    print_thread = threading.Thread(target=print_loop, daemon=True)
    print_thread.start()

    try:
        while True:
            # --- 批量接收模式 (Drain Buffer) ---
            # 每次循环尽可能多地读取缓冲区中的数据，避免堆积
            # 限制单次批量处理时间，防止 UI/统计 卡死
            batch_start_time = time.time()
            msgs_processed = 0
            
            while True:
                # 使用 timeout=0 进行非阻塞轮询
                success, rx_data, msg = transmitter._receive_can_data(
                    timeout=0, 
                    return_msg=True,
                    stop_on_error=False
                )
                
                if not success:
                    # 原版 TZUSB2CANTransmitter 在异常或无数据时都返回 False, [], None
                    # 我们无法区分是 Error 还是 Timeout，只能跳出
                    break
                
                if msg:
                    # --- 1. 外部过滤错误帧 ---
                    if getattr(msg, "is_error_frame", False):
                        # 忽略错误帧，不计入有效数据，也不计入统计
                        continue

                    # --- 2. 收到有效数据，更新时间戳 ---
                    last_valid_data_time = time.time()
                    msgs_processed += 1
                    with stats_lock:
                        total_count += 1
                    
                    # 更新 ID 统计
                    can_id = msg.arbitration_id
                    
                    # 统计所有接收到的 ID
                    with stats_lock:
                        if can_id not in id_stats:
                            id_stats[can_id] = {
                                'count': 1,
                                'period_count': 1,
                                'period_start': time.time(),
                                'last_ts': time.time(),
                                'freq': 0.0,
                                'dlc': msg.dlc,
                                'is_fd': msg.is_fd,
                                'is_ext': msg.is_extended_id
                            }
                        else:
                            stats = id_stats[can_id]
                            stats['count'] += 1
                            stats['period_count'] += 1
                            stats['dlc'] = msg.dlc
                            stats['is_fd'] = msg.is_fd
                            stats['is_ext'] = msg.is_extended_id
                            stats['last_ts'] = time.time()
                
                # 限制批量处理时间 (例如 20ms)
                if time.time() - batch_start_time > 0.02:
                    break
            
            # 如果这一轮没有处理任何消息，稍微休眠一下避免 CPU 100%
            if msgs_processed == 0:
                time.sleep(0.005)

            current_time = time.time()

            # --- 3. 基于数据流中断的自动重连逻辑 ---
            # 由于底层库吞掉了异常，我们只能通过“长时间收不到数据”来判断掉线
            time_since_last_data = current_time - last_valid_data_time
            if time_since_last_data > DEVICE_DEAD_TIMEOUT:
                print(f"\n🚨 警告: 已有 {time_since_last_data:.1f}s 未收到有效数据 (阈值 {DEVICE_DEAD_TIMEOUT}s)")
                print("   判定设备可能已掉线或死锁，尝试重置设备...")
                
                try:
                    print("🔄 关闭当前设备...")
                    TZUSB2CANTransmitter.close_can_device(dev_config)
                    time.sleep(2.0) # 等待 USB 设备复位
                    
                    print("🔄 重新初始化设备...")
                    # 重新调用初始化
                    dev_config, ch0, ch1 = TZUSB2CANTransmitter.init_can_device(
                        baud_rate=BAUD_RATE,
                        dbit_baud_rate=DATA_BAUD_RATE,
                        channels=[CHANNEL],
                        fd=True,
                        sp=SAMPLE_POINT,
                        dsp=DATA_SAMPLE_POINT,
                        backend='candle'
                    )
                    
                    # 更新 Bus 对象
                    new_bus = None
                    if CHANNEL == 0: new_bus = ch0
                    elif CHANNEL == 1: new_bus = ch1
                    else: new_bus = dev_config.get('buses', {}).get(CHANNEL)
                    
                    if new_bus:
                        transmitter.bus = new_bus
                        # 重置看门狗时间
                        last_valid_data_time = time.time()
                        print("✅ 设备重连成功，恢复监控...")
                    else:
                        print("❌ 重连后无法获取通道句柄")
                        # 稍微延时避免疯狂重试
                        last_valid_data_time = time.time() + 2.0 
                        
                except Exception as e:
                    print(f"❌ 重连失败: {e}")
                    # 避免死循环过快
                    time.sleep(2)
                    # 重置时间，给下一次重连机会
                    last_valid_data_time = time.time()

            # --- 监控逻辑 ---
            if not monitor_started:
                with stats_lock:
                    received_target_count = sum(1 for cid in TARGET_IDS if cid in id_stats)
                if received_target_count >= len(TARGET_IDS):
                    monitor_started = True
            
            if monitor_started:
                for dev_name, ids in DEVICE_MAP.items():
                    all_timeout = True
                    timeout_details = []
                    
                    for cid in ids:
                        with stats_lock:
                            stats = id_stats.get(cid)
                        if stats:
                            last_ts = stats['last_ts']
                            ago = current_time - last_ts
                            if ago <= TIMEOUT_THRESHOLD:
                                all_timeout = False
                                break 
                            else:
                                timeout_details.append(f"{hex(cid)} (超时 {ago:.1f}s)")
                        else:
                            timeout_details.append(f"{hex(cid)} (无数据)")
                    
                    if all_timeout:
                        print(f"\n\n❌ 错误: 设备 {dev_name} 掉线！")
                        print(f"   说明: 该设备下的所有 ID {ids} 均在 {TIMEOUT_THRESHOLD}s 内未收到数据。")
                        print(f"   详情: {', '.join(timeout_details)}")
                        print("程序停止。")
                        
                        sending_active = False
                        TZUSB2CANTransmitter.close_can_device(dev_config)
                        sys.exit(1)

            
    except KeyboardInterrupt:
        print("\n\n⏹️ 用户停止接收")
    except Exception as e:
        print(f"\n❌ 运行时错误: {e}")
    finally:
        stop_print_event.set()
        print("正在停止发送任务...")
        sending_active = False # 停止发送线程
        
        print("正在关闭设备...")
        TZUSB2CANTransmitter.close_can_device(dev_config)
        print("已退出。")

if __name__ == "__main__":
    main()
