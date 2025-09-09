import zmq
import numpy as np
import cv2
import lz4.frame
import struct
import time
import sys

def unpack_bundled_message(message_data):
    """解析打包消息"""
    cameras = {}
    offset = 0
    
    # 解析消息头
    sync_group_count = struct.unpack('<i', message_data[offset:offset+4])[0]
    offset += 4
    
    camera_count = struct.unpack('<i', message_data[offset:offset+4])[0]
    offset += 4
    
    print(f"Unpacking sync group #{sync_group_count} with {camera_count} cameras")
    
    # 解析每个相机的数据
    for i in range(camera_count):
        try:
            # 序列号长度和内容
            serial_len = struct.unpack('<i', message_data[offset:offset+4])[0]
            offset += 4
            serial = message_data[offset:offset+serial_len].decode('utf-8')
            offset += serial_len
            
            # 时间戳
            timestamp = struct.unpack('<q', message_data[offset:offset+8])[0]
            offset += 8
            
            # 彩色数据
            color_len = struct.unpack('<i', message_data[offset:offset+4])[0]
            offset += 4
            color_data = message_data[offset:offset+color_len]
            offset += color_len
            
            # 深度数据
            depth_len = struct.unpack('<i', message_data[offset:offset+4])[0]
            offset += 4
            depth_data = message_data[offset:offset+depth_len]
            offset += depth_len
            
            # 解码图像
            color_frame = cv2.imdecode(np.frombuffer(color_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            # 检查彩色帧解码
            if color_frame is None:
                print(f"Error: Failed to decode color frame for camera {serial}")
                continue
            
            # 获取彩色帧的实际尺寸
            color_height, color_width = color_frame.shape[:2]
            
            # 解码深度帧
            try:
                decompressed_depth = lz4.frame.decompress(depth_data)
                # 根据彩色帧的尺寸来推断深度帧的尺寸
                depth_frame = np.frombuffer(decompressed_depth, dtype=np.uint16).reshape(color_height, color_width)
            except Exception as e:
                print(f"Error: Failed to decode depth frame for camera {serial}: {e}")
                continue
            
            cameras[serial] = {
                "color": color_frame,
                "depth": depth_frame,
                "timestamp": timestamp,
                "sync_group": sync_group_count
            }
            
        except Exception as e:
            print(f"Error unpacking camera {i}: {e}")
            break
    
    return sync_group_count, cameras

def main():
    # 解析命令行参数
    if len(sys.argv) > 1 and ':' in sys.argv[1]:
        # 如果第一个参数包含冒号，认为是IP:PORT格式
        publisher_endpoint = sys.argv[1]
        if not publisher_endpoint.startswith('tcp://'):
            publisher_endpoint = f"tcp://{publisher_endpoint}"
        zmq_endpoint = publisher_endpoint
    elif len(sys.argv) > 1:
        # 只提供IP地址，使用默认端口
        publisher_ip = sys.argv[1]
        zmq_endpoint = f"tcp://{publisher_ip}:5555"
    else:
        # 使用默认配置
        # zmq_endpoint = "tcp://192.168.11.82:5555"
        zmq_endpoint = "tcp://192.168.1.33:5555"
    
    # 主题名称（可通过第二个参数覆盖）
    zmq_topic = sys.argv[2] if len(sys.argv) > 2 else "D435i_STREAM"
    
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect(zmq_endpoint)
    subscriber.setsockopt_string(zmq.SUBSCRIBE, zmq_topic)
    print(f"订阅者已连接到 {zmq_endpoint}，主题: {zmq_topic}")
    print("将根据接收到的消息自动检测相机数量")

    expected_cameras = []  # 动态检测相机数量，不依赖配置
    window_created = False  # 跟踪窗口是否已创建
    
    # 性能监控变量
    frame_count = 0
    total_network_latency = 0
    max_latency = 0
    min_latency = float('inf')
    
    # 时钟偏移检测
    clock_offset = 0
    clock_offset_samples = []
    is_clock_synced = True

    while True:
        try:
            # 记录消息接收时间
            receive_time = int(time.time() * 1000)
            processing_start = time.time()
            
            # 接收打包消息 (带超时)
            if subscriber.poll(100):  # 缩短等待时间到100ms
                topic = subscriber.recv_string(zmq.NOBLOCK)
                bundled_data = subscriber.recv(zmq.NOBLOCK)
                print(f"Received message with topic: {topic}, data size: {len(bundled_data)} bytes")
                
                # 清理积压的消息，只处理最新的
                messages_dropped = 0
                while subscriber.poll(0):  # 立即检查是否还有消息
                    try:
                        subscriber.recv_string(zmq.NOBLOCK)
                        subscriber.recv(zmq.NOBLOCK)
                        messages_dropped += 1
                    except zmq.Again:
                        break
                
                if messages_dropped > 0:
                    print(f"Dropped {messages_dropped} old messages to stay current")
                    
            else:
                print("No message received in 100ms...")
                continue
            
            # 解析打包消息
            try:
                sync_group_count, cameras = unpack_bundled_message(bundled_data)
                frame_count += 1
                print(f"Successfully unpacked {len(cameras)} cameras from sync group #{sync_group_count}")
                
                # 动态适应相机数量（根据实际接收到的消息）
                if not expected_cameras and cameras:
                    expected_cameras = sorted(list(cameras.keys()))  # 排序以保证顺序一致
                    print(f"自动检测到 {len(expected_cameras)} 个相机: {expected_cameras}")
                elif expected_cameras and set(cameras.keys()) != set(expected_cameras):
                    # 相机列表发生变化
                    old_cameras = set(expected_cameras)
                    new_cameras = set(cameras.keys())
                    added = new_cameras - old_cameras
                    removed = old_cameras - new_cameras
                    if added:
                        print(f"新增相机: {added}")
                    if removed:
                        print(f"断开相机: {removed}")
                    expected_cameras = sorted(list(new_cameras))
            except Exception as e:
                print(f"Error unpacking message: {e}")
                continue
            
            # 检查是否收到了所有期望的相机
            print(f"Processing cameras: received {len(cameras)}, expected {len(expected_cameras)}")
            if cameras and len(cameras) >= len(expected_cameras):
                print("Starting frame processing...")
                display_frames = []
                
                network_latencies = []
                
                for s_num in expected_cameras:
                    if s_num in cameras and cameras[s_num]["color"] is not None:
                        color = cameras[s_num]["color"].copy()
                        depth_frame = cameras[s_num]["depth"]
                        
                        # 创建深度彩色图
                        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
                        
                        # 计算网络延迟（发送时间戳到接收时间）
                        send_timestamp = cameras[s_num]["timestamp"]
                        raw_latency = receive_time - send_timestamp
                        
                        # 时钟偏移检测和补偿
                        if len(clock_offset_samples) < 30:  # 前30帧用于检测时钟偏移
                            clock_offset_samples.append(raw_latency)
                            if len(clock_offset_samples) == 30:
                                # 计算平均偏移
                                avg_offset = sum(clock_offset_samples) / len(clock_offset_samples)
                                if avg_offset < -10 or avg_offset > 1000:  # 异常延迟范围
                                    clock_offset = avg_offset
                                    is_clock_synced = False
                                    print(f"Detected clock offset: {clock_offset:.1f}ms")
                                else:
                                    print("Clocks appear to be synchronized")
                        
                        # 应用时钟偏移补偿
                        if not is_clock_synced and len(clock_offset_samples) >= 30:
                            # 如果是负偏移（发送端时钟快），需要减去偏移量
                            network_latency = raw_latency - clock_offset
                            latency_color = (0, 255, 255)  # 青色表示已补偿
                        else:
                            network_latency = raw_latency
                        
                        # 如果延迟仍然为负，使用绝对值并标记为异常
                        if network_latency < 0:
                            network_latency = abs(network_latency)
                            latency_color = (0, 0, 255)  # 红色表示时钟不同步
                        elif not is_clock_synced and len(clock_offset_samples) >= 30:
                            latency_color = (0, 255, 255)  # 青色表示已补偿
                        else:
                            latency_color = (0, 255, 0)  # 绿色表示正常
                        
                        network_latencies.append(network_latency)
                        
                        # 更新延迟统计
                        total_network_latency += network_latency
                        max_latency = max(max_latency, network_latency)
                        min_latency = min(min_latency, network_latency)
                        
                        # 计算处理时间
                        processing_time = (time.time() - processing_start) * 1000
                        
                        # 显示信息
                        cv2.putText(color, f"SN: {s_num}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(color, f"Net Latency: {network_latency:.1f}ms", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, latency_color, 1)
                        cv2.putText(color, f"Raw Latency: {raw_latency:.1f}ms", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)
                        cv2.putText(color, f"Proc Time: {processing_time:.1f}ms", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                        cv2.putText(color, f"Sync: #{sync_group_count}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        
                        if not is_clock_synced:
                            cv2.putText(color, f"CLOCK OFFSET: {clock_offset:.1f}ms", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                        elif latency_color == (0, 0, 255):
                            cv2.putText(color, f"CLOCK UNSYNC!", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                        else:
                            cv2.putText(color, f"PERFECT SYNC", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        
                        # 组合颜色和深度图像
                        combined_camera_view = np.hstack((color, depth_colormap))
                        display_frames.append(combined_camera_view)

                # 显示完美同步的画面
                print(f"Display frames ready: {len(display_frames)}/{len(expected_cameras)}")
                if len(display_frames) == len(expected_cameras):
                    print("Creating final view...")
                    final_view = np.vstack(display_frames)
                    window_title = f'Multi-Camera Synchronized Stream ({len(expected_cameras)} cameras)'
                    
                    # 首次创建窗口时设置属性
                    if not window_created:
                        cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
                        window_created = True
                        print(f"Created window: {window_title}")
                    
                    print(f"Showing frame #{frame_count}")
                    cv2.imshow(window_title, final_view)
                    cv2.waitKey(1)  # 立即处理事件
                    print("Frame displayed successfully")
                    
                    # 每30帧输出延迟统计
                    if frame_count % 30 == 0:
                        avg_latency = total_network_latency / frame_count
                        avg_network_latency = sum(network_latencies) / len(network_latencies)
                        print(f"Frame #{frame_count}: Avg Latency: {avg_latency:.1f}ms, "
                              f"Current Avg: {avg_network_latency:.1f}ms, "
                              f"Min: {min_latency}ms, Max: {max_latency}ms, "
                              f"Proc: {processing_time:.1f}ms")
                        if not is_clock_synced:
                            print(f"Clock offset applied: {clock_offset:.1f}ms")
                        
                        # 显示最近几帧的原始延迟用于调试
                        if len(network_latencies) >= 3:
                            recent_raw = [receive_time - cameras[s]["timestamp"] for s in expected_cameras if s in cameras]
                            print(f"Recent raw latencies: {[f'{x:.1f}' for x in recent_raw[-3:]]}")
            else:
                print(f"Warning: Received {len(cameras)} cameras, expected {len(expected_cameras)}")

            # 处理OpenCV事件并检查退出
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("User pressed 'q', exiting...")
                break
            elif key != 255:  # 如果按了任何键
                print(f"Key pressed: {key}")
            
            # 定期状态输出
            if frame_count > 0 and frame_count % 10 == 0:
                print(f"Processed {frame_count} frames so far...")

        except zmq.Again:
            continue
        except Exception as e:
            print(f"An error occurred: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    print("\n相机订阅器启动")
    print("使用方法:")
    print("  python camera_subscriber.py                     # 使用默认地址 192.168.11.82:5555")
    print("  python camera_subscriber.py <IP>                # 指定IP，默认端口5555")
    print("  python camera_subscriber.py <IP:PORT>           # 指定IP和端口")
    print("  python camera_subscriber.py <IP:PORT> <TOPIC>   # 指定IP、端口和主题")
    print("\n示例:")
    print("  python camera_subscriber.py 192.168.1.100")
    print("  python camera_subscriber.py 192.168.1.100:5556")
    print("  python camera_subscriber.py 192.168.1.100:5556 MY_TOPIC")
    print("\n说明: 订阅器将根据接收到的消息自动检测相机数量")
    print("-" * 50)
    main()