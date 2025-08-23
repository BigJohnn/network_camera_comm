import zmq
import numpy as np
import cv2
import lz4.frame
import struct
import time

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
            depth_frame = np.frombuffer(lz4.frame.decompress(depth_data), dtype=np.uint16).reshape(480, 640)
            
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
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://192.168.252.82:5555")
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "D435i_STREAM")
    print("Subscriber connected to tcp://192.168.252.82:5555 (Bundled message mode)")

    expected_cameras = ["317422071787", "243322073887"]
    
    # 性能监控变量
    frame_count = 0
    total_network_latency = 0
    total_processing_time = 0
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
            
            # 接收打包消息
            topic = subscriber.recv_string()
            bundled_data = subscriber.recv()
            
            # 解析打包消息
            sync_group_count, cameras = unpack_bundled_message(bundled_data)
            frame_count += 1
            
            # 检查是否收到了所有期望的相机
            if len(cameras) == len(expected_cameras):
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
                if len(display_frames) == len(expected_cameras):
                    final_view = np.vstack(display_frames)
                    cv2.imshow('Perfect Synchronized Dual RealSense Stream', final_view)
                    
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

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except zmq.Again:
            continue
        except Exception as e:
            print(f"An error occurred: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(0.1)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()