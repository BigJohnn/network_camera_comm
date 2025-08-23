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

    while True:
        try:
            # 接收打包消息
            topic = subscriber.recv_string()
            bundled_data = subscriber.recv()
            
            # 解析打包消息
            sync_group_count, cameras = unpack_bundled_message(bundled_data)
            
            # 检查是否收到了所有期望的相机
            if len(cameras) == len(expected_cameras):
                display_frames = []
                
                for s_num in expected_cameras:
                    if s_num in cameras and cameras[s_num]["color"] is not None:
                        color = cameras[s_num]["color"].copy()
                        depth_frame = cameras[s_num]["depth"]
                        
                        # 创建深度彩色图
                        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
                        
                        # 计算延迟
                        current_time = int(time.time() * 1000)
                        latency = current_time - cameras[s_num]["timestamp"]
                        
                        # 显示信息
                        cv2.putText(color, f"SN: {s_num}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(color, f"Latency: {latency}ms", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(color, f"Sync: #{sync_group_count}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(color, f"PERFECT SYNC", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        
                        # 组合颜色和深度图像
                        combined_camera_view = np.hstack((color, depth_colormap))
                        display_frames.append(combined_camera_view)

                # 显示完美同步的画面
                if len(display_frames) == len(expected_cameras):
                    final_view = np.vstack(display_frames)
                    cv2.imshow('Perfect Synchronized Dual RealSense Stream', final_view)
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