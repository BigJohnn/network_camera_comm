import zmq
import numpy as np
import cv2
import lz4.frame
import struct
import time

def main():
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://192.168.252.82:5555")
    subscriber.setsockopt_string(zmq.SUBSCRIBE, "D435i_STREAM")
    print("Subscriber connected to tcp://192.168.252.82:5555")

    # 儲存兩個相機的最新幀
    latest_frames = {}

    while True:
        try:
            # 接收 multipart message
            topic = subscriber.recv_string()
            serial = subscriber.recv_string()
            timestamp_bytes = subscriber.recv()
            color_jpeg = subscriber.recv()
            depth_lz4 = subscriber.recv()
            # print(f"topic: {topic}")
            # print(f"serial: {serial}")
            # print(f"timestamp: {timestamp_bytes}")
            # print(f"color_jpeg size: {len(color_jpeg)} bytes")
            # print(f"depth_lz4 size: {len(depth_lz4)} bytes")
            # 解包
            timestamp = struct.unpack('<q', timestamp_bytes)[0]
            
            # 解碼/解壓縮
            color_frame = cv2.imdecode(np.frombuffer(color_jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
            # print(f"color_frame shape: {color_frame.shape}")
            depth_frame = np.frombuffer(lz4.frame.decompress(depth_lz4), dtype=np.uint16).reshape(480, 640)
            
            # 更新最新幀
            latest_frames[serial] = {
                "color": color_frame,
                "depth": depth_frame,
                "timestamp": timestamp
            }

            display_frames = []
            # 確保按固定順序顯示 (Ensure display in a fixed order)
            for s_num in ["243222076185", "243322073850"]:
                if s_num in latest_frames and latest_frames[s_num]["color"] is not None:
                    color = latest_frames[s_num]["color"].copy()
                    
                    # --- FIX START ---
                    # Get the ALREADY DECOMPRESSED depth frame directly
                    depth_frame = latest_frames[s_num]["depth"]
                    # REMOVED: depth_data = lz4.frame.decompress(latest_frames[s_num]["depth"]) 
                    # --- FIX END ---
                    
                    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)
                    
                    # 加上序列號和延遲訊息 (Add serial number and latency info)
                    latency = int(time.time() * 1000) - latest_frames[s_num]["timestamp"]
                    cv2.putText(color, f"SN: {s_num}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.putText(color, f"Latency: {latency}ms", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    
                    # combined_camera_view = np.hstack((color, depth_colormap))
                    # display_frames.append(combined_camera_view)
                    display_frames.append(color)

            if len(display_frames) > 0:
                # If two cameras are ready, stack their combined views vertically
                if len(display_frames) == 2:
                    final_view = np.vstack(display_frames)
                    cv2.imshow('Dual RealSense Stream', final_view)
                # Otherwise, just show the one that is ready
                else:
                    cv2.imshow('Dual RealSense Stream', display_frames[0])


            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        except Exception as e:
            print(f"An error occurred: {e}")
            time.sleep(1)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()