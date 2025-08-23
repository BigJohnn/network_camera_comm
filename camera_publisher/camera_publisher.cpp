#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <csignal>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <map>
#include <zmq.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <lz4.h>
#include <lz4frame.h>

// --- USB连接优化配置 ---
// FIX: Update serial numbers to match your actual cameras
const std::vector<std::string> SERIAL_NUMBERS = {"317422071787", "243322073887"};
const std::string ZMQ_ENDPOINT = "tcp://*:5555";
const std::string ZMQ_TOPIC = "D435i_STREAM";

const int COLOR_WIDTH = 640;
const int COLOR_HEIGHT = 480;
const int DEPTH_WIDTH = 640;
const int DEPTH_HEIGHT = 480;
const int FPS = 30;

// USB连接专用同步设定 - 优化版本
const int64_t SYNC_WINDOW_MS = 30;        // 同步窗口 - 减少到30ms提高响应性
const int64_t MAX_FRAME_AGE_MS = 200;     // 最大帧年龄 - 减少到200ms
const int MAX_BUFFER_SIZE = 2;            // 每个相机最多保留2帧 - 减少缓存

// 全局变量
std::atomic<bool> g_running(true);

// 简化的帧数据结构
struct SimpleFrameData {
    std::string serial;
    int64_t relative_timestamp;  // 相对时间戳 (相机启动后的ms)
    int64_t system_time;        // 系统接收时间
    cv::Mat color_mat;
    std::vector<char> depth_compressed;
    int frame_number;
    
    SimpleFrameData() : relative_timestamp(0), system_time(0), frame_number(0) {}
    
    SimpleFrameData(const std::string& s, int64_t ts, int64_t st, 
                   const cv::Mat& color, const std::vector<char>& depth_comp, int fn)
        : serial(s), relative_timestamp(ts), system_time(st), 
          color_mat(color.clone()), depth_compressed(depth_comp), frame_number(fn) {}
};

// 简化的同步器 - 基于系统时间而不是相机时间戳
class SimplifiedUSBSynchronizer {
private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::vector<SimpleFrameData>> sync_queue_;
    std::map<std::string, std::queue<SimpleFrameData>> camera_buffers_;
    
public:
    SimplifiedUSBSynchronizer() {}
    
    void add_frame(const SimpleFrameData& frame_data) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 添加到对应相机的缓冲区
        camera_buffers_[frame_data.serial].push(frame_data);
        
        // 限制缓冲区大小并清理过旧的帧
        cleanup_buffers();
        
        // 尝试同步 - 基于系统接收时间
        auto synced_frames = attempt_synchronization();
        if (!synced_frames.empty()) {
            sync_queue_.push(synced_frames);
            cv_.notify_one();
        }
    }
    
    bool get_synchronized_frames(std::vector<SimpleFrameData>& frames) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(100), 
                    [this] { return !sync_queue_.empty() || !g_running; });
        
        if (!g_running) return false;
        
        if (!sync_queue_.empty()) {
            frames = sync_queue_.front();
            sync_queue_.pop();
            return true;
        }
        return false;
    }
    
private:
    std::vector<SimpleFrameData> attempt_synchronization() {
        std::vector<SimpleFrameData> result;
        
        // 检查是否所有相机都有数据 (改为更严格的同步要求)
        for (const std::string& serial : SERIAL_NUMBERS) {
            auto it = camera_buffers_.find(serial);
            if (it == camera_buffers_.end() || it->second.empty()) {
                static int debug_counter = 0;
                if (++debug_counter % 100 == 0) { // 每100次失败输出一次调试信息
                    std::cout << "Sync failed: Camera " << serial << " has no data" << std::endl;
                }
                return result; // 如果任何相机没有数据就不同步
            }
        }
        
        // 找到所有相机的最新帧时间戳
        std::map<std::string, SimpleFrameData> latest_frames;
        for (const std::string& serial : SERIAL_NUMBERS) {
            auto& buffer = camera_buffers_[serial];
            if (!buffer.empty()) {
                latest_frames[serial] = buffer.back(); // 获取最新帧
            }
        }
        
        // 如果不是所有相机都有最新帧，返回空
        if (latest_frames.size() != SERIAL_NUMBERS.size()) {
            return result;
        }
        
        // 计算时间戳范围
        int64_t min_time = LLONG_MAX;
        int64_t max_time = LLONG_MIN;
        
        for (const auto& pair : latest_frames) {
            min_time = std::min(min_time, pair.second.system_time);
            max_time = std::max(max_time, pair.second.system_time);
        }
        
        // 检查时间差是否在同步窗口内
        if (max_time - min_time <= SYNC_WINDOW_MS) {
            // 从每个缓冲区移除选中的帧
            for (const std::string& serial : SERIAL_NUMBERS) {
                auto& buffer = camera_buffers_[serial];
                std::queue<SimpleFrameData> new_buffer;
                bool removed = false;
                
                // 移除匹配的帧（从后往前找最新的）
                std::vector<SimpleFrameData> temp_frames;
                while (!buffer.empty()) {
                    temp_frames.push_back(buffer.front());
                    buffer.pop();
                }
                
                // 移除最后一个帧（最新的），其余放回
                for (size_t i = 0; i < temp_frames.size() - 1; ++i) {
                    new_buffer.push(temp_frames[i]);
                }
                
                buffer = new_buffer;
                result.push_back(latest_frames[serial]);
            }
            
            // 调试输出 - 增加延迟监控
            static int sync_debug_counter = 0;
            static int64_t last_sync_time = 0;
            int64_t current_sync_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            if (++sync_debug_counter % 30 == 0) {  // 每30次同步输出一次
                int64_t sync_interval = last_sync_time > 0 ? current_sync_time - last_sync_time : 0;
                std::cout << "Synchronized " << result.size() << " cameras, time range: " 
                        << (max_time - min_time) << "ms, sync interval: " << sync_interval << "ms" << std::endl;
            }
            last_sync_time = current_sync_time;
        }
        
        return result;
    }
    
    void cleanup_buffers() {
        int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        for (auto& pair : camera_buffers_) {
            std::queue<SimpleFrameData>& buffer = pair.second;
            std::queue<SimpleFrameData> new_buffer;
            
            // 转换为vector以便操作
            std::vector<SimpleFrameData> frames;
            while (!buffer.empty()) {
                frames.push_back(buffer.front());
                buffer.pop();
            }
            
            // 只保留最新的MAX_BUFFER_SIZE个帧，并且不能太旧
            int keep_count = 0;
            for (int i = frames.size() - 1; i >= 0 && keep_count < MAX_BUFFER_SIZE; --i) {
                if (current_time - frames[i].system_time <= MAX_FRAME_AGE_MS) {
                    new_buffer.push(frames[i]);
                    keep_count++;
                }
            }
            
            // 重建队列（注意：这会改变顺序，但对于我们的同步逻辑来说问题不大）
            buffer = new_buffer;
        }
    }
};

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nStopping publisher..." << std::endl;
        g_running = false;
    }
}

std::vector<char> compress_depth_lz4(const rs2::depth_frame& frame) {
    if (!frame) return std::vector<char>();

    const char* source_data = (const char*)frame.get_data();
    size_t source_size = frame.get_data_size();

    if (source_size == 0) {
        std::cerr << "!!! Error: Depth frame is empty, cannot compress." << std::endl;
        return std::vector<char>();
    }

    size_t max_compressed_size = LZ4F_compressFrameBound(source_size, NULL);
    std::vector<char> compressed_buffer(max_compressed_size);

    size_t compressed_size = LZ4F_compressFrame(
        compressed_buffer.data(), max_compressed_size,
        source_data, source_size,
        NULL
    );

    if (LZ4F_isError(compressed_size)) {
        std::cerr << "!!! LZ4 frame compression failed: " << LZ4F_getErrorName(compressed_size) << std::endl;
        return std::vector<char>();
    }

    compressed_buffer.resize(compressed_size);
    return compressed_buffer;
}

// USB优化的相机配置
rs2::pipeline setup_usb_camera(const std::string& serial) {
    rs2::context ctx;
    rs2::config cfg;
    
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_BGR8, FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, FPS);
    
    rs2::pipeline pipe(ctx);
    rs2::pipeline_profile profile = pipe.start(cfg);
    
    // USB连接优化设置
    rs2::device dev = profile.get_device();
    
    try {
        // 禁用自动曝光以减少帧间延迟差异
        auto sensors = dev.query_sensors();
        for (rs2::sensor& sensor : sensors) {
            if (sensor.is<rs2::color_sensor>()) {
                rs2::color_sensor color_sensor = sensor.as<rs2::color_sensor>();
                // 方案1：使用更低的固定曝光值
                if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                    color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
                    // 大幅降低曝光值 - 根据环境调整
                    color_sensor.set_option(RS2_OPTION_EXPOSURE, 600);  // 从8500降到3000
                    std::cout << "Set fixed exposure to 3000" << std::endl;
                }
                
                // 添加增益控制
                if (color_sensor.supports(RS2_OPTION_GAIN)) {
                    color_sensor.set_option(RS2_OPTION_GAIN, 64);  // 较低的增益值
                    std::cout << "Set gain to 64" << std::endl;
                }
                
                // 设置固定白平衡
                if (color_sensor.supports(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE)) {
                    color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0);
                    if (color_sensor.supports(RS2_OPTION_WHITE_BALANCE)) {
                        color_sensor.set_option(RS2_OPTION_WHITE_BALANCE, 4600); // 日光白平衡
                        std::cout << "Set white balance to 4600K" << std::endl;
                    }
                }
                
                // 可选：设置对比度和饱和度
                if (color_sensor.supports(RS2_OPTION_CONTRAST)) {
                    color_sensor.set_option(RS2_OPTION_CONTRAST, 50);  // 默认通常是50
                }
                if (color_sensor.supports(RS2_OPTION_SATURATION)) {
                    color_sensor.set_option(RS2_OPTION_SATURATION, 64); // 默认通常是64
                }
            }
        }
        
        std::cout << "USB camera " << serial << " configured with fixed settings" << std::endl;
        
    } catch (const rs2::error& e) {
        std::cerr << "Warning: Could not optimize settings for " << serial << ": " << e.what() << std::endl;
    }
    
    return pipe;
}

void camera_thread_func(rs2::pipeline& pipe, const std::string& serial, SimplifiedUSBSynchronizer& synchronizer) {
    std::cout << "USB camera thread started for " << serial << std::endl;
    rs2::align align_to_color(RS2_STREAM_COLOR);
    int frame_count = 0;

    while (g_running) {
        try {
            rs2::frameset frames = pipe.wait_for_frames(1000);
            if (!frames) continue;

            frames = align_to_color.process(frames);
            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();

            if (!color_frame || !depth_frame) continue;
            
            // 记录系统接收时间
            int64_t system_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            // 彩色影像处理
            cv::Mat color_mat(cv::Size(COLOR_WIDTH, COLOR_HEIGHT), CV_8UC3, 
                             (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            
            // 深度影像压缩
            std::vector<char> depth_encoded = compress_depth_lz4(depth_frame);
            if (depth_encoded.empty()) continue;

            // 获取相机相对时间戳（保持原始值用于调试）
            int64_t camera_timestamp = static_cast<int64_t>(depth_frame.get_timestamp());
            
            // 创建帧数据 - 主要基于系统时间同步
            SimpleFrameData frame_data(serial, camera_timestamp, system_time, 
                                     color_mat, depth_encoded, frame_count++);
            
            // 添加到同步器
            synchronizer.add_frame(frame_data);
            
            // 每30帧输出一次调试信息
            if (frame_count % 30 == 0) {
                std::cout << "Camera " << serial << " - Frame " << frame_count 
                          << ", Camera TS: " << camera_timestamp 
                          << ", System Time: " << system_time 
                          << " (diff: " << (system_time - camera_timestamp) << "ms)" << std::endl;
            }
            
        } catch (const rs2::error& e) {
            std::cerr << "RealSense error in thread for " << serial << ": " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Exception in thread for " << serial << ": " << e.what() << std::endl;
        }
    }
    std::cout << "USB camera thread for " << serial << " is stopping." << std::endl;
}

void zmq_publisher_thread_bundled(zmq::socket_t& publisher, SimplifiedUSBSynchronizer& synchronizer) {
    std::cout << "ZMQ publisher thread started (Bundled message mode)" << std::endl;
    int sync_group_count = 0;
    
    while (g_running) {
        std::vector<SimpleFrameData> synced_frames;
        if (!synchronizer.get_synchronized_frames(synced_frames)) {
            continue;
        }
        
        try {
            sync_group_count++;
            
            // 构建打包消息的结构
            std::vector<char> bundled_message;
            
            // 消息头：同步组ID + 相机数量
            int32_t camera_count = static_cast<int32_t>(synced_frames.size());
            bundled_message.insert(bundled_message.end(), 
                                 reinterpret_cast<char*>(&sync_group_count), 
                                 reinterpret_cast<char*>(&sync_group_count) + sizeof(sync_group_count));
            bundled_message.insert(bundled_message.end(), 
                                 reinterpret_cast<char*>(&camera_count), 
                                 reinterpret_cast<char*>(&camera_count) + sizeof(camera_count));
            
            // 为每个相机编码数据
            for (const SimpleFrameData& frame_data : synced_frames) {
                // 编码彩色影像 - 降低质量提高传输速度
                std::vector<uchar> color_encoded;
                cv::imencode(".jpg", frame_data.color_mat, color_encoded, 
                            std::vector<int>{cv::IMWRITE_JPEG_QUALITY, 65});
                
                // 序列号长度和内容
                int32_t serial_len = static_cast<int32_t>(frame_data.serial.length());
                bundled_message.insert(bundled_message.end(), 
                                     reinterpret_cast<char*>(&serial_len), 
                                     reinterpret_cast<char*>(&serial_len) + sizeof(serial_len));
                bundled_message.insert(bundled_message.end(), 
                                     frame_data.serial.begin(), frame_data.serial.end());
                
                // 时间戳
                int64_t timestamp = frame_data.system_time;
                bundled_message.insert(bundled_message.end(), 
                                     reinterpret_cast<char*>(&timestamp), 
                                     reinterpret_cast<char*>(&timestamp) + sizeof(timestamp));
                
                // 彩色数据长度和内容
                int32_t color_len = static_cast<int32_t>(color_encoded.size());
                bundled_message.insert(bundled_message.end(), 
                                     reinterpret_cast<char*>(&color_len), 
                                     reinterpret_cast<char*>(&color_len) + sizeof(color_len));
                bundled_message.insert(bundled_message.end(), 
                                     color_encoded.begin(), color_encoded.end());
                
                // 深度数据长度和内容
                int32_t depth_len = static_cast<int32_t>(frame_data.depth_compressed.size());
                bundled_message.insert(bundled_message.end(), 
                                     reinterpret_cast<char*>(&depth_len), 
                                     reinterpret_cast<char*>(&depth_len) + sizeof(depth_len));
                bundled_message.insert(bundled_message.end(), 
                                     frame_data.depth_compressed.begin(), 
                                     frame_data.depth_compressed.end());
            }
            
            // 发送单个打包消息
            publisher.send(zmq::buffer(ZMQ_TOPIC), zmq::send_flags::sndmore);
            publisher.send(zmq::buffer(bundled_message), zmq::send_flags::none);
            
            if (sync_group_count % 30 == 0) {
                // 计算发送延迟
                int64_t send_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                int64_t frame_age = send_time - synced_frames[0].system_time;
                
                std::cout << "Bundled sync group #" << sync_group_count 
                          << " sent with " << synced_frames.size() << " cameras, size: " 
                          << bundled_message.size() << " bytes, frame age: " << frame_age << "ms" << std::endl;
            }
                      
        } catch (const std::exception& e) {
            std::cerr << "Exception in ZMQ publisher: " << e.what() << std::endl;
        }
    }
    std::cout << "ZMQ publisher thread is stopping." << std::endl;
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 初始化 ZMQ
    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind(ZMQ_ENDPOINT);
    std::cout << "Publisher bound to " << ZMQ_ENDPOINT << std::endl;

    // 初始化简化的USB同步器
    SimplifiedUSBSynchronizer synchronizer;

    std::vector<rs2::pipeline> pipelines;
    std::vector<std::thread> camera_threads;

    try {
        // 初始化所有相机（USB连接模式）
        for (const std::string& serial : SERIAL_NUMBERS) {
            std::cout << "Initializing USB camera " << serial << std::endl;
            rs2::pipeline pipe = setup_usb_camera(serial);
            pipelines.push_back(std::move(pipe));
        }
        
        std::cout << "All USB cameras initialized. Waiting for stabilization..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // 启动相机执行緒
        for (size_t i = 0; i < pipelines.size(); ++i) {
            camera_threads.push_back(std::thread(camera_thread_func, 
                                               std::ref(pipelines[i]), 
                                               SERIAL_NUMBERS[i], 
                                               std::ref(synchronizer)));
        }
        
        // 启动 ZMQ 发送执行緒
        std::thread zmq_thread(zmq_publisher_thread_bundled, std::ref(publisher), std::ref(synchronizer));
        
        std::cout << "\n=== Simplified USB Multi-Camera Publisher Started ===" << std::endl;
        std::cout << "Sync window: " << SYNC_WINDOW_MS << "ms (system time based)" << std::endl;
        std::cout << "Max buffer size: " << MAX_BUFFER_SIZE << " frames per camera" << std::endl;
        std::cout << "Press Ctrl+C to stop..." << std::endl;
        
        // 等待退出信號
        while(g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 清理
        std::cout << "\nStopping all threads..." << std::endl;
        
        for (std::thread& t : camera_threads) {
            if (t.joinable()) t.join();
        }
        
        if (zmq_thread.joinable()) {
            zmq_thread.join();
        }
        
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
        g_running = false;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        g_running = false;
        return 1;
    }

    std::cout << "USB multi-camera publisher stopped." << std::endl;
    return 0;
}