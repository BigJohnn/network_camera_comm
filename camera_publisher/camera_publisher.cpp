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

// ORBBEC SDK支持
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/h/ObTypes.h>

// ORBBEC相机结构 - 添加callback模式支持
struct OrbbecCameraContext {
    std::shared_ptr<ob::Device> device;
    std::shared_ptr<ob::Pipeline> pipeline;
    std::string serial;
    
    // Callback模式的线程安全帧处理
    std::mutex frameset_mutex;
    std::shared_ptr<ob::FrameSet> latest_frameset;
    std::atomic<bool> new_frame_available{false};
    
    OrbbecCameraContext(const std::string& s) : serial(s) {}
};

// --- USB连接优化配置 ---
// 动态检测已连接的相机
std::vector<std::string> SERIAL_NUMBERS;
const std::string ZMQ_ENDPOINT = "tcp://*:5555";
const std::string ZMQ_TOPIC = "D435i_STREAM";

const int COLOR_WIDTH = 640;
const int COLOR_HEIGHT = 480;
const int DEPTH_WIDTH = 640;
const int DEPTH_HEIGHT = 480;
const int FPS = 10;  // 进一步降低帧率到10fps以减少USB带宽压力

// USB连接专用同步设定 - 优化版本
const int64_t SYNC_WINDOW_MS = 100;        // 同步窗口 - 统一时间戳后缩小到100ms
const int64_t MAX_FRAME_AGE_MS = 200;     // 最大帧年龄 - 减少到200ms
const int MAX_BUFFER_SIZE = 5;            // 每个相机最多保留2帧 - 减少缓存

// 全局变量
std::atomic<bool> g_running(true);

// 相机类型枚举
enum class CameraType {
    REALSENSE,
    ORBBEC
};

// 简化的帧数据结构
struct SimpleFrameData {
    std::string serial;
    int64_t relative_timestamp;  // 相对时间戳 (相机启动后的ms)
    int64_t system_time;        // 系统接收时间
    cv::Mat color_mat;
    std::vector<char> depth_compressed;
    int frame_number;
    CameraType camera_type;     // 相机类型
    
    SimpleFrameData() : relative_timestamp(0), system_time(0), frame_number(0), camera_type(CameraType::REALSENSE) {}
    
    SimpleFrameData(const std::string& s, int64_t ts, int64_t st, 
                   const cv::Mat& color, const std::vector<char>& depth_comp, int fn, CameraType type = CameraType::REALSENSE)
        : serial(s), relative_timestamp(ts), system_time(st), 
          color_mat(color.clone()), depth_compressed(depth_comp), frame_number(fn), camera_type(type) {}
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
                if (++debug_counter % 50 == 0) { // 每50次失败输出一次调试信息
                    std::cout << "Sync failed: Camera " << serial << " has no data (attempt #" << debug_counter << ")" << std::endl;
                    // 输出所有相机的缓冲区状态
                    for (const std::string& s : SERIAL_NUMBERS) {
                        auto it2 = camera_buffers_.find(s);
                        if (it2 != camera_buffers_.end()) {
                            std::cout << "  Camera " << s << ": " << it2->second.size() << " frames in buffer" << std::endl;
                        } else {
                            std::cout << "  Camera " << s << ": buffer not found" << std::endl;
                        }
                    }
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

// 检测ORBBEC相机 - 适配v2.4.11
std::vector<std::pair<std::string, CameraType>> detect_orbbec_cameras() {
    std::vector<std::pair<std::string, CameraType>> orbbec_cameras;
    
    try {
        // v2.4.11 Context创建方式
        auto context = std::make_shared<ob::Context>();
        auto deviceList = context->queryDeviceList();
        
        std::cout << "\n=== Detecting Connected ORBBEC Cameras (v2.4.11) ===" << std::endl;
        std::cout << "Found " << deviceList->deviceCount() << " ORBBEC device(s)" << std::endl;
        
        for (uint32_t i = 0; i < deviceList->deviceCount(); i++) {
            try {
                auto device = deviceList->getDevice(i);
                auto deviceInfo = device->getDeviceInfo();
                
                std::string serial = deviceInfo->serialNumber();
                std::string name = deviceInfo->name();
                
                std::cout << "ORBBEC Camera found:" << std::endl;
                std::cout << "  - Name: " << name << std::endl;
                std::cout << "  - Serial: " << serial << std::endl;
                
                // 检查传感器支持
                auto sensorList = device->getSensorList();
                bool has_color = false;
                bool has_depth = false;
                
                for (uint32_t j = 0; j < sensorList->count(); j++) {
                    auto sensor = sensorList->getSensor(j);
                    auto sensorType = sensor->type();
                    if (sensorType == OB_SENSOR_COLOR) has_color = true;
                    if (sensorType == OB_SENSOR_DEPTH) has_depth = true;
                }
                
                if (has_color && has_depth) {
                    // 禁用Femto Bolt相机测试
                    if (name.find("Femto Bolt") != std::string::npos) {
                        std::cout << "  ✗ Femto Bolt camera DISABLED (bolt test disabled)" << std::endl;
                    } else {
                        orbbec_cameras.push_back({serial, CameraType::ORBBEC});
                        std::cout << "  ✓ Supports color and depth streams - ADDED" << std::endl;
                    }
                } else {
                    std::cout << "  ✗ Missing required streams - SKIPPED" << std::endl;
                }
                
            } catch (const std::exception& e) {
                std::cerr << "Error reading ORBBEC device info: " << e.what() << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "ORBBEC v2.4.11 detection error: " << e.what() << std::endl;
    }
    
    return orbbec_cameras;
}

// 自动检测已连接的相机（RealSense + ORBBEC）
std::vector<std::pair<std::string, CameraType>> detect_connected_cameras() {
    std::vector<std::pair<std::string, CameraType>> detected_cameras;
    
    // 检测RealSense相机
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        
        std::cout << "\n=== Detecting Connected RealSense Cameras ===" << std::endl;
        std::cout << "Found " << devices.size() << " RealSense device(s)" << std::endl;
        
        for (auto&& dev : devices) {
            try {
                std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
                std::string product_id = dev.get_info(RS2_CAMERA_INFO_PRODUCT_ID);
                
                std::cout << "Camera found:" << std::endl;
                std::cout << "  - Name: " << name << std::endl;
                std::cout << "  - Serial: " << serial << std::endl;
                std::cout << "  - Product ID: " << product_id << std::endl;
                
                // 检查是否支持所需的流
                bool has_color = false;
                bool has_depth = false;
                
                auto sensors = dev.query_sensors();
                for (auto&& sensor : sensors) {
                    auto profiles = sensor.get_stream_profiles();
                    for (auto&& profile : profiles) {
                        if (profile.stream_type() == RS2_STREAM_COLOR) {
                            has_color = true;
                        }
                        if (profile.stream_type() == RS2_STREAM_DEPTH) {
                            has_depth = true;
                        }
                    }
                }
                
                if (has_color && has_depth) {
                    detected_cameras.push_back({serial, CameraType::REALSENSE});
                    std::cout << "  ✓ Supports color and depth streams - ADDED" << std::endl;
                } else {
                    std::cout << "  ✗ Missing required streams (color:" << has_color << " depth:" << has_depth << ") - SKIPPED" << std::endl;
                }
                
                std::cout << std::endl;
                
            } catch (const std::exception& e) {
                std::cerr << "Error getting RealSense device info: " << e.what() << std::endl;
            }
        }
        
    } catch (const rs2::error& e) {
        std::cerr << "RealSense error during detection: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error during RealSense camera detection: " << e.what() << std::endl;
    }
    
    // 检测ORBBEC相机
    auto orbbec_cameras = detect_orbbec_cameras();
    detected_cameras.insert(detected_cameras.end(), orbbec_cameras.begin(), orbbec_cameras.end());
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Total cameras to use: " << detected_cameras.size() << std::endl;
    for (size_t i = 0; i < detected_cameras.size(); ++i) {
        const char* type_str = (detected_cameras[i].second == CameraType::REALSENSE) ? "RealSense" : "ORBBEC";
        std::cout << "  " << (i+1) << ". " << detected_cameras[i].first << " (" << type_str << ")" << std::endl;
    }
    std::cout << "===============================" << std::endl;
    
    return detected_cameras;
}

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

// ORBBEC相机初始化函数 - 适配v2.4.11 + 多设备同步
std::shared_ptr<OrbbecCameraContext> setup_orbbec_camera(const std::string& serial, bool is_primary = true, std::shared_ptr<ob::Context> shared_context = nullptr) {
    try {
        // 使用共享Context以便设备同步
        auto context = shared_context ? shared_context : std::make_shared<ob::Context>();
        auto deviceList = context->queryDeviceList();
        
        // 查找指定序列号的设备
        std::shared_ptr<ob::Device> target_device = nullptr;
        for (uint32_t i = 0; i < deviceList->deviceCount(); i++) {
            auto device = deviceList->getDevice(i);
            auto deviceInfo = device->getDeviceInfo();
            if (std::string(deviceInfo->serialNumber()) == serial) {
                target_device = device;
                break;
            }
        }
        
        if (!target_device) {
            throw std::runtime_error("ORBBEC camera with serial " + serial + " not found");
        }
        
        auto camera_ctx = std::make_shared<OrbbecCameraContext>(serial);
        camera_ctx->device = target_device;
        camera_ctx->pipeline = std::make_shared<ob::Pipeline>(target_device);
        
        // === 多设备同步配置 ===
        try {
            std::cout << "ORBBEC camera " << serial << " - Configuring multi-device sync (" 
                      << (is_primary ? "PRIMARY" : "SECONDARY") << " mode)" << std::endl;
            
            // 创建多设备同步配置结构
            OBMultiDeviceSyncConfig syncConfig;
            memset(&syncConfig, 0, sizeof(syncConfig));
            
            // 基于实际相机序列号配置同步模式
            bool should_be_primary = false;
            
            // CP728410006B 工作正常，设为主设备
            if (serial == "CP728410006B") {
                should_be_primary = true;
            }
            // CP769450002G 有问题，设为从设备  
            else if (serial == "CP769450002G") {
                should_be_primary = false;
            }
            // CL8M84101W7 Femto Bolt也设为从设备
            else if (serial == "CL8M84101W7") {
                should_be_primary = false;
            }
            // 其他情况使用传入的参数
            else {
                should_be_primary = is_primary;
            }
            
            if (should_be_primary) {
                // 主设备模式：生成同步信号 (基于原始配置)
                syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
                syncConfig.depthDelayUs = 0;
                syncConfig.colorDelayUs = 0;
                syncConfig.trigger2ImageDelayUs = 0;
                syncConfig.triggerOutEnable = true;
                syncConfig.triggerOutDelayUs = 0;
                syncConfig.framesPerTrigger = 1;
                
                target_device->setMultiDeviceSyncConfig(syncConfig);
                std::cout << "ORBBEC camera " << serial << " - Set as PRIMARY device (trigger generator)" << std::endl;
            } else {
                // 从设备模式：接收同步信号 (基于原始配置，但保持triggerOutEnable=true)
                syncConfig.syncMode = OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
                syncConfig.depthDelayUs = 0;
                syncConfig.colorDelayUs = 0;
                syncConfig.trigger2ImageDelayUs = 0;
                syncConfig.triggerOutEnable = true;  // 根据原始配置，从设备也启用trigger out
                syncConfig.triggerOutDelayUs = 0;
                syncConfig.framesPerTrigger = 1;
                
                target_device->setMultiDeviceSyncConfig(syncConfig);
                std::cout << "ORBBEC camera " << serial << " - Set as SECONDARY device (trigger receiver)" << std::endl;
            }
            
            // 启用设备时钟同步
            if (shared_context) {
                shared_context->enableDeviceClockSync(3000);  // 3秒同步窗口
                std::cout << "ORBBEC camera " << serial << " - Enabled device clock synchronization" << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Warning: Failed to configure multi-device sync for " << serial << ": " << e.what() << std::endl;
            std::cerr << "Continuing without hardware sync..." << std::endl;
        }
        
        // 配置流 - 尝试多种配置策略
        auto config = std::make_shared<ob::Config>();
        bool color_configured = false;
        bool depth_configured = false;
        
        // 检查是否是Femto Bolt相机
        bool is_femto_bolt = false;
        try {
            auto deviceInfo = target_device->getDeviceInfo();
            std::string device_name = deviceInfo->name();
            if (device_name.find("Femto Bolt") != std::string::npos) {
                is_femto_bolt = true;
                std::cout << "ORBBEC camera " << serial << " - Detected as Femto Bolt, using 720p configuration" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "Warning: Could not get device info: " << e.what() << std::endl;
        }
        
        // 策略1: 根据相机类型选择合适的分辨率配置
        if (is_femto_bolt) {
            std::cout << "ORBBEC camera " << serial << " - Trying strategy 1: Femto Bolt 1280x720 configuration" << std::endl;
        } else {
            std::cout << "ORBBEC camera " << serial << " - Trying strategy 1: Exact 640x480 configuration" << std::endl;
        }
        
        try {
            auto colorProfiles = camera_ctx->pipeline->getStreamProfileList(OB_SENSOR_COLOR);
            auto depthProfiles = camera_ctx->pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
            
            // 查找合适的彩色配置
            std::shared_ptr<ob::VideoStreamProfile> selectedColorProfile = nullptr;
            if (colorProfiles) {
                if (is_femto_bolt) {
                    // Femto Bolt: 查找1280x720配置
                    for (int fps : {15, 10, 30}) {
                        for (uint32_t i = 0; i < colorProfiles->count(); i++) {
                            auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                            if (profile->width() == 1280 && profile->height() == 720 && profile->fps() == static_cast<uint32_t>(fps)) {
                                selectedColorProfile = profile;
                                break;
                            }
                        }
                        if (selectedColorProfile) break;
                    }
                } else {
                    // 其他相机: 查找640x480配置
                    for (int fps : {15, 6, 30}) {
                        for (uint32_t i = 0; i < colorProfiles->count(); i++) {
                            auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                            if (profile->width() == 640 && profile->height() == 480 && profile->fps() == static_cast<uint32_t>(fps)) {
                                selectedColorProfile = profile;
                                break;
                            }
                        }
                        if (selectedColorProfile) break;
                    }
                }
            }
            
            // 查找合适的深度配置
            std::shared_ptr<ob::VideoStreamProfile> selectedDepthProfile = nullptr;
            if (depthProfiles) {
                if (is_femto_bolt) {
                    // Femto Bolt: 查找1280x720深度配置
                    for (int fps : {15, 10}) {
                        for (uint32_t i = 0; i < depthProfiles->count(); i++) {
                            auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                            if (profile->width() == 1280 && profile->height() == 720 && profile->fps() == static_cast<uint32_t>(fps)) {
                                selectedDepthProfile = profile;
                                break;
                            }
                        }
                        if (selectedDepthProfile) break;
                    }
                } else {
                    // 其他相机: 查找640x480@10fps深度配置
                    for (uint32_t i = 0; i < depthProfiles->count(); i++) {
                        auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                        if (profile->width() == 640 && profile->height() == 480 && profile->fps() == 10) {
                            selectedDepthProfile = profile;
                            break;
                        }
                    }
                }
            }
            
            if (selectedColorProfile && selectedDepthProfile) {
                config->enableStream(selectedColorProfile);
                config->enableStream(selectedDepthProfile);
                color_configured = true;
                depth_configured = true;
                if (is_femto_bolt) {
                    std::cout << "ORBBEC camera " << serial << " - Femto Bolt 1280x720 configuration successful: Color@" 
                              << selectedColorProfile->fps() << "fps, Depth@" << selectedDepthProfile->fps() << "fps (will resize to 640x480)" << std::endl;
                } else {
                    std::cout << "ORBBEC camera " << serial << " - Exact 640x480 configuration successful: Color@" 
                              << selectedColorProfile->fps() << "fps, Depth@" << selectedDepthProfile->fps() << "fps" << std::endl;
                }
            } else {
                if (is_femto_bolt) {
                    std::cout << "ORBBEC camera " << serial << " - No 1280x720 profiles found for Femto Bolt" << std::endl;
                } else {
                    std::cout << "ORBBEC camera " << serial << " - No 640x480@10fps profiles found" << std::endl;
                }
            }
        } catch (const std::exception& e) {
            std::cout << "ORBBEC camera " << serial << " - Exact configuration failed: " << e.what() << std::endl;
            config = std::make_shared<ob::Config>();  // 重置配置
        }
        
        // 策略2: 如果自动配置失败，尝试手动配置最低要求
        if (!color_configured || !depth_configured) {
            std::cout << "ORBBEC camera " << serial << " - Trying strategy 2: Manual minimal configuration" << std::endl;
            
            // 配置彩色流 - 使用任何可用配置
            try {
                auto colorProfiles = camera_ctx->pipeline->getStreamProfileList(OB_SENSOR_COLOR);
                if (colorProfiles && colorProfiles->count() > 0) {
                    // 使用第一个可用的彩色配置
                    auto profile = colorProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
                    config->enableStream(profile);
                    color_configured = true;
                    std::cout << "ORBBEC Color stream: " << profile->width() << "x" << profile->height() 
                              << " @" << profile->fps() << "fps (minimal config)" << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "ORBBEC camera " << serial << " - Failed to configure color stream: " << e.what() << std::endl;
            }
            
            // 配置深度流 - 使用任何可用配置
            try {
                auto depthProfiles = camera_ctx->pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
                if (depthProfiles && depthProfiles->count() > 0) {
                    // 使用第一个可用的深度配置
                    auto profile = depthProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
                    config->enableStream(profile);
                    depth_configured = true;
                    std::cout << "ORBBEC Depth stream: " << profile->width() << "x" << profile->height() 
                              << " @" << profile->fps() << "fps (minimal config)" << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "ORBBEC camera " << serial << " - Failed to configure depth stream: " << e.what() << std::endl;
            }
        }
        
        // 策略3: 如果仍然失败，尝试单独配置每个流
        if (!color_configured || !depth_configured) {
            std::cout << "ORBBEC camera " << serial << " - Trying strategy 3: Individual stream configuration" << std::endl;
            config = std::make_shared<ob::Config>();  // 重置配置
            
            if (!color_configured) {
                try {
                    // 仅启用彩色流
                    config->enableVideoStream(OB_STREAM_COLOR);
                    std::cout << "ORBBEC camera " << serial << " - Color stream only configuration" << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "ORBBEC camera " << serial << " - Failed to configure color-only: " << e.what() << std::endl;
                }
            }
            
            if (!depth_configured && color_configured) {
                try {
                    // 尝试添加深度流
                    config->enableVideoStream(OB_STREAM_DEPTH);
                    std::cout << "ORBBEC camera " << serial << " - Added depth stream to configuration" << std::endl;
                } catch (const std::exception& e) {
                    std::cerr << "ORBBEC camera " << serial << " - Failed to add depth stream: " << e.what() << std::endl;
                }
            }
        }
        
        if (!color_configured && !depth_configured) {
            throw std::runtime_error("Failed to configure any streams for ORBBEC camera " + serial);
        }
        
        // 启动Pipeline - v2.4.11方式，使用callback模式
        try {
            // 添加帧同步配置，确保彩色和深度帧同步
            try {
                camera_ctx->pipeline->enableFrameSync();
                std::cout << "ORBBEC camera " << serial << " - Frame synchronization enabled" << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Warning: Could not enable frame sync for " << serial << ": " << e.what() << std::endl;
            }
            
            // 启动pipeline时使用callback模式，基于官方示例
            camera_ctx->pipeline->start(config, [camera_ctx](std::shared_ptr<ob::FrameSet> frameset) {
                if (!frameset) return;
                
                // 线程安全地更新最新帧
                std::lock_guard<std::mutex> lock(camera_ctx->frameset_mutex);
                camera_ctx->latest_frameset = frameset;
                camera_ctx->new_frame_available = true;
            });
            
            // 验证设备是否真正开始流传输
            std::cout << "ORBBEC camera " << serial << " pipeline started, testing frameset availability..." << std::endl;
            
            // 检查设备连接状态
            try {
                auto deviceInfo = target_device->getDeviceInfo();
                std::cout << "ORBBEC camera " << serial << " device info:" << std::endl;
                std::cout << "  - Name: " << deviceInfo->name() << std::endl;
                std::cout << "  - Serial: " << deviceInfo->serialNumber() << std::endl;
                std::cout << "  - Firmware: " << deviceInfo->firmwareVersion() << std::endl;
                std::cout << "  - Hardware: " << deviceInfo->hardwareVersion() << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "Warning: Failed to get device info: " << e.what() << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Failed to start ORBBEC pipeline for " << serial << ": " << e.what() << std::endl;
            throw;
        }
        
        std::cout << "ORBBEC camera " << serial << " (v2.4.11) configured successfully" << std::endl;
        return camera_ctx;
        
    } catch (const std::exception& e) {
        std::cerr << "ORBBEC v2.4.11 camera setup error for " << serial << ": " << e.what() << std::endl;
        throw;
    }
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

void orbbec_camera_thread_func(std::shared_ptr<OrbbecCameraContext> camera_ctx, SimplifiedUSBSynchronizer& synchronizer) {
    std::cout << "ORBBEC camera thread (v2.4.11) started for " << camera_ctx->serial << std::endl;
    int frame_count = 0;
    int consecutive_failures = 0;

    while (g_running) {
        try {
            // v2.4.11 Callback模式 - 从缓存获取最新帧
            std::shared_ptr<ob::FrameSet> frameset = nullptr;
            
            // 检查是否有新帧可用
            {
                std::lock_guard<std::mutex> lock(camera_ctx->frameset_mutex);
                if (camera_ctx->new_frame_available && camera_ctx->latest_frameset) {
                    frameset = camera_ctx->latest_frameset;
                    camera_ctx->new_frame_available = false;  // 标记已处理
                }
            }
            
            // 如果没有新帧，等待一小段时间后继续
            if (!frameset) {
                consecutive_failures++;
                if (consecutive_failures % 50 == 0) {  // 每50次失败输出一次
                    std::cerr << "ORBBEC Camera " << camera_ctx->serial << " - No new frameset available (failure #" 
                              << consecutive_failures << ")" << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 短暂等待
                continue;
            }

            auto colorFrame = frameset->colorFrame();
            auto depthFrame = frameset->depthFrame();

            // 改进的帧检查逻辑：允许一定程度的帧不匹配
            bool has_color = (colorFrame != nullptr);
            bool has_depth = (depthFrame != nullptr);
            
            if (!has_color && !has_depth) {
                consecutive_failures++;
                std::cerr << "ORBBEC Camera " << camera_ctx->serial << " - No frames available (failure #" 
                          << consecutive_failures << ")" << std::endl;
                continue;
            }
            
            // 如果只有一种帧类型，尝试等待匹配的帧
            if (!has_color || !has_depth) {
                consecutive_failures++;
                std::cerr << "ORBBEC Camera " << camera_ctx->serial << " - Missing frames (color: " 
                          << (has_color ? "OK" : "MISSING") << ", depth: " 
                          << (has_depth ? "OK" : "MISSING") << ", failure #" 
                          << consecutive_failures << ")" << std::endl;
                          
                // 如果连续失败太多次，跳过这次处理但不退出
                if (consecutive_failures > 20) {
                    std::cerr << "ORBBEC Camera " << camera_ctx->serial 
                              << " - Too many frame mismatches, may have sync issues" << std::endl;
                    consecutive_failures = 0;  // 重置计数器
                }
                continue;
            }
            
            // 重置失败计数器
            consecutive_failures = 0;
            int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            // 记录系统接收时间
            int64_t system_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            // 转换彩色帧为OpenCV格式
            cv::Mat color_mat;
            if (colorFrame->format() == OB_FORMAT_RGB888) {
                color_mat = cv::Mat(colorFrame->height(), colorFrame->width(), CV_8UC3, (void*)colorFrame->data());
                cv::cvtColor(color_mat, color_mat, cv::COLOR_RGB2BGR);
            } else if (colorFrame->format() == OB_FORMAT_MJPG) {
                // MJPEG格式需要解码
                std::vector<uint8_t> jpeg_data(
                    static_cast<const uint8_t*>(colorFrame->data()),
                    static_cast<const uint8_t*>(colorFrame->data()) + colorFrame->dataSize()
                );
                color_mat = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
                if (color_mat.empty()) {
                    std::cerr << "Failed to decode MJPEG frame from ORBBEC camera" << std::endl;
                    continue;
                }
            } else {
                std::cerr << "Unsupported color format for ORBBEC camera: " << colorFrame->format() << std::endl;
                continue;
            }
            
            // 调试: 输出彩色帧尺寸（特别是Femto Bolt）
            static std::map<std::string, bool> logged_color_size;
            if (!logged_color_size[camera_ctx->serial]) {
                std::cout << "ORBBEC camera " << camera_ctx->serial << " - Color frame size: " 
                          << color_mat.cols << "x" << color_mat.rows << std::endl;
                logged_color_size[camera_ctx->serial] = true;
            }
            
            // 统一分辨率：智能调整ORBBEC相机帧到640x480
            if (color_mat.cols != COLOR_WIDTH || color_mat.rows != COLOR_HEIGHT) {
                // 特殊处理Femto Bolt的1280x720分辨率
                bool is_femto_size = (color_mat.cols == 1280 && color_mat.rows == 720);
                
                // 计算宽高比
                float src_ratio = (float)color_mat.cols / color_mat.rows;
                float dst_ratio = (float)COLOR_WIDTH / COLOR_HEIGHT;
                
                if (is_femto_size) {
                    // Femto Bolt: 1280x720 -> 640x480, 需要裁剪或缩放
                    // 720p (16:9) -> 480p (4:3), 宽高比不同，需要裁剪
                    int crop_width = color_mat.rows * 4 / 3; // 720 * 4/3 = 960
                    int start_x = (color_mat.cols - crop_width) / 2; // (1280-960)/2 = 160
                    
                    cv::Rect crop_rect(start_x, 0, crop_width, color_mat.rows);
                    cv::Mat cropped_color = color_mat(crop_rect).clone();
                    
                    // 然后缩放到640x480
                    cv::Mat resized_color;
                    cv::resize(cropped_color, resized_color, cv::Size(COLOR_WIDTH, COLOR_HEIGHT));
                    color_mat = resized_color;
                    
                    static std::map<std::string, bool> logged_femto_resize;
                    if (!logged_femto_resize[camera_ctx->serial]) {
                        std::cout << "ORBBEC Femto Bolt " << camera_ctx->serial << " - Cropped 1280x720 to 960x720 then resized to 640x480" << std::endl;
                        logged_femto_resize[camera_ctx->serial] = true;
                    }
                } else if (abs(src_ratio - dst_ratio) < 0.01) {
                    // 宽高比一致，使用resize
                    cv::Mat resized_color;
                    cv::resize(color_mat, resized_color, cv::Size(COLOR_WIDTH, COLOR_HEIGHT));
                    color_mat = resized_color;
                    
                    static std::map<std::string, bool> logged_resize;
                    if (!logged_resize[camera_ctx->serial]) {
                        std::cout << "ORBBEC camera " << camera_ctx->serial << " resized from " << colorFrame->width() << "x" << colorFrame->height() 
                                  << " to " << COLOR_WIDTH << "x" << COLOR_HEIGHT << " (same aspect ratio)" << std::endl;
                        logged_resize[camera_ctx->serial] = true;
                    }
                } else if (color_mat.cols == COLOR_WIDTH) {
                    // 宽度相同，高度不同，使用crop
                    int start_y = (color_mat.rows - COLOR_HEIGHT) / 2;
                    cv::Rect crop_rect(0, start_y, COLOR_WIDTH, COLOR_HEIGHT);
                    color_mat = color_mat(crop_rect).clone();
                    
                    static bool logged_crop = false;
                    if (!logged_crop) {
                        std::cout << "ORBBEC camera cropped from " << colorFrame->width() << "x" << colorFrame->height() 
                                  << " to " << COLOR_WIDTH << "x" << COLOR_HEIGHT << " (center crop)" << std::endl;
                        logged_crop = true;
                    }
                } else {
                    // 其他情况，默认使用resize
                    cv::Mat resized_color;
                    cv::resize(color_mat, resized_color, cv::Size(COLOR_WIDTH, COLOR_HEIGHT));
                    color_mat = resized_color;
                    
                    static bool logged_fallback = false;
                    if (!logged_fallback) {
                        std::cout << "ORBBEC camera fallback resize from " << colorFrame->width() << "x" << colorFrame->height() 
                                  << " to " << COLOR_WIDTH << "x" << COLOR_HEIGHT << std::endl;
                        logged_fallback = true;
                    }
                }
            }
            
            // 处理深度帧 - 压缩深度数据
            cv::Mat depth_mat(depthFrame->height(), depthFrame->width(), CV_16UC1, (void*)depthFrame->data());
            
            // 调试: 输出深度帧尺寸（特别是Femto Bolt）
            static std::map<std::string, bool> logged_depth_size;
            if (!logged_depth_size[camera_ctx->serial]) {
                std::cout << "ORBBEC camera " << camera_ctx->serial << " - Depth frame size: " 
                          << depth_mat.cols << "x" << depth_mat.rows << std::endl;
                logged_depth_size[camera_ctx->serial] = true;
            }
            
            // 统一深度帧分辨率：智能调整ORBBEC深度帧到640x480
            if (depth_mat.cols != DEPTH_WIDTH || depth_mat.rows != DEPTH_HEIGHT) {
                // 计算宽高比
                float src_ratio = (float)depth_mat.cols / depth_mat.rows;
                float dst_ratio = (float)DEPTH_WIDTH / DEPTH_HEIGHT;
                
                if (abs(src_ratio - dst_ratio) < 0.01) {
                    // 宽高比一致，使用resize
                    cv::Mat resized_depth;
                    cv::resize(depth_mat, resized_depth, cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), 0, 0, cv::INTER_NEAREST);
                    depth_mat = resized_depth;
                } else if (depth_mat.cols == DEPTH_WIDTH) {
                    // 宽度相同，高度不同，使用crop (640x576 -> 640x480)
                    int start_y = (depth_mat.rows - DEPTH_HEIGHT) / 2;
                    cv::Rect crop_rect(0, start_y, DEPTH_WIDTH, DEPTH_HEIGHT);
                    depth_mat = depth_mat(crop_rect).clone();
                    
                    static bool logged_depth_crop = false;
                    if (!logged_depth_crop) {
                        std::cout << "ORBBEC depth cropped from " << depthFrame->width() << "x" << depthFrame->height() 
                                  << " to " << DEPTH_WIDTH << "x" << DEPTH_HEIGHT << " (center crop)" << std::endl;
                        logged_depth_crop = true;
                    }
                } else {
                    // 其他情况，默认使用resize
                    cv::Mat resized_depth;
                    cv::resize(depth_mat, resized_depth, cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), 0, 0, cv::INTER_NEAREST);
                    depth_mat = resized_depth;
                }
            }
            
            // 使用LZ4压缩深度数据
            std::vector<char> depth_compressed;
            size_t depth_size = depth_mat.total() * depth_mat.elemSize();
            size_t max_compressed_size = LZ4F_compressFrameBound(depth_size, NULL);
            depth_compressed.resize(max_compressed_size);
            
            size_t compressed_size = LZ4F_compressFrame(
                depth_compressed.data(), max_compressed_size,
                depth_mat.data, depth_size, NULL
            );
            
            if (LZ4F_isError(compressed_size)) {
                std::cerr << "LZ4 compression failed for ORBBEC depth frame" << std::endl;
                continue;
            }
            
            depth_compressed.resize(compressed_size);
            
            // 获取时间戳 - ORBBEC相机时间戳不可靠，使用系统时间
            int64_t camera_timestamp = system_time;
            
            // 创建帧数据
            SimpleFrameData frame_data(camera_ctx->serial, camera_timestamp, system_time, 
                                     color_mat, depth_compressed, frame_count++, CameraType::ORBBEC);
            
            // 添加到同步器
            synchronizer.add_frame(frame_data);
            
            // 每30帧输出一次详细调试信息
            if (frame_count % 30 == 0) {
                std::cout << "ORBBEC Camera " << camera_ctx->serial << " - Frame " << frame_count 
                          << ", Timestamp: " << camera_timestamp 
                          << ", System Time: " << system_time 
                          << ", Health: " << consecutive_failures << " failures" << std::endl;
            }
            
            // 每10帧输出简单状态（包含健康状态）
            if (frame_count % 10 == 0) {
                std::cout << "ORBBEC " << camera_ctx->serial << " alive: Frame " << frame_count 
                          << " (health: " << (consecutive_failures == 0 ? "GOOD" : "DEGRADED") << ")" << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "ORBBEC error in thread for " << camera_ctx->serial << ": " << e.what() << std::endl;
        }
    }
    std::cout << "ORBBEC camera thread for " << camera_ctx->serial << " is stopping." << std::endl;
}

void camera_thread_func(rs2::pipeline& pipe, const std::string& serial, SimplifiedUSBSynchronizer& synchronizer) {
    std::cout << "RealSense camera thread started for " << serial << std::endl;
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

            // 统一使用系统时间戳实现多相机同步
            int64_t camera_timestamp = system_time;
            
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
        
        // 添加调试信息
        static int zmq_attempt_count = 0;
        if (++zmq_attempt_count % 50 == 0) {
            std::cout << "ZMQ thread: Attempt #" << zmq_attempt_count << " to get synchronized frames" << std::endl;
        }
        
        if (!synchronizer.get_synchronized_frames(synced_frames)) {
            continue;
        }
        
        std::cout << "ZMQ: Got " << synced_frames.size() << " synchronized frames" << std::endl;
        
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
            
            std::cout << "Sent sync group #" << sync_group_count 
                      << " with " << synced_frames.size() << " cameras, size: " 
                      << bundled_message.size() << " bytes" << std::endl;
            
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

    // 自动检测已连接的相机
    auto detected_cameras = detect_connected_cameras();
    
    if (detected_cameras.empty()) {
        std::cerr << "No compatible cameras found!" << std::endl;
        std::cerr << "Please ensure your cameras are connected and have both color and depth sensors." << std::endl;
        return 1;
    }

    // 提取序列号并保存相机类型信息
    SERIAL_NUMBERS.clear();
    std::map<std::string, CameraType> camera_types;
    for (const auto& camera : detected_cameras) {
        SERIAL_NUMBERS.push_back(camera.first);
        camera_types[camera.first] = camera.second;
    }

    std::cout << "\nUsing " << detected_cameras.size() << " camera(s) for streaming" << std::endl;

    // 初始化 ZMQ
    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind(ZMQ_ENDPOINT);
    std::cout << "Publisher bound to " << ZMQ_ENDPOINT << std::endl;

    // 初始化简化的USB同步器
    SimplifiedUSBSynchronizer synchronizer;

    std::vector<rs2::pipeline> pipelines;
    std::vector<std::shared_ptr<OrbbecCameraContext>> orbbec_cameras;
    std::vector<std::thread> camera_threads;

    // 为ORBBEC相机创建共享Context以支持多设备同步
    std::shared_ptr<ob::Context> orbbec_context = nullptr;
    std::vector<std::string> orbbec_serials;
    
    // 分离ORBBEC相机序列号
    for (const auto& camera : detected_cameras) {
        if (camera.second == CameraType::ORBBEC) {
            orbbec_serials.push_back(camera.first);
        }
    }
    
    // 如果有多个ORBBEC相机，创建共享Context
    if (orbbec_serials.size() > 1) {
        orbbec_context = std::make_shared<ob::Context>();
        std::cout << "Created shared ORBBEC context for " << orbbec_serials.size() << " cameras" << std::endl;
    }

    try {
        // 初始化所有相机 - 改进的ORBBEC多设备同步序列
        
        // 1. 先初始化所有RealSense相机
        for (size_t i = 0; i < SERIAL_NUMBERS.size(); ++i) {
            const std::string& serial = SERIAL_NUMBERS[i];
            
            if (camera_types[serial] == CameraType::REALSENSE) {
                std::cout << "Initializing RealSense camera " << serial << std::endl;
                rs2::pipeline pipe = setup_usb_camera(serial);
                pipelines.push_back(std::move(pipe));
            }
        }
        
        // 2. 然后按照ORBBEC官方建议：先初始化从设备，再初始化主设备
        if (!orbbec_serials.empty()) {
            std::cout << "\n=== ORBBEC Multi-Device Sync Sequence ===" << std::endl;
            
            // 先初始化所有从设备 (CP769450002G) - 禁用Femto Bolt测试
            std::vector<std::string> secondary_devices = {"CP769450002G"};
            for (const std::string& secondary_serial : secondary_devices) {
                // 检查该从设备是否存在于orbbec_serials中
                bool found = false;
                for (const std::string& serial : orbbec_serials) {
                    if (serial == secondary_serial) {
                        found = true;
                        break;
                    }
                }
                
                if (!found) continue; // 跳过不存在的从设备
                
                if (secondary_serial == "CP769450002G") {
                    std::cout << "Initializing ORBBEC SECONDARY device " << secondary_serial << " (Gemini 336 - problematic)" << std::endl;
                } else if (secondary_serial == "CL8M84101W7") {
                    std::cout << "Initializing ORBBEC SECONDARY device " << secondary_serial << " (Femto Bolt - 720p)" << std::endl;
                }
                
                // 添加延迟避免USB冲突
                int delay_ms = 3000;
                std::cout << "Adding " << delay_ms << "ms delay before initializing secondary camera" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
                
                // 重试机制初始化从设备
                int max_retries = 3;
                bool success = false;
                
                for (int retry = 0; retry < max_retries && !success; ++retry) {
                    try {
                        if (retry > 0) {
                            std::cout << "Retrying ORBBEC SECONDARY camera " << secondary_serial << " initialization (attempt " << (retry + 1) << "/" << max_retries << ")" << std::endl;
                            std::this_thread::sleep_for(std::chrono::milliseconds(1000 * retry));
                        }
                        
                        auto orbbec_ctx = setup_orbbec_camera(secondary_serial, false, orbbec_context);  // false = 从设备（但实际基于序列号判断）
                        orbbec_cameras.push_back(orbbec_ctx);
                        success = true;
                        
                    } catch (const std::exception& e) {
                        std::cerr << "Failed to initialize ORBBEC SECONDARY camera " << secondary_serial 
                                  << " (attempt " << (retry + 1) << "/" << max_retries << "): " << e.what() << std::endl;
                        
                        if (retry == max_retries - 1) {
                            std::cerr << "WARNING: Failed to initialize ORBBEC SECONDARY camera " << secondary_serial << " after " << max_retries << " attempts!" << std::endl;
                            // 不抛出异常，继续处理其他相机
                        }
                    }
                }
            }
            
            // 然后初始化主设备 CP728410006B
            for (const std::string& serial : orbbec_serials) {
                if (serial == "CP728410006B") {  // 工作正常的相机作为主设备
                    std::cout << "Initializing ORBBEC PRIMARY device " << serial << std::endl;
                    
                    // 给从设备稍微多一些启动时间
                    std::cout << "Allowing 5s for secondary device to stabilize..." << std::endl;
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                    
                    // 重试机制初始化主设备
                    int max_retries = 3;
                    bool success = false;
                    
                    for (int retry = 0; retry < max_retries && !success; ++retry) {
                        try {
                            if (retry > 0) {
                                std::cout << "Retrying ORBBEC PRIMARY camera " << serial << " initialization (attempt " << (retry + 1) << "/" << max_retries << ")" << std::endl;
                                std::this_thread::sleep_for(std::chrono::milliseconds(1000 * retry));
                            }
                            
                            auto orbbec_ctx = setup_orbbec_camera(serial, true, orbbec_context);  // true = 主设备（但实际基于序列号判断）
                            orbbec_cameras.push_back(orbbec_ctx);  // 按顺序添加
                            success = true;
                            
                        } catch (const std::exception& e) {
                            std::cerr << "Failed to initialize ORBBEC PRIMARY camera " << serial 
                                      << " (attempt " << (retry + 1) << "/" << max_retries << "): " << e.what() << std::endl;
                            
                            if (retry == max_retries - 1) {
                                std::cerr << "ERROR: Failed to initialize ORBBEC PRIMARY camera " << serial << " after " << max_retries << " attempts!" << std::endl;
                                throw;
                            }
                        }
                    }
                    break;  // 只处理一个主设备
                }
            }
        }
        
        std::cout << "All cameras initialized. Waiting for stabilization..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // 启动所有相机线程
        size_t pipeline_index = 0;
        size_t orbbec_index = 0;
        
        for (size_t i = 0; i < SERIAL_NUMBERS.size(); ++i) {
            if (camera_types[SERIAL_NUMBERS[i]] == CameraType::REALSENSE) {
                camera_threads.push_back(std::thread(camera_thread_func, 
                                                   std::ref(pipelines[pipeline_index]), 
                                                   SERIAL_NUMBERS[i], 
                                                   std::ref(synchronizer)));
                pipeline_index++;
            } else if (camera_types[SERIAL_NUMBERS[i]] == CameraType::ORBBEC) {
                camera_threads.push_back(std::thread(orbbec_camera_thread_func, 
                                                   orbbec_cameras[orbbec_index], 
                                                   std::ref(synchronizer)));
                orbbec_index++;
            }
        }
        
        // 启动 ZMQ 发送执行緒
        std::thread zmq_thread(zmq_publisher_thread_bundled, std::ref(publisher), std::ref(synchronizer));
        
        // 统计启用的相机数量
        int realsense_count = 0;
        int orbbec_count = 0;
        for (const auto& camera : detected_cameras) {
            if (camera.second == CameraType::REALSENSE) realsense_count++;
            else orbbec_count++;
        }
        
        std::cout << "\n=== Multi-Camera Publisher Started ===" << std::endl;
        std::cout << "Active RealSense cameras: " << realsense_count << std::endl;
        std::cout << "Active ORBBEC cameras: " << orbbec_count << std::endl;
        std::cout << "Sync window: " << SYNC_WINDOW_MS << "ms (optimized for low latency)" << std::endl;
        std::cout << "Max buffer size: " << MAX_BUFFER_SIZE << " frames per camera" << std::endl;
        std::cout << "JPEG quality: 65 (optimized for speed)" << std::endl;
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