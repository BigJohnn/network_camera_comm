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

// ORBBEC相机结构 - 简化的callback模式
struct OrbbecCameraContext {
    std::shared_ptr<ob::Device> device;
    std::shared_ptr<ob::Pipeline> pipeline;
    std::string serial;
    int device_index;
    
    // Callback模式的线程安全帧处理
    std::mutex frameset_mutex;
    std::shared_ptr<ob::FrameSet> latest_frameset;
    std::atomic<bool> new_frame_available{false};
    
    OrbbecCameraContext(const std::string& s, int idx) : serial(s), device_index(idx) {}
};

// --- USB连接优化配置 ---
std::vector<std::string> SERIAL_NUMBERS;
const std::string ZMQ_ENDPOINT = "tcp://*:5555";
const std::string ZMQ_TOPIC = "D435i_STREAM";

const int COLOR_WIDTH = 640;
const int COLOR_HEIGHT = 480;
const int DEPTH_WIDTH = 640;
const int DEPTH_HEIGHT = 480;
const int FPS = 10;

// USB连接专用同步设定
const int64_t SYNC_WINDOW_MS = 100;
const int64_t MAX_FRAME_AGE_MS = 200;
const int MAX_BUFFER_SIZE = 5;

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
    int64_t relative_timestamp;
    int64_t system_time;
    cv::Mat color_mat;
    std::vector<char> depth_compressed;
    int frame_number;
    CameraType camera_type;
    
    SimpleFrameData() : relative_timestamp(0), system_time(0), frame_number(0), camera_type(CameraType::REALSENSE) {}
    
    SimpleFrameData(const std::string& s, int64_t ts, int64_t st, 
                   const cv::Mat& color, const std::vector<char>& depth_comp, int fn, CameraType type = CameraType::REALSENSE)
        : serial(s), relative_timestamp(ts), system_time(st), 
          color_mat(color.clone()), depth_compressed(depth_comp), frame_number(fn), camera_type(type) {}
};

// 简化的同步器
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
        
        camera_buffers_[frame_data.serial].push(frame_data);
        cleanup_buffers();
        
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
        
        for (const std::string& serial : SERIAL_NUMBERS) {
            auto it = camera_buffers_.find(serial);
            if (it == camera_buffers_.end() || it->second.empty()) {
                return result;
            }
        }
        
        std::map<std::string, SimpleFrameData> latest_frames;
        for (const std::string& serial : SERIAL_NUMBERS) {
            auto& buffer = camera_buffers_[serial];
            if (!buffer.empty()) {
                latest_frames[serial] = buffer.back();
            }
        }
        
        if (latest_frames.size() != SERIAL_NUMBERS.size()) {
            return result;
        }
        
        int64_t min_time = LLONG_MAX;
        int64_t max_time = LLONG_MIN;
        
        for (const auto& pair : latest_frames) {
            min_time = std::min(min_time, pair.second.system_time);
            max_time = std::max(max_time, pair.second.system_time);
        }
        
        if (max_time - min_time <= SYNC_WINDOW_MS) {
            for (const std::string& serial : SERIAL_NUMBERS) {
                auto& buffer = camera_buffers_[serial];
                std::queue<SimpleFrameData> new_buffer;
                
                std::vector<SimpleFrameData> temp_frames;
                while (!buffer.empty()) {
                    temp_frames.push_back(buffer.front());
                    buffer.pop();
                }
                
                for (size_t i = 0; i < temp_frames.size() - 1; ++i) {
                    new_buffer.push(temp_frames[i]);
                }
                
                buffer = new_buffer;
                result.push_back(latest_frames[serial]);
            }
            
            static int sync_debug_counter = 0;
            if (++sync_debug_counter % 30 == 0) {
                std::cout << "Synchronized " << result.size() << " cameras, time range: " 
                        << (max_time - min_time) << "ms" << std::endl;
            }
        }
        
        return result;
    }
    
    void cleanup_buffers() {
        int64_t current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        for (auto& pair : camera_buffers_) {
            std::queue<SimpleFrameData>& buffer = pair.second;
            std::queue<SimpleFrameData> new_buffer;
            
            std::vector<SimpleFrameData> frames;
            while (!buffer.empty()) {
                frames.push_back(buffer.front());
                buffer.pop();
            }
            
            int keep_count = 0;
            for (int i = frames.size() - 1; i >= 0 && keep_count < MAX_BUFFER_SIZE; --i) {
                if (current_time - frames[i].system_time <= MAX_FRAME_AGE_MS) {
                    new_buffer.push(frames[i]);
                    keep_count++;
                }
            }
            
            buffer = new_buffer;
        }
    }
};

// Format conversion helper for ORBBEC
cv::Mat convertOrbbecFrame(std::shared_ptr<ob::ColorFrame> colorFrame, const std::string& serial) {
    cv::Mat color_mat;
    
    if (!colorFrame || colorFrame->dataSize() == 0) {
        std::cerr << "Invalid color frame from ORBBEC camera " << serial << std::endl;
        return color_mat;
    }
    
    auto format = colorFrame->format();
    int width = colorFrame->width();
    int height = colorFrame->height();
    
    // Debug format once per camera
    static std::map<std::string, bool> logged_format;
    if (!logged_format[serial]) {
        std::cout << "ORBBEC camera " << serial << " - Color format: " << format 
                  << " (" << width << "x" << height << ")" << std::endl;
        
        // Format code reference for ORBBEC
        if (format == OB_FORMAT_MJPG) {
            std::cout << "  Format " << format << " = MJPG" << std::endl;
        } else if (format == OB_FORMAT_YUYV) {
            std::cout << "  Format " << format << " = YUYV" << std::endl;
        } else if (format == OB_FORMAT_RGB888) {
            std::cout << "  Format " << format << " = RGB888" << std::endl;
        }
        logged_format[serial] = true;
    }
    
    try {
        switch(format) {
            case OB_FORMAT_RGB888:
                {
                    cv::Mat rgb_mat(height, width, CV_8UC3, (void*)colorFrame->data());
                    cv::cvtColor(rgb_mat, color_mat, cv::COLOR_RGB2BGR);
                }
                break;
                
            case OB_FORMAT_BGR:
                color_mat = cv::Mat(height, width, CV_8UC3, (void*)colorFrame->data()).clone();
                break;
                
            case OB_FORMAT_YUYV:
                {
                    cv::Mat yuyv_mat(height, width, CV_8UC2, (void*)colorFrame->data());
                    cv::cvtColor(yuyv_mat, color_mat, cv::COLOR_YUV2BGR_YUYV);
                }
                break;
                
            case OB_FORMAT_UYVY:
                {
                    cv::Mat uyvy_mat(height, width, CV_8UC2, (void*)colorFrame->data());
                    cv::cvtColor(uyvy_mat, color_mat, cv::COLOR_YUV2BGR_UYVY);
                }
                break;
                
            case OB_FORMAT_MJPG:  // This is format code 5 in your SDK
                {
                    std::vector<uint8_t> jpeg_data(
                        static_cast<const uint8_t*>(colorFrame->data()),
                        static_cast<const uint8_t*>(colorFrame->data()) + colorFrame->dataSize()
                    );
                    color_mat = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
                }
                break;
                
            case OB_FORMAT_BGRA:
                {
                    cv::Mat bgra_mat(height, width, CV_8UC4, (void*)colorFrame->data());
                    cv::cvtColor(bgra_mat, color_mat, cv::COLOR_BGRA2BGR);
                }
                break;
                
            case OB_FORMAT_RGBA:
                {
                    cv::Mat rgba_mat(height, width, CV_8UC4, (void*)colorFrame->data());
                    cv::cvtColor(rgba_mat, color_mat, cv::COLOR_RGBA2BGR);
                }
                break;
                
            case OB_FORMAT_UNKNOWN:
            default:
                // Try to detect format based on data size
                size_t data_size = colorFrame->dataSize();
                size_t expected_rgb = width * height * 3;
                size_t expected_yuyv = width * height * 2;
                size_t expected_rgba = width * height * 4;
                
                if (data_size == expected_rgb) {
                    std::cout << "ORBBEC camera " << serial << " - Unknown format (code " << format 
                              << "), trying BGR interpretation for 3-channel data" << std::endl;
                    color_mat = cv::Mat(height, width, CV_8UC3, (void*)colorFrame->data()).clone();
                } else if (data_size == expected_yuyv) {
                    std::cout << "ORBBEC camera " << serial << " - Unknown format (code " << format 
                              << "), trying YUYV interpretation for 2-channel data" << std::endl;
                    cv::Mat yuyv_mat(height, width, CV_8UC2, (void*)colorFrame->data());
                    cv::cvtColor(yuyv_mat, color_mat, cv::COLOR_YUV2BGR_YUYV);
                } else if (data_size == expected_rgba) {
                    std::cout << "ORBBEC camera " << serial << " - Unknown format (code " << format 
                              << "), trying BGRA interpretation for 4-channel data" << std::endl;
                    cv::Mat bgra_mat(height, width, CV_8UC4, (void*)colorFrame->data());
                    cv::cvtColor(bgra_mat, color_mat, cv::COLOR_BGRA2BGR);
                } else {
                    std::cerr << "Cannot determine format for ORBBEC camera " << serial 
                              << ". Format code: " << format << ", DataSize: " << data_size 
                              << ", Expected RGB: " << expected_rgb 
                              << ", Expected YUYV: " << expected_yuyv 
                              << ", Expected RGBA: " << expected_rgba << std::endl;
                    // Last resort: try to interpret as BGR anyway
                    if (data_size >= expected_rgb) {
                        std::cerr << "Attempting last resort BGR interpretation..." << std::endl;
                        color_mat = cv::Mat(height, width, CV_8UC3, (void*)colorFrame->data()).clone();
                    }
                }
                break;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error converting ORBBEC frame: " << e.what() << std::endl;
    }
    
    return color_mat;
}

// Detect ORBBEC cameras
std::vector<std::pair<std::string, CameraType>> detect_orbbec_cameras() {
    std::vector<std::pair<std::string, CameraType>> orbbec_cameras;
    
    try {
        ob::Context context;
        auto deviceList = context.queryDeviceList();
        
        std::cout << "\n=== Detecting ORBBEC Cameras ===" << std::endl;
        std::cout << "Found " << deviceList->getCount() << " ORBBEC device(s)" << std::endl;
        
        for (uint32_t i = 0; i < deviceList->getCount(); i++) {
            try {
                auto device = deviceList->getDevice(i);
                auto deviceInfo = device->getDeviceInfo();
                
                std::string serial = deviceInfo->serialNumber();
                std::string name = deviceInfo->name();
                
                std::cout << "ORBBEC Camera found:" << std::endl;
                std::cout << "  - Name: " << name << std::endl;
                std::cout << "  - Serial: " << serial << std::endl;
                
                orbbec_cameras.push_back({serial, CameraType::ORBBEC});
                
            } catch (const std::exception& e) {
                std::cerr << "Error reading ORBBEC device info: " << e.what() << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "ORBBEC detection error: " << e.what() << std::endl;
    }
    
    return orbbec_cameras;
}

// Setup ORBBEC camera - force 640x480 resolution
std::shared_ptr<OrbbecCameraContext> setup_orbbec_camera(std::shared_ptr<ob::Device> device, const std::string& serial, int device_index) {
    try {
        auto camera_ctx = std::make_shared<OrbbecCameraContext>(serial, device_index);
        camera_ctx->device = device;
        camera_ctx->pipeline = std::make_shared<ob::Pipeline>(device);
        
        auto config = std::make_shared<ob::Config>();
        
        // Get available stream profiles and select 640x480
        try {
            // Get color stream profiles
            auto colorProfiles = camera_ctx->pipeline->getStreamProfileList(OB_SENSOR_COLOR);
            std::shared_ptr<ob::VideoStreamProfile> selectedColorProfile = nullptr;
            
            if (colorProfiles) {
                std::cout << "ORBBEC camera " << serial << " - Available color profiles:" << std::endl;
                
                // First try to find exact 640x480 profile
                for (uint32_t i = 0; i < colorProfiles->count(); i++) {
                    auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    int width = profile->width();
                    int height = profile->height();
                    int fps = profile->fps();
                    auto format = profile->format();
                    
                    // Debug output
                    if (width == 640 || width == 1280) {
                        std::cout << "  " << width << "x" << height << " @" << fps << "fps, format: " << format << std::endl;
                    }
                    
                    // Look for 640x480 with any supported format, prioritize 15fps
                    if (width == 640 && height == 480) {
                        if (!selectedColorProfile || (profile->fps() <= 30 && profile->fps() >= 15)) {  // Prefer 15-30fps for stability
                            selectedColorProfile = profile;
                            std::cout << "  -> Selected 640x480 @" << fps << "fps" << std::endl;
                        }
                    }
                }
                
                // If no 640x480 found, we'll resize from 1280x720 later
                if (!selectedColorProfile) {
                    std::cout << "No 640x480 color profile found, will use 1280x720 and resize" << std::endl;
                    // Find 1280x720 profile as fallback, prefer 15-30fps
                    for (uint32_t i = 0; i < colorProfiles->count(); i++) {
                        auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                        if (profile->width() == 1280 && profile->height() == 720) {
                            if (!selectedColorProfile || (profile->fps() <= 30 && profile->fps() >= 15)) {
                                selectedColorProfile = profile;
                                std::cout << "  -> Using 1280x720 @" << profile->fps() << "fps for resizing" << std::endl;
                            }
                            if (profile->fps() <= 30 && profile->fps() >= 15) {
                                break; // Found good fps range, stop looking
                            }
                        }
                    }
                }
                
                if (selectedColorProfile) {
                    config->enableStream(selectedColorProfile);
                } else {
                    // Fallback to auto selection
                    config->enableVideoStream(OB_STREAM_COLOR);
                    std::cout << "Using auto color stream selection" << std::endl;
                }
            }
            
            // Get depth stream profiles  
            auto depthProfiles = camera_ctx->pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
            std::shared_ptr<ob::VideoStreamProfile> selectedDepthProfile = nullptr;
            
            if (depthProfiles) {
                // Try to find 640x480 depth, prefer 15-30fps
                for (uint32_t i = 0; i < depthProfiles->count(); i++) {
                    auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (profile->width() == 640 && profile->height() == 480) {
                        if (!selectedDepthProfile || (profile->fps() <= 30 && profile->fps() >= 15)) {
                            selectedDepthProfile = profile;
                            std::cout << "Selected depth: 640x480 @" << profile->fps() << "fps" << std::endl;
                        }
                        if (profile->fps() <= 30 && profile->fps() >= 15) {
                            break; // Found good fps range, stop looking
                        }
                    }
                }
                
                // If no 640x480, try 640x576 (common ORBBEC depth resolution), prefer 15-30fps
                if (!selectedDepthProfile) {
                    for (uint32_t i = 0; i < depthProfiles->count(); i++) {
                        auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                        if (profile->width() == 640 && profile->height() == 576) {
                            if (!selectedDepthProfile || (profile->fps() <= 30 && profile->fps() >= 15)) {
                                selectedDepthProfile = profile;
                                std::cout << "Selected depth: 640x576 @" << profile->fps() << "fps (will crop)" << std::endl;
                            }
                            if (profile->fps() <= 30 && profile->fps() >= 15) {
                                break; // Found good fps range, stop looking
                            }
                        }
                    }
                }
                
                // Last resort: use any available depth profile
                if (!selectedDepthProfile && depthProfiles->count() > 0) {
                    selectedDepthProfile = depthProfiles->getProfile(0)->as<ob::VideoStreamProfile>();
                    std::cout << "Using depth: " << selectedDepthProfile->width() << "x" 
                              << selectedDepthProfile->height() << " @" << selectedDepthProfile->fps() << "fps" << std::endl;
                }
                
                if (selectedDepthProfile) {
                    config->enableStream(selectedDepthProfile);
                } else {
                    config->enableVideoStream(OB_STREAM_DEPTH);
                }
            }
            
        } catch (const std::exception& e) {
            std::cerr << "Error configuring streams: " << e.what() << std::endl;
            // Fallback to simple configuration
            config->enableVideoStream(OB_STREAM_COLOR);
            config->enableVideoStream(OB_STREAM_DEPTH);
        }
        
        // Enable frame sync if available
        try {
            camera_ctx->pipeline->enableFrameSync();
            std::cout << "ORBBEC camera " << serial << " - Frame sync enabled" << std::endl;
        } catch (...) {
            std::cout << "ORBBEC camera " << serial << " - Frame sync not available" << std::endl;
        }
        
        // Start pipeline with callback
        camera_ctx->pipeline->start(config, [camera_ctx](std::shared_ptr<ob::FrameSet> frameset) {
            if (!frameset) return;
            
            std::lock_guard<std::mutex> lock(camera_ctx->frameset_mutex);
            camera_ctx->latest_frameset = frameset;
            camera_ctx->new_frame_available = true;
        });
        
        std::cout << "ORBBEC camera " << serial << " configured successfully" << std::endl;
        return camera_ctx;
        
    } catch (const std::exception& e) {
        std::cerr << "ORBBEC camera setup error for " << serial << ": " << e.what() << std::endl;
        throw;
    }
}

// Detect all cameras
std::vector<std::pair<std::string, CameraType>> detect_connected_cameras() {
    std::vector<std::pair<std::string, CameraType>> detected_cameras;
    
    // Detect RealSense cameras
    try {
        rs2::context ctx;
        auto devices = ctx.query_devices();
        
        std::cout << "\n=== Detecting RealSense Cameras ===" << std::endl;
        std::cout << "Found " << devices.size() << " RealSense device(s)" << std::endl;
        
        for (auto&& dev : devices) {
            try {
                std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                std::string name = dev.get_info(RS2_CAMERA_INFO_NAME);
                
                std::cout << "RealSense Camera found:" << std::endl;
                std::cout << "  - Name: " << name << std::endl;
                std::cout << "  - Serial: " << serial << std::endl;
                
                detected_cameras.push_back({serial, CameraType::REALSENSE});
                
            } catch (const std::exception& e) {
                std::cerr << "Error getting RealSense device info: " << e.what() << std::endl;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error during RealSense detection: " << e.what() << std::endl;
    }
    
    // Detect ORBBEC cameras
    auto orbbec_cameras = detect_orbbec_cameras();
    detected_cameras.insert(detected_cameras.end(), orbbec_cameras.begin(), orbbec_cameras.end());
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Total cameras: " << detected_cameras.size() << std::endl;
    for (size_t i = 0; i < detected_cameras.size(); ++i) {
        const char* type_str = (detected_cameras[i].second == CameraType::REALSENSE) ? "RealSense" : "ORBBEC";
        std::cout << "  " << (i+1) << ". " << detected_cameras[i].first << " (" << type_str << ")" << std::endl;
    }
    
    return detected_cameras;
}

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nStopping publisher..." << std::endl;
        g_running = false;
    }
}

std::vector<char> compress_depth_lz4(const void* data, size_t size) {
    if (!data || size == 0) return std::vector<char>();

    size_t max_compressed_size = LZ4F_compressFrameBound(size, NULL);
    std::vector<char> compressed_buffer(max_compressed_size);

    size_t compressed_size = LZ4F_compressFrame(
        compressed_buffer.data(), max_compressed_size,
        data, size,
        NULL
    );

    if (LZ4F_isError(compressed_size)) {
        std::cerr << "LZ4 compression failed: " << LZ4F_getErrorName(compressed_size) << std::endl;
        return std::vector<char>();
    }

    compressed_buffer.resize(compressed_size);
    return compressed_buffer;
}

// ORBBEC camera recovery function
bool restart_orbbec_camera(std::shared_ptr<OrbbecCameraContext> camera_ctx) {
    try {
        std::cout << "ORBBEC " << camera_ctx->serial << " - Attempting recovery..." << std::endl;
        
        // Stop current pipeline
        if (camera_ctx->pipeline) {
            camera_ctx->pipeline->stop();
        }
        
        // Wait for hardware to reset
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        
        // Reinitialize device
        ob::Context ctx;
        auto devList = ctx.queryDeviceList();
        
        for (uint32_t i = 0; i < devList->getCount(); i++) {
            auto dev = devList->getDevice(i);
            auto devInfo = dev->getDeviceInfo();
            
            if (std::string(devInfo->serialNumber()) == camera_ctx->serial) {
                camera_ctx->device = dev;
                camera_ctx->pipeline = std::make_shared<ob::Pipeline>(dev);
                break;
            }
        }
        
        if (!camera_ctx->device || !camera_ctx->pipeline) {
            std::cerr << "ORBBEC " << camera_ctx->serial << " - Device not found during recovery" << std::endl;
            return false;
        }
        
        // Configure with conservative settings
        auto config = std::make_shared<ob::Config>();
        
        // Try to get 640x480 @15fps specifically
        try {
            auto colorProfiles = camera_ctx->pipeline->getStreamProfileList(OB_SENSOR_COLOR);
            if (colorProfiles) {
                for (uint32_t i = 0; i < colorProfiles->count(); i++) {
                    auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (profile->width() == 640 && profile->height() == 480 && profile->fps() == 15) {
                        config->enableStream(profile);
                        std::cout << "ORBBEC " << camera_ctx->serial << " - Recovery: Using 640x480@15fps color" << std::endl;
                        break;
                    }
                }
            }
        } catch (...) {
            config->enableVideoStream(OB_STREAM_COLOR);
        }
        
        try {
            auto depthProfiles = camera_ctx->pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
            if (depthProfiles) {
                for (uint32_t i = 0; i < depthProfiles->count(); i++) {
                    auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
                    if (profile->width() == 640 && profile->height() == 480 && profile->fps() <= 30) {
                        config->enableStream(profile);
                        std::cout << "ORBBEC " << camera_ctx->serial << " - Recovery: Using depth @" << profile->fps() << "fps" << std::endl;
                        break;
                    }
                }
            }
        } catch (...) {
            config->enableVideoStream(OB_STREAM_DEPTH);
        }
        
        // Restart pipeline
        camera_ctx->pipeline->start(config, [camera_ctx](std::shared_ptr<ob::FrameSet> frameset) {
            if (!frameset) return;
            std::lock_guard<std::mutex> lock(camera_ctx->frameset_mutex);
            camera_ctx->latest_frameset = frameset;
            camera_ctx->new_frame_available = true;
        });
        
        std::cout << "ORBBEC " << camera_ctx->serial << " - Recovery successful" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "ORBBEC " << camera_ctx->serial << " - Recovery failed: " << e.what() << std::endl;
        return false;
    }
}

// ORBBEC camera thread with recovery
void orbbec_camera_thread_func(std::shared_ptr<OrbbecCameraContext> camera_ctx, SimplifiedUSBSynchronizer& synchronizer) {
    std::cout << "ORBBEC camera thread started for " << camera_ctx->serial << std::endl;
    int frame_count = 0;
    int consecutive_failures = 0;
    auto last_recovery_attempt = std::chrono::steady_clock::now();
    const int RECOVERY_THRESHOLD = 1000;
    const auto RECOVERY_COOLDOWN = std::chrono::minutes(5);

    while (g_running) {
        try {
            std::shared_ptr<ob::FrameSet> frameset = nullptr;
            
            {
                std::lock_guard<std::mutex> lock(camera_ctx->frameset_mutex);
                if (camera_ctx->new_frame_available && camera_ctx->latest_frameset) {
                    frameset = camera_ctx->latest_frameset;
                    camera_ctx->new_frame_available = false;
                }
            }
            
            if (!frameset) {
                consecutive_failures++;
                if (consecutive_failures % 500 == 0) {
                    std::cerr << "ORBBEC " << camera_ctx->serial << " - No frameset (failures: " 
                              << consecutive_failures << ")" << std::endl;
                }
                
                // Attempt recovery if failures exceed threshold and cooldown period has passed
                if (consecutive_failures >= RECOVERY_THRESHOLD) {
                    auto now = std::chrono::steady_clock::now();
                    if (now - last_recovery_attempt >= RECOVERY_COOLDOWN) {
                        last_recovery_attempt = now;
                        if (restart_orbbec_camera(camera_ctx)) {
                            consecutive_failures = 0;
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                            continue;
                        }
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            auto colorFrame = frameset->colorFrame();
            auto depthFrame = frameset->depthFrame();

            if (!colorFrame || !depthFrame) {
                consecutive_failures++;
                continue;
            }
            
            consecutive_failures = 0;
            
            int64_t system_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            // Convert color frame
            cv::Mat color_mat = convertOrbbecFrame(colorFrame, camera_ctx->serial);
            if (color_mat.empty()) {
                continue;
            }
            
            // Resize to standard resolution if needed
            if (color_mat.cols != COLOR_WIDTH || color_mat.rows != COLOR_HEIGHT) {
                cv::Mat resized;
                cv::resize(color_mat, resized, cv::Size(COLOR_WIDTH, COLOR_HEIGHT));
                color_mat = resized;
            }
            
            // Process depth frame
            cv::Mat depth_mat(depthFrame->height(), depthFrame->width(), CV_16UC1, (void*)depthFrame->data());
            
            if (depth_mat.cols != DEPTH_WIDTH || depth_mat.rows != DEPTH_HEIGHT) {
                cv::Mat resized;
                cv::resize(depth_mat, resized, cv::Size(DEPTH_WIDTH, DEPTH_HEIGHT), 0, 0, cv::INTER_NEAREST);
                depth_mat = resized;
            }
            
            // Compress depth
            std::vector<char> depth_compressed = compress_depth_lz4(depth_mat.data, 
                                                                   depth_mat.total() * depth_mat.elemSize());
            
            if (depth_compressed.empty()) continue;
            
            // Create frame data
            SimpleFrameData frame_data(camera_ctx->serial, system_time, system_time, 
                                     color_mat, depth_compressed, frame_count++, CameraType::ORBBEC);
            
            synchronizer.add_frame(frame_data);
            
            if (frame_count % 30 == 0) {
                std::cout << "ORBBEC " << camera_ctx->serial << " - Frame " << frame_count << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "ORBBEC error for " << camera_ctx->serial << ": " << e.what() << std::endl;
        }
    }
    
    std::cout << "ORBBEC camera thread for " << camera_ctx->serial << " stopping." << std::endl;
}

// RealSense camera setup
rs2::pipeline setup_realsense_camera(const std::string& serial) {
    rs2::context ctx;
    rs2::config cfg;
    
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_BGR8, FPS);
    cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, FPS);
    
    rs2::pipeline pipe(ctx);
    pipe.start(cfg);
    
    std::cout << "RealSense camera " << serial << " configured" << std::endl;
    return pipe;
}

// RealSense camera thread
void realsense_camera_thread_func(rs2::pipeline& pipe, const std::string& serial, SimplifiedUSBSynchronizer& synchronizer) {
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
            
            int64_t system_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            cv::Mat color_mat(cv::Size(COLOR_WIDTH, COLOR_HEIGHT), CV_8UC3, 
                             (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            
            std::vector<char> depth_compressed = compress_depth_lz4(depth_frame.get_data(), 
                                                                   depth_frame.get_data_size());
            if (depth_compressed.empty()) continue;

            SimpleFrameData frame_data(serial, system_time, system_time, 
                                     color_mat, depth_compressed, frame_count++);
            
            synchronizer.add_frame(frame_data);
            
            if (frame_count % 30 == 0) {
                std::cout << "RealSense " << serial << " - Frame " << frame_count << std::endl;
            }
            
        } catch (const std::exception& e) {
            std::cerr << "RealSense error for " << serial << ": " << e.what() << std::endl;
        }
    }
    
    std::cout << "RealSense camera thread for " << serial << " stopping." << std::endl;
}

// ZMQ publisher thread
void zmq_publisher_thread(zmq::socket_t& publisher, SimplifiedUSBSynchronizer& synchronizer) {
    std::cout << "ZMQ publisher thread started" << std::endl;
    int sync_group_count = 0;
    
    while (g_running) {
        std::vector<SimpleFrameData> synced_frames;
        
        if (!synchronizer.get_synchronized_frames(synced_frames)) {
            continue;
        }
        
        try {
            sync_group_count++;
            
            std::vector<char> bundled_message;
            
            int32_t camera_count = static_cast<int32_t>(synced_frames.size());
            bundled_message.insert(bundled_message.end(), 
                                 reinterpret_cast<char*>(&sync_group_count), 
                                 reinterpret_cast<char*>(&sync_group_count) + sizeof(sync_group_count));
            bundled_message.insert(bundled_message.end(), 
                                 reinterpret_cast<char*>(&camera_count), 
                                 reinterpret_cast<char*>(&camera_count) + sizeof(camera_count));
            
            for (const SimpleFrameData& frame_data : synced_frames) {
                std::vector<uchar> color_encoded;
                cv::imencode(".jpg", frame_data.color_mat, color_encoded, 
                            std::vector<int>{cv::IMWRITE_JPEG_QUALITY, 65});
                
                int32_t serial_len = static_cast<int32_t>(frame_data.serial.length());
                bundled_message.insert(bundled_message.end(), 
                                     reinterpret_cast<char*>(&serial_len), 
                                     reinterpret_cast<char*>(&serial_len) + sizeof(serial_len));
                bundled_message.insert(bundled_message.end(), 
                                     frame_data.serial.begin(), frame_data.serial.end());
                
                int64_t timestamp = frame_data.system_time;
                bundled_message.insert(bundled_message.end(), 
                                     reinterpret_cast<char*>(&timestamp), 
                                     reinterpret_cast<char*>(&timestamp) + sizeof(timestamp));
                
                int32_t color_len = static_cast<int32_t>(color_encoded.size());
                bundled_message.insert(bundled_message.end(), 
                                     reinterpret_cast<char*>(&color_len), 
                                     reinterpret_cast<char*>(&color_len) + sizeof(color_len));
                bundled_message.insert(bundled_message.end(), 
                                     color_encoded.begin(), color_encoded.end());
                
                int32_t depth_len = static_cast<int32_t>(frame_data.depth_compressed.size());
                bundled_message.insert(bundled_message.end(), 
                                     reinterpret_cast<char*>(&depth_len), 
                                     reinterpret_cast<char*>(&depth_len) + sizeof(depth_len));
                bundled_message.insert(bundled_message.end(), 
                                     frame_data.depth_compressed.begin(), 
                                     frame_data.depth_compressed.end());
            }
            
            publisher.send(zmq::buffer(ZMQ_TOPIC), zmq::send_flags::sndmore);
            publisher.send(zmq::buffer(bundled_message), zmq::send_flags::none);
            
            if (sync_group_count % 30 == 0) {
                std::cout << "Sent sync group #" << sync_group_count 
                          << " with " << synced_frames.size() << " cameras" << std::endl;
            }
                      
        } catch (const std::exception& e) {
            std::cerr << "ZMQ publisher error: " << e.what() << std::endl;
        }
    }
    
    std::cout << "ZMQ publisher thread stopping." << std::endl;
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Detect all cameras
    auto detected_cameras = detect_connected_cameras();
    
    if (detected_cameras.empty()) {
        std::cerr << "No cameras found!" << std::endl;
        return 1;
    }

    // Extract serial numbers
    SERIAL_NUMBERS.clear();
    std::map<std::string, CameraType> camera_types;
    for (const auto& camera : detected_cameras) {
        SERIAL_NUMBERS.push_back(camera.first);
        camera_types[camera.first] = camera.second;
    }

    // Initialize ZMQ
    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind(ZMQ_ENDPOINT);
    std::cout << "Publisher bound to " << ZMQ_ENDPOINT << std::endl;

    // Initialize synchronizer
    SimplifiedUSBSynchronizer synchronizer;

    std::vector<rs2::pipeline> realsense_pipelines;
    std::vector<std::shared_ptr<OrbbecCameraContext>> orbbec_cameras;
    std::vector<std::thread> camera_threads;

    try {
        // Initialize ORBBEC cameras (if any) - based on official example
        std::vector<std::string> orbbec_serials;
        for (const auto& camera : detected_cameras) {
            if (camera.second == CameraType::ORBBEC) {
                orbbec_serials.push_back(camera.first);
            }
        }
        
        if (!orbbec_serials.empty()) {
            ob::Context ctx;
            auto devList = ctx.queryDeviceList();
            
            for (const std::string& serial : orbbec_serials) {
                for (uint32_t i = 0; i < devList->getCount(); i++) {
                    auto dev = devList->getDevice(i);
                    auto devInfo = dev->getDeviceInfo();
                    
                    if (std::string(devInfo->serialNumber()) == serial) {
                        try {
                            auto orbbec_ctx = setup_orbbec_camera(dev, serial, i);
                            orbbec_cameras.push_back(orbbec_ctx);
                            std::cout << "ORBBEC camera " << serial << " initialized" << std::endl;
                        } catch (const std::exception& e) {
                            std::cerr << "Failed to initialize ORBBEC " << serial << ": " << e.what() << std::endl;
                        }
                        break;
                    }
                }
            }
        }
        
        // Initialize RealSense cameras
        for (const auto& camera : detected_cameras) {
            if (camera.second == CameraType::REALSENSE) {
                try {
                    rs2::pipeline pipe = setup_realsense_camera(camera.first);
                    realsense_pipelines.push_back(std::move(pipe));
                } catch (const std::exception& e) {
                    std::cerr << "Failed to initialize RealSense " << camera.first << ": " << e.what() << std::endl;
                }
            }
        }
        
        std::cout << "\nStarting camera threads..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Start ORBBEC camera threads
        for (auto& orbbec_ctx : orbbec_cameras) {
            camera_threads.push_back(std::thread(orbbec_camera_thread_func, 
                                               orbbec_ctx, 
                                               std::ref(synchronizer)));
        }
        
        // Start RealSense camera threads
        size_t rs_index = 0;
        for (const auto& camera : detected_cameras) {
            if (camera.second == CameraType::REALSENSE && rs_index < realsense_pipelines.size()) {
                camera_threads.push_back(std::thread(realsense_camera_thread_func, 
                                                   std::ref(realsense_pipelines[rs_index]), 
                                                   camera.first, 
                                                   std::ref(synchronizer)));
                rs_index++;
            }
        }
        
        // Start ZMQ publisher thread
        std::thread zmq_thread(zmq_publisher_thread, std::ref(publisher), std::ref(synchronizer));
        
        std::cout << "\n=== Multi-Camera Publisher Started ===" << std::endl;
        std::cout << "Active cameras: " << camera_threads.size() << std::endl;
        std::cout << "Press Ctrl+C to stop..." << std::endl;
        
        // Wait for exit signal
        while(g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Cleanup
        std::cout << "\nStopping all threads..." << std::endl;
        
        for (std::thread& t : camera_threads) {
            if (t.joinable()) t.join();
        }
        
        if (zmq_thread.joinable()) {
            zmq_thread.join();
        }
        
        // Stop ORBBEC pipelines
        for (auto& orbbec_ctx : orbbec_cameras) {
            if (orbbec_ctx->pipeline) {
                orbbec_ctx->pipeline->stop();
            }
        }
        
        // Stop RealSense pipelines
        for (auto& pipe : realsense_pipelines) {
            pipe.stop();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        g_running = false;
        return 1;
    }

    std::cout << "Multi-camera publisher stopped." << std::endl;
    return 0;
}