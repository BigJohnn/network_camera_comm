#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <csignal>
#include <atomic>
#include <zmq.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <lz4.h> // LZ4 標頭檔
#include <lz4frame.h>
// --- 全域設定 ---
const std::vector<std::string> SERIAL_NUMBERS = {"243222076185", "243322073850"};
const std::string ZMQ_ENDPOINT = "tcp://*:5555";
const std::string ZMQ_TOPIC = "D435i_STREAM";

const int COLOR_WIDTH = 640;
const int COLOR_HEIGHT = 480;
const int DEPTH_WIDTH = 640;
const int DEPTH_HEIGHT = 480;
const int FPS = 30;

// 全局退出標誌
std::atomic<bool> g_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nStopping publisher..." << std::endl;
        g_running = false;
    }
}

std::vector<char> compress_depth_lz4(const rs2::depth_frame& frame) {
    if (!frame) return {};

    const char* source_data = (const char*)frame.get_data();
    size_t source_size = frame.get_data_size();

    if (source_size == 0) {
        std::cerr << "!!! Error: Depth frame is empty, cannot compress." << std::endl;
        return {};
    }

    // LZ4F_compressFrameBound provides the maximum size for a frame
    size_t max_compressed_size = LZ4F_compressFrameBound(source_size, NULL);
    std::vector<char> compressed_buffer(max_compressed_size);

    // LZ4F_compressFrame creates a complete, valid LZ4 frame in one go
    size_t compressed_size = LZ4F_compressFrame(
        compressed_buffer.data(), max_compressed_size,
        source_data, source_size,
        NULL // Use default preferences
    );

    // Check for errors
    if (LZ4F_isError(compressed_size)) {
        std::cerr << "!!! LZ4 frame compression failed: " << LZ4F_getErrorName(compressed_size) << std::endl;
        return {};
    }

    compressed_buffer.resize(compressed_size);
    return compressed_buffer;
}

// 相機處理執行緒
void camera_thread_func(rs2::pipeline& pipe, const std::string& serial, zmq::socket_t& publisher, std::mutex& zmq_mutex) {
    std::cout << "Thread started for camera " << serial << std::endl;
    rs2::align align_to_color(RS2_STREAM_COLOR);

    while (g_running) {
        try {
            rs2::frameset frames = pipe.wait_for_frames(1000); // 1秒超時
            if (!frames) continue;

            frames = align_to_color.process(frames);
            rs2::video_frame color_frame = frames.get_color_frame();
            rs2::depth_frame depth_frame = frames.get_depth_frame();

            if (!color_frame || !depth_frame) continue;
            
            // 壓縮彩色影像為 JPEG
            cv::Mat color_mat(cv::Size(COLOR_WIDTH, COLOR_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            std::vector<uchar> color_encoded;
            cv::imencode(".jpg", color_mat, color_encoded, {cv::IMWRITE_JPEG_QUALITY, 80});

            // 壓縮深度影像為 LZ4
            std::vector<char> depth_encoded = compress_depth_lz4(depth_frame);
            if (depth_encoded.empty()) continue;

            // 獲取時間戳
            int64_t timestamp = static_cast<int64_t>(color_frame.get_timestamp());
            
            // 鎖定 ZMQ socket 並發送 multipart message
            std::lock_guard<std::mutex> lock(zmq_mutex);
            
            // Part 1: Topic
            publisher.send(zmq::buffer(ZMQ_TOPIC), zmq::send_flags::sndmore);
            // Part 2: Serial Number
            publisher.send(zmq::buffer(serial), zmq::send_flags::sndmore);
            // Part 3: Timestamp (int64_t)
            publisher.send(zmq::buffer(&timestamp, sizeof(timestamp)), zmq::send_flags::sndmore);
            // Part 4: Color JPEG data
            publisher.send(zmq::buffer(color_encoded), zmq::send_flags::sndmore);
            // Part 5: Depth LZ4 data
            publisher.send(zmq::buffer(depth_encoded), zmq::send_flags::none); // 最後一幀

        } catch (const rs2::error& e) {
            std::cerr << "RealSense error in thread for " << serial << ": " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Exception in thread for " << serial << ": " << e.what() << std::endl;
        }
    }
    std::cout << "Thread for camera " << serial << " is stopping." << std::endl;
}

int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    zmq::context_t context(1);
    zmq::socket_t publisher(context, zmq::socket_type::pub);
    publisher.bind(ZMQ_ENDPOINT);
    std::cout << "Publisher bound to " << ZMQ_ENDPOINT << std::endl;

    std::vector<rs2::pipeline> pipelines;
    std::vector<std::thread> threads;
    std::mutex zmq_mutex; // ZMQ socket 是執行緒安全的，但為了避免訊息交錯，最好還是加鎖

    try {
        rs2::context ctx;
        for (const auto& serial : SERIAL_NUMBERS) {
            rs2::config cfg;
            cfg.enable_device(serial);
            cfg.enable_stream(RS2_STREAM_COLOR, COLOR_WIDTH, COLOR_HEIGHT, RS2_FORMAT_BGR8, FPS);
            cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_Z16, FPS);
            
            rs2::pipeline pipe(ctx);
            pipe.start(cfg);
            pipelines.push_back(pipe);
            std::cout << "Pipeline started for camera: " << serial << std::endl;
        }
    } catch (const rs2::error & e) {
        std::cerr << "Failed to initialize RealSense cameras: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "Waiting for cameras to stabilize..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 啟動執行緒
    for (size_t i = 0; i < pipelines.size(); ++i) {
        threads.emplace_back(camera_thread_func, std::ref(pipelines[i]), SERIAL_NUMBERS[i], std::ref(publisher), std::ref(zmq_mutex));
    }

    // 等待退出信號
    while(g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 清理
    std::cout << "Waiting for threads to join..." << std::endl;
    for (auto& t : threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    std::cout << "Publisher stopped." << std::endl;
    return 0;
}
