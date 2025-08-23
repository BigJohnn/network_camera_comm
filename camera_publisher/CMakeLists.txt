cmake_minimum_required(VERSION 3.10)
project(D435iCameraPublisher)

# C++14标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 首先尝试查找可能的依赖
find_package(PkgConfig)

# 查找realsense2 - 使用更灵活的方式
find_package(realsense2 QUIET)
if(NOT realsense2_FOUND)
    # 如果标准方式失败，尝试pkg-config
    if(PkgConfig_FOUND)
        pkg_check_modules(REALSENSE2 realsense2)
        if(REALSENSE2_FOUND)
            set(realsense2_FOUND TRUE)
            set(realsense2_INCLUDE_DIR ${REALSENSE2_INCLUDE_DIRS})
            set(realsense2_LIBRARY ${REALSENSE2_LIBRARIES})
            link_directories(${REALSENSE2_LIBRARY_DIRS})
        endif()
    endif()
    
    # 如果还是没找到，手动查找
    if(NOT realsense2_FOUND)
        find_path(realsense2_INCLUDE_DIR
            NAMES librealsense2/rs.hpp
            PATHS
            /usr/include
            /usr/local/include
        )
        
        find_library(realsense2_LIBRARY
            NAMES realsense2
            PATHS
            /usr/lib
            /usr/local/lib
            /usr/lib/x86_64-linux-gnu
        )
        
        if(realsense2_INCLUDE_DIR AND realsense2_LIBRARY)
            set(realsense2_FOUND TRUE)
            message(STATUS "Found RealSense2 manually")
        endif()
    endif()
endif()

if(NOT realsense2_FOUND)
    message(FATAL_ERROR "RealSense2 not found! Please install librealsense2-dev")
endif()

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 查找线程库
find_package(Threads REQUIRED)

# 查找ZeroMQ
if(PkgConfig_FOUND)
    pkg_check_modules(PC_ZeroMQ QUIET zmq)
endif()

# 查找ZeroMQ库
find_path(ZeroMQ_INCLUDE_DIR
    NAMES zmq.hpp
    PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
    /usr/include
    /usr/local/include
    /usr/include/cppzmq
    /usr/local/include/cppzmq
)

find_library(ZeroMQ_LIBRARY
    NAMES zmq
    PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
)

# 检查是否找到所有必需的组件
if(NOT ZeroMQ_INCLUDE_DIR OR NOT ZeroMQ_LIBRARY)
    message(FATAL_ERROR "ZeroMQ not found! Please install libzmq3-dev and cppzmq-dev")
endif()

# 添加可执行文件
add_executable(camera_publisher camera_publisher.cpp)

# 设置包含目录
if(realsense2_INCLUDE_DIR)
    target_include_directories(camera_publisher PRIVATE ${realsense2_INCLUDE_DIR})
else()
    # 如果使用的是CMake配置方式
    target_include_directories(camera_publisher PRIVATE ${realsense2_INCLUDE_DIRS})
endif()

target_include_directories(camera_publisher PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${ZeroMQ_INCLUDE_DIR}
)


# 查找LZ4库
find_library(LZ4_LIBRARY
    NAMES lz4
    PATHS
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
)

if(NOT LZ4_LIBRARY)
    message(FATAL_ERROR "LZ4 library not found! Please install liblz4-dev")
endif()

# 查找ORBBEC SDK - 根据服务器路径结构
set(ORBBEC_SDK_PATHS
    # 服务器上的路径
    ~/Code/OrbbecSDK/OrbbecSDK_v1.10.22/SDK
    # 相对路径备选
    ../../OrbbecSDK/OrbbecSDK_v1.10.22/SDK
    # 传统安装路径
    /usr/local/OrbbecSDK
    # 环境变量路径
    ${ORBBEC_SDK_ROOT}
)

# 展开用户主目录
file(TO_CMAKE_PATH "$ENV{HOME}" HOME_DIR)

# 查找头文件
find_path(ORBBEC_INCLUDE_DIR
    NAMES libobsensor/ObSensor.hpp
    PATHS 
    ${HOME_DIR}/Code/OrbbecSDK/OrbbecSDK_v1.10.22/SDK/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../OrbbecSDK/OrbbecSDK_v1.10.22/SDK/include
    /usr/local/OrbbecSDK/include
    /usr/include
    /usr/local/include
    ${ORBBEC_SDK_ROOT}/include
)

# 查找库文件
find_library(ORBBEC_LIBRARY
    NAMES libOrbbecSDK OrbbecSDK
    PATHS
    ${HOME_DIR}/Code/OrbbecSDK/OrbbecSDK_v1.10.22/SDK/lib
    ${CMAKE_CURRENT_SOURCE_DIR}/../../OrbbecSDK/OrbbecSDK_v1.10.22/SDK/lib
    /usr/local/OrbbecSDK/lib
    /usr/lib
    /usr/local/lib
    /usr/lib/x86_64-linux-gnu
    ${ORBBEC_SDK_ROOT}/lib
)

if(ORBBEC_INCLUDE_DIR AND ORBBEC_LIBRARY)
    message(STATUS "ORBBEC SDK found")
    message(STATUS "ORBBEC include: ${ORBBEC_INCLUDE_DIR}")
    message(STATUS "ORBBEC library: ${ORBBEC_LIBRARY}")
    
    # 添加ORBBEC库目录到运行时库路径
    get_filename_component(ORBBEC_LIB_DIR ${ORBBEC_LIBRARY} DIRECTORY)
    set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_RPATH} ${ORBBEC_LIB_DIR})
    
else()
    message(FATAL_ERROR "ORBBEC SDK not found! Please check path: ~/Code/OrbbecSDK/OrbbecSDK_v1.10.22/SDK")
endif()

# 链接库
set(CAMERA_LIBRARIES ${OpenCV_LIBS} ${ZeroMQ_LIBRARY} ${LZ4_LIBRARY} ${ORBBEC_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})

if(realsense2_LIBRARY)
    target_link_libraries(camera_publisher
        ${realsense2_LIBRARY}
        ${CAMERA_LIBRARIES}
    )
else()
    # 使用CMake目标方式
    target_link_libraries(camera_publisher
        realsense2::realsense2
        ${CAMERA_LIBRARIES}
    )
endif()

# 添加ORBBEC包含目录
if(ORBBEC_INCLUDE_DIR)
    target_include_directories(camera_publisher PRIVATE ${ORBBEC_INCLUDE_DIR})
    message(STATUS "Adding ORBBEC include directory: ${ORBBEC_INCLUDE_DIR}")
endif()

# 设置运行时库路径，确保能找到ORBBEC动态库
set_target_properties(camera_publisher PROPERTIES
    BUILD_RPATH "${ORBBEC_LIB_DIR}"
    INSTALL_RPATH "${ORBBEC_LIB_DIR}"
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# 设置编译选项
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(camera_publisher PRIVATE -Wall -Wextra -O2)
endif()

# 输出配置信息
message(STATUS "=== Configuration Summary ===")
message(STATUS "RealSense2 found: ${realsense2_FOUND}")
if(realsense2_INCLUDE_DIR)
    message(STATUS "RealSense2 include: ${realsense2_INCLUDE_DIR}")
    message(STATUS "RealSense2 library: ${realsense2_LIBRARY}")
endif()
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "ZeroMQ include: ${ZeroMQ_INCLUDE_DIR}")
message(STATUS "ZeroMQ library: ${ZeroMQ_LIBRARY}")

# 安装目标
install(TARGETS camera_publisher
    RUNTIME DESTINATION bin
)