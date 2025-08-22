#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== D435i Camera Publisher 依赖安装脚本 ===${NC}"

# 检查是否为root或使用sudo
if [ "$EUID" -ne 0 ] && [ -z "$SUDO_USER" ]; then 
   echo -e "${YELLOW}请使用 sudo 运行此脚本${NC}"
   exit 1
fi

# 更新包列表
echo -e "${GREEN}1. 更新包列表...${NC}"
apt update

# 安装基础工具
echo -e "${GREEN}2. 安装基础工具...${NC}"
apt install -y \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl

# 安装Intel RealSense SDK
echo -e "${GREEN}3. 安装Intel RealSense SDK...${NC}"

# 添加Intel RealSense存储库
apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# 安装RealSense包
apt install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

# 安装Fast-RTPS依赖（解决你遇到的问题）
echo -e "${GREEN}4. 安装Fast-RTPS依赖...${NC}"
apt install -y \
    libfastcdr-dev \
    libfastrtps-dev \
    libasio-dev \
    libtinyxml2-dev

# 安装OpenCV
echo -e "${GREEN}5. 安装OpenCV...${NC}"
apt install -y \
    libopencv-dev \
    python3-opencv

# 安装ZeroMQ
echo -e "${GREEN}6. 安装ZeroMQ...${NC}"
apt install -y \
    libzmq3-dev

# 安装cppzmq (C++ bindings for ZeroMQ)
echo -e "${GREEN}7. 安装cppzmq...${NC}"
# 首先尝试从包管理器安装
apt install -y cppzmq-dev 2>/dev/null

# 如果包不存在，从源码安装
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}从源码安装cppzmq...${NC}"
    cd /tmp
    git clone https://github.com/zeromq/cppzmq.git
    cd cppzmq
    mkdir build && cd build
    cmake ..
    make -j$(nproc)
    make install
    cd /
    rm -rf /tmp/cppzmq
fi

# 验证安装
echo -e "${GREEN}8. 验证安装...${NC}"

# 检查RealSense
if command -v rs-enumerate-devices &> /dev/null; then
    echo -e "${GREEN}✓ RealSense SDK 安装成功${NC}"
    # 列出连接的设备
    echo -e "${YELLOW}检测到的RealSense设备：${NC}"
    rs-enumerate-devices | grep "Device Name\|Serial Number" || echo "未检测到设备（正常，如果设备未连接）"
else
    echo -e "${RED}✗ RealSense SDK 安装失败${NC}"
fi

# 检查OpenCV
if pkg-config --modversion opencv4 &> /dev/null; then
    echo -e "${GREEN}✓ OpenCV $(pkg-config --modversion opencv4) 安装成功${NC}"
else
    echo -e "${RED}✗ OpenCV 安装失败${NC}"
fi

# 检查ZeroMQ
if pkg-config --modversion libzmq &> /dev/null; then
    echo -e "${GREEN}✓ ZeroMQ $(pkg-config --modversion libzmq) 安装成功${NC}"
else
    echo -e "${RED}✗ ZeroMQ 安装失败${NC}"
fi

# 设置udev规则（用于USB权限）
echo -e "${GREEN}9. 设置USB权限...${NC}"
# RealSense udev规则通常已经由librealsense2-dkms设置
udevadm control --reload-rules && udevadm trigger

# 清理
echo -e "${GREEN}10. 清理临时文件...${NC}"
apt autoremove -y
apt autoclean

echo -e "${GREEN}=== 安装完成！===${NC}"
echo -e "${YELLOW}注意事项：${NC}"
echo "1. 如果是首次安装RealSense，建议重启系统"
echo "2. 确保D435i相机连接到USB 3.0端口"
echo "3. 运行 'rs-enumerate-devices' 检查相机是否被识别"
echo "4. 如果遇到权限问题，将用户添加到 'plugdev' 组："
echo "   sudo usermod -a -G plugdev $USER"
echo "   然后重新登录"
echo ""
echo -e "${GREEN}现在可以编译项目了：${NC}"
echo "  mkdir build && cd build"
echo "  cmake .."
echo "  make -j$(nproc)"
