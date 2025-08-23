# Network Camera Communication

Send camera stream by ZMQ message from machine A to machine B, using LAN or WLAN instead of USB 3.0 cable.

Machine A should have enough power to handle its cameras/sensors data, and the network latency should be as small as possible between machine A & B.

## Supported Cameras

- **Intel RealSense**: D435i, D455, D415, etc. (using librealsense2)
- **ORBBEC**: Femto Bolt, Astra+, Gemini 2, etc. (using OrbbecSDK)

The code automatically detects connected cameras and supports mixing different camera brands.

## Installation Guide

### 1. Install System Dependencies

#### Ubuntu/Debian:
```bash
# Update package list
sudo apt update

# Install build tools and basic dependencies
sudo apt install -y cmake build-essential pkg-config git wget
sudo apt install -y libopencv-dev libzmq3-dev cppzmq-dev liblz4-dev
sudo apt install -y libusb-1.0-0-dev libudev-dev

# Python dependencies for receiver
pip3 install opencv-python zmq numpy lz4
```

### 2. Install Intel RealSense SDK

#### Method 1: Package Manager (Recommended)
```bash
# Add Intel's public key
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

# Add repository
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

# Update and install
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

#### Method 2: From Source (if package method fails)
```bash
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo apt install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true
make -j4
sudo make install
```

#### Verify RealSense Installation:
```bash
# Test camera detection
rs-enumerate-devices

# Launch viewer (optional)
realsense-viewer
```

### 3. Install ORBBEC SDK

#### Download and Install:
```bash
# Create download directory
mkdir -p ~/Downloads && cd ~/Downloads

# Download OrbbecSDK (latest version)
wget https://github.com/orbbec/OrbbecSDK/releases/download/v1.10.22/OrbbecSDK_C_C++_v1.10.22_20250410_46139de_linux_x64_release.zip

# Extract to ~/下载/ directory (required for relative path in CMake)
cd ~/下载/
unzip OrbbecSDK_C_C++_v1.10.22_20250410_46139de_linux_x64_release.zip
cd OrbbecSDK_C_C++_v1.10.22_20250410_46139de_linux_x64_release/OrbbecSDK_v1.10.22

# Install SDK
sudo ./install.sh

# Set up udev rules (IMPORTANT!)
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### Set Environment Variables:
```bash
# Add to ~/.bashrc
echo 'export ORBBEC_SDK_ROOT=/usr/local/OrbbecSDK' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/OrbbecSDK/lib' >> ~/.bashrc

# Apply changes
source ~/.bashrc
```

#### Verify ORBBEC Installation:
```bash
# Test camera detection and viewer
cd /usr/local/OrbbecSDK/bin
./OBSensorViewer
```

### 4. Fix Permissions (if needed)
```bash
# Add user to video group
sudo usermod -a -G video $USER

# Logout and login again, or restart system
```

## Build Instructions

### Build Publisher (C++)
```bash
cd camera_publisher
mkdir build && cd build

# Configure and build
cmake ..
make -j4
```

### Build Output:
The build process will show:
- RealSense SDK detection status
- ORBBEC SDK detection status  
- All required libraries found/linked

## Usage

### 1. Start Publisher (Sender)
```bash
cd camera_publisher/build
./camera_publisher
```

The program will:
- Automatically detect all connected RealSense and ORBBEC cameras
- Display camera information (name, serial, type)
- Start streaming at optimized settings

### 2. Start Subscriber (Receiver)
```bash
# Edit IP address in camera_subscriber.py to match sender's IP
python3 camera_subscriber.py
```

## Performance Optimization

Current optimizations for low latency:
- **Frame Rate**: 15fps (reduced from 30fps)
- **Sync Window**: 30ms (optimized for responsiveness)
- **Buffer Size**: 2 frames (reduced latency accumulation)
- **JPEG Quality**: 65 (balance quality/speed)
- **Resolution**: 640x480

### Further Optimization Options:

If experiencing lag, consider:

1. **Lower Resolution**: Modify `COLOR_WIDTH/HEIGHT` to 424x240
2. **Lower Frame Rate**: Set `FPS` to 10 or less
3. **Network Optimization**: Use wired connection, ensure sufficient bandwidth
4. **Time Synchronization**: Use NTP to sync clocks between machines

## Troubleshooting

### Camera Detection Issues

#### RealSense Cameras:
```bash
# Check USB connection
lsusb | grep Intel

# Check RealSense detection
rs-enumerate-devices

# Check permissions
rs-sensor-control
```

#### ORBBEC Cameras:
```bash
# Check USB connection
lsusb | grep Orbbec

# Test with viewer
cd /usr/local/OrbbecSDK/bin
./OBSensorViewer

# Check library path
echo $LD_LIBRARY_PATH
```

### Build Issues

#### Missing RealSense:
```bash
# If package install failed, try from source
sudo apt remove librealsense2-dev
# Then follow "From Source" method above
```

#### Missing ORBBEC:
```bash
# Verify SDK installation
ls -la /usr/local/OrbbecSDK/
ls -la /usr/local/OrbbecSDK/lib/
ls -la /usr/local/OrbbecSDK/include/
```

#### CMake Configuration Errors:
```bash
# Clear build cache
rm -rf build
mkdir build && cd build

# Verbose cmake for debugging
cmake .. -DCMAKE_VERBOSE_MAKEFILE=ON
```

### Runtime Issues

#### Permission Denied:
```bash
# Fix udev rules
sudo cp /usr/local/OrbbecSDK/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add to video group
sudo usermod -a -G video $USER
```

#### Network Latency:
```bash
# Test network latency
ping <sender_ip>

# Sync system clocks
sudo ntpdate -s time.nist.gov
```

#### Library Not Found:
```bash
# Update library cache
sudo ldconfig

# Check library path
ldd camera_publisher | grep -i orbbec
ldd camera_publisher | grep -i realsense
```

## Camera Support Matrix

| Camera Model | RealSense SDK | ORBBEC SDK | Tested |
|--------------|---------------|------------|--------|
| D435i        | ✅            | ❌         | ✅     |
| D455         | ✅            | ❌         | ⚠️     |
| Femto Bolt   | ❌            | ✅         | ⚠️     |
| Astra+       | ❌            | ✅         | ⚠️     |
| Gemini 2     | ❌            | ✅         | ⚠️     |

✅ Supported and tested  
⚠️ Supported but needs testing  
❌ Not supported