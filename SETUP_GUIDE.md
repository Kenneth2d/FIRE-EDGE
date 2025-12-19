# FIRE-EDGE Complete Setup Guide

Comprehensive installation and configuration guide for the FIRE-EDGE system.

**⏱️ Estimated Time**: 2-4 hours depending on hardware  
**💻 Difficulty**: Intermediate to Advanced

---

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Setup Options](#setup-options)
3. [Option A: Simulation Environment](#option-a-simulation-environment-recommended)
4. [Option B: Jetson Xavier NX Deployment](#option-b-jetson-xavier-nx-deployment)
5. [Option C: Demo/Development Mode](#option-c-demodevelopment-mode)
6. [Hardware Assembly](#hardware-assembly)
7. [Sensor Calibration](#sensor-calibration)
8. [Troubleshooting](#troubleshooting)
9. [Verification Tests](#verification-tests)

---

## System Requirements

### Minimum Requirements (Simulation)

|  Component  |          Specification          |
|-------------|---------------------------------|
|   **OS**    |         Ubuntu 20.04 LTS        |
|   **CPU**   |       Intel Core i5 (4 cores)   |
|   **RAM**   |               8GB               |
|   **GPU**   |      Not required (CPU only)    |
| **Storage** |          20GB free space        |
| **Network** | Internet (for dataset download) |

### Recommended (Full Hardware)

|   Component  |           Specification           |
|--------------|-----------------------------------|
|    **OS**    |          Ubuntu 20.04 LTS         |
|    **CPU**   |       Intel Core i7 (6+ cores)    |
|    **RAM**   |                16GB               |
|    **GPU**   |      NVIDIA GPU with CUDA 11+     |
| **Storage**  |              50GB SSD             |
| **Hardware** | NVIDIA Jetson Xavier NX + sensors |

### Required Hardware (Full System)

- ✅ NVIDIA Jetson Xavier NX Developer Kit ($599)
- ✅ FLIR Lepton 3.5 Thermal Camera ($299)
- ✅ Garmin LIDAR-Lite v3HP ($149)
- ✅ TDK MPU-9250 IMU ($15)
- ✅ A02YYUW Ultrasonic Sensors x4 ($40)
- ✅ Kopin Lightning OLED Display ($1,200)
- ✅ Custom 4S LiPo Battery 10,000mAh ($120)
- ✅ Breadboard, wires, connectors (~$50)

**Total Hardware Cost**: ~$2,470

---

## Setup Options

Choose your setup based on your goals:

|      Option       |             Use Case            | Hardware Required  | Time | Difficulty |
|-------------------|---------------------------------|--------------------|------|------------|
| **A: Simulation** | Development, algorithm testing  |   PC with webcam   |  2h  |   Medium   |
|  **B: Jetson NX** | Edge deployment, real hardware  | Full hardware list |  4h+ |    Hard    |
|  **C: Demo Mode** | Quick evaluation, presentations |       PC only      |  30m |    Easy    |

---

## Option A: Simulation Environment (Recommended)

### Step 1: Install Ubuntu 20.04

If not already installed:
```bash
# Download Ubuntu 20.04 LTS from:
# https://ubuntu.com/download/desktop

# Create bootable USB (on existing Linux):
sudo dd if=ubuntu-20.04.iso of=/dev/sdX bs=4M status=progress
```

### Step 2: Install ROS Noetic

```bash
# 1. Setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 3. Update package list
sudo apt update

# 4. Install ROS Noetic Desktop Full
sudo apt install ros-noetic-desktop-full

# 5. Initialize rosdep
sudo rosdep init
rosdep update

# 6. Setup environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 7. Install build tools
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Verify installation
rosversion -d  # Should output: noetic
```

### Step 3: Install Gazebo 11

```bash
# Install Gazebo (should be included with ROS Noetic Desktop Full)
sudo apt install gazebo11 ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Verify installation
gazebo --version  # Should output: Gazebo multi-robot simulator, version 11.x.x
```

### Step 4: Install Python Dependencies

```bash
# 1. Update system Python
sudo apt update
sudo apt install python3-pip python3-dev python3-opencv

# 2. Create virtual environment (recommended)
sudo apt install python3-venv
python3 -m venv ~/fire-edge-venv
source ~/fire-edge-venv/bin/activate

# 3. Upgrade pip
pip install --upgrade pip

# 4. Clone repository
cd ~
git clone https://github.com/yourusername/fire-edge.git
cd fire-edge

# 5. Install requirements
pip install -r requirements.txt

# This will install:
# - opencv-python
# - ultralytics (YOLOv8)
# - torch, torchvision
# - numpy, scipy, matplotlib
# - filterpy (Kalman filter)
# - and more...
```

### Step 5: Setup Catkin Workspace

```bash
# 1. Create workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 2. Link FIRE-EDGE package
ln -s ~/fire-edge .

# 3. Initialize workspace
cd ~/catkin_ws
catkin_make

# 4. Source workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

# Verify
rospack find fire_edge  # Should show path to fire-edge
```

### Step 6: Download YOLOv8 Model

```bash
cd ~/fire-edge

# Option 1: Download pre-trained thermal model (recommended)
# This downloads our trained model from research paper
python3 scripts/download_models.py

# Option 2: Use default YOLOv8n (for testing only)
# Model will auto-download on first run
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

### Step 7: Download Thermal Dataset (Optional)

```bash
# For training or validation
python3 scripts/download_dataset.py

# This downloads the foduucom thermal dataset (~2GB)
# Location: data/thermal_dataset/
```

### Step 8: Test Installation

```bash
# Test 1: ROS
roscore &
rostopic list  # Should show ROS topics

# Test 2: Gazebo
gazebo  # GUI should open
# Close Gazebo (Ctrl+C)

# Test 3: Python imports
python3 -c "
import cv2
import numpy as np
from ultralytics import YOLO
import rospy
print('All imports successful!')
"

# Test 4: Run simple demo
python3 demo/ar_firefighter_helmet_cv.py
# Should open 3 windows with camera feed
# Press 'q' to quit
```

### Step 9: Run Full Simulation

```bash
# Terminal 1: Launch ROS + Gazebo
source ~/catkin_ws/devel/setup.bash
roslaunch fire_edge simulation.launch

# Terminal 2: Start thermal processor
source ~/catkin_ws/devel/setup.bash
rosrun fire_edge thermal_processor.py

# Terminal 3: Start sensor fusion
source ~/catkin_ws/devel/setup.bash
rosrun fire_edge sensor_fusion.py

# Terminal 4: Launch AR display
source ~/catkin_ws/devel/setup.bash
rosrun fire_edge ar_display.py
```

---

## Option B: Jetson Xavier NX Deployment

### Step 1: Flash JetPack

```bash
# On HOST PC (Ubuntu 18.04/20.04):

# 1. Download NVIDIA SDK Manager
# https://developer.nvidia.com/nvidia-sdk-manager

# 2. Install SDK Manager
sudo dpkg -i sdkmanager_[version].deb
sudo apt install -f

# 3. Connect Jetson NX via USB-C (in recovery mode)
# - Power off Jetson
# - Connect USB-C cable to host PC
# - Hold RECOVERY button
# - Press POWER button
# - Release RECOVERY after 2 seconds

# 4. Launch SDK Manager
sdkmanager

# 5. Follow GUI to flash JetPack 5.1+ with:
# - CUDA 11.4+
# - cuDNN 8.6+
# - TensorRT 8.5+
# - OpenCV 4.5.4+ (with CUDA)
```

### Step 2: Initial Jetson Setup

```bash
# ON JETSON (after flashing):

# 1. Update system
sudo apt update && sudo apt upgrade -y

# 2. Install essential tools
sudo apt install -y git nano htop python3-pip

# 3. Check CUDA installation
nvcc --version  # Should show CUDA 11.4+

# 4. Install jetson-stats (system monitoring)
sudo -H pip3 install jetson-stats
sudo reboot

# After reboot, run:
jtop  # Interactive system monitor
```

### Step 3: Install PyTorch for Jetson

```bash
# Download PyTorch wheel from NVIDIA
# Check version: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048

# For JetPack 5.1 + Python 3.8:
wget https://nvidia.box.com/shared/static/[...].whl
# (Get actual link from NVIDIA forums)

# Install PyTorch
sudo apt install -y libopenblas-base libopenmpi-dev
pip3 install torch-*.whl

# Install torchvision
sudo apt install -y libjpeg-dev zlib1g-dev
git clone --branch v0.15.0 https://github.com/pytorch/vision torchvision
cd torchvision
pip3 install -e .

# Verify
python3 -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
# Should print: 2.x.x and True
```

### Step 4: Install ROS Noetic on Jetson

```bash
# Add ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

# Install ROS Noetic
sudo apt install -y ros-noetic-ros-base python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 5: Clone and Build FIRE-EDGE

```bash
# Clone repository
cd ~
git clone https://github.com/yourusername/fire-edge.git

# Install Python dependencies
cd fire-edge
pip3 install -r requirements.txt

# Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
ln -s ~/fire-edge .
cd ~/catkin_ws
catkin_make

# Source workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 6: Optimize YOLOv8 with TensorRT

```bash
cd ~/fire-edge

# Export YOLOv8 to TensorRT engine
python3 scripts/export_tensorrt.py \
    --model models/yolov8n_thermal_yellow.pt \
    --imgsz 640 \
    --device cuda:0 \
    --fp16  # Use FP16 precision for speed

# This creates: models/yolov8n_thermal_yellow.engine
# Inference will be ~2-3x faster than PyTorch
```

### Step 7: Configure Sensors

```bash
# Enable I2C, SPI, Serial
sudo usermod -aG i2c,dialout,gpio $USER
sudo reboot

# Test I2C (IMU)
sudo i2cdetect -y -r 1
# Should detect MPU-9250 at address 0x68

# Test SPI (FLIR Lepton) - if using Breakout Board
sudo apt install python3-spidev
python3 tests/test_flir_lepton.py

# Test Serial (Ultrasonic sensors)
python3 tests/test_ultrasonic.py
```

---

## Option C: Demo/Development Mode

### Quick Start (No Hardware)

```bash
# 1. Clone repository
git clone https://github.com/yourusername/fire-edge.git
cd fire-edge

# 2. Create virtual environment
python3 -m venv venv
source venv/bin/activate

# 3. Install minimal dependencies
pip install opencv-python numpy ultralytics

# 4. Run basic demo
python demo/ar_firefighter_helmet_cv.py

# 5. Run navigation demo
python demo/ar_navigation.py

# 6. Run full simulator
python demo/firefighter_simulator.py
```

---

## Hardware Assembly

### Wiring Diagram

```
┌────────────────────────────────────────────────────┐
│         NVIDIA Jetson Xavier NX                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐          │
│  │   USB    │  │   I2C    │  │   SPI    │          │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘          │
└───────┼─────────────┼─────────────┼────────────────┘
        │             │             │
        │             │             └──> FLIR Lepton 3.5
        │             │                  (SPI0, GPIO for VSync)
        │             │
        │             └──> MPU-9250 IMU
        │                  (I2C Bus 1, Address 0x68)
        │
        └──> Garmin LiDAR-Lite v3HP
             (I2C Bus 1, Address 0x62)

┌─────────────────────────────────────────────────────┐
│         Serial/GPIO Connections                     │
├─────────────────────────────────────────────────────┤
│  GPIO Pin 7  -> Ultrasonic #1 (Front)               │
│  GPIO Pin 11 -> Ultrasonic #2 (Right)               │
│  GPIO Pin 13 -> Ultrasonic #3 (Left)                │
│  GPIO Pin 15 -> Ultrasonic #4 (Back)                │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│         Power Distribution                          │
├─────────────────────────────────────────────────────┤
│  14.8V Battery -> DC-DC Converter -> Jetson NX (5V) │
│                -> LDO Regulator  -> Sensors (3.3V)  │
│                -> Boost Converter -> Display (12V)  │
└─────────────────────────────────────────────────────┘
```

### Assembly Steps

1. **Mount Jetson NX** on carrier board
2. **Connect FLIR Lepton** via SPI breakout board
3. **Wire MPU-9250** to I2C Bus 1
4. **Connect LiDAR** to I2C Bus 1 (different address)
5. **Attach Ultrasonic Sensors** to GPIO pins
6. **Connect Kopin Display** via HDMI/DisplayPort
7. **Wire Power System** with battery management
8. **Secure Components** in enclosure

---

## Sensor Calibration

### IMU Calibration (MPU-9250)

```bash
# Run calibration script
python3 scripts/calibrate_imu.py

# Follow instructions:
# 1. Place IMU on flat surface
# 2. Keep stationary for 60 seconds
# 3. Rotate slowly in all axes
# 4. Keep stationary again for 30 seconds

# Saves calibration to: config/imu_calibration.yaml
```

### LiDAR Calibration

```bash
# Test LiDAR range accuracy
python3 scripts/test_lidar.py

# Place objects at known distances (1m, 2m, 5m, 10m)
# Verify readings are within ±2.5cm
```

### Thermal Camera Calibration

```bash
# Run flat-field correction
python3 scripts/calibrate_thermal.py

# 1. Point camera at uniform temperature surface
# 2. Capture 100 frames
# 3. Generates offset correction matrix

# Saves to: config/thermal_ffc.npy
```

### System Calibration

```bash
# End-to-end calibration
roslaunch fire_edge calibrate.launch

# Walks through:
# 1. IMU bias estimation
# 2. Kalman filter Q/R tuning
# 3. Camera-IMU extrinsic calibration
# 4. Display alignment (homography)
```

---

## Troubleshooting

### Issue: Camera Not Detected

```bash
# List video devices
ls -l /dev/video*

# Try different indices
python3 -c "
import cv2
for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f'Camera {i}: Available')
        cap.release()
    else:
        print(f'Camera {i}: Not found')
"

# Grant camera permissions (Linux)
sudo usermod -a -G video $USER
sudo reboot
```

### Issue: CUDA Out of Memory (Jetson)

```bash
# Check memory usage
jtop  # Interactive monitor

# Reduce model size
# In config/yolov8_config.yaml:
imgsz: 416  # Down from 640

# Enable Jetson power mode
sudo nvpmodel -m 0  # MAXN mode (highest performance)
sudo jetson_clocks  # Lock clocks to maximum
```

### Issue: ROS Nodes Not Communicating

```bash
# Check ROS master
rosnode list
rostopic list

# Test topic
rostopic echo /thermal/image_raw

# Check network
echo $ROS_MASTER_URI
echo $ROS_IP

# Reset if needed
killall roscore rosmaster
roscore &
```

### Issue: Low FPS Performance

```bash
# Option 1: Reduce resolution
# In launch/simulation.launch:
<param name="camera/width" value="640"/>
<param name="camera/height" value="480"/>

# Option 2: Disable GUI
roslaunch fire_edge simulation.launch gui:=false

# Option 3: Use TensorRT engine (Jetson)
# See Step 6 of Jetson setup

# Option 4: Profile bottlenecks
python3 -m cProfile -o profile.stats scripts/test_detection.py
python3 -c "import pstats; p = pstats.Stats('profile.stats'); p.sort_stats('cumtime'); p.print_stats(20)"
```

---

## Verification Tests

### Test 1: Thermal Detection

```bash
python3 tests/test_detection.py \
    --video data/smoke_corridor.mp4 \
    --model models/yolov8n_thermal_yellow.pt \
    --save-video results/detection_output.mp4

# Expected output:
# - Detection mAP@0.5: >80%
# - Average latency: <50ms
# - No crashes
```

### Test 2: Kalman Filter

```bash
python3 tests/test_kalman.py --visualize

# Should show:
# - Smooth trajectory
# - Drift <0.5m/min
# - Converging covariance
```

### Test 3: Full System

```bash
# Run full integration test
rostest fire_edge test_system.test

# Checks:
# [PASS] All nodes running
# [PASS] Messages publishing at correct rates
# [PASS] Detection latency <100ms
# [PASS] Navigation drift <0.5m/min
# [PASS] No memory leaks
```

### Test 4: Hardware (if available)

```bash
python3 tests/test_sensors.py --hardware

# Tests:
# - FLIR Lepton: Image capture
# - IMU: Gyro/Accel readings
# - LiDAR: Distance measurement
# - Ultrasonic: All 4 sensors
# - Display: Output test pattern
```

---

## Next Steps

After successful setup:

1. ✅ Read [User Manual](USER_MANUAL.md)
2. ✅ Review [Architecture Documentation](ARCHITECTURE.md)
3. ✅ Try sample scenarios in Gazebo
4. ✅ Experiment with different Kalman filter parameters
5. ✅ Train YOLOv8 on your own thermal data

---

## Getting Help

- 📧 **Email**: adhamt864@gmail.com

---

**Setup complete! You're ready to develop with FIRE-EDGE.** 🔥🚒
