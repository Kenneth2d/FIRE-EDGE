# FIRE-EDGE System Architecture

**Technical Design Document** | Version 1.0 | December 2024

Comprehensive architectural overview of the FIRE-EDGE real-time firefighting edge system.

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Hardware Layer](#hardware-layer)
4. [Edge Processing Layer](#edge-processing-layer)
5. [Software Components](#software-components)
6. [Data Flow](#data-flow)
7. [Sensor Fusion Algorithm](#sensor-fusion-algorithm)
8. [Object Detection Pipeline](#object-detection-pipeline)
9. [Navigation System](#navigation-system)
10. [AR Overlay Rendering](#ar-overlay-rendering)
11. [Communication Architecture](#communication-architecture)
12. [Performance Optimization](#performance-optimization)
13. [Failure Modes](#failure-modes)
14. [Future Extensions](#future-extensions)

---

## Overview

### Design Philosophy

FIRE-EDGE follows a **distributed edge computing architecture** with these core principles:

1. **Edge-First Processing**: All critical computations on-device (no cloud dependency)
2. **Sensor Fusion**: Multiple complementary sensors for robustness
3. **Real-Time Constraints**: Hard deadlines for life-safety operations (<50ms latency)
4. **Graceful Degradation**: System continues operating with partial sensor failures
5. **Modularity**: Loosely-coupled components via ROS middleware

### Key Design Decisions

| Decision | Rationale |
|----------|-----------|
| **NVIDIA Jetson Xavier NX** | Only edge device with 21 TOPS for real-time YOLOv8 |
| **ROS Noetic** | Industry-standard middleware, mature ecosystem |
| **YOLOv8 Nano** | Best accuracy/speed tradeoff for edge deployment |
| **Kalman Filter** | Optimal for linear Gaussian systems, real-time performance |
| **FLIR Lepton 3.5** | Lowest-cost radiometric thermal camera |
| **Kopin OLED** | Highest brightness (10,000 nits) for smoke visibility |

---

## System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         HARDWARE LAYER                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │ Thermal  │  │  LiDAR   │  │   IMU    │  │Ultrasonic│      │
│  │ (30Hz)   │  │  (10Hz)  │  │ (100Hz)  │  │  (5Hz)   │      │
│  └─────┬────┘  └─────┬────┘  └─────┬────┘  └─────┬────┘      │
└────────┼─────────────┼─────────────┼─────────────┼────────────┘
         │             │             │             │
         └─────────────┴─────────────┴─────────────┘
                       │
         ┌─────────────▼─────────────┐
         │    ROS Middleware Layer   │
         │  (Message Passing, Sync)  │
         └─────────────┬─────────────┘
                       │
    ┌──────────────────┼──────────────────┐
    │                  │                  │
    ▼                  ▼                  ▼
┌─────────┐    ┌──────────────┐    ┌─────────┐
│ Thermal │    │   Sensor     │    │   AR    │
│Processor│───▶│   Fusion     │───▶│ Display │
│ YOLOv8  │    │Kalman Filter │    │ OpenCV  │
└─────────┘    └──────────────┘    └─────────┘
    │                  │                  │
    │                  │                  │
    └──────────────────┴──────────────────┘
                       │
                       ▼
              ┌────────────────┐
              │  Kopin Display │
              │  1280x720 OLED │
              └────────────────┘
```

### Layer Breakdown

#### Layer 1: Hardware Abstraction
- **Responsibility**: Interface with physical sensors
- **Components**: Device drivers, I/O handlers
- **Technologies**: I2C, SPI, USB, GPIO
- **Fault Tolerance**: Retry logic, timeout handling

#### Layer 2: ROS Middleware
- **Responsibility**: Message passing, time synchronization
- **Components**: ROS nodes, topics, services
- **Technologies**: ROS Noetic, message_filters
- **Fault Tolerance**: Automatic reconnection, buffering

#### Layer 3: Processing Nodes
- **Responsibility**: Sensor processing and fusion
- **Components**: thermal_processor, sensor_fusion, ar_display
- **Technologies**: PyTorch, NumPy, OpenCV
- **Fault Tolerance**: Watchdog timers, exception handling

#### Layer 4: Display Output
- **Responsibility**: AR visualization
- **Components**: Overlay renderer, homography transform
- **Technologies**: OpenCV, Kopin SDK
- **Fault Tolerance**: Fallback to simple text display

---

## Hardware Layer

### Sensor Suite Specifications

#### 1. FLIR Lepton 3.5 Thermal Camera

**Technical Specifications**:
```
Resolution:        160×120 pixels
Sensor Type:       Uncooled VOx Microbolometer
Spectral Range:    8-14 μm (LWIR)
Frame Rate:        8.7 Hz (default), configurable
Field of View:     57° × 44° (horizontal × vertical)
Thermal Sensitivity: <50 mK (NETD)
Temperature Range: -10°C to +140°C (high gain)
                   -10°C to +450°C (low gain)
Interface:         SPI (up to 20 MHz)
Power:             150 mW typical
Dimensions:        8.5 × 8.5 × 11.0 mm
```

**Integration Details**:
- Connected via SPI0 on Jetson GPIO header
- VSync interrupt on GPIO7 for frame synchronization
- Flat-Field Correction (FFC) runs every 3 minutes
- Radiometric mode enabled for absolute temperature

**Driver Architecture**:
```python
class FLIRLeptonDriver:
    def __init__(self, spi_dev='/dev/spidev0.0'):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)  # Bus 0, Device 0
        self.spi.max_speed_hz = 20000000  # 20 MHz
        self.vsync_pin = 7
        GPIO.setup(self.vsync_pin, GPIO.IN)
        
    def capture_frame(self) -> np.ndarray:
        """Capture 16-bit thermal frame via SPI."""
        # Wait for VSync pulse
        GPIO.wait_for_edge(self.vsync_pin, GPIO.RISING, timeout=200)
        
        # Read frame data (160*120*2 = 38,400 bytes)
        frame_data = self.spi.readbytes(38400)
        
        # Convert to numpy array and reshape
        frame = np.frombuffer(frame_data, dtype=np.uint16)
        frame = frame.reshape(120, 160)
        
        # Convert to temperature (if radiometric)
        temp_celsius = self.raw_to_celsius(frame)
        
        return temp_celsius
```

#### 2. Garmin LIDAR-Lite v3HP

**Technical Specifications**:
```
Range:             5 cm to 40 m
Accuracy:          ±2.5 cm
Update Rate:       ~10 Hz (adjustable)
Resolution:        1 cm
Beam Divergence:   ~8 mrad
Interface:         I2C (0x62 default address)
Power:             5V, 105 mA peak (1.8W)
Dimensions:        40 × 48 × 20 mm
```

**Integration Details**:
- Connected to I2C Bus 1 on Jetson
- Trigger measurement, wait for ready flag, read distance
- Measurement delay: ~20-100ms depending on range
- Adaptive acquisition count for noise rejection

**Driver Architecture**:
```python
class LiDARLiteDriver:
    I2C_ADDRESS = 0x62
    MEASURE_REG = 0x00
    STATUS_REG = 0x01
    DISTANCE_REG = 0x8f
    
    def __init__(self, bus=1):
        self.bus = smbus2.SMBus(bus)
        
    def measure_distance(self) -> float:
        """Trigger measurement and return distance in meters."""
        # Write 0x04 to register 0x00 to start measurement
        self.bus.write_byte_data(self.I2C_ADDRESS, 
                                 self.MEASURE_REG, 0x04)
        
        # Poll status register until bit 0 is clear (ready)
        timeout = time.time() + 0.2  # 200ms timeout
        while time.time() < timeout:
            status = self.bus.read_byte_data(self.I2C_ADDRESS, 
                                             self.STATUS_REG)
            if (status & 0x01) == 0:
                break
            time.sleep(0.001)
        
        # Read 2-byte distance from high and low registers
        distance_cm = self.bus.read_word_data(self.I2C_ADDRESS, 
                                              self.DISTANCE_REG)
        
        # Convert to meters and return
        return distance_cm / 100.0
```

#### 3. TDK InvenSense MPU-9250 IMU

**Technical Specifications**:
```
Gyroscope:
  - Range: ±250, ±500, ±1000, ±2000 °/s
  - Sensitivity: 131 LSB/(°/s) @ ±250°/s
  - Output Rate: up to 8 kHz
  
Accelerometer:
  - Range: ±2, ±4, ±8, ±16 g
  - Sensitivity: 16384 LSB/g @ ±2g
  - Output Rate: up to 4 kHz
  
Magnetometer (AK8963):
  - Range: ±4800 μT
  - Sensitivity: 0.6 μT/LSB (16-bit)
  - Output Rate: 100 Hz

Interface:         I2C (0x68) or SPI
Digital Output:    16-bit ADC
Power:            3.3V, 3.5 mA (gyro+accel)
```

**Integration Details**:
- Connected to I2C Bus 1 at address 0x68
- Configured for ±2000°/s gyro, ±16g accel
- Digital Low-Pass Filter (DLPF) at 20 Hz
- Sampling at 100 Hz for real-time tracking

**Sensor Fusion Algorithm**:
```python
class MPU9250Driver:
    def __init__(self, bus=1, address=0x68):
        self.bus = smbus2.SMBus(bus)
        self.address = address
        self._configure()
        
    def _configure(self):
        """Configure sensor ranges and filters."""
        # Wake up (disable sleep)
        self.bus.write_byte_data(self.address, 0x6B, 0x00)
        
        # Gyro: ±2000°/s
        self.bus.write_byte_data(self.address, 0x1B, 0x18)
        
        # Accel: ±16g
        self.bus.write_byte_data(self.address, 0x1C, 0x18)
        
        # DLPF: 20 Hz bandwidth
        self.bus.write_byte_data(self.address, 0x1A, 0x04)
        
    def read_imu(self) -> Dict[str, np.ndarray]:
        """Read all IMU data."""
        # Read 14 bytes starting at ACCEL_XOUT_H (0x3B)
        data = self.bus.read_i2c_block_data(self.address, 0x3B, 14)
        
        # Parse data
        accel_x = self._bytes_to_int(data[0], data[1]) / 2048.0  # g
        accel_y = self._bytes_to_int(data[2], data[3]) / 2048.0
        accel_z = self._bytes_to_int(data[4], data[5]) / 2048.0
        
        gyro_x = self._bytes_to_int(data[8], data[9]) / 16.4  # °/s
        gyro_y = self._bytes_to_int(data[10], data[11]) / 16.4
        gyro_z = self._bytes_to_int(data[12], data[13]) / 16.4
        
        return {
            'accel': np.array([accel_x, accel_y, accel_z]),
            'gyro': np.array([gyro_x, gyro_y, gyro_z]),
            'timestamp': time.time()
        }
```

#### 4. A02YYUW Ultrasonic Sensors (×4)

**Technical Specifications**:
```
Range:             20 cm to 450 cm
Accuracy:          ±1 cm
Resolution:        1 mm
Beam Angle:        ~15°
Interface:         UART (9600 baud)
Power:             3.3-5V, 3 mA
Waterproof:        IP67 rated
Response Time:     100 ms
```

**Array Configuration**:
- **Front**: 0° (forward direction)
- **Right**: 90° (starboard)
- **Left**: 270° (port)
- **Back**: 180° (aft)

**Driver Architecture**:
```python
class UltrasonicArray:
    def __init__(self):
        self.sensors = {
            'front': serial.Serial('/dev/ttyTHS1', 9600),
            'right': serial.Serial('/dev/ttyTHS2', 9600),
            'left': serial.Serial('/dev/ttyTHS3', 9600),
            'back': serial.Serial('/dev/ttyTHS4', 9600)
        }
        
    def read_all(self) -> Dict[str, float]:
        """Read distance from all sensors."""
        distances = {}
        for name, port in self.sensors.items():
            if port.in_waiting >= 4:
                # Read 4-byte packet: 0xFF 0xFF (H-data) (L-data) (checksum)
                data = port.read(4)
                if data[0] == 0xFF and data[1] == 0xFF:
                    distance_mm = (data[2] << 8) | data[3]
                    distances[name] = distance_mm / 1000.0  # Convert to meters
                else:
                    distances[name] = None  # Invalid reading
            else:
                distances[name] = None  # No data
        return distances
```

---

## Edge Processing Layer

### NVIDIA Jetson Xavier NX

**Hardware Specifications**:
```
GPU:              384-core NVIDIA Volta
                  48 Tensor Cores
                  21 TOPS (INT8)
                  
CPU:              6-core NVIDIA Carmel ARM v8.2 (64-bit)
                  6 MB L2 + 4 MB L3 cache
                  
Memory:           16 GB LPDDR4x (137 GB/s)

Storage:          16 GB eMMC 5.1
                  microSD slot (expandable)
                  
Power Modes:      MODE_10W (10W), MODE_15W (15W)
                  MODE_2CORE (CPU only)
                  
Connectivity:     Gigabit Ethernet
                  M.2 Key E (WiFi/Bluetooth)
                  4× USB 3.1
                  
Video:            2× MIPI CSI-2 (camera input)
                  HDMI 2.0 + DP 1.4 output
                  
Dimensions:       103 × 90 × 31 mm
```

**Power Management**:
```bash
# Set to maximum performance (15W mode)
sudo nvpmodel -m 0

# Lock clocks to maximum
sudo jetson_clocks

# Monitor power consumption
jtop  # Interactive GUI
tegrastats  # Command-line stats
```

**Thermal Management**:
- Passive cooling: Heat sink (required)
- Active cooling: 5V PWM fan (recommended for sustained load)
- Thermal throttling: Kicks in at 95°C
- Safe operating: <80°C recommended

---

## Software Components

### ROS Node Architecture

#### 1. Thermal Processor Node (`thermal_processor.py`)

**Responsibilities**:
- Capture frames from FLIR Lepton
- Run YOLOv8 inference for fire/human detection
- Publish detection results and thermal images

**Node Configuration**:
```yaml
thermal_processor:
  node_name: thermal_processor
  rate: 30  # Hz
  
  camera:
    device: /dev/spidev0.0
    resolution: [160, 120]
    ffc_interval: 180  # seconds
    
  yolo:
    model_path: models/yolov8n_thermal_yellow.pt
    confidence_threshold: 0.5
    iou_threshold: 0.45
    device: cuda:0
    fp16: true
    
  publishing:
    thermal_image_topic: /thermal/image_raw
    detections_topic: /fire_edge/detections
    visualization_topic: /fire_edge/thermal_viz
```

**Main Loop**:
```python
class ThermalProcessorNode:
    def __init__(self):
        rospy.init_node('thermal_processor')
        
        # Load YOLOv8 model
        self.model = YOLO(rospy.get_param('~model_path'))
        self.model.to('cuda')
        
        # Initialize thermal camera
        self.camera = FLIRLeptonDriver()
        
        # Publishers
        self.thermal_pub = rospy.Publisher(
            '/thermal/image_raw', Image, queue_size=1
        )
        self.detection_pub = rospy.Publisher(
            '/fire_edge/detections', Detections, queue_size=1
        )
        
        # Processing rate
        self.rate = rospy.Rate(30)
        
    def run(self):
        while not rospy.is_shutdown():
            # Capture thermal frame
            thermal_frame = self.camera.capture_frame()
            
            # Convert to 8-bit for YOLOv8
            thermal_8bit = self.normalize_thermal(thermal_frame)
            
            # Run inference
            results = self.model(thermal_8bit, verbose=False)[0]
            
            # Parse detections
            detections = self.parse_yolo_results(results)
            
            # Publish thermal image
            thermal_msg = self.numpy_to_ros_image(thermal_8bit)
            self.thermal_pub.publish(thermal_msg)
            
            # Publish detections
            detection_msg = self.create_detection_msg(detections)
            self.detection_pub.publish(detection_msg)
            
            self.rate.sleep()
```

#### 2. Sensor Fusion Node (`sensor_fusion.py`)

**Responsibilities**:
- Fuse IMU, LiDAR, and ultrasonic data
- Implement Kalman filter for pose estimation
- Publish current position and path history

**Node Configuration**:
```yaml
sensor_fusion:
  node_name: sensor_fusion
  rate: 100  # Hz (driven by IMU)
  
  kalman:
    state_dim: 5  # [x, y, vx, vy, theta]
    measurement_dim: 3  # [x, y, theta]
    process_noise_Q: [0.1, 0.1, 0.05, 0.05, 0.01]
    measurement_noise_R: [0.5, 0.5, 0.1]
    
  sensors:
    imu_topic: /imu/data
    lidar_topic: /lidar/range
    ultrasonic_topic: /ultrasonic/ranges
    
  output:
    pose_topic: /fire_edge/pose
    path_topic: /fire_edge/path
    odom_topic: /fire_edge/odom
```

**Kalman Filter Implementation**:
```python
class SensorFusionNode:
    def __init__(self):
        rospy.init_node('sensor_fusion')
        
        # Initialize Kalman filter
        self.kf = KalmanFilter(
            dim_x=5,  # State: [x, y, vx, vy, theta]
            dim_z=3   # Measurement: [x, y, theta]
        )
        
        # State transition matrix (constant velocity model)
        dt = 0.01  # 100 Hz
        self.kf.F = np.array([
            [1, 0, dt, 0,  0],  # x
            [0, 1, 0,  dt, 0],  # y
            [0, 0, 1,  0,  0],  # vx
            [0, 0, 0,  1,  0],  # vy
            [0, 0, 0,  0,  1]   # theta
        ])
        
        # Measurement matrix
        self.kf.H = np.array([
            [1, 0, 0, 0, 0],  # Measure x
            [0, 1, 0, 0, 0],  # Measure y
            [0, 0, 0, 0, 1]   # Measure theta
        ])
        
        # Process noise
        Q_diag = rospy.get_param('~kalman/process_noise_Q')
        self.kf.Q = np.diag(Q_diag)
        
        # Measurement noise
        R_diag = rospy.get_param('~kalman/measurement_noise_R')
        self.kf.R = np.diag(R_diag)
        
        # Subscribers (with time synchronization)
        self.imu_sub = message_filters.Subscriber('/imu/data', Imu)
        self.lidar_sub = message_filters.Subscriber('/lidar/range', Range)
        
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.imu_sub, self.lidar_sub], 
            queue_size=10, 
            slop=0.05
        )
        ts.registerCallback(self.sensor_callback)
        
        # Publisher
        self.pose_pub = rospy.Publisher(
            '/fire_edge/pose', PoseStamped, queue_size=1
        )
        
    def sensor_callback(self, imu_msg, lidar_msg):
        """Fuse sensor data with Kalman filter."""
        # Predict step
        self.kf.predict()
        
        # Extract measurements
        # (Simplified - actual implementation more complex)
        z = np.array([
            self.estimate_x_from_lidar(lidar_msg),
            self.estimate_y_from_lidar(lidar_msg),
            self.estimate_theta_from_imu(imu_msg)
        ])
        
        # Update step
        self.kf.update(z)
        
        # Publish estimated pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = self.kf.x[0]
        pose_msg.pose.position.y = self.kf.x[1]
        # ... (set orientation from theta)
        
        self.pose_pub.publish(pose_msg)
```

#### 3. AR Display Node (`ar_display.py`)

**Responsibilities**:
- Subscribe to thermal, detections, and pose
- Render AR overlays (bounding boxes, navigation arrows, mini-map)
- Output to Kopin OLED display

**Node Configuration**:
```yaml
ar_display:
  node_name: ar_display
  rate: 30  # Hz
  
  display:
    width: 1280
    height: 720
    device: /dev/fb0  # Framebuffer
    
  rendering:
    detection_color:
      fire: [0, 0, 255]      # Red
      human: [0, 255, 255]   # Yellow
      hazard: [0, 165, 255]  # Orange
    
    minimap_size: 320
    minimap_scale: 40  # pixels per meter
    
  subscriptions:
    thermal_topic: /thermal/image_raw
    detections_topic: /fire_edge/detections
    pose_topic: /fire_edge/pose
    path_topic: /fire_edge/path
```

**Rendering Pipeline**:
```python
class ARDisplayNode:
    def __init__(self):
        rospy.init_node('ar_display')
        
        # Subscribe to data
        self.thermal_sub = rospy.Subscriber(
            '/thermal/image_raw', Image, self.thermal_callback
        )
        self.detection_sub = rospy.Subscriber(
            '/fire_edge/detections', Detections, self.detection_callback
        )
        self.pose_sub = rospy.Subscriber(
            '/fire_edge/pose', PoseStamped, self.pose_callback
        )
        
        # Display configuration
        self.width = 1280
        self.height = 720
        self.display = cv2.namedWindow('FIRE-EDGE', cv2.WND_PROP_FULLSCREEN)
        
        # State
        self.latest_thermal = None
        self.latest_detections = []
        self.latest_pose = None
        
    def render_frame(self) -> np.ndarray:
        """Compose final AR display frame."""
        if self.latest_thermal is None:
            return self.create_blank_frame()
        
        # Start with thermal image (resize to display resolution)
        frame = cv2.resize(self.latest_thermal, (self.width, self.height))
        
        # Draw detection bounding boxes
        for det in self.latest_detections:
            self.draw_detection(frame, det)
        
        # Draw navigation overlay
        if self.latest_pose is not None:
            self.draw_navigation(frame, self.latest_pose)
        
        # Draw mini-map in corner
        minimap = self.render_minimap()
        frame[20:20+minimap.shape[0], -340:-20] = minimap
        
        # Draw status bar
        self.draw_status_bar(frame)
        
        return frame
    
    def draw_detection(self, frame, detection):
        """Draw bounding box and label for detection."""
        x1, y1, x2, y2 = detection.bbox
        class_name = detection.class_name
        confidence = detection.confidence
        
        # Choose color based on class
        if class_name == 'fire':
            color = (0, 0, 255)  # Red
        elif class_name == 'human':
            color = (0, 255, 255)  # Yellow
        else:
            color = (0, 165, 255)  # Orange
        
        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        
        # Draw label background
        label = f"{class_name} {confidence:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (x1, y1-th-10), (x1+tw, y1), color, -1)
        
        # Draw label text
        cv2.putText(frame, label, (x1, y1-5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
```

---

## Data Flow

### End-to-End Processing Pipeline

```
 SENSORS          ROS TOPICS              PROCESSING              OUTPUT
    │                 │                        │                     │
┌───▼────┐      ┌─────▼─────┐          ┌──────▼──────┐      ┌──────▼──────┐
│Thermal │─────▶│/thermal/   │─────────▶│   YOLOv8    │─────▶│ Detection   │
│ Lepton │      │image_raw   │   30Hz   │  Inference  │ 35ms │   Results   │
└────────┘      └────────────┘          └─────────────┘      └──────┬──────┘
                                                                      │
┌────────┐      ┌────────────┐          ┌─────────────┐            │
│  IMU   │─────▶│/imu/data   │──┐       │   Kalman    │◀───────────┘
│MPU9250 │      │            │  └──────▶│   Filter    │
└────────┘      └────────────┘  100Hz   │  Prediction │
                                         │   + Update  │      ┌─────────────┐
┌────────┐      ┌────────────┐   │      └──────┬──────┘─────▶│   Pose      │
│ LiDAR  │─────▶│/lidar/     │───┘             │             │  Estimate   │
│  Lite  │      │range       │   10Hz          │             └──────┬──────┘
└────────┘      └────────────┘                 │                    │
                                                │                    │
┌────────┐      ┌────────────┐                 │                    │
│Ultrasonic─────▶│/ultrasonic/│─────────────────┘                    │
│ Array  │      │ranges      │   5Hz                                │
└────────┘      └────────────┘                                      │
                                                                     │
                                        ┌────────────────────────────┘
                                        │
                                        ▼
                                 ┌──────────────┐
                                 │  AR Overlay  │
                                 │   Renderer   │
                                 └──────┬───────┘
                                        │
                                        ▼
                                 ┌──────────────┐
                                 │    Kopin     │
                                 │OLED Display  │
                                 └──────────────┘
```

### Timing Diagram

```
Time (ms)  0     10    20    30    35    40    50    60    70    80
           │     │     │     │     │     │     │     │     │     │
Thermal    ├─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┴─────┤
(30 Hz)    │ Capture │ Capture │ Capture │ Capture │ Capture │
