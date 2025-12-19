# FIRE-EDGE User Manual

**Version 1.0** | December 2024

Complete operational guide for the FIRE-EDGE augmented reality firefighting navigation system.

---

## Table of Contents

1. [Introduction](#introduction)
2. [System Overview](#system-overview)
3. [Getting Started](#getting-started)
4. [Basic Operations](#basic-operations)
5. [Simulation Mode](#simulation-mode)
6. [Hardware Mode](#hardware-mode)
7. [Understanding the Display](#understanding-the-display)
8. [Navigation Features](#navigation-features)
9. [Thermal Detection](#thermal-detection)
10. [Emergency Procedures](#emergency-procedures)
11. [Maintenance](#maintenance)
12. [Troubleshooting](#troubleshooting)
13. [Best Practices](#best-practices)

---

## Introduction

### What is FIRE-EDGE?

FIRE-EDGE (Firefighting Edge) is an augmented reality navigation and hazard detection system designed to help firefighters operate safely in smoke-filled, zero-visibility environments. The system combines:

- **Thermal imaging** to see through smoke
- **AI-powered detection** to identify fires and humans
- **Sensor fusion** for accurate indoor positioning
- **AR overlays** for intuitive navigation guidance

### Who Should Use This Manual?

- **Firefighters** - Operational procedures and emergency use
- **Training Officers** - System setup and training scenarios
- **Researchers** - Understanding system capabilities
- **Developers** - Integration and customization

### Safety Notice

⚠️ **IMPORTANT**: FIRE-EDGE is a research prototype. It is **NOT** certified for operational use. Always:
- Follow standard firefighting protocols
- Use certified primary equipment
- Treat FIRE-EDGE as supplementary information only
- Never rely solely on the system in life-threatening situations

---

## System Overview

### Hardware Components

| Component | Function | Status Indicator |
|-----------|----------|------------------|
| **Jetson Xavier NX** | Main processor | Green LED when active |
| **FLIR Lepton 3.5** | Thermal camera | Red LED when capturing |
| **Garmin LiDAR** | Distance measurement | Blue pulse when ranging |
| **MPU-9250 IMU** | Motion tracking | N/A (internal) |
| **Ultrasonic Array** | Close-range obstacles | N/A (internal) |
| **Kopin OLED Display** | AR visualization | Image output |
| **Battery Pack** | Power supply | LED battery indicator |

### System Specifications

- **Detection Range**: 0.5m - 15m (thermal), up to 40m (LiDAR)
- **Update Rate**: 30 Hz (display), 100 Hz (IMU), 10 Hz (LiDAR)
- **Battery Life**: 45 minutes continuous operation
- **Operating Temperature**: -10°C to 50°C (ambient)
- **Weight**: ~1.2 kg (complete system)
- **Detection Accuracy**: 85% for fire/human detection

---

## Getting Started

### First-Time Setup

#### Simulation Mode (Training/Development)

```bash
# 1. Launch the system
cd ~/fire-edge
source ~/catkin_ws/devel/setup.bash

# 2. Start simulation environment
roslaunch fire_edge simulation.launch scenario:=smoke_corridor

# 3. In new terminal, start detection
rosrun fire_edge thermal_processor.py

# 4. In new terminal, start navigation
rosrun fire_edge sensor_fusion.py

# 5. In new terminal, launch display
rosrun fire_edge ar_display.py
```

#### Hardware Mode (Field Deployment)

```bash
# 1. Power on Jetson Xavier NX (press power button)
# Wait for boot (30-45 seconds)

# 2. SSH into Jetson (if remote)
ssh fire-edge@192.168.1.100

# 3. Launch system
cd ~/fire-edge
./scripts/start_system.sh

# 4. Wait for "SYSTEM READY" message
# All sensors should show green status
```

### Pre-Operation Checklist

Before each use:

- [ ] **Battery charged** (>80% recommended)
- [ ] **All sensors connected** (check LED indicators)
- [ ] **Display functional** (test pattern visible)
- [ ] **Thermal camera calibrated** (run flat-field correction if needed)
- [ ] **IMU initialized** (keep stationary for 10 seconds at startup)
- [ ] **Emergency stop accessible** (power button)

---

## Basic Operations

### Power On/Off

**Power On**:
1. Press power button on battery pack
2. Wait for Jetson boot (~30 seconds)
3. System auto-starts if configured
4. Look for "SYSTEM READY" on display

**Power Off**:
1. Press `Ctrl+C` in main terminal (if manual launch)
2. Or run: `./scripts/stop_system.sh`
3. Wait for clean shutdown message
4. Press power button to cut battery

**Emergency Shutdown**:
- Hold power button for 5 seconds (force shutdown)
- Only use in emergencies (may cause data loss)

### Display Controls

The Kopin OLED display shows real-time information:

**Brightness Adjustment**:
- Default: 10,000 nits (maximum)
- Reduce in low-smoke: `rosservice call /display/brightness 5000`
- Restore: `rosservice call /display/brightness 10000`

**Overlay Toggle**:
- Hide/show navigation: Press `N` key (simulation) or voice command (hardware)
- Hide/show detections: Press `D` key
- Mini-map only: Press `M` key

### Voice Commands (Hardware Only)

If voice interface is enabled:

- **"Navigate to exit"** - Activate return path
- **"Add waypoint"** - Mark current location
- **"Show map"** - Enlarge mini-map view
- **"Clear path"** - Reset navigation history
- **"System status"** - Read out sensor status

---

## Simulation Mode

### Available Scenarios

#### 1. Smoke-Filled Corridor
```bash
roslaunch fire_edge simulation.launch scenario:=smoke_corridor
```
- **Description**: 30m straight corridor with spreading fire
- **Difficulty**: Easy
- **Training Goals**: Basic navigation, fire detection

#### 2. Collapsing Structure
```bash
roslaunch fire_edge simulation.launch scenario:=collapsing_structure
```
- **Description**: Multi-room building with falling debris
- **Difficulty**: Hard
- **Training Goals**: Dynamic obstacle avoidance, emergency navigation

#### 3. Multi-Story Building
```bash
roslaunch fire_edge simulation.launch scenario:=multi_story
```
- **Description**: Three floors with stairs and multiple rooms
- **Difficulty**: Medium
- **Training Goals**: Vertical navigation, complex pathfinding

#### 4. Custom Scenario
```bash
roslaunch fire_edge simulation.launch scenario:=custom world_file:=/path/to/custom.world
```

### Keyboard Controls (Simulation)

When Gazebo window is active:

| Key | Action |
|-----|--------|
| `W` | Move forward |
| `S` | Move backward |
| `A` | Turn left |
| `D` | Turn right |
| `Space` | Add waypoint |
| `N` | Toggle navigation mode |
| `C` | Clear waypoints |
| `R` | Reset position |
| `Q` | Quit simulation |

### Training Exercises

**Exercise 1: Basic Navigation**
1. Start smoke_corridor scenario
2. Move forward 10 meters
3. Add waypoint
4. Move forward 10 more meters
5. Activate navigation to return to start

**Exercise 2: Fire Detection**
1. Start collapsing_structure scenario
2. Navigate to find all fire sources
3. Count detected fires (should find 3)
4. Mark each fire location with waypoint

**Exercise 3: Emergency Egress**
1. Navigate to farthest room in multi_story
2. Simulate loss of visibility (increase smoke)
3. Use navigation system to return to entrance
4. Goal: Exit in <2 minutes

---

## Hardware Mode

### System Initialization

When using actual hardware:

**Step 1: Physical Inspection**
- Check all cables connected
- Verify sensor mounting is secure
- Ensure display is clean and visible
- Battery indicator shows charge level

**Step 2: Sensor Calibration**
```bash
# Run automatic calibration
./scripts/calibrate_all.sh

# Or calibrate individually:
python3 scripts/calibrate_imu.py      # 60 seconds, keep still
python3 scripts/calibrate_thermal.py  # 30 seconds, point at wall
python3 scripts/calibrate_lidar.py    # 10 seconds, automatic
```

**Step 3: System Check**
```bash
# Verify all sensors
python3 tests/test_sensors.py --hardware

# Expected output:
# [PASS] Thermal camera: 160x120 @ 9Hz
# [PASS] IMU: Reading at 100Hz
# [PASS] LiDAR: Range 0.5-40m
# [PASS] Ultrasonic: All 4 sensors responding
# [PASS] Display: 1280x720 output
```

**Step 4: Start Mission**
```bash
./scripts/start_mission.sh

# System will:
# 1. Initialize all sensors
# 2. Load calibration data
# 3. Start detection and fusion
# 4. Begin recording (optional)
# 5. Display "READY" status
```

### Field Operation

**Normal Operation Loop**:
1. System continuously processes thermal feed
2. YOLOv8 detects fires, humans, hazards
3. Kalman filter updates position estimate
4. AR overlay shows navigation guidance
5. Path history automatically recorded

**Adding Waypoints**:
- Button press (if configured)
- Voice command: "Add waypoint"
- Automatic at significant events (fire detected, obstacle encountered)

**Return Navigation**:
- Button press (if configured)
- Voice command: "Navigate to exit"
- System displays arrows following recorded path in reverse

**Monitoring Status**:
- Top-left: System state (TRACKING / LOST)
- Top-right: Battery percentage
- Bottom: Navigation instructions
- Mini-map: Real-time position and obstacles

---

## Understanding the Display

### Main Display Layout

```
┌─────────────────────────────────────────────────────────┐
│  Status: TRACKING     Battery: 85%    12:45:32         │  <- Header
├─────────────────────────────────────────────────────────┤
│                                                          │
│                    THERMAL VIEW                          │
│                 (Fire/Human Detection)                   │
│                                                          │
│   🔥 Fire detected                                       │
│   👤 Human detected                                      │
│                                                          │
├─────────────────────────────────────────────────────────┤
│  ← Turn Left        Walk 15 steps →                     │  <- Navigation
├─────────────────────────────────────────────────────────┤
│  Mini-Map:                                              │
│  ┌─────────────┐                                        │
│  │    ·  ←  ·  │  Your position and heading            │
│  │   · YOU ·   │  Dots = path history                  │
│  │    ·  ·  ·  │  Squares = waypoints                  │
│  └─────────────┘  Red lines = obstacles                │
└─────────────────────────────────────────────────────────┘
```

### Detection Overlays

**Fire Detection**:
- 🔥 **Red bounding box** around detected flames
- **Confidence score** (0-100%) shown above box
- **Distance estimate** if within range
- **Alert priority**: Critical (red flash)

**Human Detection**:
- 👤 **Yellow bounding box** around person
- **Confidence score** displayed
- **Approximate distance** shown
- **Alert priority**: High (yellow pulse)

**Hazard Detection**:
- ⚠️ **Orange bounding box** around obstacles
- **Type label** (debris, furniture, etc.)
- **Clearance distance** if close (<2m)
- **Alert priority**: Medium (orange)

### Navigation Indicators

**Direction Arrows**:
- ⬆️ **Green arrow**: Straight ahead
- ⬅️ **Yellow arrow**: Turn left
- ➡️ **Yellow arrow**: Turn right
- ⬇️ **Red arrow**: Turn around

**Distance Information**:
- "Walk 5 steps" - Estimated steps to next waypoint
- "Turn left" - Change direction
- "Arrived" - Waypoint reached
- "EXIT AHEAD" - Final destination

**Mini-Map Symbols**:
- 🟢 **Green dot**: Your current position
- 🔵 **Blue dots**: Path history (breadcrumbs)
- 🟡 **Yellow squares**: Waypoints
- 🔴 **Red lines**: Detected walls/obstacles
- 🟣 **Purple square**: Detected fires
- 🟠 **Orange square**: Detected humans

---

## Navigation Features

### Automatic Path Recording

**How It Works**:
- System records your position every 0.5 seconds
- Creates a "breadcrumb trail" of your movement
- Stores up to 100 positions (adjustable)
- No user action required

**Using Recorded Path**:
1. When ready to exit, activate navigation
2. System displays arrows following path in reverse
3. Follow arrows to retrace your steps
4. System alerts when you reach entrance

**Path Accuracy**:
- Typical drift: 0.3 meters per minute
- Cumulative over long missions
- More accurate in rooms with distinct features
- May require manual correction in featureless areas

### Waypoint System

**Creating Waypoints**:
- **Manual**: Press button or voice command at important locations
- **Automatic**: System adds waypoints at significant events:
  - Fire detected
  - Person found
  - Major direction change
  - Branching corridor

**Managing Waypoints**:
```bash
# List all waypoints
rostopic echo /fire_edge/waypoints

# Clear waypoints
rosservice call /fire_edge/clear_waypoints

# Navigate to specific waypoint
rosservice call /fire_edge/navigate_to_waypoint 3
```

**Waypoint Information**:
- Position (X, Y coordinates)
- Timestamp
- Event type (manual, fire, person, etc.)
- Notes/description (optional)

### Dead Reckoning

**What is Dead Reckoning?**
- Navigation using IMU to track movement
- No GPS required (indoor operation)
- Estimates position based on:
  - Acceleration (from accelerometer)
  - Rotation (from gyroscope)
  - Heading (from magnetometer)

**Limitations**:
- Accumulates error over time (drift)
- Affected by magnetic interference
- Requires occasional correction from other sensors

**Drift Correction**:
- LiDAR provides absolute distance measurements
- Ultrasonic sensors verify close-range position
- Kalman filter fuses all data to minimize drift
- Expected accuracy: ±0.3m after 1 minute

---

## Thermal Detection

### Understanding Thermal Imaging

**How Thermal Cameras Work**:
- Detect infrared radiation (heat)
- Show temperature differences as colors
- Can "see" through smoke and darkness
- FLIR Lepton 3.5 range: 8-14 micrometers

**Color Scale (INFERNO colormap)**:
- 🟣 **Purple/Blue**: Cold (<20°C)
- 🟢 **Green**: Ambient (~25°C)
- 🟡 **Yellow**: Warm (30-50°C)
- 🟠 **Orange**: Hot (50-100°C)
- 🔴 **Red/White**: Very hot (>100°C, fire)

### Fire Detection Accuracy

**Detection Performance**:
- **Overall Accuracy**: 85% mAP@0.5
- **Precision**: 89.2% (few false positives)
- **Recall**: 86.1% (catches most fires)
- **Inference Time**: 35ms (real-time)

**What Gets Detected**:
- ✅ Open flames
- ✅ Smoldering fires
- ✅ Hot surfaces (walls, doors)
- ✅ Heated gases (smoke plume)
- ✅ Glowing embers

**Potential False Positives**:
- ⚠️ Steam (high humidity environments)
- ⚠️ Hot machinery or appliances
- ⚠️ Direct sunlight on reflective surfaces
- ⚠️ Welding/cutting torches

**Distance Limitations**:
- **0-5m**: Excellent accuracy (>90%)
- **5-10m**: Good accuracy (85-90%)
- **10-15m**: Moderate accuracy (70-85%)
- **>15m**: Reduced accuracy (<70%)

### Human Detection

**Why Detect Humans?**
- Locate victims for rescue
- Identify fellow firefighters
- Avoid friendly fire hazards
- Coordinate team movements

**Detection Characteristics**:
- Human body heat signature distinct from environment
- Typically 36-37°C core temperature
- Infrared signature even through light smoke
- Can detect unconscious victims

**Limitations**:
- Difficult if victim covered by debris
- Challenging if body temperature cooled
- May confuse with large animals
- Clothing can partially obscure signature

---

## Emergency Procedures

### System Failure

**If Display Goes Black**:
1. Check battery indicator LED
2. If battery dead: Replace/recharge
3. If battery OK: Press reset button on Jetson
4. Wait 30 seconds for reboot
5. System should auto-restart
6. If not: Revert to standard firefighting protocols

**If Navigation Lost**:
- Display shows "LOST" status
- Can still see thermal imagery and detections
- Dead reckoning position unreliable
- **Action**: Use thermal view only, navigate manually
- Try to return to last known waypoint
- System may recover tracking automatically

**If Thermal Camera Fails**:
- Black or frozen thermal image
- YOLOv8 detection stops
- **Action**: Switch to standard vision (if available)
- Navigation may still work with LiDAR/IMU
- Exit structure using conventional methods

### Low Battery Warning

**Battery Levels**:
- **100-50%**: Green indicator, normal operation
- **50-20%**: Yellow indicator, plan exit soon
- **20-10%**: Orange indicator, exit immediately
- **<10%**: Red flashing, emergency power

**When Low Battery Warning Appears**:
1. Note current position on mini-map
2. Activate return navigation immediately
3. Expedite egress
4. System will attempt graceful shutdown at 5%
5. Below 5%: Immediate power loss possible

### Emergency Egress

**Activating Emergency Exit**:
```bash
# Voice command (hardware)
"Navigate to exit"

# Or button press (if configured)
# Hold emergency button for 2 seconds

# Or ROS command
rosservice call /fire_edge/emergency_egress
```

**Emergency Navigation Mode**:
- Displays brightest arrows (max visibility)
- Prioritizes fastest/safest route
- Ignores unexplored waypoints
- Continuous audio alerts (if enabled)
- Shortest path calculation every 5 seconds

**If Path Blocked**:
- System shows "ROUTE BLOCKED" warning
- Attempts to find alternate path
- If no alternate: Displays manual exit direction
- Revert to standard egress procedures
- Mark obstacle position for future reference

---

## Maintenance

### Daily Maintenance

**Before Each Mission**:
- [ ] Charge battery to >80%
- [ ] Clean display lens with microfiber cloth
- [ ] Check thermal camera lens (no debris)
- [ ] Verify all cable connections secure
- [ ] Run sensor test: `python3 tests/test_sensors.py`
- [ ] Update system if new version available

**After Each Mission**:
- [ ] Download mission logs: `./scripts/download_logs.sh`
- [ ] Inspect for physical damage
- [ ] Clean exterior (avoid harsh chemicals)
- [ ] Store in protective case
- [ ] Charge battery if <50%

### Weekly Maintenance

- [ ] Run full calibration sequence
- [ ] Check IMU drift over 5-minute test
- [ ] Verify LiDAR accuracy with known distances
- [ ] Thermal camera flat-field correction
- [ ] Review system logs for errors
- [ ] Update software if patches available

### Monthly Maintenance

- [ ] Deep clean all sensors
- [ ] Replace thermal camera lens if scratched
- [ ] Battery health check (capacity test)
- [ ] Inspect all cables for wear
- [ ] Backup system configuration
- [ ] Performance benchmark tests

### Calibration Schedule

| Component | Frequency | Duration | Procedure |
|-----------|-----------|----------|-----------|
| **IMU** | Before each mission | 60 sec | Keep stationary, auto-calibrate |
| **Thermal** | Weekly | 30 sec | Point at uniform surface |
| **LiDAR** | Monthly | 10 sec | Verify against known distances |
| **Display** | As needed | 5 sec | Adjust brightness/alignment |
| **Full System** | Quarterly | 10 min | Complete recalibration suite |

---

## Troubleshooting

### Common Issues

#### Issue: Thermal Image Appears Frozen

**Symptoms**: Same thermal frame repeated, no updates

**Causes**:
- Camera communication failure
- USB/SPI connection loose
- Driver crash

**Solutions**:
1. Check camera connection LED
2. Restart thermal processor:
   ```bash
   rosnode kill /thermal_processor
   rosrun fire_edge thermal_processor.py
   ```
3. If persistent, reboot Jetson
4. Check `dmesg | grep lepton` for errors

#### Issue: High Navigation Drift

**Symptoms**: Mini-map position drifts far from actual location

**Causes**:
- IMU not calibrated
- Magnetic interference
- Poor feature tracking
- Kalman filter divergence

**Solutions**:
1. Re-calibrate IMU:
   ```bash
   python3 scripts/calibrate_imu.py
   ```
2. Check for metal/electronic interference
3. Increase Kalman filter measurement noise (R):
   ```bash
   rosparam set /sensor_fusion/R_diag "[1.0, 1.0, 0.2]"
   ```
4. Manually reset position if severely diverged

#### Issue: No Fire Detections

**Symptoms**: Thermal view shows fire, but no bounding boxes

**Causes**:
- YOLOv8 model not loaded
- Inference threshold too high
- Model file corrupted
- GPU out of memory

**Solutions**:
1. Check model file exists:
   ```bash
   ls -lh models/yolov8n_thermal_yellow.pt
   ```
2. Lower confidence threshold:
   ```bash
   rosparam set /thermal_processor/confidence_threshold 0.3
   ```
3. Restart thermal processor with verbose mode:
   ```bash
   rosrun fire_edge thermal_processor.py --verbose
   ```
4. Check GPU memory:
   ```bash
   jtop  # On Jetson
   nvidia-smi  # On PC
   ```

#### Issue: Display Not Showing

**Symptoms**: Blank OLED display, no image

**Causes**:
- Display not powered
- HDMI/cable disconnected
- Wrong display output selected
- Driver issue

**Solutions**:
1. Check display power LED
2. Verify cable connections
3. Set correct display:
   ```bash
   export DISPLAY=:0
   xrandr --output HDMI-0 --mode 1280x720
   ```
4. Test with simple image:
   ```bash
   feh test_pattern.png
   ```

### Error Messages

| Error | Meaning | Solution |
|-------|---------|----------|
| `TRACKING LOST` | Visual odometry failed | Move to area with more features |
| `LOW FEATURES` | Not enough keypoints detected | Increase lighting or change angle |
| `SENSOR TIMEOUT` | Sensor not responding | Check connection, restart node |
| `MODEL LOAD FAILED` | YOLOv8 model error | Verify model file, check GPU memory |
| `KALMAN DIVERGENCE` | Filter estimates unstable | Re-initialize filter, check Q/R matrices |
| `BATTERY CRITICAL` | <10% power remaining | Exit immediately |

### Getting Help

If issues persist:

1. **Check Logs**:
   ```bash
   # ROS logs
   cat ~/.ros/log/latest/rosout.log
   
   # System logs
   journalctl -u fire-edge.service
   
   # Thermal processor logs
   cat /tmp/thermal_processor.log
   ```

2. **Run Diagnostics**:
   ```bash
   rosrun fire_edge diagnostics.py --full
   ```

3. **Contact Support**:
   - Email: adhamt864@gmail.com
   - Include: logs, error messages, system specs

---

## Best Practices

### Operational Guidelines

**Before Entering Structure**:
1. Complete pre-operation checklist
2. Verify all team members have working systems
3. Establish communication protocol
4. Designate entry point as "home" position
5. Activate path recording

**During Operation**:
1. Keep display visible but don't rely solely on it
2. Add waypoints at major decision points
3. Monitor battery level continuously
4. Verbally confirm fire detections
5. Use thermal view to complement, not replace, visual inspection

**When Exiting**:
1. Activate return navigation early
2. Verify path is clear before following
3. Update team on egress status
4. Download logs after mission for review
5. Report any system anomalies

### Training Recommendations

**New Users**:
1. Start with simulation mode (5+ hours practice)
2. Learn all keyboard/voice commands
3. Practice emergency egress scenarios
4. Understand display symbols and colors
5. Complete all training exercises

**Experienced Users**:
1. Monthly proficiency exercises
2. Test in various smoke conditions
3. Practice with partial system failures
4. Train on new building layouts
5. Review mission logs for improvement

### Optimization Tips

**Improving Detection Accuracy**:
- Keep thermal camera lens clean
- Optimize lighting when possible
- Adjust confidence threshold for environment
- Use multiple viewpoints for confirmation
- Trust the system but verify visually

**Maximizing Battery Life**:
- Reduce display brightness in low smoke
- Disable unused sensors if possible
- Use power-saving mode when stationary
- Carry spare battery for extended missions
- Monitor power consumption in real-time

**Enhancing Navigation**:
- Add waypoints liberally
- Re-calibrate IMU if drift suspected
- Use LiDAR for distance verification
- Mark obstacles clearly
- Review path periodically

---

## Appendix

### Keyboard Shortcuts (Quick Reference)

| Key | Function | Mode |
|-----|----------|------|
| `W` | Move forward | Simulation |
| `S` | Move backward | Simulation |
| `A` | Turn left | Simulation |
| `D` | Turn right | Simulation |
| `Space` | Add waypoint | Both |
| `N` | Toggle navigation | Both |
| `M` | Show mini-map | Both |
| `C` | Clear waypoints | Both |
| `R` | Reset position | Simulation |
| `Q` | Quit | Both |
| `Esc` | Emergency stop | Both |

### ROS Topics (Reference)

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/thermal/image_raw` | `sensor_msgs/Image` | Raw thermal camera feed |
| `/fire_edge/detections` | `fire_edge/Detections` | YOLOv8 detection results |
| `/fire_edge/pose` | `geometry_msgs/PoseStamped` | Estimated position |
| `/fire_edge/waypoints` | `fire_edge/WaypointArray` | Current waypoints |
| `/fire_edge/path` | `nav_msgs/Path` | Recorded path history |
| `/fire_edge/navigation_cmd` | `std_msgs/String` | Navigation commands |

### Configuration Files

| File | Purpose | Location |
|------|---------|----------|
| `yolov8_config.yaml` | Detection parameters | `config/` |
| `kalman_params.yaml` | Sensor fusion settings | `config/` |
| `display_config.yaml` | AR overlay settings | `config/` |
| `imu_calibration.yaml` | IMU calibration data | `config/` |
| `thermal_ffc.npy` | Thermal correction | `config/` |

---

**For additional support, contact the FIRE-EDGE team:**
- 📧 Email: adhamt864@gmail.com

**Stay safe and save lives!** 🔥🚒
