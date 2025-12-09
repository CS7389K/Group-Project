# Jetson Deployment Checklist

Use this checklist when deploying to a new Jetson Xavier NX.

## Pre-Installation Checks

- [ ] **Hardware Setup**
  - [ ] Jetson Xavier NX powered on and connected
  - [ ] USB camera connected (or RPi camera)
  - [ ] Network connection available (WiFi or Ethernet)
  - [ ] Sufficient storage space (>20GB free)
  - [ ] TurtleBot3 robot accessible (if testing on hardware)

- [ ] **Software Prerequisites**
  - [ ] Ubuntu 20.04 or 22.04 installed
  - [ ] JetPack 5.x installed (check: `cat /etc/nv_tegra_release`)
  - [ ] ROS2 Humble installed (check: `echo $ROS_DISTRO`)
  - [ ] ROS2 sourced in terminal: `source /opt/ros/humble/setup.bash`
  - [ ] Internet connection working: `ping google.com`

## Installation Steps

### 1. Clone Repository

```bash
cd ~/
git clone https://github.com/CS7389K/Group-Project.git moondream2_turtlebot3
cd moondream2_turtlebot3
```

- [ ] Repository cloned successfully
- [ ] Current directory is `~/moondream2_turtlebot3`

### 2. Run Installation Script

```bash
chmod +x tools/install_jetson.sh
./tools/install_jetson.sh
```

- [ ] Script executable
- [ ] Installation started
- [ ] System dependencies installed (apt packages)
- [ ] PyTorch for Jetson downloaded/installed
- [ ] Python dependencies installed (transformers, ultralytics, etc.)
- [ ] ROS2 workspace created at `~/ros2_ws`
- [ ] Packages linked to workspace
- [ ] rosdep initialized
- [ ] Packages built with colcon
- [ ] Environment configured (.bashrc updated)
- [ ] Model pre-downloaded (if selected)
- [ ] No errors in installation log

### 3. Source Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Or start new terminal (auto-sourced from .bashrc):

- [ ] Workspace sourced
- [ ] ROS2 environment active: `echo $ROS_DISTRO` shows "humble"
- [ ] Workspace in path: `echo $AMENT_PREFIX_PATH` includes ros2_ws

## Post-Installation Verification

### 4. Package Installation Check

```bash
ros2 pkg list | grep -E "vlm|turtlebot3"
```

Expected output:
```
turtlebot3_vlm_perception
vlm_bridge
```

- [ ] Both packages appear in package list
- [ ] No errors when listing packages

### 5. Camera Test

```bash
# Check camera devices
v4l2-ctl --list-devices

# Test camera node
ros2 launch turtlebot3_vlm_perception camera_only.launch.py
```

In another terminal:
```bash
# View camera feed
ros2 run rqt_image_view rqt_image_view
```

- [ ] Camera device detected (e.g., `/dev/video0`)
- [ ] Camera node launches without errors
- [ ] Topic `/camera/image_raw` is publishing: `ros2 topic hz /camera/image_raw`
- [ ] Image visible in rqt_image_view
- [ ] Frame rate stable (15-30 FPS)

### 6. VLM Bridge Test

```bash
# Launch VLM server (takes 1-2 minutes to load)
ros2 launch vlm_bridge vlm_bridge.launch.py with_client:=true
```

- [ ] Model loads without errors (wait 1-2 minutes)
- [ ] No CUDA out-of-memory errors
- [ ] VLM server node appears: `ros2 node list` shows `/vlm_server`
- [ ] VLM client node appears: `ros2 node list` shows `/vlm_client`
- [ ] Response topic exists: `ros2 topic list` shows `/vlm/response`

Monitor responses:
```bash
ros2 topic echo /vlm/response
```

- [ ] VLM responses appearing on topic
- [ ] Responses are coherent (describe scene)
- [ ] No repeated error messages

### 7. Full System Test

```bash
# Launch complete perception stack
ros2 launch turtlebot3_vlm_perception vlm.launch.py
```

- [ ] Camera window opens (OpenCV)
- [ ] YOLO detections appear (bounding boxes)
- [ ] Terminal dashboard displays:
  - [ ] Frame counter incrementing
  - [ ] VISION section shows detections
  - [ ] REASONING section shows VLM analysis
  - [ ] SYSTEM STATUS shows memory usage
- [ ] No crashes or frozen states
- [ ] Performance acceptable:
  - [ ] Camera: ~30 FPS
  - [ ] VLM: 2-3 Hz (as expected)
  - [ ] GPU memory: <6GB

## Performance Optimization

### 8. Enable Max Performance

```bash
# Set max power mode
sudo nvpmodel -m 0

# Max clock speeds
sudo jetson_clocks

# Verify mode
sudo nvpmodel -q
```

- [ ] Power mode set to Mode 0 (MAXN)
- [ ] Clocks maximized
- [ ] System feels responsive

### 9. Monitor Resources

```bash
# GPU/CPU/Memory stats
tegrastats

# Process monitor
htop
```

Check:
- [ ] CPU usage reasonable (<80% average)
- [ ] GPU usage active when VLM running
- [ ] Memory not swapping (swap near 0)
- [ ] Temperature acceptable (<75°C)

## Configuration Tuning

### 10. Adjust for Your Hardware

Edit `~/ros2_ws/src/vlm_bridge/config/vlm_params.yaml`:

```yaml
/**:
  ros__parameters:
    quantization: "8bit"  # Try "4bit" if memory issues
    query_rate: 2.0       # Lower to 1.0 if needed
```

- [ ] Config file exists and editable
- [ ] Changes take effect after relaunch

### 11. Test Different Modes

```bash
# Test 8-bit quantization (default)
ros2 launch vlm_bridge vlm_bridge.launch.py quantization:=8bit

# Test 4-bit quantization (lower memory)
ros2 launch vlm_bridge vlm_bridge.launch.py quantization:=4bit

# Test with different camera
ros2 launch turtlebot3_vlm_perception camera_only.launch.py device:=/dev/video1
```

- [ ] 8-bit mode works
- [ ] 4-bit mode works (if tested)
- [ ] Correct camera device selected

## Troubleshooting

### Common Issues

#### Out of Memory
- [ ] Tried 4-bit quantization: `quantization:=4bit`
- [ ] Closed other applications
- [ ] Checked with `tegrastats` and `free -h`
- [ ] Reduced query rate in config

#### Camera Not Found
- [ ] Checked `ls -la /dev/video*`
- [ ] Verified camera connected
- [ ] Tested with: `v4l2-ctl --list-formats-ext -d /dev/video0`
- [ ] Added user to video group: `sudo usermod -aG video $USER` (then logout/login)

#### Model Download Issues
- [ ] Check internet: `ping huggingface.co`
- [ ] Set cache dir: `export HF_HOME=~/hf_cache`
- [ ] Pre-download manually with Python script

#### Type Annotation Errors
- [ ] Verified transformers version: `pip3 show transformers`
- [ ] Should be <4.46.0 for Python 3.8
- [ ] Reinstall if needed: `pip3 install 'transformers>=4.30.0,<4.46.0'`

#### Build Failures
- [ ] Check ROS2 sourced: `echo $ROS_DISTRO`
- [ ] Clean build: `cd ~/ros2_ws && rm -rf build/ install/ log/`
- [ ] Rebuild: `colcon build --packages-select vlm_bridge turtlebot3_vlm_perception`
- [ ] Check dependencies: `rosdep install --from-paths src --ignore-src -r -y`

## Integration with TurtleBot3

### 12. Robot Integration (Optional)

If deploying on actual TurtleBot3:

- [ ] TurtleBot3 packages installed
- [ ] Robot connected and operational
- [ ] Base navigation working
- [ ] OpenMANIPULATOR connected (if using)

Test:
```bash
# Terminal 1: Launch TurtleBot3
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: Launch VLM perception
ros2 launch turtlebot3_vlm_perception vlm.launch.py

# Terminal 3: Monitor topics
ros2 topic list
```

- [ ] Both systems running simultaneously
- [ ] No topic conflicts
- [ ] VLM decisions could be used for navigation/manipulation

## Final Checks

### 13. Documentation Review

- [ ] Read `INSTALL.md` for detailed info
- [ ] Read `QUICKSTART.md` for quick reference
- [ ] Read `ros2_bridge/README.md` for VLM bridge details
- [ ] Read `ARCHITECTURE.md` for system architecture

### 14. Backup Configuration

Save working configuration:

```bash
# Backup config files
cp ~/ros2_ws/src/vlm_bridge/config/vlm_params.yaml ~/vlm_params.yaml.backup
cp ~/ros2_ws/src/turtlebot3_vlm_perception/config/perception_params.yaml ~/perception_params.yaml.backup

# Note working settings
echo "Working configuration:" > ~/deployment_notes.txt
echo "Quantization: 8bit" >> ~/deployment_notes.txt
echo "Query rate: 2.0 Hz" >> ~/deployment_notes.txt
echo "Camera device: /dev/video0" >> ~/deployment_notes.txt
```

- [ ] Configurations backed up
- [ ] Working settings documented

### 15. Test Reboot

```bash
sudo reboot
```

After reboot:

```bash
# Should auto-source from .bashrc
ros2 pkg list | grep vlm

# Test launch
ros2 launch vlm_bridge vlm_bridge.launch.py
```

- [ ] Workspace auto-sources after reboot
- [ ] Packages still available
- [ ] System launches normally

## Deployment Complete! ✅

Your TurtleBot3 VLM perception system is now deployed and ready to use.

### Quick Reference Commands

**Launch VLM Bridge:**
```bash
ros2 launch vlm_bridge vlm_bridge.launch.py
```

**Launch Full System:**
```bash
ros2 launch turtlebot3_vlm_perception vlm.launch.py
```

**Monitor Performance:**
```bash
tegrastats  # GPU/CPU/Memory
htop        # Process monitor
```

**Check Topics:**
```bash
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic echo /vlm/response
```

## Support

If issues persist:
1. Check logs: `~/.ros/log/`
2. Review documentation in repository
3. Check GitHub Issues
4. Contact: vsj23@txstate.edu

---

**Deployment Date:** _________________

**Deployed By:** _________________

**Jetson Serial:** _________________

**Notes:**
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________
