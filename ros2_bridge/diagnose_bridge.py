#!/usr/bin/env python3
"""
Diagnostic script for ROS2 Network Bridge
==========================================
Run this to check what's working and what's not.
"""

import subprocess
import sys
import time

def run_command(cmd, description):
    """Run a command and show output"""
    print(f"\n{'='*60}")
    print(f"Checking: {description}")
    print(f"Command: {cmd}")
    print('='*60)
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=3)
        print("STDOUT:", result.stdout if result.stdout else "(empty)")
        if result.stderr:
            print("STDERR:", result.stderr)
        print(f"Exit code: {result.returncode}")
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print("TIMEOUT - command took too long")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False

def main():
    print("\n" + "="*60)
    print("ROS2 Network Bridge Diagnostic Tool")
    print("="*60)
    
    checks = []
    
    # Check 1: ROS2 installed
    checks.append(("ROS2 Installation", run_command("which ros2", "ROS2 binary exists")))
    
    # Check 2: Camera topic
    checks.append(("Camera Topic", run_command("ros2 topic list | grep /camera/image_raw", "Camera topic exists")))
    
    # Check 3: Camera publishing
    print(f"\n{'='*60}")
    print("Checking: Camera Publishing Rate")
    print("Command: ros2 topic hz /camera/image_raw --window 5")
    print('='*60)
    print("Running for 5 seconds...")
    try:
        result = subprocess.run(
            "timeout 5 ros2 topic hz /camera/image_raw --window 5",
            shell=True, capture_output=True, text=True
        )
        if "average rate" in result.stdout:
            print("✓ Camera is publishing!")
            print(result.stdout)
            checks.append(("Camera Hz", True))
        else:
            print("✗ Camera not publishing or no data")
            print(result.stdout if result.stdout else "(no output)")
            checks.append(("Camera Hz", False))
    except Exception as e:
        print(f"ERROR: {e}")
        checks.append(("Camera Hz", False))
    
    # Check 4: Bridge node running
    checks.append(("Bridge Node", run_command("ros2 node list | grep ros2_network_bridge", "Bridge node running")))
    
    # Check 5: VLM result topic
    checks.append(("VLM Result Topic", run_command("ros2 topic list | grep /vlm/inference_result", "VLM result topic exists")))
    
    # Check 6: Check if VLM server is reachable
    print(f"\n{'='*60}")
    print("Checking: VLM Server Connection")
    print('='*60)
    print("Trying to read VLM server URL from bridge node parameters...")
    try:
        result = subprocess.run(
            "ros2 param get /ros2_network_bridge vlm_server_url",
            shell=True, capture_output=True, text=True, timeout=3
        )
        if result.returncode == 0 and result.stdout:
            url = result.stdout.strip().split()[-1]
            print(f"VLM Server URL: {url}")
            
            # Try to ping the server
            import requests
            try:
                print(f"Testing connection to {url}/health...")
                response = requests.get(f"{url}/health", timeout=5)
                if response.status_code == 200:
                    print(f"✓ VLM Server is reachable!")
                    print(f"Response: {response.json()}")
                    checks.append(("VLM Server", True))
                else:
                    print(f"✗ VLM Server responded with status {response.status_code}")
                    checks.append(("VLM Server", False))
            except requests.exceptions.ConnectionError:
                print(f"✗ Cannot connect to VLM server")
                print(f"Make sure:")
                print(f"  1. Server is running: python3 standalone_vlm_server.py")
                print(f"  2. Server URL is correct")
                print(f"  3. Firewall allows connection")
                checks.append(("VLM Server", False))
            except Exception as e:
                print(f"✗ Error: {e}")
                checks.append(("VLM Server", False))
        else:
            print("✗ Could not get VLM server URL from bridge node")
            print("Is the bridge node running?")
            checks.append(("VLM Server", False))
    except Exception as e:
        print(f"ERROR: {e}")
        checks.append(("VLM Server", False))
    
    # Check 7: Camera node running
    checks.append(("Camera Node", run_command("ros2 node list | grep camera_publisher", "Camera node running")))
    
    # Check 8: Show preview parameter
    print(f"\n{'='*60}")
    print("Checking: Preview Window Parameter")
    print('='*60)
    try:
        result = subprocess.run(
            "ros2 param get /ros2_network_bridge show_preview",
            shell=True, capture_output=True, text=True, timeout=3
        )
        if "true" in result.stdout.lower():
            print("✓ Preview is ENABLED")
            checks.append(("Preview Enabled", True))
        else:
            print("✗ Preview is DISABLED")
            print("To enable: ros2 param set /ros2_network_bridge show_preview true")
            checks.append(("Preview Enabled", False))
    except Exception as e:
        print(f"ERROR: {e}")
        checks.append(("Preview Enabled", False))
    
    # Summary
    print("\n" + "="*60)
    print("DIAGNOSTIC SUMMARY")
    print("="*60)
    for name, status in checks:
        symbol = "✓" if status else "✗"
        print(f"{symbol} {name:25s} {'OK' if status else 'FAILED'}")
    
    print("\n" + "="*60)
    print("TROUBLESHOOTING TIPS")
    print("="*60)
    
    if not checks[0][1]:  # ROS2
        print("❌ ROS2 not found - source your ROS2 workspace:")
        print("   source /opt/ros/humble/setup.bash")
        print("   source ~/ros2_ws/install/setup.bash")
    
    if not checks[1][1] or not checks[2][1]:  # Camera
        print("\n❌ Camera not publishing:")
        print("   1. Check if camera node is running:")
        print("      ros2 node list")
        print("   2. Launch the camera:")
        print("      ros2 launch vlm_bridge complete_network_bridge.launch.py vlm_server_url:=http://YOUR_IP:5000")
        print("   3. Check camera hardware:")
        print("      ls /dev/video*")
        print("      v4l2-ctl --list-devices")
    
    if not checks[3][1]:  # Bridge
        print("\n❌ Bridge node not running:")
        print("   Launch it with:")
        print("      ros2 launch vlm_bridge complete_network_bridge.launch.py vlm_server_url:=http://YOUR_IP:5000")
    
    if not checks[5][1]:  # VLM Server
        print("\n❌ VLM Server not reachable:")
        print("   1. Start the server on your PC:")
        print("      python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000")
        print("   2. Check firewall:")
        print("      sudo ufw allow 5000/tcp")
        print("   3. Verify IP address in launch command")
    
    if not checks[7][1]:  # Preview
        print("\n❌ Preview window disabled:")
        print("   Enable it with:")
        print("      ros2 param set /ros2_network_bridge show_preview true")
        print("   Or relaunch with:")
        print("      ros2 launch vlm_bridge complete_network_bridge.launch.py show_preview:=true")
    
    print("\n" + "="*60)
    print("To monitor live data:")
    print("="*60)
    print("  Camera images:    ros2 topic echo /camera/image_raw")
    print("  VLM results:      ros2 topic echo /vlm/inference_result")
    print("  Camera rate:      ros2 topic hz /camera/image_raw")
    print("  Bridge logs:      ros2 node info /ros2_network_bridge")
    print("  All topics:       ros2 topic list")
    print("="*60)

if __name__ == '__main__':
    main()
