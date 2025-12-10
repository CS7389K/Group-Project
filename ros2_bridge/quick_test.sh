#!/bin/bash
# Quick test script for ROS2 Network Bridge

echo "======================================"
echo "ROS2 Network Bridge Quick Test"
echo "======================================"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 not sourced!"
    echo "Run: source /opt/ros/humble/setup.bash"
    echo "     source ~/ros2_ws/install/setup.bash"
    exit 1
else
    echo "✓ ROS2 distro: $ROS_DISTRO"
fi

# Check if nodes are running
echo ""
echo "Checking running nodes..."
NODES=$(ros2 node list 2>/dev/null)

if echo "$NODES" | grep -q "camera_publisher"; then
    echo "✓ Camera publisher is running"
else
    echo "❌ Camera publisher NOT running"
fi

if echo "$NODES" | grep -q "ros2_network_bridge"; then
    echo "✓ Network bridge is running"
else
    echo "❌ Network bridge NOT running"
fi

# Check topics
echo ""
echo "Checking topics..."
TOPICS=$(ros2 topic list 2>/dev/null)

if echo "$TOPICS" | grep -q "/camera/image_raw"; then
    echo "✓ Camera topic exists"
    
    # Check if data is being published
    echo "  Checking if camera is publishing (5 second test)..."
    RATE=$(timeout 5 ros2 topic hz /camera/image_raw 2>&1 | grep "average rate")
    if [ -n "$RATE" ]; then
        echo "  ✓ $RATE"
    else
        echo "  ❌ Camera topic exists but no data being published"
    fi
else
    echo "❌ Camera topic NOT found"
fi

if echo "$TOPICS" | grep -q "/vlm/inference_result"; then
    echo "✓ VLM result topic exists"
else
    echo "❌ VLM result topic NOT found"
fi

# Check bridge parameters
echo ""
echo "Checking bridge parameters..."
if echo "$NODES" | grep -q "ros2_network_bridge"; then
    VLM_URL=$(ros2 param get /ros2_network_bridge vlm_server_url 2>/dev/null | grep -oP 'String value is: \K.*')
    SHOW_PREVIEW=$(ros2 param get /ros2_network_bridge show_preview 2>/dev/null | grep -oP 'Boolean value is: \K.*')
    RATE=$(ros2 param get /ros2_network_bridge inference_rate 2>/dev/null | grep -oP 'Double value is: \K.*')
    
    echo "  VLM Server URL: $VLM_URL"
    echo "  Show Preview: $SHOW_PREVIEW"
    echo "  Inference Rate: $RATE Hz"
    
    if [ "$SHOW_PREVIEW" = "True" ] || [ "$SHOW_PREVIEW" = "true" ]; then
        echo "  ✓ Preview is ENABLED"
    else
        echo "  ❌ Preview is DISABLED - won't see window!"
        echo "     Enable with: ros2 param set /ros2_network_bridge show_preview true"
    fi
    
    # Test VLM server connection
    if [ -n "$VLM_URL" ]; then
        echo ""
        echo "Testing VLM server connection..."
        if curl -s -f "$VLM_URL/health" > /dev/null 2>&1; then
            echo "  ✓ VLM server is reachable at $VLM_URL"
            RESPONSE=$(curl -s "$VLM_URL/health")
            echo "  Response: $RESPONSE"
        else
            echo "  ❌ Cannot connect to VLM server at $VLM_URL"
            echo "     Make sure the server is running on your PC:"
            echo "     python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000"
        fi
    fi
fi

# Check for X display (needed for CV windows)
echo ""
echo "Checking display..."
if [ -n "$DISPLAY" ]; then
    echo "✓ DISPLAY is set: $DISPLAY"
    if xset q &>/dev/null; then
        echo "✓ X server is accessible"
    else
        echo "❌ X server is NOT accessible"
        echo "   If using SSH, connect with: ssh -X user@host"
    fi
else
    echo "❌ DISPLAY not set"
    echo "   CV windows won't work without X11"
    echo "   If using SSH, connect with: ssh -X user@host"
fi

echo ""
echo "======================================"
echo "Summary"
echo "======================================"
echo ""
echo "To see real-time VLM results:"
echo "  ros2 topic echo /vlm/inference_result"
echo ""
echo "To check camera rate:"
echo "  ros2 topic hz /camera/image_raw"
echo ""
echo "To enable preview window:"
echo "  ros2 param set /ros2_network_bridge show_preview true"
echo ""
echo "Full diagnostic:"
echo "  python3 ~/moondream2_turtlebot3/ros2_bridge/diagnose_bridge.py"
echo ""
