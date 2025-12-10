#!/bin/bash
# Quick diagnostic for network bridge issues

echo "=========================================="
echo "Network Bridge Diagnostics"
echo "=========================================="
echo ""

# Check if nodes are running
echo "1. Checking ROS2 nodes..."
NODES=$(ros2 node list 2>/dev/null)
if echo "$NODES" | grep -q "camera_publisher"; then
    echo "   ✓ camera_publisher is running"
else
    echo "   ✗ camera_publisher NOT running"
fi

if echo "$NODES" | grep -q "ros2_network_bridge"; then
    echo "   ✓ ros2_network_bridge is running"
else
    echo "   ✗ ros2_network_bridge NOT running"
    echo ""
    echo "Start the system with:"
    echo "  ros2 launch vlm_bridge complete_network_bridge.launch.py vlm_server_url:=http://YOUR_IP:5000"
    exit 1
fi

echo ""
echo "2. Checking camera topic..."
if ros2 topic info /camera/image_raw &>/dev/null; then
    echo "   ✓ /camera/image_raw exists"
    
    # Check if data is flowing
    echo "   Testing data flow (3 seconds)..."
    RATE=$(timeout 3 ros2 topic hz /camera/image_raw 2>&1 | grep "average rate")
    if [ -n "$RATE" ]; then
        echo "   ✓ Camera publishing: $RATE"
    else
        echo "   ✗ Camera topic exists but NO DATA"
    fi
else
    echo "   ✗ /camera/image_raw does NOT exist"
fi

echo ""
echo "3. Checking bridge subscription..."
SUBS=$(ros2 topic info /camera/image_raw 2>/dev/null | grep "Subscription count")
echo "   $SUBS"
if echo "$SUBS" | grep -q ": 0"; then
    echo "   ⚠ No subscriptions! Bridge may not be connected."
fi

echo ""
echo "4. Checking bridge parameters..."
if ros2 node info /ros2_network_bridge &>/dev/null; then
    VLM_URL=$(ros2 param get /ros2_network_bridge vlm_server_url 2>/dev/null | awk '{print $NF}')
    RATE=$(ros2 param get /ros2_network_bridge inference_rate 2>/dev/null | awk '{print $NF}')
    PREVIEW=$(ros2 param get /ros2_network_bridge show_preview 2>/dev/null | awk '{print $NF}')
    
    echo "   VLM Server: $VLM_URL"
    echo "   Inference Rate: $RATE Hz"
    echo "   Show Preview: $PREVIEW"
    
    # Test VLM server
    echo ""
    echo "5. Testing VLM server connection..."
    if command -v curl &>/dev/null; then
        CLEAN_URL=$(echo "$VLM_URL" | tr -d "'\"")
        if curl -s -f --connect-timeout 3 "$CLEAN_URL/health" &>/dev/null; then
            echo "   ✓ VLM server is reachable at $CLEAN_URL"
            RESPONSE=$(curl -s "$CLEAN_URL/health")
            echo "   Response: $RESPONSE"
        else
            echo "   ✗ Cannot reach VLM server at $CLEAN_URL"
            echo ""
            echo "   Make sure the server is running:"
            echo "   python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000"
        fi
    else
        echo "   (curl not available, skipping server test)"
    fi
fi

echo ""
echo "6. Checking for errors in bridge logs..."
echo "   (Last 5 ERROR messages)"
ros2 node info /ros2_network_bridge 2>&1 | grep -i error | tail -5

echo ""
echo "=========================================="
echo "Quick Checks:"
echo "=========================================="
echo ""
echo "Monitor camera frames:"
echo "  ros2 topic hz /camera/image_raw"
echo ""
echo "Monitor VLM results:"
echo "  ros2 topic echo /vlm/inference_result"
echo ""
echo "See bridge logs in the terminal where you launched it"
echo ""
