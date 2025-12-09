#!/usr/bin/env python3
"""
Test script for Network Bridge architecture.
Tests connection between ROS2 bridge and standalone VLM server.
"""

import requests
import base64
from io import BytesIO
from PIL import Image
import time

def test_vlm_server(server_url: str = 'http://localhost:5000'):
    """Test standalone VLM server"""
    
    print("=" * 70)
    print("Testing Standalone VLM Server")
    print("=" * 70)
    
    # Test 1: Health check
    print("\n1. Testing health endpoint...")
    try:
        response = requests.get(f'{server_url}/health', timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Server is healthy")
            print(f"   Model loaded: {data['model_loaded']}")
            print(f"   Inference count: {data['inference_count']}")
        else:
            print(f"   ✗ Health check failed: {response.status_code}")
            return False
    except Exception as e:
        print(f"   ✗ Cannot connect to server: {e}")
        print(f"\n   Make sure server is running:")
        print(f"   python3 standalone_vlm_server.py --host 0.0.0.0 --port 5000")
        return False
    
    # Test 2: Inference with test image
    print("\n2. Testing inference endpoint...")
    try:
        # Create a simple test image
        test_image = Image.new('RGB', (640, 480), color='red')
        
        # Add some text to make it interesting
        from PIL import ImageDraw, ImageFont
        draw = ImageDraw.Draw(test_image)
        try:
            # Try to use a default font
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 40)
        except:
            font = ImageFont.load_default()
        draw.text((50, 200), "TEST IMAGE", fill='white', font=font)
        
        # Encode to base64
        buffered = BytesIO()
        test_image.save(buffered, format="JPEG")
        img_base64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
        
        # Send inference request
        payload = {
            'image': img_base64,
            'prompt': 'Describe this image',
            'timestamp': time.time()
        }
        
        print("   Sending test image...")
        start_time = time.time()
        response = requests.post(f'{server_url}/inference', json=payload, timeout=30)
        elapsed = time.time() - start_time
        
        if response.status_code == 200:
            data = response.json()
            print(f"   ✓ Inference successful!")
            print(f"   Result: {data['result'][:100]}...")
            print(f"   Inference time: {data['inference_time_ms']:.0f}ms")
            print(f"   Total round-trip: {elapsed*1000:.0f}ms")
        else:
            print(f"   ✗ Inference failed: {response.status_code}")
            print(f"   Error: {response.text}")
            return False
            
    except Exception as e:
        print(f"   ✗ Inference test failed: {e}")
        return False
    
    print("\n" + "=" * 70)
    print("✓ All tests passed!")
    print("=" * 70)
    print("\nYou can now start the ROS2 bridge on TurtleBot3:")
    print(f"  ros2 launch vlm_bridge network_bridge.launch.py vlm_server_url:={server_url}")
    print("\n" + "=" * 70)
    
    return True


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Test VLM server connection')
    parser.add_argument('--url', type=str, default='http://localhost:5000',
                       help='VLM server URL (default: http://localhost:5000)')
    
    args = parser.parse_args()
    
    success = test_vlm_server(args.url)
    exit(0 if success else 1)
