#!/usr/bin/env python3
"""
NODE B: HYBRID VISION AGENT (Subscriber)
----------------------------------------
1. Subscribes to /image_raw (from Node A)
2. Detects objects (YOLO11)
3. Reasons about physics (Moondream2)
4. Displays "See & Think" Dashboard
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import gc
import os
import time
import psutil
from PIL import Image as PILImage
from transformers import AutoModelForCausalLM, AutoTokenizer, BitsAndBytesConfig

# --- OPTIMIZATION & CONFIG ---
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:128,expandable_segments:True'

try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False

def cleanup():
    gc.collect()
    if torch.cuda.is_available(): torch.cuda.empty_cache()

# --- VLM CLASS ---
class MoondreamJetsonAgent:
    def __init__(self):
        self.model = None
        self.tokenizer = None
        self.is_loaded = False
        
    def load(self):
        print("⏳ Loading Moondream2 (8-bit Optimized for Jetson)...")
        cleanup()
        model_id = "vikhyatk/moondream2"
        bnb_config = BitsAndBytesConfig(
            load_in_8bit=True,
            llm_int8_enable_fp32_cpu_offload=True,
            llm_int8_skip_modules=["vision_encoder", "vision", "input_layernorm"]
        )
        try:
            self.model = AutoModelForCausalLM.from_pretrained(
                model_id, trust_remote_code=True, quantization_config=bnb_config,
                device_map="auto", low_cpu_mem_usage=True
            )
            self.tokenizer = AutoTokenizer.from_pretrained(model_id)
            # Jetson Stability Patch
            if hasattr(self.model, "vision_encoder"): self.model.vision_encoder.to(dtype=torch.float32)
            elif hasattr(self.model, "vision"): self.model.vision.to(dtype=torch.float32)
            self.is_loaded = True
            print("✅ VLM Loaded")
        except Exception as e:
            print(f"❌ VLM Error: {e}")

    def query(self, img, prompt):
        if not self.is_loaded: return "Loading..."
        try:
            res = self.model.query(img, prompt)
            cleanup()
            return res['answer'].strip()
        except Exception as e: return "Error"

# --- MAIN ROS NODE ---
class VisionReasoningNode(Node):
    def __init__(self):
        super().__init__('vision_reasoning_node')
        self.bridge = CvBridge()
        
        # Load Models
        self.get_logger().info("Initializing AI Models...")
        self.vlm = MoondreamJetsonAgent()
        self.vlm.load()
        self.yolo = YOLO("yolo11n.pt") if HAS_YOLO else None
        
        # Subscribe to Node A
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, qos_profile_sensor_data)
            
        self.last_analysis_time = 0
        self.analysis_interval = 0.5 # 2 Hz Max
        self.get_logger().info("🚀 Vision Agent Listening on /image_raw...")

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pil_img = PILImage.fromarray(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))
            
            # 1. YOLO
            detections = []
            if self.yolo:
                results = self.yolo(cv_img, verbose=False)
                for r in results:
                    for box in r.boxes:
                        if float(box.conf[0]) > 0.5:
                            lbl = self.yolo.names[int(box.cls[0])]
                            detections.append((lbl, float(box.conf[0])))
            
            # 2. VLM Logic
            target = detections[0] if detections else None
            action = "SCANNING"
            thought = "No significant object detected"
            
            if target and (time.time() - self.last_analysis_time > self.analysis_interval):
                self.last_analysis_time = time.time()
                lbl = target[0]
                
                # Combined Query (Material + Action)
                prompt = f"Robot (500g payload). Object: {lbl}. 1. Material/Weight? 2. Reachable? 3. Action (GRASP/PUSH/IGNORE)?"
                thought = self.vlm.query(pil_img, prompt)
                
                if "GRASP" in thought.upper(): action = "GRASP"
                elif "PUSH" in thought.upper(): action = "PUSH"
                elif "IGNORE" in thought.upper(): action = "IGNORE"
                else: action = "THINKING"

            self.print_dashboard(detections, thought, action)
            
        except Exception as e:
            self.get_logger().error(f"Pipeline Error: {e}")

    def print_dashboard(self, detections, thought, action):
        print("\033[2J\033[H", end="") # Clear
        print("="*60)
        print("👁️  VISUAL CORTEX (YOLO + MOONDREAM)")
        print("="*60)
        print(f"Seeing: {len(detections)} Objects")
        for d in detections: print(f" > {d[0].upper()} ({d[1]:.2f})")
        print("-" * 60)
        print(f"🧠 THOUGHT: {thought}")
        print("-" * 60)
        print(f"⚡ ACTION:  [{action}]")
        print("="*60)
        mem = psutil.virtual_memory()
        print(f"RAM Usage: {mem.used/(1024**3):.1f}GB / {mem.total/(1024**3):.1f}GB")

def main(args=None):
    rclpy.init(args=args)
    node = VisionReasoningNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
