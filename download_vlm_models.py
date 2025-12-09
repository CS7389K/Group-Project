#!/usr/bin/env python3
"""
Download and cache Moondream2 VLM models for offline use

This script downloads all necessary model files and weights to a local directory,
allowing the VLM server to run without internet access.

Usage:
    python3 download_vlm_models.py --output-dir ./vlm_models
    
Then run the server in offline mode:
    python3 standalone_vlm_server.py --model-cache-dir ./vlm_models
"""

import argparse
import logging
from pathlib import Path
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('ModelDownloader')


def download_models(output_dir: str, use_8bit: bool = True, use_4bit: bool = False):
    """
    Download Moondream2 model and tokenizer to local directory
    
    Args:
        output_dir: Directory to save model files
        use_8bit: Download with 8-bit quantization support
        use_4bit: Download with 4-bit quantization support
    """
    output_path = Path(output_dir).expanduser().resolve()
    output_path.mkdir(parents=True, exist_ok=True)
    
    logger.info("="*70)
    logger.info("Moondream2 VLM Model Downloader")
    logger.info("="*70)
    logger.info(f"Output directory: {output_path}")
    logger.info(f"8-bit quantization: {use_8bit}")
    logger.info(f"4-bit quantization: {use_4bit}")
    logger.info("")
    
    # Model configuration
    model_id = "vikhyatk/moondream2"
    revision = "2024-08-26"
    
    try:
        # Step 1: Download tokenizer
        logger.info("[1/2] Downloading tokenizer...")
        tokenizer = AutoTokenizer.from_pretrained(
            model_id,
            revision=revision,
            trust_remote_code=True,
            cache_dir=str(output_path)
        )
        
        # Save tokenizer to output directory
        tokenizer.save_pretrained(str(output_path))
        logger.info(f"✓ Tokenizer saved to {output_path}")
        
        # Step 2: Download model
        logger.info("[2/2] Downloading model (this may take several minutes)...")
        
        # Determine device
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        logger.info(f"Using device: {device}")
        
        if use_4bit:
            logger.info("Downloading with 4-bit quantization support...")
            model = AutoModelForCausalLM.from_pretrained(
                model_id,
                revision=revision,
                trust_remote_code=True,
                load_in_4bit=True,
                device_map={"": device},
                cache_dir=str(output_path)
            )
        elif use_8bit:
            logger.info("Downloading with 8-bit quantization support...")
            model = AutoModelForCausalLM.from_pretrained(
                model_id,
                revision=revision,
                trust_remote_code=True,
                load_in_8bit=True,
                device_map={"": device},
                cache_dir=str(output_path)
            )
        else:
            logger.info("Downloading full precision model...")
            model = AutoModelForCausalLM.from_pretrained(
                model_id,
                revision=revision,
                trust_remote_code=True,
                torch_dtype=torch.float16 if device == 'cuda' else torch.float32,
                cache_dir=str(output_path)
            )
        
        # Save model to output directory
        model.save_pretrained(str(output_path))
        logger.info(f"✓ Model saved to {output_path}")
        
        # Summary
        logger.info("")
        logger.info("="*70)
        logger.info("✓ Download Complete!")
        logger.info("="*70)
        logger.info(f"Models saved to: {output_path}")
        logger.info("")
        logger.info("To run the VLM server in offline mode, use:")
        logger.info(f"  python3 standalone_vlm_server.py --model-cache-dir {output_path}")
        logger.info("")
        
        # Show directory size
        total_size = sum(f.stat().st_size for f in output_path.rglob('*') if f.is_file())
        size_mb = total_size / (1024 * 1024)
        size_gb = size_mb / 1024
        
        if size_gb >= 1.0:
            logger.info(f"Total size: {size_gb:.2f} GB")
        else:
            logger.info(f"Total size: {size_mb:.2f} MB")
        
        logger.info("")
        logger.info("You can now copy this directory to your router/offline device")
        logger.info("="*70)
        
        return True
        
    except Exception as e:
        logger.error(f"Error downloading models: {e}")
        logger.error("Make sure you have:")
        logger.error("  1. Internet connection")
        logger.error("  2. Sufficient disk space (~2-4 GB)")
        logger.error("  3. Required packages: transformers, torch, accelerate, bitsandbytes")
        return False


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Download Moondream2 VLM models for offline use',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Download to default directory
  python3 download_vlm_models.py
  
  # Download to custom directory
  python3 download_vlm_models.py --output-dir /path/to/models
  
  # Download with 4-bit quantization
  python3 download_vlm_models.py --use-4bit
  
  # Then run server offline
  python3 standalone_vlm_server.py --model-cache-dir ./vlm_models
        """
    )
    
    parser.add_argument(
        '--output-dir',
        type=str,
        default='./vlm_models',
        help='Directory to save downloaded models (default: ./vlm_models)'
    )
    parser.add_argument(
        '--use-8bit',
        action='store_true',
        default=True,
        help='Download with 8-bit quantization support (default: True)'
    )
    parser.add_argument(
        '--use-4bit',
        action='store_true',
        help='Download with 4-bit quantization support (more aggressive)'
    )
    
    args = parser.parse_args()
    
    # Download models
    success = download_models(
        output_dir=args.output_dir,
        use_8bit=args.use_8bit and not args.use_4bit,
        use_4bit=args.use_4bit
    )
    
    exit(0 if success else 1)


if __name__ == '__main__':
    main()
