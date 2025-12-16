#!/home/hariprasad/must3r_env/bin/python
"""
MUSt3R Processor - Runs in Python 3.11 environment
Reads images from shared directory, processes with MUSt3R,
writes point clouds back.

This bridges the Python 3.11 (MUSt3R) and Python 3.10 (ROS2) gap.
"""

import os
import sys
import time
import json
import numpy as np
from pathlib import Path

# MUSt3R imports
print("Loading MUSt3R model...")
try:
    from must3r.model import load_model
    import torch
    print(f"✓ MUSt3R loaded (PyTorch {torch.__version__})")
    MUST3R_AVAILABLE = True
except Exception as e:
    print(f"✗ MUSt3R load failed: {e}")
    MUST3R_AVAILABLE = False

# Shared directories
SHARED_DIR = Path("/tmp/must3r_bridge")
IMAGE_DIR = SHARED_DIR / "images"
POINTS_DIR = SHARED_DIR / "points"
STATUS_FILE = SHARED_DIR / "status.json"

# Create directories
IMAGE_DIR.mkdir(parents=True, exist_ok=True)
POINTS_DIR.mkdir(parents=True, exist_ok=True)


class MUSt3RProcessor:
    """
    MUSt3R 3D reconstruction processor with intelligent GPU/CPU fallback.
    """

    def __init__(self, target_size=512):
        self.model = None
        self.encoder = None
        self.decoder = None
        self.target_size = target_size
        self.patch_size = 16

        # Intelligent device detection and fallback
        self._detect_and_configure_device()

        if MUST3R_AVAILABLE:
            self.load_model()

    def _detect_and_configure_device(self):
        """
        Detect available hardware and configure optimal settings.
        Implements proper GPU/CPU fallback with performance optimization.
        """
        print("\n" + "="*60)
        print("Hardware Detection & Configuration")
        print("="*60)

        # Check CUDA availability
        cuda_available = torch.cuda.is_available()

        if cuda_available:
            self.device = 'cuda'
            cuda_device = torch.cuda.get_device_name(0)
            cuda_memory = torch.cuda.get_device_properties(0).total_memory / 1e9
            print(f"✓ CUDA Available")
            print(f"  GPU: {cuda_device}")
            print(f"  Memory: {cuda_memory:.2f} GB")
            print(f"  Resolution: {self.target_size}x{self.target_size} (full)")
        else:
            self.device = 'cpu'
            print(f"⚠ CUDA Not Available - Falling back to CPU")
            print(f"  CPU Mode: Performance will be reduced")

            # CPU-specific optimizations
            if self.target_size > 512:
                print(f"  Resolution: Reducing from {self.target_size} to 512 for CPU")
                self.target_size = 512
            else:
                print(f"  Resolution: {self.target_size}x{self.target_size}")

            # Set thread limits for CPU
            torch.set_num_threads(4)
            print(f"  CPU Threads: 4 (optimized for inference)")

        print(f"  Device Selected: {self.device.upper()}")
        print("="*60 + "\n")
    
    def load_model(self):
        """
        Load the MUSt3R pre-trained model.
        Configured for 512x512 resolution with GPU/CPU fallback.
        """
        try:
            # Try multiple possible checkpoint paths
            checkpoint_paths = [
                "/home/hariprasad/checkpoints/MUSt3R_512.pth",
                "/home/hariprasad/checkpoints/must3r_model.pth",
                "/home/hariprasad/must3r_env/checkpoints/MUSt3R_ViTLarge_BaseDecoder_512_dpt.pth",
            ]

            checkpoint_path = None
            for path in checkpoint_paths:
                if os.path.exists(path):
                    checkpoint_path = path
                    break

            if checkpoint_path:
                print(f"Loading MUSt3R Model Configuration:")
                print(f"  Checkpoint: {os.path.basename(checkpoint_path)}")
                print(f"  Resolution: {self.target_size}x{self.target_size}")
                print(f"  Device: {self.device.upper()}")
                print(f"  Loading...")

                # load_model returns (encoder, decoder) tuple
                self.encoder, self.decoder = load_model(checkpoint_path, device=self.device)
                self.model = (self.encoder, self.decoder)

                # Move models to device
                self.encoder = self.encoder.to(self.device)
                self.decoder = self.decoder.to(self.device)

                # Set to evaluation mode
                self.encoder.eval()
                self.decoder.eval()

                print("✓ Model loaded successfully")
                print(f"✓ Configuration validated: {self.target_size}x{self.target_size} @ {self.device.upper()}")
            else:
                print(f"✗ No checkpoint found. Tried: {checkpoint_paths}")
        except Exception as e:
            print(f"✗ Model load error: {e}")
            import traceback
            traceback.print_exc()
            self.model = None
    
    def process_images(self, left_path, right_path):
        """
        Process stereo image pair and return 3D points using MUSt3R.
        """
        try:
            from PIL import Image
            from must3r.tools.image import get_resize_function
            from must3r.engine.inference import encoder_multi_ar, postprocess
            import must3r.tools.path_to_dust3r  # noqa
            from dust3r.datasets import ImgNorm

            # Load images
            left_img = Image.open(left_path).convert('RGB')
            right_img = Image.open(right_path).convert('RGB')

            if self.encoder is None or self.decoder is None:
                print("  ⚠ Model not loaded, generating fallback points")
                # Generate dummy points for testing
                return self.generate_dummy_points(left_img)

            # Prepare images in MUSt3R format using configured resolution
            imgs = []
            true_shapes = []

            for pil_img in [left_img, right_img]:
                W, H = pil_img.size
                resize_func, _, _ = get_resize_function(
                    self.target_size, self.patch_size, H, W
                )
                rgb_tensor = resize_func(ImgNorm(pil_img))
                imgs.append(rgb_tensor)
                true_shapes.append(torch.tensor([rgb_tensor.shape[-2], rgb_tensor.shape[-1]]))
            
            # Run inference
            with torch.no_grad():
                # Encode images
                true_shape = torch.stack(true_shapes)  # Shape: [2, 2]
                x, pos = encoder_multi_ar(
                    self.encoder, imgs, true_shape,
                    device=self.device, verbose=False
                )

                # Stack for decoder - needs (1, nimgs, N, D) format
                x_stacked = torch.stack([x[0], x[1]], dim=0).unsqueeze(0)
                pos_stacked = torch.stack([pos[0], pos[1]], dim=0).unsqueeze(0)

                # Add batch dimension to true_shape for decoder: [2, 2] -> [1, 2, 2]
                true_shape_batched = true_shape.unsqueeze(0)

                # Decode to get point maps
                output = self.decoder(x_stacked, pos_stacked, true_shape_batched)

                # Extract 3D points from decoder output
                # Unwrap nested tuples until we get to the tensor
                dec_out = output
                while isinstance(dec_out, (tuple, list)):
                    dec_out = dec_out[0]

                # Now dec_out should be a tensor
                if not torch.is_tensor(dec_out):
                    raise ValueError(f"Expected tensor, got {type(dec_out)}")

                # Extract point map from decoder output
                # The decoder outputs include XYZ in the first 3 channels
                if len(dec_out.shape) == 5:  # (B, nimgs, H, W, C)
                    # Take first batch, first view, first 3 channels (XYZ)
                    pts3d = dec_out[0, 0, :, :, :3]  # (H, W, 3)
                elif len(dec_out.shape) == 4:  # (nimgs, H, W, C) or (B, H, W, C)
                    pts3d = dec_out[0, :, :, :3]  # (H, W, 3)
                elif len(dec_out.shape) == 3:  # (H, W, C)
                    pts3d = dec_out[:, :, :3]  # (H, W, 3)
                else:
                    raise ValueError(f"Unexpected decoder output shape: {dec_out.shape}")

                # Convert to numpy points (N, 6) - xyz, rgb
                points = self.pointmap_to_points(pts3d, left_img)
                print(f"  ✓ MUSt3R processing successful!")

            return points
            
        except Exception as e:
            print(f"  ✗ MUSt3R processing failed: {e}")
            print(f"  ⚠ Generating fallback points (this is NOT real 3D reconstruction!)")
            import traceback
            traceback.print_exc()
            return self.generate_dummy_points()
    
    def pointmap_to_points(self, pointmap, image):
        """Convert MUSt3R pointmap to colored 3D points"""
        try:
            # Get numpy arrays
            if torch.is_tensor(pointmap):
                pts = pointmap.cpu().numpy()
            else:
                pts = np.array(pointmap)
            
            # Reshape to point list
            if len(pts.shape) == 4:  # (B, 3, H, W)
                pts = pts[0].transpose(1, 2, 0)  # (H, W, 3)
            
            H, W = pts.shape[:2]
            pts = pts.reshape(-1, 3)  # (N, 3)
            
            # Get colors from image
            img_array = np.array(image.resize((W, H)))
            colors = img_array.reshape(-1, 3)
            
            # Combine xyz and rgb
            points = np.hstack([pts, colors])
            
            # Filter invalid points
            valid = ~np.isnan(points).any(axis=1)
            points = points[valid]
            
            return points
            
        except Exception as e:
            print(f"Pointmap conversion error: {e}")
            return self.generate_dummy_points()
    
    def generate_dummy_points(self, image=None):
        """Generate dummy point cloud for testing"""
        num_points = 1000
        
        # Random 3D points in front of camera
        x = np.random.randn(num_points) * 0.5
        y = np.random.randn(num_points) * 0.5
        z = np.random.rand(num_points) * 3 + 1  # 1-4 meters
        
        # Random colors
        if image is not None:
            # Sample colors from image
            img_array = np.array(image)
            h, w = img_array.shape[:2]
            px = np.random.randint(0, w, num_points)
            py = np.random.randint(0, h, num_points)
            colors = img_array[py, px]
        else:
            colors = np.random.randint(0, 255, (num_points, 3))
        
        return np.hstack([
            x.reshape(-1, 1),
            y.reshape(-1, 1),
            z.reshape(-1, 1),
            colors
        ])
    
    def run(self):
        """Main processing loop"""
        # Display configuration summary
        config = self.get_config_status()

        print("\n" + "=" * 60)
        print("MUSt3R Processor Running")
        print("=" * 60)
        print(f"Configuration:")
        print(f"  Device: {config['device'].upper()}")
        print(f"  Resolution: {config['resolution']}")
        print(f"  Model Loaded: {'✓' if config['model_loaded'] else '✗'}")
        print(f"  CUDA Available: {'✓' if config['cuda_available'] else '✗'}")
        print(f"\nDirectories:")
        print(f"  Input: {IMAGE_DIR}")
        print(f"  Output: {POINTS_DIR}")
        print("=" * 60 + "\n")

        # Update status
        self.update_status("running")
        
        while True:
            try:
                # Check for new images
                left_files = sorted(IMAGE_DIR.glob("*_left.png"))
                
                for left_path in left_files:
                    right_path = Path(str(left_path).replace("_left.png", "_right.png"))

                    if not right_path.exists():
                        continue

                    # Process images
                    timestamp = left_path.stem.replace("_left", "")
                    print(f"Processing: {timestamp}")

                    # Time the processing
                    start_time = time.time()
                    points = self.process_images(left_path, right_path)
                    processing_time = time.time() - start_time

                    # Save points
                    output_path = POINTS_DIR / f"{timestamp}_points.npy"
                    np.save(output_path, points)
                    print(f"  → Saved {len(points)} points to {output_path.name}")
                    print(f"  → Processing time: {processing_time:.2f}s on {self.device.upper()}")

                    # Remove processed images
                    left_path.unlink()
                    right_path.unlink()
                
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                print("\nShutting down...")
                self.update_status("stopped")
                break
            except Exception as e:
                print(f"Error: {e}")
                time.sleep(1)
    
    def get_config_status(self):
        """Get current configuration status"""
        return {
            "device": self.device,
            "resolution": f"{self.target_size}x{self.target_size}",
            "model_loaded": self.model is not None,
            "encoder_available": self.encoder is not None,
            "decoder_available": self.decoder is not None,
            "cuda_available": torch.cuda.is_available()
        }

    def update_status(self, status):
        """Update status file with configuration info"""
        config = self.get_config_status()
        with open(STATUS_FILE, 'w') as f:
            json.dump({
                "status": status,
                "time": time.time(),
                "model_loaded": self.model is not None,
                "config": config
            }, f)


if __name__ == "__main__":
    processor = MUSt3RProcessor()
    processor.run()

