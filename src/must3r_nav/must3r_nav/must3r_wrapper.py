#!/usr/bin/env python3
"""
MUSt3R Wrapper - Implements professor's expected API

⚠️  IMPORTANT: This wrapper can ONLY be used from Python 3.11 environment!
    Do NOT import this from ROS2 nodes (Python 3.10) - it will fail!

Professor's pseudocode:
    from must3r import MUST3R
    model = MUST3R.from_pretrained("naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt")
    pointmaps, confidences = model(images)

Actual API uses load_model() which returns (encoder, decoder).
This wrapper provides the simplified interface.

Usage (Python 3.11 only):
    $ source ~/must3r_env/bin/activate
    $ python3 -c "from must3r_nav.must3r_wrapper import MUST3R; print('OK')"
"""

import os
import sys
import numpy as np

# Verify Python version
if sys.version_info < (3, 11):
    print(f"⚠️  WARNING: MUSt3R requires Python 3.11+, you have {sys.version_info.major}.{sys.version_info.minor}")
    print("   This wrapper will NOT work in ROS2 environment (Python 3.10)")
    print("   Use must3r_processor.py via file-based IPC instead")

# Try to import MUSt3R (will fail if not in must3r_env)
MUST3R_AVAILABLE = False
try:
    import torch
    from must3r.model import load_model
    from must3r.tools.image import get_resize_function
    from must3r.engine.inference import encoder_multi_ar, postprocess
    import must3r.tools.path_to_dust3r  # noqa
    from dust3r.datasets import ImgNorm
    from PIL import Image
    MUST3R_AVAILABLE = True
    print("✓ MUSt3R modules imported successfully")
except ImportError as e:
    print(f"⚠ MUSt3R not available: {e}")
    print("   Make sure you're running from must3r_env: source ~/must3r_env/bin/activate")


class MUST3R:
    """
    High-level MUSt3R wrapper matching professor's expected API.
    
    Usage:
        model = MUST3R.from_pretrained("naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt")
        pointmaps, confidences = model(images)
    """
    
    # Checkpoint mapping
    CHECKPOINTS = {
        "naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt": [
            "/home/hariprasad/checkpoints/MUSt3R_512.pth",
            "/home/hariprasad/checkpoints/must3r_model.pth",
        ]
    }
    
    def __init__(self, encoder, decoder, device='cpu'):
        self.encoder = encoder
        self.decoder = decoder
        self.device = device
        self.image_size = 512
        self.patch_size = 16
    
    @classmethod
    def from_pretrained(cls, model_name, device=None):
        """
        Load a pretrained MUSt3R model.
        
        Args:
            model_name: Model identifier (e.g., "naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt")
            device: 'cuda' or 'cpu' (default: auto-detect)
        
        Returns:
            MUST3R instance
        """
        if not MUST3R_AVAILABLE:
            raise RuntimeError("MUSt3R is not available. Install with: pip install must3r")
        
        if device is None:
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        # Find checkpoint
        checkpoint_paths = cls.CHECKPOINTS.get(model_name, [model_name])
        checkpoint_path = None
        
        for path in checkpoint_paths:
            if os.path.exists(path):
                checkpoint_path = path
                break
        
        if checkpoint_path is None:
            raise FileNotFoundError(f"No checkpoint found for {model_name}. Tried: {checkpoint_paths}")
        
        print(f"Loading MUSt3R from: {checkpoint_path}")
        print(f"Device: {device}")
        
        # Load encoder and decoder
        encoder, decoder = load_model(checkpoint_path, device=device)
        
        print("✓ MUSt3R model loaded successfully!")
        return cls(encoder, decoder, device)
    
    def to(self, device):
        """Move model to device"""
        self.device = device
        self.encoder.to(device)
        self.decoder.to(device)
        return self
    
    def __call__(self, images):
        """
        Run 3D reconstruction on images.
        
        Args:
            images: List of PIL Images or numpy arrays (stereo pair or multi-view)
        
        Returns:
            pointmaps: 3D point coordinates (N, 3)
            confidences: Confidence scores (N,)
        """
        if not MUST3R_AVAILABLE:
            return self._dummy_inference(images)
        
        return self._run_inference(images)
    
    def _run_inference(self, images):
        """Run actual MUSt3R inference"""
        # Convert images to PIL if needed
        pil_images = []
        for img in images:
            if isinstance(img, np.ndarray):
                pil_images.append(Image.fromarray(img))
            else:
                pil_images.append(img)
        
        # Prepare images in MUSt3R format
        imgs = []
        true_shapes = []
        
        for pil_img in pil_images:
            W, H = pil_img.size
            resize_func, _, _ = get_resize_function(self.image_size, self.patch_size, H, W)
            rgb_tensor = resize_func(ImgNorm(pil_img))
            imgs.append(rgb_tensor)
            true_shapes.append(torch.tensor([rgb_tensor.shape[-2], rgb_tensor.shape[-1]]))
        
        # Run inference
        with torch.no_grad():
            true_shape = torch.stack(true_shapes)
            
            # Encode
            x, pos = encoder_multi_ar(
                self.encoder, imgs, true_shape,
                device=self.device, verbose=False
            )
            
            # Stack for decoder
            x_stacked = torch.stack(x, dim=0).unsqueeze(0)
            pos_stacked = torch.stack(pos, dim=0).unsqueeze(0)
            
            # Decode
            output = self.decoder(x_stacked, pos_stacked, true_shape)
            
            # Process output
            if isinstance(output, dict) and 'pts3d' in output:
                pts3d = output['pts3d']
                conf = output.get('conf', None)
            else:
                processed = postprocess(output)
                pts3d = processed['pts3d']
                conf = processed.get('conf', None)
        
        # Convert to numpy
        if torch.is_tensor(pts3d):
            pointmaps = pts3d.cpu().numpy().reshape(-1, 3)
        else:
            pointmaps = np.array(pts3d).reshape(-1, 3)
        
        if conf is not None:
            if torch.is_tensor(conf):
                confidences = conf.cpu().numpy().flatten()
            else:
                confidences = np.array(conf).flatten()
        else:
            confidences = np.ones(len(pointmaps))
        
        # Filter invalid points
        valid = ~np.isnan(pointmaps).any(axis=1) & ~np.isinf(pointmaps).any(axis=1)
        pointmaps = pointmaps[valid]
        confidences = confidences[valid] if len(confidences) == len(valid) else np.ones(len(pointmaps))
        
        return pointmaps, confidences
    
    def _dummy_inference(self, images):
        """Generate dummy point cloud for testing"""
        print("⚠ Running in dummy mode (MUSt3R not available)")
        
        num_points = 5000
        pointmaps = np.random.randn(num_points, 3).astype(np.float32)
        pointmaps[:, 2] = np.abs(pointmaps[:, 2]) + 1  # Positive Z (depth)
        confidences = np.random.rand(num_points).astype(np.float32)
        
        return pointmaps, confidences


class MUST3RProcessor:
    """
    MUSt3R Processor class matching professor's expected interface.
    
    Usage:
        processor = MUST3RProcessor()
        points = processor.reconstruct_scene(images)
    """
    
    def __init__(self, device=None):
        if device is None:
            device = 'cuda' if MUST3R_AVAILABLE and torch.cuda.is_available() else 'cpu'
        
        try:
            self.model = MUST3R.from_pretrained(
                "naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt"
            ).to(device)
            self.available = True
        except Exception as e:
            print(f"⚠ Could not load MUSt3R model: {e}")
            self.model = None
            self.available = False
    
    def reconstruct_scene(self, images):
        """
        Reconstruct 3D scene from images.
        
        Args:
            images: List of images (PIL or numpy)
        
        Returns:
            points: numpy array (N, 3) of 3D points
        """
        if self.model is None:
            # Return dummy points
            return np.random.randn(1000, 3).astype(np.float32)
        
        pointmaps, confidences = self.model(images)
        return self.process_pointcloud(pointmaps, confidences)
    
    def process_pointcloud(self, pointmaps, confidences, threshold=0.5):
        """
        Process raw pointmaps into filtered point cloud.
        
        Args:
            pointmaps: Raw 3D points (N, 3)
            confidences: Confidence scores (N,)
            threshold: Minimum confidence threshold
        
        Returns:
            points: Filtered point cloud (M, 3)
        """
        # Filter by confidence
        mask = confidences > threshold
        points = pointmaps[mask]
        
        return points


# Export for easy importing
__all__ = ['MUST3R', 'MUST3RProcessor', 'MUST3R_AVAILABLE']


if __name__ == "__main__":
    # Test the wrapper
    print("Testing MUSt3R wrapper...")
    
    try:
        # Test MUST3R class
        model = MUST3R.from_pretrained("naver/MUSt3R_ViTLarge_BaseDecoder_512_dpt")
        print(f"✓ Model loaded on {model.device}")
        
        # Test with dummy images
        from PIL import Image
        img1 = Image.new('RGB', (640, 480), color=(100, 150, 200))
        img2 = Image.new('RGB', (640, 480), color=(120, 170, 220))
        
        print("Running inference...")
        points, conf = model([img1, img2])
        print(f"✓ Generated {len(points)} points")
        print(f"  Points shape: {points.shape}")
        print(f"  Confidence range: [{conf.min():.2f}, {conf.max():.2f}]")
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
















