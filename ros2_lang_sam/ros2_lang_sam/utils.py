"""
Utility functions for ros2_lang_sam package.
"""

import os
import shutil
import tempfile
import urllib.request
from pathlib import Path


class ModelDownloader:
    """
    Utility class for downloading model files.
    """
    
    def __init__(self, cache_dir: str = None):
        """
        Initialize the model downloader.
        
        Args:
            cache_dir: Directory to cache downloaded models (default: ~/.cache/ros2_lang_sam)
        """
        if cache_dir is None:
            cache_dir = os.path.expanduser("~/.cache/ros2_lang_sam")
        
        self.cache_dir = Path(cache_dir)
        os.makedirs(self.cache_dir, exist_ok=True)
    
    def download_file(self, url: str, filename: str = None) -> str:
        """
        Download a file from a URL to the cache directory.
        
        Args:
            url: URL to download
            filename: Name to save the file as (default: basename of URL)
            
        Returns:
            Path to the downloaded file
        """
        if filename is None:
            filename = os.path.basename(url)
        
        output_path = self.cache_dir / filename
        
        if output_path.exists():
            return str(output_path)
        
        # Download to a temporary file first
        with tempfile.NamedTemporaryFile(delete=False) as tmp_file:
            with urllib.request.urlopen(url) as response:
                shutil.copyfileobj(response, tmp_file)
        
        # Move the temporary file to the final location
        shutil.move(tmp_file.name, output_path)
        
        return str(output_path)


def visualize_masks(image, masks, boxes=None, scores=None, alpha=0.5):
    """
    Visualize masks on the input image.
    
    Args:
        image: Input image as a numpy array
        masks: List of binary masks
        boxes: List of bounding boxes (optional)
        scores: List of scores (optional)
        alpha: Transparency of the masks (default: 0.5)
        
    Returns:
        Visualization image with masks and bounding boxes
    """
    import cv2
    import numpy as np
    
    # Create a copy of the input image
    vis_image = image.copy()
    
    # Draw masks with random colors
    for i, mask in enumerate(masks):
        # Generate random color
        color = np.random.randint(0, 255, 3).tolist()
        
        # Create colored mask
        colored_mask = np.zeros_like(vis_image)
        colored_mask[mask > 0] = color
        
        # Blend with original image
        vis_image = cv2.addWeighted(
            vis_image, 1.0, colored_mask, alpha, 0.0
        )
    
    # Draw bounding boxes if provided
    if boxes is not None:
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = [int(coord) for coord in box]
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw score if provided
            if scores is not None and i < len(scores):
                cv2.putText(
                    vis_image,
                    f"Score: {scores[i]:.2f}",
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
    
    return vis_image
