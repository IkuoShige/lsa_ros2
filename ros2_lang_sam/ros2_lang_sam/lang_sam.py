"""
LangSAM module for ROS 2 wrapper.
This module provides a wrapper for the LangSAM class from lang-segment-anything.
"""

import numpy as np
from PIL import Image
import sys
import os

# Add the lang-segment-anything path to Python path to allow direct imports
if '/opt/lang-segment-anything' not in sys.path:
    sys.path.append('/opt/lang-segment-anything')

try:
    # Try to import directly from the source directory
    from lang_sam.lang_sam import LangSAM as OriginalLangSAM
except ImportError:
    # If that fails, try to use the installed package (for development environments)
    try:
        from lang_sam import LangSAM as OriginalLangSAM
    except ImportError:
        raise ImportError(
            "Could not import LangSAM. Make sure lang-segment-anything is either "
            "installed or available at /opt/lang-segment-anything"
        )


class LangSAM:
    """
    ROS 2 wrapper for the LangSAM class from lang-segment-anything.
    """

    def __init__(
        self, 
        sam_type: str = "sam2.1_hiera_small", 
        ckpt_path: str = None, 
        device: str = "cuda"
    ) -> None:
        """
        Initialize the LangSAM instance.
        
        Args:
            sam_type: Type of SAM model to use (default: "sam2.1_hiera_small")
            ckpt_path: Path to checkpoint (default: None)
            device: Device to run inference on (default: "cuda")
        """
        self.model = OriginalLangSAM(
            sam_type=sam_type,
            ckpt_path=ckpt_path,
            device=device
        )
        self.sam_type = sam_type
        self.device = device

    def segment(
        self, 
        image: np.ndarray, 
        text_prompt: str, 
        box_threshold: float = 0.3, 
        text_threshold: float = 0.25
    ) -> dict:
        """
        Segment objects in the image based on the text prompt.
        
        Args:
            image: Input image as a numpy array (RGB format)
            text_prompt: Text prompt describing the object to segment
            box_threshold: Threshold for box predictions (default: 0.3)
            text_threshold: Threshold for text predictions (default: 0.25)
            
        Returns:
            Dictionary containing masks, boxes, scores and other outputs
        """
        # Convert numpy array to PIL Image
        pil_image = Image.fromarray(image.astype('uint8'), 'RGB')
        
        # Run segmentation
        results = self.model.predict(
            [pil_image], 
            [text_prompt], 
            box_threshold=box_threshold, 
            text_threshold=text_threshold
        )
        
        # The output of model.predict is a list of dictionaries, one per image
        # Since we only process one image, we return the first element
        return results[0]
