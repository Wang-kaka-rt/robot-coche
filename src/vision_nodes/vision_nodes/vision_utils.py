"""
Vision utilities for object detection and tracking.

This module provides functions for detecting and tracking objects in images,
primarily using OpenCV and color-based detection (HSV color space).

Suitable for resource-constrained environments like Raspberry Pi 4.
"""

import cv2
import numpy as np
from typing import Tuple, Optional, List


class BallDetector:
    """
    Detects red transport card (square) using HSV color space segmentation.
    
    HSV-based detection is computationally efficient and works well
    for color-based object detection on Raspberry Pi.
    """
    
    def __init__(self, min_area: int = 500, max_area: int = 100000):
        """
        Initialize the card detector.
        
        Args:
            min_area: Minimum contour area to be considered a card (pixels)
            max_area: Maximum contour area to be considered a card (pixels)
        """
        self.min_area = min_area
        self.max_area = max_area
        
        # HSV ranges for red color (transport card)
        # Red wraps around in HSV, so we need two ranges
        # Tightened saturation (≥140) to reject washed-out colors, skin tones, and white objects
        self.lower_red1 = np.array([0, 140, 100], dtype=np.uint8)
        self.upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
        self.lower_red2 = np.array([160, 140, 100], dtype=np.uint8)
        self.upper_red2 = np.array([180, 255, 255], dtype=np.uint8)
        
        # Store last mask for visualization
        self.last_mask = None
    
    def detect(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        """
        Detect red transport card (square) in frame.
        
        Args:
            frame: Input image (BGR)
            
        Returns:
            Tuple of (x, y, width, height) if card found, None otherwise
        """
        if frame is None or frame.size == 0:
            return None

        # Gaussian blur to reduce noise before color segmentation
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)

        # Convert to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Create mask for red color (two ranges needed for red)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply morphological operations to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Store mask for visualization
        self.last_mask = mask
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return None
        
        # Find largest rectangular contour within size constraints
        best_contour = None
        best_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area or area > self.max_area:
                continue
            
            # Check if it's roughly rectangular
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
            
            # Look for 4 corners (rectangle)
            if len(approx) >= 4 and area > best_area:
                best_contour = contour
                best_area = area
        
        if best_contour is None:
            return None
        
        # Get bounding rectangle
        x, y, w, h = cv2.boundingRect(best_contour)
        
        return (int(x), int(y), int(w), int(h))
    
    def draw_detection(self, frame: np.ndarray, detection: Optional[Tuple[int, int, int, int]]) -> np.ndarray:
        """
        Draw detection result on frame.
        
        Args:
            frame: Input image
            detection: Detection result (x, y, width, height) or None
            
        Returns:
            Frame with drawn detection
        """
        result = frame.copy()
        
        if detection is not None:
            x, y, w, h = detection
            # Draw rectangle
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Draw center point
            cx, cy = x + w // 2, y + h // 2
            cv2.circle(result, (cx, cy), 5, (0, 255, 0), -1)
            
            # Add label
            text = f"Red Card"
            cv2.putText(result, text, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return result
