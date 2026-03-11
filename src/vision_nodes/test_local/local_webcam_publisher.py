#!/usr/bin/env python3
"""
Local webcam test script for red card detection + follower decision preview.

This script can be run on a development machine to test card detection
before deploying to the robot. It does NOT require ROS.

It also shows what movement command the robot would choose in real time.

Usage:
    python3 local_webcam_publisher.py [--camera_index 0]

Controls:
    'q' or ESC: Quit
    's': Save current frame with detection
"""

import cv2
import sys
import argparse
from pathlib import Path

# Add parent directory to path to import vision_nodes
sys.path.insert(0, str(Path(__file__).parent.parent))

from vision_nodes.vision_utils import BallDetector


class LocalWebcamTester:
    """Webcam red-card detection tester with follower decision simulation."""
    
    def __init__(self, camera_index: int = 0, width: int = 640, height: int = 480):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        
        # Open camera
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_index}")
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # Initialize detector
        self.detector = BallDetector(min_area=500, max_area=100000)
        
        # State
        self.frame_count = 0
        self.detection_count = 0
        
        print(f"Camera opened: {self.width}x{self.height}")
        print("Detecting: RED TRANSPORT CARD (square)")
        print("Controls: q/ESC=quit, s=save")
        print("Showing: mask + detection + robot decision overlay")

        # Follower logic parameters (same idea as follower node)
        self.frame_center_x = self.width / 2.0
        self.min_card_size = 20
        self.max_card_size = 150
        self.target_card_size = 80
        self.center_tol = 50
        self.linear_spd = 0.1
        self.angular_spd = 0.5
    
    def run(self):
        """Main loop."""
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to read frame")
                break
            
            self.frame_count += 1
            
            # Detect card
            detection = self.detector.detect(frame)
            
            if detection:
                self.detection_count += 1

            # Simulate follower logic
            movement = self._simulate_follower(detection)
            
            # Get the mask (what computer sees)
            if self.detector.last_mask is not None:
                # Convert mask to 3-channel for visualization
                mask_colored = cv2.cvtColor(self.detector.last_mask, cv2.COLOR_GRAY2BGR)
                
                # Draw detection on mask
                if detection:
                    x, y, w, h = detection
                    cv2.rectangle(mask_colored, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cx, cy = x + w // 2, y + h // 2
                    cv2.circle(mask_colored, (cx, cy), 5, (0, 255, 0), -1)
                    
                    # Add info
                    text = f"RED CARD DETECTED"
                    cv2.putText(mask_colored, text, (x, y - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                display_frame = mask_colored
            else:
                display_frame = frame
            
            # Add stats overlay
            info_text = f"Frame: {self.frame_count} | Detections: {self.detection_count}"
            cv2.putText(display_frame, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if detection:
                x, y, w, h = detection
                det_text = f"Card at ({x}, {y}), size={w}x{h}"
                cv2.putText(display_frame, det_text, (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, "SEARCHING...", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Draw movement decisions
            self._draw_movement_info(display_frame, movement)
            
            cv2.imshow('Red Card Detection - Computer Vision', display_frame)
            
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # ESC
                break
            elif key == ord('s'):
                filename = f"card_detection_{self.frame_count}.jpg"
                cv2.imwrite(filename, display_frame)
                print(f"Saved: {filename}")
        
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        print(f"\nStats: {self.frame_count} frames processed, "
              f"{self.detection_count} detections ({100*self.detection_count/max(1,self.frame_count):.1f}%)")
        self.cap.release()
        cv2.destroyAllWindows()

    def _simulate_follower(self, detection):
        """Simulate what movement the robot would make."""
        if detection is None:
            return {
                'linear': 0.0,
                'angular': 0.0,
                'state': 'STOPPED',
                'reason': 'No card detected',
                'color': (0, 0, 255),
            }

        x, y, w, h = detection
        card_size = (w + h) / 2.0
        card_center_x = x + w // 2

        if card_size < self.min_card_size:
            return {
                'linear': 0.0,
                'angular': 0.0,
                'state': 'NOISE FILTER',
                'reason': f'Card too small ({card_size:.0f} < {self.min_card_size})',
                'color': (0, 165, 255),
            }

        if card_size > self.max_card_size:
            return {
                'linear': -0.05,
                'angular': 0.0,
                'state': 'BACKING UP',
                'reason': f'Card too close ({card_size:.0f} > {self.max_card_size})',
                'color': (0, 0, 255),
            }

        error_x = self.frame_center_x - card_center_x
        angular_vel = (error_x / self.frame_center_x) * self.angular_spd

        size_error = card_size - self.target_card_size
        if size_error > 15:
            linear_vel = self.linear_spd * 0.3
            distance_state = 'TOO CLOSE'
        elif size_error > 5:
            linear_vel = self.linear_spd * 0.6
            distance_state = 'SLOWING'
        else:
            linear_vel = self.linear_spd
            distance_state = 'GOOD DISTANCE'

        if abs(error_x) < self.center_tol:
            linear_vel *= 0.8

        if error_x > 50:
            horizontal = 'Turn LEFT'
        elif error_x < -50:
            horizontal = 'Turn RIGHT'
        else:
            horizontal = 'CENTERED'

        return {
            'linear': linear_vel,
            'angular': angular_vel,
            'state': 'FOLLOWING',
            'distance': distance_state,
            'horizontal': horizontal,
            'reason': f'{distance_state} + {horizontal}',
            'color': (0, 255, 0),
        }

    def _draw_movement_info(self, frame, movement):
        """Draw movement decision info on frame."""
        panel_x = 10
        panel_y = 120
        line_height = 25

        state_color = movement['color']
        state_text = f"ROBOT STATE: {movement['state']}"
        cv2.putText(frame, state_text, (panel_x, panel_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, state_color, 2)

        panel_y += line_height

        reason_text = f"Reason: {movement['reason']}"
        cv2.putText(frame, reason_text, (panel_x, panel_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        panel_y += line_height + 5

        forward_text = f"Forward: {movement['linear']:+.2f} m/s"
        cv2.putText(frame, forward_text, (panel_x, panel_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 2)

        panel_y += line_height

        angular_text = f"Rotate: {movement['angular']:+.3f} rad/s"
        cv2.putText(frame, angular_text, (panel_x, panel_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 200, 255), 2)


def main():
    parser = argparse.ArgumentParser(
        description='Test red card detection with local webcam')
    parser.add_argument('--camera_index', type=int, default=0,
                       help='Camera device index (default: 0)')
    
    args = parser.parse_args()
    
    try:
        tester = LocalWebcamTester(camera_index=args.camera_index)
        tester.run()
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
