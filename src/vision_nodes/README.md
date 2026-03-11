# Vision Nodes

Red transport card detection for the Freenove 4WD Smart Car robot.

## Overview

Detects red transport card (square) using HSV color segmentation:

- **Ball Detector Node**: Detects red card, publishes position
- **Ball Follower Node**: Makes robot follow detected card (autonomous)

## Local Testing

Test on your Mac before deploying to robot:

```bash
python3 test_local/local_webcam_publisher.py
```

Press **q** or **ESC** to quit, **s** to save frame.

## Robot Deployment

```bash
# Build
colcon build --packages-select vision_nodes

# Run with camera
ros2 launch vision_nodes ball_detection.launch.py camera_topic:=/image_raw/compressed
```

## Technical Details

- **Method**: HSV color segmentation (efficient, no GPU needed)
- **Language**: Python 3
- **Framework**: ROS2 (Jazzy Jalisco)
- **Library**: OpenCV 4.5+
- **Target**: Raspberry Pi 4, Ubuntu 24.04 LTS

## Adjusting Red Detection

Edit `vision_nodes/vision_utils.py` lines 35-38 to change HSV ranges:

```python
self.lower_red1 = np.array([0, 140, 100], dtype=np.uint8)    # Adjust these
self.upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
```

- **Hue** (0-180): Color type
- **Saturation** (100-255): Color purity (higher = more vivid)
- **Value** (100-255): Brightness
