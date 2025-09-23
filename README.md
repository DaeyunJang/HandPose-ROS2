# Welcome to HandPose-ROS
This system reconstructs hand joint coordinate frames from 21 hand landmarks. It includes both a standalone script version and a ROS 2 version.

## DEMO
<!-- https://github.com/user-attachments/assets/956f45fd-a752-4233-a894-6325916b71ae -->
https://github.com/user-attachments/assets/b2a561e7-b902-4bd1-a802-426902151187

# Handpose ROS Integration

## Summary
This package integrates **MediaPipe Hands** with **ROS 2 (Humble)**.  
It detects hand landmarks from a camera stream, scales them into canonical and world coordinates, builds per-joint coordinate frames, and publishes them into the ROS TF system for downstream robotics applications (e.g., teleoperation, grasp planning).

---

## Detail Logic

### 1. Landmark extraction
- MediaPipe Hands is used to get 21 hand landmarks per hand.
- Landmarks are published as normalized coordinates `(x, y, z)`.

#### 1-1. Canonical conversion
- Normalized coordinates are multiplied by image width/height to obtain **canonical pixel space coordinates**.

#### 1-2. World absolute scaling
- To approximate metric scale, the wrist–index MCP distance is assumed to be `0.08 m (80 mm)` (based on author’s hand).
- This scaling is applied uniformly, so that the hand size remains constant regardless of the screen position.
- The resulting transforms are suffixed with `world_abs`.

---

### 2. Coordinate frame generation (ROS TF)
- Each landmark is just a point, so local coordinate systems must be defined.

#### 2-1. Wrist frame
- Palm direction → **Y axis**  
- Middle finger MCP direction → **X axis**  
- Z axis is defined as the cross product, forming a right-handed system.

#### 2-2. Finger frames
- Each finger has joints: `MCP → PIP → DIP → TIP`.
- For each joint:
  - Project onto wrist XZ plane to determine **Y axis**.
  - Joint-to-joint vector defines **X axis**.
  - **Z axis** is set by cross product.

#### 2-3. Thumb special case
- For thumb MCP, an additional ~60° rotation about X axis is applied to better align with human thumb kinematics.

#### 2-4. TF broadcasting
- All transforms are published into ROS TF tree.

---

## System
- **ROS 2 Humble**
- **MediaPipe Hands**
- **Realsense2 Camera** (tested)
`note`
TF will be updated with respect to Hz of image callback (fps)

#### OS
Ubuntu 22.04.6 LTS

#### Software
- [ROS2 humble](https://docs.ros.org/en/humble/index.html)
- [Hand landmarks detection](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker)

#### System Requirements
- Ubuntu 22.04 (ROS2 humble)
- Realsense camera (D415)

---

## Installation
#### virtual environment
[Using Python Packages with ROS 2](https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Packages.html#installing-via-a-virtual-environment)
```bash
source {PATH_OF_YOUR_VIRTUAL_ENV}/bin/activate
```

#### python modules
```bash
  $ pip install -r requirements.txt
```

## Execute
#### Clone repositories
`HandPose` packages is required for run this project.
```bash
cd ~
git clone https://github.com/DaeyunJang/HandPose.git
```

#### Build and source
```bash
  # activate python environment
  source {PATH_OF_YOUR_VIRTUAL_ENV}/bin/activate

  # build dependent package
  cd ~/HandPose
  colcon build
  ./install/setup.bash # must be enter this line befor build 'Hand-Gripper-ROS2' package.
```

#### Launch
```bash
  ros2 launch handpose_ros handpose_launch.py
```

## Parameters
check `handpose_ros/config/params.yaml`