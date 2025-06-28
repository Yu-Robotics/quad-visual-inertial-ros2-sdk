# Quad-Camera Visual-Inertial Module ROS2 SDK

This repository provides the official ROS2 SDK for the [Quad-Camera Visual-Inertial Module](https://github.com/Yu-Robotics/quad-visual-inertial-hardware). The SDK is designed to enable seamless integration and operation of the hardware in ROS2-based projects, providing camera and IMU data streams with synchronized, high-performance interfaces.

## Overview

This SDK is tailored for the Quad-Camera Visual-Inertial Module, as described in the [hardware documentation](https://github.com/Yu-Robotics/quad-visual-inertial-hardware/files/README.md). The hardware features four hardware-synchronized OV7251 (or OV7750) cameras and a BMI088 IMU, managed by a Rockchip RV1106 processor and an OV680 ISP bridge, providing real-time visual-inertial data for robotics and computer vision applications.

## Synchronization Accuracy

- **Between the four cameras:** < 50µs  
- **Between IMU and camera sensors:** < 1ms

## Build Instructions

1. **Extract the SDK:**
   Clone SDK package.

2. **Navigate to the Source Directory:**
   ```bash
   cd <unpacked_sdk_directory>
   ```

3. **Build with colcon:**
   ```bash
   colcon build
   ```

## Launch Instructions

1. **Connect the Module:**
   Ensure the Quad-Camera Visual-Inertial Module is properly connected to your PC via USB.

2. **Launch the ROS2 Node:**
   ```bash
   ros2 launch libuvc_cam quad_cam_inertial_launch.py
   ```

## Verifying a Successful Launch

After starting the launch file, verify the following topics are available using:

```bash
ros2 topic list
```

You should see these topics:

- `/cam/compressed`
- `/cam0/image_raw`
- `/cam1/image_raw`
- `/cam2/image_raw`
- `/cam3/image_raw`
- `/imu0`
- `/parameter_events`
- `/rosout`

## Topic Descriptions

- **`/cam/compressed`**: Raw, unsegmented and undecoded image stream from the module.
- **`/camX/image_raw`** (`/cam0/image_raw` ... `/cam3/image_raw`): Segmented and decoded image streams for each camera.
- **`/imu0`**: IMU data topic.

## Acknowledgements

- The `libuvc` component included in this SDK is a forked and modified version from the [@saki4510t/UVCCamera](https://github.com/saki4510t/UVCCamera) project. Special thanks to the original author and contributors for their foundational work.

- This SDK is inspired by and built upon the contributions of the open-source community and is intended to support further research and development in robotics and computer vision.

---
For hardware details and further documentation, please refer to the [hardware repository](https://github.com/Yu-Robotics/quad-visual-inertial-hardware).
