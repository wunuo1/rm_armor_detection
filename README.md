English| [简体中文](./README_cn.md)

Getting Started with rm armor detection demo
=======


# Introduction

The rm armor detection package is an example of using the Dnn Node package. By inheriting the DnnNode virtual base class, it utilizes the YOLOv8s-pose model and image data to perform algorithm inference on a BPU processor.

The image data comes from subscribed image data messages, supporting image data (nv12 format) published using DaHeng cameras(MER-139-210U3C). After the inference is completed, a custom algorithm output parsing method is used to parse the algorithm's output tensor. Once parsed, the intelligent results are published, and real-time rendering effects can be viewed through a web interface.

# Development Environment

- Programming Language: C/C++
- Development Platform: X5
- System Version: Ubuntu 22.04

# Compilation

1. Confirm Compilation Environment

- X5 Ubuntu system is installed on the board.

- The current compilation terminal has set the TROS environment variable: `source /opt/tros/humble/setup.bash`.

- ROS2 compile tool colcon is installed.

2. Compilation

- Compilation command: `colcon build --packages-select rm_armor_detection`

# Usage Guide

##  X5 Ubuntu

```shell
#Set CPU overclocking
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_boost_frequencies
echo 1 >/sys/devices/system/cpu/cpufreq/boost
echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

#Run DaHeng camera
ros2 run rm_camera_driver rm_camera_driver_node

#Run detection node
#If you need to start the WEB display, set the environment variable by 'export WEB_SHOW=TRUE'. Please note that after starting the display, running encoding and decoding functions may cause a decrease in frame rate
ros2 launch rm_armor_detection rm_armor_detection.launch.py
```

## Notes
1. Before starting the function, please set the CPU overclocking to ensure a more stable frame rate
2. The exposure time of the camera will affect the frame rate. Please set an appropriate exposure time
3. Starting the web display will result in a decrease in frame rate. Please ensure that the web display is not started during actual use
4. Ensure normal heat dissipation of the board to avoid frame rate drop caused by high temperature
5. The model was trained using an open-source dataset from the RM community, but due to some issues with the dataset itself, the performance was not particularly good. This case mainly verifies that there is no problem with the overall pathway, and provides rough results and actual data. It is recommended to train with a new dataset for practical use, or use other models for quantitative deployment through the RDK X5 quantization toolchain.

# Results Analysis

## Terminal Output

Log Output:

```text
root@ubuntu:~/rm_ws# ros2 launch rm_armor_detection rm_armor_detection.launch.py
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-01-22-19-42-53-824784-ubuntu-52035
[INFO] [launch]: Default logging verbosity is set to INFO
web_show is  FALSE
Hobot shm pkg enables zero-copy with fastrtps profiles file: /opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
Hobot shm pkg sets RMW_FASTRTPS_USE_QOS_FROM_XML: 1
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
[INFO] [rm_armor_detection-1]: process started with pid [52036]
[rm_armor_detection-1] [BPU_PLAT]BPU Platform Version(1.3.6)!
[rm_armor_detection-1] [HBRT] set log level as 0. version = 3.15.54.0
[rm_armor_detection-1] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
[rm_armor_detection-1] [A][DNN][packed_model.cpp:247][Model](2025-01-22,19:42:54.886.756) [HorizonRT] The model builder version = 1.23.5
[rm_armor_detection-1] [WARN] [1737546175.104195012] [dnn_node_sample]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[rm_armor_detection-1] [WARN] [1737546176.155800829] [dnn_node_sample]: input fps: 76.39, out fps: 76.46, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546177.158795826] [dnn_node_sample]: input fps: 76.92, out fps: 76.77, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546178.167298476] [dnn_node_sample]: input fps: 79.29, out fps: 79.37, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546179.168653423] [dnn_node_sample]: input fps: 75.92, out fps: 75.92, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546180.176483194] [dnn_node_sample]: input fps: 78.53, out fps: 78.37, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546181.182426956] [dnn_node_sample]: input fps: 80.44, out fps: 80.60, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546182.186146874] [dnn_node_sample]: input fps: 78.69, out fps: 78.76, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546183.186917778] [dnn_node_sample]: input fps: 78.69, out fps: 82.00, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546184.192057661] [dnn_node_sample]: input fps: 82.59, out fps: 78.61, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546185.194026778] [dnn_node_sample]: input fps: 78.30, out fps: 77.84, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546186.196924359] [dnn_node_sample]: input fps: 77.84, out fps: 77.84, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546187.204075043] [dnn_node_sample]: input fps: 78.22, out fps: 79.44, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546188.213937324] [dnn_node_sample]: input fps: 78.97, out fps: 78.30, infer time ms: 12, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737546189.225088528] [dnn_node_sample]: input fps: 78.37, out fps: 82.10, infer time ms: 12, post process time ms: 3
```

The log shows that both the frame rates of subscribing image messages and publishing AI messages are around 75fps, with the algorithm taking about 13 milliseconds for single-frame inference and about 3 milliseconds for output parsing.

## Web Display

Web effect screenshot:

![image](./render/result.jpg)

Detected target bounding boxes, key points and categories are rendered.

## Data
|           |       |
|----------|-------|
|Frame rate   |75fps  |
|CPU usage    |430%（相机250% + 检测180%）|
|BPU usage    |85%    |
|delay         |60ms   |