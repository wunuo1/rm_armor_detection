[English](./README.md) | 简体中文

Getting Started with rm armor detection demo
=======


# 功能介绍

rm_armor_detection package是Dnn Node package的使用示例，通过继承DnnNode虚基类，使用YOLOv8s-pose模型和图像数据利用BPU处理器进行算法推理。

图像数据来源于订阅到的图像数据消息，支持使用大恒摄像头（MER-139-210U3C）发布的图像数据（nv12格式）；推理完成后，使用自定义的算法输出解析方法解析算法输出的tensor；解析完成后发布智能结果，可通过web查看实时的渲染效果。

# 开发环境

- 编程语言: C/C++
- 开发平台: X5
- 系统版本：Ubuntu 22.04

# 编译

1、编译环境确认

- 板端已安装X5 Ubuntu系统。

- 当前编译终端已设置TROS环境变量：`source /opt/tros/humble/setup.bash`。

- 已安装ROS2编译工具colcon。

2、编译

- 编译命令：`colcon build --packages-select rm_armor_detection`

# 使用介绍

## X5 Ubuntu系统上运行

```shell
#设置CPU超频
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_boost_frequencies
echo 1 >/sys/devices/system/cpu/cpufreq/boost
echo performance >/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor

#启动大恒摄像头
ros2 run rm_camera_driver rm_camera_driver_node

#启动检测节点
#若需启动WEB端展示，则设置环境变量 export WEB_SHOW=TRUE。需注意启动显示后因编解码等功能运行，会导致帧率下降
ros2 launch rm_armor_detection rm_armor_detection.launch.py
```

## 注意事项
1. 功能启动前请设置CPU超频，帧率更加稳定
2. 摄像头曝光时间会影响帧率，请设置合适的曝光时间
3. 启动WEB展示将导致帧率下降，实际使用请确保web端展示未启动
4. 保证板卡散热正常，避免因温度过高导致帧率下降
5. 该模型使用RM社区开源数据集训练获得，由于数据集本身存在一些问题，效果没有特别好。该案例主要验证整体通路没有问题，并提供大致效果以及实际数据。实际使用建议用新的数据集进行训练，也可使用其他模型，通过RDK X5量化工具链进行量化部署。

# 结果分析

## 终端输出

log输出：

```text
root@ubuntu:~/rm_ws# ros2 launch rm_armor_detection rm_armor_detection.launch.py
[INFO] [launch]: All log files can be found below /root/.ros/log/2025-01-22-21-42-09-988969-ubuntu-3634
[INFO] [launch]: Default logging verbosity is set to INFO
web_show is  FALSE
Hobot shm pkg enables zero-copy with fastrtps profiles file: /opt/tros/humble/lib/hobot_shm/config/shm_fastdds.xml
Hobot shm pkg sets RMW_FASTRTPS_USE_QOS_FROM_XML: 1
env of RMW_FASTRTPS_USE_QOS_FROM_XML is  1 , ignore env setting
[INFO] [rm_armor_detection-1]: process started with pid [3635]
[rm_armor_detection-1] [BPU_PLAT]BPU Platform Version(1.3.6)!
[rm_armor_detection-1] [HBRT] set log level as 0. version = 3.15.54.0
[rm_armor_detection-1] [DNN] Runtime version = 1.23.10_(3.15.54 HBRT)
[rm_armor_detection-1] [A][DNN][packed_model.cpp:247][Model](2025-01-22,21:42:11.113.724) [HorizonRT] The model builder version = 1.23.5
[rm_armor_detection-1] [WARN] [1737553331.352026854] [dnn_node_sample]: Loaned messages are only safe with const ref subscription callbacks. If you are using any other kind of subscriptions, set the ROS_DISABLE_LOANED_MESSAGES environment variable to 1 (the default).
[rm_armor_detection-1] [WARN] [1737553332.403618074] [dnn_node_sample]: input fps: 71.08, out fps: 71.29, infer time ms: 14, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553333.410584641] [dnn_node_sample]: input fps: 68.66, out fps: 68.59, infer time ms: 14, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553334.416761018] [dnn_node_sample]: input fps: 72.71, out fps: 72.56, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553335.416888954] [dnn_node_sample]: input fps: 71.00, out fps: 71.00, infer time ms: 14, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553336.426095202] [dnn_node_sample]: input fps: 71.36, out fps: 71.36, infer time ms: 14, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553337.432650176] [dnn_node_sample]: input fps: 67.59, out fps: 67.59, infer time ms: 14, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553338.439750294] [dnn_node_sample]: input fps: 72.64, out fps: 72.49, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553339.447164638] [dnn_node_sample]: input fps: 72.35, out fps: 72.49, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553340.459308794] [dnn_node_sample]: input fps: 69.93, out fps: 70.16, infer time ms: 14, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553341.471564468] [dnn_node_sample]: input fps: 73.41, out fps: 73.12, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553342.481729076] [dnn_node_sample]: input fps: 72.13, out fps: 72.28, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553343.493939536] [dnn_node_sample]: input fps: 74.26, out fps: 74.11, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553344.505093429] [dnn_node_sample]: input fps: 72.21, out fps: 72.21, infer time ms: 13, post process time ms: 3
[rm_armor_detection-1] [WARN] [1737553345.517376352] [dnn_node_sample]: input fps: 71.22, out fps: 71.15, infer time ms: 14, post process time ms: 3

```

log显示订阅图像消息和发布AI消息的帧率都为70fps左右，算法单帧推理耗时为13毫秒左右，算法输出解析耗时为3毫秒左右。

## web效果展示

web效果截图：

![image](./render/result.jpg)

渲染出了检测出来的目标检测框、关键点和类别。

## 具体数据
|           |       |
|----------|-------|
|识别帧率   |75fps  |
|CPU占用    |430%（相机250% + 检测180%）|
|BPU占用    |85%    |
|处理延迟   |60ms   |