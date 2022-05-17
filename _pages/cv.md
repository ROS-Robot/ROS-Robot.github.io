---
layout: archive
title: "CV"
permalink: /cv/
author_profile: true
redirect_from:
  - /resume
---

# ROS2资源汇总
## Awesome Robot Operating System 2 (ROS 2)中文版 [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)



> 精心编排的机器人操作系统2.0版（ROS 2）资源和库列表

机器人操作系统2（ROS 2）是一套软件库和工具，可以帮助你建立机器人应用。从驱动程序到最先进的算法，以及强大的开发者工具，ROS 2拥有你的下一个机器人项目所需要的东西。而且都是开源的。

## 主要内容

- [软件包](#软件包)
- [操作系统](#操作系统)
- [分支](#分支)
- [文档](#文档)
- [社区](#社区)
- [书籍](#书籍)
- [课程](#课程)
- [会议](#会议)
- [论文](#论文)
- [播客](#播客)
- [服务](#服务)
- [公司](#公司)
- [组织](#组织)
- [工作组](#工作组)

## 软件包

### 展示

- [adlink_ddsbot](https://github.com/Adlink-ROS/adlink_ddsbot) - 基于ROS 2.0/1.0的机器人集群架构(opensplice DDS). ![adlink_ddsbot](https://img.shields.io/github/stars/Adlink-ROS/adlink_ddsbot.svg)
- [adlink_neuronbot](https://github.com/Adlink-ROS/adlink_neuronbot) - ROS2/DDS机器人软件包，用于人类的跟踪和集群. ![adlink_neuronbot](https://img.shields.io/github/stars/Adlink-ROS/adlink_neuronbot.svg)
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/ros2) - 基于ROS2的TurtleBot3演示，包括Bringup、Teleop和Cartographer. ![turtlebot3](https://img.shields.io/github/stars/ROBOTIS-GIT/turtlebot3.svg)

### 示例

- [turtlebot2_demo](https://github.com/ros2/turtlebot2_demo) - 使用ROS 2的TurtleBot 2演示. ![turtlebot2_demo](https://img.shields.io/github/stars/ros2/turtlebot2_demo.svg)
- [examples/rclcpp](https://github.com/ros2/examples/tree/master/rclcpp) - C++ 示例. ![ros2/examples](https://img.shields.io/github/stars/ros2/examples.svg)
- [examples/rclpy](https://github.com/ros2/examples/tree/master/rclpy) - Python 示例. ![ros2/examples](https://img.shields.io/github/stars/ros2/examples.svg)
- [rcljava_examples](https://github.com/esteve/ros2_java_examples/tree/master/rcljava_examples) - 包含如何使用rcljava API的例子的软件包. ![ros2_java_examples](https://img.shields.io/github/stars/esteve/ros2_java_examples.svg)
- [ros2_talker_android, ros2_listener_android](https://github.com/esteve/ros2_android_examples) - ROS2 Java绑定的Android应用实例. ![ros2_android_examples](https://img.shields.io/github/stars/esteve/ros2_android_examples.svg)

### 对比分析

- [ros2_benchmarking](https://github.com/piappl/ros2_benchmarking) - ROS2基准测试的框架。ROS2的通信特性可以在几个轴上进行快速和自动化的评估. ![ros2_benchmarking](https://img.shields.io/github/stars/piappl/ros2_benchmarking.svg)
- [performance_test](https://github.com/ApexAI/performance_test) - 测试各种通信手段的性能和延迟，如ROS 2、FastRTPS和Connext DDS Micro. ![performance_test](https://img.shields.io/github/stars/ApexAI/performance_test.svg)

### 容器化

- [docker-ros2-ospl-ce](https://github.com/Adlink-ROS/docker-ros2-ospl-ce) - 建立ROS2+OpenSplice CE容器的docker文件. ![docker-ros2-ospl-ce](https://img.shields.io/github/stars/Adlink-ROS/docker-ros2-ospl-ce.svg)
- [ros2_java_docker](https://github.com/esteve/ros2_java_docker) - 用OpenJDK和Android构建ros2_java的Dockerfiles. ![ros2_java_docker](https://img.shields.io/github/stars/esteve/ros2_java_docker.svg)
- [micro-ROS/docker](https://github.com/micro-ROS/docker) - 设置、配置和开发微型ROS硬件的Docker相关材料. 
- [ros-tooling/cross_compile](https://github.com/ros-tooling/cross_compile) - 将ROS和ROS 2工作空间交叉编译到非本地架构，并生成相应的Docker镜像.
- [ros2-docker](https://husarnet.com/blog/ros2-docker) - 通过互联网连接运行在Docker容器中的ROS 2节点.

### 网络

- [Husarnet VPN](https://github.com/husarnet/husarnet) - 一个专门用于ROS和ROS 2的P2P、安全网络层. ![husarnet](https://img.shields.io/github/stars/husarnet/husarnet.svg)

### 生态

- [Link ROS](https://www.freedomrobotics.ai/blog/link-ros-cloud-logging-for-ros) - 用于ROS 1和ROS 2的云日志.
- [rosbag2](https://github.com/ros2/rosbag2) - ROS2原生的rosbag. ![rosbag2](https://img.shields.io/github/stars/ros2/rosbag2.svg)
- [rviz](https://github.com/ros2/rviz) - 3D机器人展示台. ![rviz](https://img.shields.io/github/stars/ros2/rviz.svg)
- [urdfdom](https://github.com/ros2/urdfdom) - URDF（U-Robot描述格式）库，提供核心数据结构和一个简单的XML解析器 ![urdfdom](https://img.shields.io/github/stars/ros2/urdfdom.svg)
- [urdfdom_headers](https://github.com/ros2/urdfdom_headers) - 用于URDF分析器的头文件. ![urdfdom_headers](https://img.shields.io/github/stars/ros2/urdfdom_headers.svg)
- [ros2cli](https://github.com/ros2/ros2cli) - ROS 2命令行工具. ![ros2cli](https://img.shields.io/github/stars/ros2/ros2cli.svg)
- [orocos_kinematics_dynamics](https://github.com/ros2/orocos_kinematics_dynamics) - Orocos运动学和动力学C++库. ![orocos_kinematics_dynamics](https://img.shields.io/github/stars/ros2/orocos_kinematics_dynamics.svg)
- [pydds](https://github.com/atolab/pydds) - 用于Vortex Lite和OpenSplice的简单DDS Python API. ![pydds](https://img.shields.io/github/stars/atolab/pydds.svg)
- [Webots](https://cyberbotics.com) - 用于ROS 2的机器人模拟器. ![webots](https://img.shields.io/github/stars/cyberbotics/webots.svg)
- [LGSVL](https://www.lgsvlsimulator.com/) - 仿真软件加速自动驾驶汽车的安全开发.
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub) - 这是一个关于Unity中机器人模拟的工具、教程、资源和文档的中心库.
- [Foxglove Studio](https://github.com/foxglove/studio) - 用于机器人的综合可视化和诊断工具. ![foxglove studio](https://img.shields.io/github/stars/foxglove/studio.svg)
- [ROS2 For Unity](https://github.com/RobotecAI/ros2-for-unity) - 一个资产包，使Unity3D模拟和ROS2生态系统之间的高性能通信成为可能. ![ros2-for-unity](https://img.shields.io/github/stars/RobotecAI/ros2-for-unity.svg)

### 交互

- [Jupyter ROS2](https://github.com/zmk5/jupyter-ros2) - 用于ROS2的Jupyter小组件帮助程序.

### 渗透测试

- [aztarna](https://github.com/aliasrobotics/aztarna) - 一个用于机器人的足迹工具.
- [ros2_fuzzer](https://github.com/aliasrobotics/ros2_fuzzer) - ROS2主题和服务模糊器.

### 应用层

- [Apex.Autonomy](https://www.apex.ai/apex-autonomy) - Apex.Autonomy提供了作为独立构件的自治算法，并与Autoware.Auto兼容.
- [Autoware.Auto](https://www.autoware.auto/) - Autoware.Auto为自动驾驶技术提供一个基于ROS 2的开源软件栈.
- [ros2_control](https://github.com/ros-controls/ros2_control) - `ros2_control` 是对ROS 2中的新功能如何在机器人控制方面进行阐述和使用的一个概念证明 (`ros2_controllers`). ![ros2_control](https://img.shields.io/github/stars/ros-controls/ros2_control.svg)
- [ros2_controllers](https://github.com/ros-controls/ros2_controllers) - ros_controllers的描述. ![ros2_controllers](https://img.shields.io/github/stars/ros-controls/ros2_controllers.svg)
- [geometry2](https://github.com/ros2/geometry2) - 一组用于跟踪坐标变换的ROS包. ![geometry2](https://img.shields.io/github/stars/ros2/geometry2.svg)
- [ros2-ORB_SLAM2](https://github.com/alsora/ros2-ORB_SLAM2) - 封装ORB_SLAM2库的ROS2节点. ![ros2-ORB_SLAM2](https://img.shields.io/github/stars/alsora/ros2-ORB_SLAM2.svg)
- [basalt_ros2](https://github.com/berndpfrommer/basalt_ros2) - Basalt VIO的ROS2包装器. ![basalt_ros2](https://img.shields.io/github/stars/berndpfrommer/basalt_ros2.svg)
- [cartographer](https://github.com/ros2/cartographer) - 跨越多个平台和传感器配置的二维和三维实时同步定位和测绘（SLAM）. ![cartographer](https://img.shields.io/github/stars/ros2/cartographer.svg)
- [slam_gmapping](https://github.com/Project-MANAS/slam_gmapping) - 用于ROS2的Slam Gmapping. ![slam_gmapping](https://img.shields.io/github/stars/Project-MANAS/slam_gmapping.svg)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) - 用ROS在潜在的大规模地图中进行终身制图和定位的Slam工具箱. ![slam_toolbox](https://img.shields.io/github/stars/SteveMacenski/slam_toolbox.svg)
- [lidarslam_ros2](https://github.com/rsasaki0109/lidarslam_ros2) - 使用ndt/gicp注册和姿势优化的三维激光雷达slam的ROS2软件包. ![lidarslam_ros2](https://img.shields.io/github/stars/rsasaki0109/lidarslam_ros2.svg)
- [li_slam_ros2](https://github.com/rsasaki0109/li_slam_ros2) - 参考LIO-SAM的紧耦合激光雷达惯性ndt/gicp slam的ROS2软件包. ![li_slam_ros2](https://img.shields.io/github/stars/rsasaki0109/li_slam_ros2.svg)
- [octomap_server2](https://github.com/iKrishneel/octomap_server2) - 用OctoMap绘图的ROS2栈。ROS1的端口 [octomap_mapping](https://github.com/OctoMap/octomap_mapping) 软件包. ![octomap_server2](https://img.shields.io/github/stars/iKrishneel/octomap_server2.svg)
- [vision_opencv](https://github.com/ros-perception/vision_opencv/tree/ros2) - 用于连接ROS2和OpenCV的软件包. ![vision_opencv](https://img.shields.io/github/stars/ros-perception/vision_opencv.svg)
- [teleop_twist_keyboard](https://github.com/ros2/teleop_twist_keyboard) - 用于ROS2的通用键盘远程操作. ![teleop_twist_keyboard](https://img.shields.io/github/stars/ros2/teleop_twist_keyboard.svg)
- [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy) - 用于扭转机器人的简单操纵杆远程操作. ![teleop_twist_joy](https://img.shields.io/github/stars/ros2/teleop_twist_joy.svg)
- [navigation](https://github.com/ros-planning/navigation2/) - ROS2导航栈. ![navigation](https://img.shields.io/github/stars/ros-planning/navigation2.svg)
- [diagnostics](https://github.com/bponsler/diagnostics/tree/ros2-devel) - 原始ROS1诊断程序的分叉版本，适用于ROS 2（目前只有诊断程序_updater）. ![diagnostics](https://img.shields.io/github/stars/bponsler/diagnostics.svg)
- [robot_state_publisher](https://github.com/bponsler/robot_state_publisher/tree/publish-robot-model) - 原始ROS机器人状态发布器的分叉版本，经过所有修改，可以在ROS2生态系统中编译. ![robot_state_publisher](https://img.shields.io/github/stars/bponsler/robot_state_publisher.svg)
- [common_interfaces](https://github.com/ros2/common_interfaces) - 一组包含通用接口文件（.msg和.srv）的软件包. ![common_interfaces](https://img.shields.io/github/stars/ros2/common_interfaces.svg)
- [ros2_object_map](https://github.com/intel/ros2_object_map) - "SLAM时在地图上标记物体的标签"
. ![ros2_object_map](https://img.shields.io/github/stars/intel/ros2_object_map.svg)
- [ros2_object_analytics](https://github.com/intel/ros2_object_analytics) - 物体分析（OA）是ROS2封装器，用于实时物体检测、定位和跟踪. ![ros2_object_analytics](https://img.shields.io/github/stars/intel/ros2_object_analytics.svg)
- [ros2_intel_movidius_ncs](https://github.com/intel/ros2_intel_movidius_ncs) - Movidius™神经计算棒（NCS）神经元计算API的ROS2包装器. ![ros2_intel_movidius_ncs](https://img.shields.io/github/stars/intel/ros2_intel_movidius_ncs.svg)
- [ros2_moving_object](https://github.com/intel/ros2_moving_object) - 根据物体分析所产生的信息对移动物体进行寻址
 `ros2_object_analytics`. ![ros2_moving_object](https://img.shields.io/github/stars/intel/ros2_moving_object.svg)
- [ros2_openvino_toolkit](https://github.com/intel/ros2_openvino_toolkit) - OpenVINO™的CV API的ROS2封装器（人类视觉仿真）. ![ros2_openvino_toolkit](https://img.shields.io/github/stars/intel/ros2_openvino_toolkit.svg)
- [ros2_grasp_library](https://github.com/intel/ros2_grasp_library) - 可能是一个把握的图书馆 :). ![ros2_grasp_library](https://img.shields.io/github/stars/intel/ros2_grasp_library.svg)
- [apriltag_ros](https://github.com/christianrauch/apriltag_ros) - 用于检测AprilTag的ROS2节点. ![apriltag_ros](https://img.shields.io/github/stars/christianrauch/apriltag_ros.svg)
- [ros2-web-bridge](https://github.com/RobotWebTools/ros2-web-bridge) - 将你的浏览器与ROS 2.0相连接. ![ros2-web-bridge](https://img.shields.io/github/stars/RobotWebTools/ros2-web-bridge.svg)
- [ros2_message_filters](https://github.com/intel/ros2_message_filters) - ros2_message_filters根据过滤器需要满足的条件来混合各种消息，源于ROS2移植的ROS message_filters. ![ros2_message_filters](https://img.shields.io/github/stars/intel/ros2_message_filters.svg)
- [ros2-tensorflow](https://github.com/alsora/ros2-tensorflow) - ROS2 nodes for computer vision tasks in Tensorflow. ![ros2-tensorflow](https://img.shields.io/github/stars/alsora/ros2-tensorflow.svg)
- [ros2_pytorch](https://github.com/klintan/ros2_pytorch) - PyTorch中用于计算机视觉任务的ROS2节点 ![ros2_pytorch](https://img.shields.io/github/stars/klintan/ros2_pytorch.svg).
- [pid](https://github.com/UTNuclearRoboticsPublic/pid) - 一个用于ROS2的PID控制器. ![pid](https://img.shields.io/github/stars/UTNuclearRoboticsPublic/pid.svg)
- [system-modes](https://github.com/micro-ROS/system_modes) - ROS 2和微型ROS的系统模式.
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros/tree/ros2) - 用于部署Darknet的YOLO计算机视觉模型的ROS2包装器.
- [easy_perception_deployment](https://github.com/ros-industrial/easy_perception_deployment) - 加快计算机视觉模型的培训和部署的软件包，适用于各行业. ![easy_perception_deployment](https://img.shields.io/github/stars/ros-industrial/easy_perception_deployment.svg)
- [easy_manipulation_deployment](https://github.com/ros-industrial/easy_manipulation_deployment) - 整合感知元素以建立一个端到端的取放任务的软件包. ![easy_manipulation_deployment](https://img.shields.io/github/stars/ros-industrial/easy_manipulation_deployment.svg)

### Middleware

- [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent) - Micro XRCE-DDS代理在DDS网络和Micro XRCE-DDS客户端之间充当服务器.
- [Micro XRCE-DDS Agent docker](https://hub.docker.com/r/eprosima/micro-xrce-dds-agent/) - 包含Micro XRCE-DDS代理的Docker镜像.
- [Micro XRCE-DDS Client](https://github.com/eProsima/Micro-XRCE-DDS-Client) - Micro XRCE-DDS实现了一个客户-服务器协议，使资源有限的设备（客户）能够参与DDS通信.
- [micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent) - 使用Micro XRCE-DDS代理的ROS 2软件包.
- [Eclipse Zenoh](https://github.com/eclipse-zenoh/zenoh) - [Zenoh](https://zenoh.io) 是一个可扩展的、性能极好的协议，可以透明地用于与其他国家进行交互 [ROS2 applications](https://zenoh.io/blog/2021-04-28-ros2-integration/) 以及为 [R2X communication](https://zenoh.io/blog/2021-03-23-discovery/). (https://img.shields.io/github/stars/eclipse-zenoh/zenoh)
- [Eclipse Zenoh-Plugin-DDS](https://github.com/eclipse-zenoh/zenoh-plugin-dds) - 这是 [zenoh](https://zenoh.io) 插件它允许在zenoh上透明地路由ROS2/DDS数据. 这通常用于 [R2X communication](https://zenoh.io/blog/2021-03-23-discovery/) over Wireless network or across the Internet. (https://img.shields.io/github/stars/eclipse-zenoh/zenoh-plugin-dds)

### "系统" 捆绑

- [rclandroid](https://github.com/esteve/ros2_android/tree/master/rclandroid) - 用于ROS2的Android API. ![rclandroid](https://img.shields.io/github/stars/esteve/ros2_android.svg)
- [rclnodejs](https://github.com/RobotWebTools/rclnodejs) - ROS2.0客户端的Node.js版本. ![rclnodejs](https://img.shields.io/github/stars/RobotWebTools/rclnodejs.svg)
- [riot-ros2](https://github.com/astralien3000/riot-ros2) - 这个项目使ROS2能够在使用RIOT操作系统的微控制器上运行. ![riot-ros2](https://img.shields.io/github/stars/astralien3000/riot-ros2.svg)
- [ROS2-Integration-Service](https://github.com/eProsima/ROS2-Integration-Service) - ROS2集成和路由，提供了一个完整的工具，使其他技术与ROS2轻松集成，并在广域网/互联网上启用ROS2.
- [soss](https://github.com/osrf/soss) - 系统合成器用于通过ROS2-集成服务将ROS2与其他（通信）系统集成.
- [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino) - 将微型ROS集成到Arduino软件平台项目中.
- [micro_ros_zephyr_module](https://github.com/micro-ROS/micro_ros_zephyr_module) - 在基于Zeyphr操作系统的项目中整合微型ROS.

### 驱动层

- [Autoware.IO](https://www.autoware.io/) - Autoware.IO提供了一个异构的硬件参考平台，使成员公司的解决方案能够整合到支持Autoware.Auto和Autoware.AI软件栈的平台上.
- [ros2_xmlrpc_interface](https://github.com/aarushsesto/ros2_xmlrpc_interface) - 带有xmlrpc的ros2接口包，使用Sesto API与Sesto服务器进行通信. ![ros2_xmlrpc](https://img.shields.io/github/stars/aarushsesto/ros2_xmlrpc_interface.svg)
- [cozmo_driver_ros2](https://github.com/FurqanHabibi/cozmo_driver_ros2) - 用于ROS2的非官方Anki Cozmo节点. ![cozmo_driver_ros2](https://img.shields.io/github/stars/FurqanHabibi/cozmo_driver_ros2.svg)
- [sphero_ros2](https://github.com/athackst/sphero_ros2) - ROS2 sphero的驱动程序. ![sphero_ros2](https://img.shields.io/github/stars/athackst/sphero_ros2.svg)
- [flock2](https://github.com/clydemcqueen/flock2) - 用于DJI Tello无人机的ROS2驱动程序. ![flock2](https://img.shields.io/github/stars/clydemcqueen/flock2.svg)
- [ros2_raspicam_node](https://github.com/Misterblue/ros2_raspicam_node) - 用于Raspberry Pi相机的ROS2节点. ![ros2_raspicam_node](https://img.shields.io/github/stars/Misterblue/ros2_raspicam_node.svg)
- [joystick_drivers](https://github.com/ros2/joystick_drivers) - 用于操纵杆的ROS2驱动程序. ![joystick_drivers](https://img.shields.io/github/stars/ros2/joystick_drivers.svg)
- [joystick_drivers_from_scratch](https://github.com/ros2/joystick_drivers_from_scratch) - 用于ROS 2的操纵杆驱动包. ![joystick_drivers_from_scratch](https://img.shields.io/github/stars/ros2/joystick_drivers_from_scratch.svg)
- [joystick_ros2](https://github.com/FurqanHabibi/joystick_ros2) - ROS2的操纵杆驱动程序，支持所有平台。Linux, macOS, Windows. ![joystick_ros2](https://img.shields.io/github/stars/FurqanHabibi/joystick_ros2.svg)
- [ros2_teleop_keyboard](https://github.com/rohbotics/ros2_teleop_keyboard) - 用于ROS2的Teleop Twist键盘. ![ros2_teleop_keyboard](https://img.shields.io/github/stars/rohbotics/ros2_teleop_keyboard.svg)
- [ros_astra_camera](https://github.com/ros2/ros_astra_camera) - 用于Astra相机的ROS2封装. ![ros_astra_camera](https://img.shields.io/github/stars/ros2/ros_astra_camera.svg)
- [ros2_usb_camera](https://github.com/klintan/ros2_usb_camera) - ROS2通用USB摄像头驱动程序. ![ros_astra_camera](https://img.shields.io/github/stars/klintan/ros2_usb_camera.svg)
- [ros2_android_drivers](https://github.com/esteve/ros2_android_drivers) - 用于几个安卓传感器的ROS2驱动集合. ![ros2_android_drivers](https://img.shields.io/github/stars/esteve/ros2_android_drivers.svg)
- [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense) - 用于Intel® RealSense™设备的ROS2封装
. ![ros2_intel_realsense](https://img.shields.io/github/stars/intel/ros2_intel_realsense.svg)
- [raspicam2_node](https://github.com/christianrauch/raspicam2_node) - 用于树莓派相机模块的ROS2节点. ![raspicam2_node](https://img.shields.io/github/stars/christianrauch/raspicam2_node.svg)
- [ros2_track_imu](https://github.com/klintan/ros2_track_imu) - TrackIMU IMU传感器的ROS2节点![ros2_track_imu](https://img.shields.io/github/stars/klintan/ros2_track_imu.svg).
- [HRIM](https://github.com/AcutronicRobotics/HRIM) - 一个机器人模块的标准接口.
- [FIROS2](https://github.com/eProsima/FIROS2) - 专注于ROS2和FIWARE之间互通的ROS2可整合工具. ![FIROS2](https://img.shields.io/github/stars/eProsima/FIROS2.svg)
- [lino2_upper](https://github.com/linorobot2/lino2_upper) - ROS2上的Linorobot. ![lino2_upper](https://img.shields.io/github/stars/linorobot2/lino2_upper.svg)
- [RysROS2](https://github.com/GroupOfRobots/RysROS2) - 用于MiniRys机器人的ROS2软件栈. ![RysROS2](https://img.shields.io/github/stars/GroupOfRobots/RysROS2.svg)
- [px4_to_ros](https://github.com/eProsima/px4_to_ros) - ROS2/ROS软件包，用于将PX4与ROS进行通信. ![px4_to_ros](https://img.shields.io/github/stars/eProsima/px4_to_ros.svg)
- [multiwii_ros2](https://github.com/christianrauch/multiwii_ros2) - 用于MultiWii和Cleanflight飞行控制器的ROS2节点. ![multiwii_ros2](https://img.shields.io/github/stars/christianrauch/multiwii_ros2.svg)
- [ydlidar_ros2](https://github.com/Adlink-ROS/ydlidar_ros2) - ydlidar的ROS2包装器. ![ydlidar_ros2](https://img.shields.io/github/stars/Adlink-ROS/ydlidar_ros2.svg)
- [zed-ros2-wrapper](https://github.com/stereolabs/zed-ros2-wrapper) - 用于ZED SDK的ROS 2包装器测试版.
- [ros2_denso_radar](https://github.com/klintan/ros2_denso_radar) - 丰田/雷克萨斯2015-2017年电装雷达驱动器，用于ROS2.
- [sick_scan2](https://github.com/SICKAG/sick_scan2) - 用于SICK TiM系列激光扫描器的ROS2驱动程序 (TiM551/TiM561/TiM571).
- [ros2_ouster_drivers](https://github.com/SteveMacenski/ros2_ouster_drivers) - 用于Ouster OS-1激光雷达的ROS2驱动器. ![ros2_ouster_drivers](https://img.shields.io/github/stars/SteveMacenski/ros2_ouster_drivers)
- [micro-ROS/hardware](https://github.com/micro-ROS/hardware) - 有关微型ROS项目中使用和支持的硬件平台的信息和文件.
- [Blickfeld Cube 1 & Cube Range](https://docs.blickfeld.com/cube/latest/external/ros/driver-v2/README.html) - 用于Blickfeld Cube 1和Cube系列的ROS2驱动器.

### 客户库

- [rclada](https://github.com/ada-ros/rclada) - 用于Ada的ROS客户端库. ![rclada](https://img.shields.io/github/stars/ada-ros/rclada.svg)
- [rclcpp](https://github.com/ros2/rclcpp) - 用于C++的ROS客户端库. ![rclcpp](https://img.shields.io/github/stars/ros2/rclcpp.svg)
- [rclgo](https://github.com/juaruipav/rclgo) - 用于Go的ROS客户端库. ![rclgo](https://img.shields.io/github/stars/juaruipav/rclgo.svg)
- [rclpy](https://github.com/ros2/rclpy) - 用于Python的ROS客户端库. ![rclpy](https://img.shields.io/github/stars/ros2/rclpy.svg)
- [rcljava](https://github.com/esteve/ros2_java/tree/master/rcljava) - Java的ROS客户端库. ![rcljava](https://img.shields.io/github/stars/esteve/ros2_java.svg)
- [rclnodejs](https://github.com/RobotWebTools/rclnodejs) - 用于Node.js的ROS客户端库. ![rclnodejs](https://img.shields.io/github/stars/RobotWebTools/rclnodejs.svg)
- [rclobjc](https://github.com/esteve/ros2_objc) - 适用于Objective C的ROS客户端库（iOS）. ![rclobjc](https://img.shields.io/github/stars/esteve/ros2_objc.svg)
- [rclc](https://github.com/ros2/rclc) - 用于C语言的ROS客户端库. ![rclc](https://img.shields.io/github/stars/ros2/rclc.svg)
- [ros2_rust](https://github.com/ros2-rust/ros2_rust) - ROS2的Rust绑定. ![ros2_rust](https://img.shields.io/github/stars/esteve/ros2_rust.svg)
- [ros2_dotnet](https://github.com/esteve/ros2_dotnet) - 用于ROS2的.NET绑定. ![ros2_dotnet](https://img.shields.io/github/stars/esteve/ros2_dotnet.svg)
- [ros2cs](https://github.com/RobotecAI/ros2cs) - an alternative to ros2_dotnet, a ROS2 C# interface supporting full range of messages and modern ROS2. ![ros2cs](https://img.shields.io/github/stars/RobotecAI/ros2cs.svg)

### 常见的客户库

- [rcl](https://github.com/ros2/rcl) - 一个替代ros2_dotnet的ROS2 C#接口，支持所有的信息和现代ROS2的功能. ![rcl](https://img.shields.io/github/stars/ros2/rcl.svg)
- [system_tests](https://github.com/ros2/system_tests) - rclcpp和rclpy的测试. ![system_tests](https://img.shields.io/github/stars/ros2/system_tests.svg)
- [rcl_interfaces](https://github.com/ros2/rcl_interfaces) - ROS客户端库所使用的信息和服务的存储库. ![rcl_interfaces](https://img.shields.io/github/stars/ros2/rcl_interfaces.svg)

### IDL生成器

- [rosidl_generator_java](https://github.com/esteve/ros2_java/tree/master/rosidl_generator_java) - 在Java中生成ROS接口. ![ros2_java](https://img.shields.io/github/stars/esteve/ros2_java.svg)
- [rosidl_generator_objc](https://github.com/esteve/ros2_objc/tree/master/rosidl_generator_objc) - 在Objective C中生成ROS接口. ![ros2_objc](https://img.shields.io/github/stars/esteve/ros2_objc.svg)
- [rosidl_generator_cpp](https://github.com/ros2/rosidl/tree/master/rosidl_generator_cpp) - 用C++生成ROS接口. ![rosidl](https://img.shields.io/github/stars/ros2/rosidl.svg)
- [rosidl_generator_c](https://github.com/ros2/rosidl/tree/master/rosidl_generator_c) - 用C语言生成ROS接口. ![rosidl](https://img.shields.io/github/stars/ros2/rosidl.svg)
- [rosidl](https://github.com/ros2/rosidl) - 提供ROS IDL（.msg）定义和代码生成的软件包. ![rosidl](https://img.shields.io/github/stars/ros2/rosidl.svg)
- [rosidl_dds](https://github.com/ros2/rosidl_dds) - 为ROS接口生成DDS接口. ![rosidl_dds](https://img.shields.io/github/stars/ros2/rosidl_dds.svg)

### RMW（ROS中间件）

- [rmw](https://github.com/ros2/rmw/tree/master/rmw) - 包含ROS中间件API. ![rmw](https://img.shields.io/github/stars/ros2/rmw.svg)
- [rmw_connext_cpp](https://github.com/ros2/rmw_connext/tree/master/rmw_connext_cpp) - 使用RTI Connext静态代码生成的C++语言实现ROS中间件接口. ![rmw_connext_cpp](https://img.shields.io/github/stars/ros2/rmw_connext.svg)
- [rmw_fastrtps_cpp](https://github.com/ros2/rmw_fastrtps/tree/master/rmw_fastrtps_cpp) - 使用eProsima FastRTPS静态代码生成的C++实现ROS中间件接口. ![rmw_fastrtps_cpp](https://img.shields.io/github/stars/ros2/rmw_fastrtps.svg)
- [rmw_dps](https://github.com/ros2/rmw_dps) - 使用英特尔的分布式发布和订阅实现ROS中间件（rmw）的接口. ![rmw_dps](https://img.shields.io/github/stars/ros2/rmw_dps.svg)
- [rmw_opensplice_cpp](https://github.com/ros2/rmw_opensplice/tree/master/rmw_opensplice_cpp) - 使用PrismTech OpenSplice静态代码生成器在C++中实现ROS中间件接口. ![rmw_opensplice_cpp](https://img.shields.io/github/stars/ros2/rmw_opensplice.svg)
- [rmw_coredx](https://github.com/tocinc/rmw_coredx) - 用于ROS2的CoreDX DDS集成层. ![tocinc/rmw_coredx](https://img.shields.io/github/stars/tocinc/rmw_coredx.svg)
- [rmw_freertps](https://github.com/ros2/rmw_freertps) - 使用freertps实现RMW. ![tocinc/rmw_coredx](https://img.shields.io/github/stars/ros2/rmw_freertps.svg)
- [rmw_zenoh](https://github.com/atolab/rmw_zenoh) - 使用Eclipse zenoh实现RMW。零开销的Pub/sub、存储/查询和计算. ![atolab/rmw_zenoh](https://img.shields.io/github/stars/atolab/rmw_zenoh.svg)
- [rcutils](https://github.com/ros2/rcutils) - ROS 2中使用的常见C函数和数据结构. ![rmw](https://img.shields.io/github/stars/ros2/rcutils.svg)
- [freertps](https://github.com/ros2/freertps) - 一个免费的、可移植的、简约的、正在进行中的RTPS实现. ![rmw](https://img.shields.io/github/stars/ros2/freertps.svg)
- [rmw_cyclonedds](https://github.com/atolab/rmw_cyclonedds) - 用于Eclipse Cyclone DDS的ROS2 RMW层. ![rmw_cyclonedds](https://img.shields.io/github/stars/atolab/rmw_cyclonedds.svg)
- [rmw_zenoh](https://github.com/atolab/rmw_zenoh) - ROS2 RMW层为 [zenoh](https://zenoh.io).
- [rmw_iceoryx](https://github.com/ros2/rmw_iceoryx) - 启用进程间通信中间件的使用 [Eclipse iceoryx](https://iceoryx.io).

### DDS communication mechanism implementations

- [Connext DDS](https://www.rti.com/products/connext-dds-professional) -  用于开发和整合IIoT系统的连接软件. :heavy_dollar_sign:
- [Fast-RTPS](https://github.com/eProsima/Fast-RTPS) - 实施RTPS标准（RTPS是DDS的线上互操作协议）. ![Fast-RTPS](https://img.shields.io/github/stars/eProsima/Fast-RTPS.svg)
- [OpenSplice](https://github.com/ADLINK-IST/opensplice) - OMG DDS标准的实施. ![opensplice](https://img.shields.io/github/stars/ADLINK-IST/opensplice.svg) :heavy_dollar_sign:
- [CoreDX DDS](http://www.twinoakscomputing.com/coredx) - 双橡树计算公司的实施 :heavy_dollar_sign:
- [freertps](https://github.com/ros2/freertps) - 一个免费的、可移植的、简约的、正在进行中的RTPS实现. ![freertps](https://img.shields.io/github/stars/ros2/freertps.svg)
- [cdds](https://github.com/atolab/cdds) - Cyclone DDS是完全公开开发的，正在接受过程中，以成为Eclipse IoT的一部分. ![cdds](https://img.shields.io/github/stars/atolab/cdds.svg)
- [Micro-XRCE-DDS)](https://github.com/eProsima/Micro-XRCE-DDS) - 一个XRCE DDS实现（由microROS支持）. ![Micro-XRCE-DDS](https://img.shields.io/github/stars/eProsima/Micro-XRCE-DDS.svg)

### 系统编译(Linux)

- [meta-ros2](https://github.com/erlerobot/meta-ros2) - 用于OpenEmbedded Linux的ROS2层. ![meta-ros2](https://img.shields.io/github/stars/erlerobot/meta-ros2.svg)

### 系统编译 (ROS2)

- [ci](https://github.com/ros2/ci) - ROS 2 CI 基础设施. ![ci](https://img.shields.io/github/stars/ros2/ci.svg)
- [ament_cmake_export_jars](https://github.com/esteve/ros2_java/tree/master/ament_cmake_export_jars) - 在CMake的ament构建系统中，能够将Java档案导出为下游包. ![ros2_java](https://img.shields.io/github/stars/esteve/ros2_java.svg)
- [rmw_implementation_cmake](https://github.com/ros2/rmw/tree/master/rmw_implementation_cmake) - 可以发现和列举可用实现的CMake函数. ![rmw](https://img.shields.io/github/stars/ros2/rmw.svg)
- [rmw_implementation](https://github.com/ros2/rmw_implementation) - rmw实现的CMake基础设施和依赖性. ![rmw](https://img.shields.io/github/stars/ros2/rmw_implementation.svg)

## 操作系统

- [NuttX](https://github.com/micro-ROS/NuttX) - 用于微型ROS的官方NuttX分支.
- [RIOT](https://github.com/RIOT-OS/RIOT) - RIOT是一个实时多线程操作系统(...)，实时能力强，内存占用小，(...)API提供部分POSIX兼容性.
- [eMCOS](https://www.esol.com/embedded/emcos.html) - 用于多核处理器的符合POSIX标准的实时操作系统，有望在未来支持AUTOSAR.
- [PYNQ](http://www.pynq.io/) - 在XILINX FPGA上运行的基于Python的高性能ML应用的快速原型开发.
- [ReconROS](https://github.com/Lien182/ReconROS) - 基于ROS2 FPGA的硬件加速的框架。基于 [ReconOS](https://github.com/reconos/reconos). ![ReconROS](https://img.shields.io/github/stars/Lien182/ReconROS.svg)
- [Ubuntu Core](https://ubuntu.com/core) - 用Ubuntu Core构建安全的物联网设备.
- [Ubuntu Server](https://ubuntu.com/server)
- [VxWorks](https://github.com/Wind-River/vxworks7-ros2-build) - 用于关键基础设施的安全、可靠和可认证的实时操作系统
- [Zephyr](https://www.zephyrproject.org/) - Linux基金会项目的RTOS，旨在实现安全和安保.

## 分支

- [Apex.OS](https://www.apex.ai/apex-os) - Apex.OS是ROS 2的一个分叉，它已经变得非常强大和可靠，可以用于安全关键型应用.

## 文档

- [ROS Index](https://index.ros.org/) - 未来进入ROS2文档的单一入口点（BETA）.
  - [Foxy packages](https://index.ros.org/packages/page/1/time/#foxy).
  - [Dashing packages](https://index.ros.org/packages/page/1/time/#dashing).
  - [Crystal packages](https://index.ros.org/packages/page/1/time/#crystal).
  - [Bouncy packages](https://index.ros.org/packages/page/1/time/#bouncy).
  - [Ardent packages](https://index.ros.org/packages/page/1/time/#ardent).
- [ROS 2 Design](http://design.ros2.org/) - 为ROS 2.0设计工作提供信息和指导的文章.
- [ROS 2 Docs (Overview)](http://docs.ros2.org/beta2/index.html#) - 关于ROS 2内部设计和组织的细节.
- [ROS 2 Tutorials](https://github.com/ros2/ros2/wiki/Tutorials) - 学习ROS2的概念、库、构建和开发的演示/示例.
- [ROS 2 Wiki](https://github.com/ros2/ros2/wiki) - 查找有关ROS 2的各种信息的入口点.
- [ROS 2 Distribution (rosdistro)](https://github.com/ros2/rosdistro) - 关于发行版和所含软件包的信息.
- [ROS2 package status](http://repo.ros2.org/).
  - [Bouncy package status](http://repo.ros2.org/status_page/ros_bouncy_default.html) - ROS Bouncy软件包的状态.
  - [Ardent package status](http://repo.ros2.org/status_page/ros_ardent_default.html) - ROS2 Ardent软件包的状态.
- [ROS2 Buildfarm](http://build.ros2.org) - 构建信息（Jenkins构建场）.
- [ROS2 CLI cheats sheet](https://github.com/artivis/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf) - ROS 2命令行界面的作弊表.
- [ROS2 Quality Assurance Guidelines](https://github.com/ros-industrial/ros2_quality_assurance_guidelines) - 一套提高软件包质量、遵循REP-2004质量标准和整合持续集成的指南和教程.

## 社区

- [ROS Discourse](https://discourse.ros.org/c/ng-ros)
- [ROS Answers](https://answers.ros.org/questions/scope:all/sort:activity-desc/tags:ROS2/)
- [ROS News](http://www.ros.org/news/)
- [ROS Planet](http://planet.ros.org/)
- [Stack Overflow](https://stackoverflow.com/questions/tagged/ros2)

## 书籍

**尚无书籍出版**

## 课程

- [ROS2 How To: Discover Next Generation ROS (Udemy)](https://www.udemy.com/ros2-how-to/)
- [ROS 2 New Features: Skill-up with the latest features of Robot Operating System 2  (Udemy)](https://www.udemy.com/course/ros-2-new-features/)
- [ROS 2 Basics in 5 Days (C++) - Learn how to start working with ROS 2 (The Construct)](http://www.theconstructsim.com/construct-learn-develop-robots-using-ros/robotigniteacademy_learnros/ros-courses-library/ros2-basics-course/)
- ROS2 Autoware Course
  - [Autoware Course Lecture 1: Development Environment](https://www.youtube.com/watch?v=XTmlhvlmcf8)
  - [Autoware Course Lecture 2: ROS2 101](https://www.youtube.com/watch?v=FTA4Ia2vLS8)
  - [Autoware Course Lecture 3: ROS 2 Tooling - Develop Like a Pro](https://www.youtube.com/watch?v=wcibIqiRb04)
  - [Autoware Course Lecture 4: Platform HW, RTOS and DDS](https://www.youtube.com/watch?v=IyycN6ldsIs)
  - [Autoware Course Lecture 5: Autonomous Driving Stacks](https://www.youtube.com/watch?v=nTI4fnn2tuU)
  - [Autoware Course Lecture 6: Autoware 101](https://www.youtube.com/watch?v=eSHHmJrqpMU)
  - [Autoware Course Lecture 7: Object Perception: LIDAR](https://www.youtube.com/watch?v=xSGCpb24dhI)
  - [Autoware Course Lecture 8: Object Perception: CAMERA](https://www.youtube.com/watch?v=OtjTa-meJ-E)
  - [Autoware Course Lecture 9: Object Perception: Radar](https://www.youtube.com/watch?v=PcVIO-xoNv8)
  - [Autoware Course Lecture 10: State Estimation for Localization](https://www.youtube.com/watch?v=g2YURb-d9vY)
  - [Autoware Course Lecture 11: LGSVL Simulator](https://www.youtube.com/watch?v=OcB6FUbjZXo)
  - [Autoware Course Lecture 12: Motion Control](https://www.youtube.com/watch?v=fQJpAVRQBrI)
- [ROS2-Industrial training material](https://github.com/ros-industrial/ros2_i_training)

## 会议

### ROSCon 2019

[Program announcement](https://roscon.ros.org/2019/#program)(slides + videos)

### ROSCon Fr 2019

[Program announcement](https://roscon.fr/#program)(slides + videos)

### ROS-I EU Spring 2019 Workshop

- Current Status of ROS 2 Hands-on Feature Overview [Slides](https://static1.squarespace.com/static/51df34b1e4b08840dcfd2841/t/5ce6c85ca4222fe0ccbd5309/1558628472094/2019-05-07_Current_Status_of_ROS_2.pdf)

### 2019

- 使用赛灵思和H-ROS的机器人模块化（赛灵思公司） [Video](https://www.xilinx.com/video/events/robot-modularity-with-xilinx-and-h-ros.html)

### ROSCon JP 2018 (english slide presentations only)

- What's next for ROS? (from slide 24 onwards) [Slides](https://roscon.jp/2018/presentations/ROSCon_JP_2018_presentation_2.pdf) [Video](https://vimeo.com/292064161)

### ROSCon 2018

[program announcement](https://roscon.ros.org/2018/#program)

- 实践ROS 2：演练
- 关于自动驾驶汽车的ROS 2
- RViz - 迁移到ROS 2.0的故事
- 为ROS 2启动
- 参与到ROS 2的开发中来
- 计划到计划。一路走来的插件
- 在ROS2中利用DDS的安全性
- Arm DDS安全库：为ROS2增加安全保障
- ROS2：对捷豹4x4进行增压
- 性能测试--通信中间件性能测量的工具
- 适用于安卓、iOS和通用Windows平台的ROS2：展示ROS2的可移植性，以及跨平台和跨语言的能力
- 在基于嵌入式异构平台的混合关键型机器人系统上整合ROS和ROS2
- 争取实现ROS 2微控制器元交叉编译
- 为ROS 2.0准备的Node.js客户端和网桥
- RCLAda：ROS2的Ada客户端库

### 2018年嵌入式世界大会

- ADLink神经元：一个面向工业的基于ROS2的平台 [Slides](https://raw.githubusercontent.com/Adlink-ROS/adlink_neuronbot/master/document/ADLINK_NeuronBot_20180313.pdf) [Video](https://www.youtube.com/watch?v=RC6XvTvTs9Y&feature=youtu.be) [Video](https://www.youtube.com/watch?v=qA4_Hmnd_tM&feature=youtu.be)

### 2018

- ROS2 - 机器人操作系统第二版 (TNG Technology Consulting GmbH) [Slides](https://www.tngtech.com/fileadmin/Public/Images/BigTechday/BTD11/Folien/ROS2.pdf) [Video](https://www.youtube.com/watch?v=6Vzi0Grrlp8)

### 2017年ROS工业会议

- 微型机器人操作系统。用于高度资源受限设备的ROS [Slides](https://static1.squarespace.com/static/51df34b1e4b08840dcfd2841/t/5a3bb6d524a6947d9d0cbc68/1513862873907/07_Losa.pdf)
- ROS2 - it's coming [Slides](https://static1.squarespace.com/static/51df34b1e4b08840dcfd2841/t/5a3bb787e4966b606fe227d7/1513863070599/11_Thomas.pdf)

### ROSCon 2017

- 推进机器人技术发展的未来的ROS 2愿景 [Slides](https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Vision.pdf) [Video](https://vimeo.com/236161417)
- ROS2微调 [Slides](https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Fine%20Tuning.pdf) [Video](https://vimeo.com/236168591)
- 使用ROS2在Turtlebot2上进行SLAM测试 [Slides](https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20SLAM.pdf) [Video](https://vimeo.com/236172294)
- 使用ROS2对工业机器人进行基于视觉的操纵 [Slides](https://roscon.ros.org/2017/presentations/ROSCon%202017%20ROS2%20Vision-Based%20Manipulation.pdf) [Video](https://vimeo.com/236182180)

### 2017

- HyphaROS ROS 2.0介绍 [slides](https://drive.google.com/file/d/1MW_w7MS1DNg1EzhprgbJKY2cqmxksPaw/view)

### 2016年ROS工业会议

- ros 2.0和opc ua：现状更新 [Slides](https://static1.squarespace.com/static/51df34b1e4b08840dcfd2841/t/58235f2eb8a79be587899891/1478713139775/ROS-I-Conf2016-day1-09-keinert.pdf)

### ROSCon 2016

- ROS 2 Update [Slides](https://roscon.ros.org/2016/presentations/ROSCon%202016%20-%20ROS%202%20Update.pdf) [Video](https://vimeo.com/187696091)
- 评估ROS2通信层的复原力 [Slides](https://roscon.ros.org/2016/presentations/rafal.kozik-ros2evaluation.pdf) [Video](https://vimeo.com/187705229)

### ROSCon 2015

- 在 "小型 "嵌入式系统上的ROS 2 [Slides](https://roscon.ros.org/2015/presentations/ros2_on_small_embedded_systems.pdf) [Video](https://vimeo.com/142150576)
- ROS 2的状况--演示和背后的技术 [Slides](https://roscon.ros.org/2015/presentations/state-of-ros2.pdf) [Video](https://vimeo.com/142151734)
- ROS 2中的实时性能 [Slides](https://roscon.ros.org/2015/presentations/RealtimeROS2.pdf) [Video](https://vimeo.com/142621778)

## 论文

- [Distributed and Synchronized Setup towards Real-Time Robotic Control using ROS2 on Linux](https://www.semanticscholar.org/paper/Distributed-and-Synchronized-Setup-towards-Robotic-Puck-Keller/10c4eeef9da0c5aa87664037f18a0ab746853757)
- [Time Synchronization in modular collaborative robots](https://arxiv.org/pdf/1809.07295.pdf)
- [Open Problems in Robotic Anomaly Detection](https://arxiv.org/pdf/1809.03565.pdf)
- [Towards a distributed and real-time framework for robots: Evaluation of ROS 2.0 communications for real-time robotic applications](https://arxiv.org/pdf/1809.02595.pdf)
- [An information model for modular robots: the Hardware Robot Information Model (HRIM)](https://arxiv.org/pdf/1802.01459.pdf)
- [Introducting the Robot Security Framework (RSF), A standardized methodology to perform security assessments in robotics](https://arxiv.org/pdf/1806.04042.pdf)
- [Towards an open standard for assessing the severity of robot security vulnerabilities, The Robot Vulnerability Scoring System (RVSS)](https://arxiv.org/pdf/1807.10357.pdf)
- [Real-Time Characteristics of ROS 2.0 in Multiagent Robot Systems: An Empirical Study](https://www.semanticscholar.org/paper/Real-Time-Characteristics-of-ROS-2.0-in-Multiagent-Park-Delgado/8fa5b9443b33dd20c33be9a4259d92b238310a5c)
- [Response-Time Analysis of ROS 2 Processing Chains Under Reservation-Based Scheduling](https://www.semanticscholar.org/paper/Response-Time-Analysis-of-ROS-2-Processing-Chains-Casini-Bla%C3%9F/6fa472cc45f6de22f2a26114441d595534a80a92)
- [Robot Operating System 2 - The need for a holistic security approach to robotic architectures](http://journals.sagepub.com/doi/pdf/10.1177/1729881418770011) - Ubuntu 16.04, ROS 2 Beta 2/3, and RTI 5.3 DDS with
DDS Security.
- [Maruyama, Yuya et al. “Exploring the performance of ROS2.” 2016 International Conference on Embedded Software (EMSOFT) (2016): 1-10.](https://www.semanticscholar.org/paper/Exploring-the-performance-of-ROS2-Maruyama-Kato/07b895f3b584dea4f64e91844f243de382026b20)

## 播客

- [与HaoChih Lin讨论ROS 2和物联网设备的DDS（从第5分钟开始）](http://www.theconstructsim.com/rdp-017-ros-2-dds-iot-haochih/)
- [与德克-托马斯的ROS 2的一切（从第16分钟开始）](http://www.theconstructsim.com/rdp-012-all-about-ros2-with-dirk-thomas/)

## 服务

### 机器人夺旗(RCTF)

- [rctf-list](https://github.com/aliasrobotics/RCTF) - 机器人CTF（RCTF）方案清单.

## Companies

- [Acutronic Robotics](https://github.com/AcutronicRobotics) - 不复存在. 硬件机器人信息模型（HRIM）、硬件机器人操作系统（H-ROS）的发起者和世界上第一个模块化工业机器人手臂MARA的创造者.
- [ADLINK](https://www.adlinktech.com/en/index.aspx) - "领先的边缘计算机".
- [Alias Robotics](https://aliasrobotics.com/) - 机器人网络安全方面的产品和服务.
- [Amazon](https://github.com/aws-robotics) - 亚马逊的机器人团队 亚马逊网络服务（AWS）.
- [Apex.AI](https://www.apex.ai/) -  "安全和经过认证的自主流动软件".
- [AutonomouStuff](https://autonomoustuff.com) - "自主化系统和解决方案的世界领导者".
- [Bosch](https://github.com/boschresearch) - 博世研究院的机器人团队.
- [Canonical](https://canonical.com/) - 乌班图背后的公司.
- [Eprosima](https://www.eprosima.com/) - "中间件专家".
- [Ericsson Research](https://discourse.ros.org/t/transport-priority-qos-policy-to-solve-ip-flow-ambiguity-while-requesting-5g-network-qos/15332) - 将ROS2应用程序连接到5G网络，用于M2M通信.
- [FARobot](https://www.farobottech.com/) - 集群机器人系统，一个基于ROS 2/DDS的车队管理系统.
- [Fraunhofer Institute for Manufacturing Engineering and Automation IPA](https://www.ipa.fraunhofer.de/en/expertise/robot-and-assistive-systems.html) - 机器人和辅助系统.
- [GESTALT ROBOTICS](https://www.gestalt-robotics.com/en/home) - 智能自动化的服务提供商.
- [Husarnet](https://husarnet.com) - 开源的、P2P的、低延迟的机器人专用VPN.
- [iRobot](https://www.irobot.de/) - 吸尘和拖地机器人制造商.
- [Klepsydra Technologies](https://www.klepsydra.com/).
- [MathWorks](https://de.mathworks.com/help/ros/index.html) - ROS Toolbox.
- [Mission Robotics](https://missionrobotics.us/) - 用于海洋智能新时代的硬件和软件.
- [Roboception GmbH](https://roboception.com/en/) - 为您的机器人提供实时感知.
- [ROBOOX](https://roboox.co/) - 消费机器人的开源软件生态系统.
- [Rover Robotics](https://roverrobotics.com/) - 坚固的工业级机器人.
- [Sony Corporation](https://www.sony.net/SonyInfo/technology/element/robotics/).
- [synapticon](https://www.synapticon.com/technology) - 兼容ROS的运动控制和驱动产品，努力支持ROS2.
- [Wind River](https://labs.windriver.com/ros2-for-vxworks/) - 用于VxWorks的ROS2.

## 组织

- [U.S. Department of Transportation](https://discourse.ros.org/t/carma-migrating-to-ros-2-with-cyclonedds-and-zenoh/17541)

## 工作组

- 边缘人工智能工作小组
  - [Discourse threads tagged "wg-edgeai"](https://discourse.ros.org/tag/wg-edgeai)
- 嵌入式工作组
  - [Discourse threads tagged "wg-embedded"](https://discourse.ros.org/tags/wg-embedded)
- 硬件加速工作组
  - [Discourse threads tagged "wg-acceleration"](https://discourse.ros.org/tag/wg-acceleration)
- 导航工作小组
  - [Discourse threads tagged "wg-navigation"](https://discourse.ros.org/tags/wg-navigation)
- 安全工作小组
  - [Safety Working Group Landing Page](http://www.ros2.org/safety_working_group/)
  - [Safety Design Pattern Catalogue](http://www.ros2.org/safety_working_group/safety_patterns_catalogue.html)
- 安全工作小组
  - [Discourse threads tagged "wg-security"](https://discourse.ros.org/tags/wg-security)
  - [ros-security/community](https://github.com/ros-security/community) - 概述了ROS 2安全工作组的治理情况.
- 技术指导委员会
  - [Discourse threads tagged "tsc"](https://discourse.ros.org/tags/tsc)
- 工具工作小组
  - [Discourse threads tagged "wg-tooling"](https://discourse.ros.org/tags/wg-tooling)

## 授权

[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
