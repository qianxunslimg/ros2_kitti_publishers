# ROS2 Kitti Publishers

## 概述
ROS2 Kitti Publishers 是一个 ROS 2 包，提供了 KITTI 数据集文件的发布器，包括点云、图像、里程计、IMU 和 GPS 数据。该包被设计用于与 KITTI 数据集配合使用，允许用户将数据集的数据发布为 ROS 2 消息。

## 先决条件
- ROS 2：该包已在 ROS 2 Foxy Fitzroy 上进行测试。
- OpenCV：用于图像处理。
- PCL：用于点云处理。

## 快速开始
1. 将此存储库克隆到您的 ROS 2 工作空间。
2. 构建您的 ROS 2 工作空间。
3. 运行以下命令启动 ROS 2 Kitti Publishers：

```bash
colcon build
source install/setup.bash
ros2 launch ros2_kitti_publishers kitti_publishers.launch.py
```

## 配置文件
1. default.rviz：RViz2 的配置文件，用于可视化发布的数据。
2. kitti_rqt.perspective：RQt 的配置文件，用于可视化发布的数据。
## 节点
- kitti_publishers：发布 KITTI 数据集文件的节点。
- rviz2：启动 RViz2 可视化工具的节点。
- rqt：启动 RQt 可视化工具的节点。