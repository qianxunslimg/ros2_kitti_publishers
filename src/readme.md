# Kitti Publishers Node
该节点是一个ROS2节点，用于发布KITTI数据集中的各种消息类型，如点云数据、图像数据、IMU数据和导航数据。

## 依赖
该节点依赖以下库和软件包：
- ROS2
- OpenCV
- PCL
- 安装
要安装该节点，请按照以下步骤进行操作：

1. 下载并安装ROS2。
2. 安装OpenCV和PCL库。
3. 克隆该节点的源代码。

```shell
$ git clone https://github.com/your_repository.git
```
4. 编译源代码。
```shell
cd your_repository
colcon build
```
## 运行节点
要运行该节点，请执行以下命令：

```shell
source /path/to/ros2/install/setup.bash
ros2 run kitti_publishers_node kitti_publishers_node
```
## 发布的消息
该节点发布以下消息类型：
- /kitti/point_cloud：点云数据 (sensor_msgs/PointCloud2)
- /kitti/image/gray/left：左灰度图像 (sensor_msgs/Image)
- /kitti/image/gray/right：右灰度图像 (sensor_msgs/Image)
- /kitti/image/color/left：左彩色图像 (sensor_msgs/Image)
- /kitti/image/color/right：右彩色图像 (sensor_msgs/Image)
- /kitti/imu：IMU数据 (sensor_msgs/Imu)
- /kitti/nav_sat_fix：导航卫星修复数据 (sensor_msgs/NavSatFix)
- /kitti/marker_array：标记数组数据 (visualization_msgs/MarkerArray)
## 参数
该节点有以下参数可供配置：
- path_point_cloud_：点云数据文件路径
- path_image_gray_left_：左灰度图像文件路径
- path_image_gray_right_：右灰度图像文件路径
- path_image_color_left_：左彩色图像文件路径
- path_image_color_right_：右彩色图像文件路径
- path_oxts_：导航数据文件路径