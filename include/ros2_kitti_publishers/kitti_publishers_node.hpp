#ifndef ROS2_KITTI_PUBLISHERS__KITTI_PUBLISHERS_NODE_HPP_
#define ROS2_KITTI_PUBLISHERS__KITTI_PUBLISHERS_NODE_HPP_

// 包括所需的ROS 2头文件
#include "rclcpp/rclcpp.hpp" // ROS 2 C++ 客户端库的主要头文件

// 包括所需的ROS 2消息类型的头文件
#include "std_msgs/msg/string.hpp"          // std_msgs/String消息类型的头文件
#include <sensor_msgs/msg/point_cloud2.hpp> // sensor_msgs/PointCloud2消息类型的头文件

// 包括将点云数据转换为ROS 2消息格式的头文件
#include <pcl_conversions/pcl_conversions.h> // PCL转换ROS 2消息格式的头文件

// 包括进行几何变换的ROS 2库的头文件
#include <tf2/LinearMath/Quaternion.h>             // 用于进行四元数变换的头文件
#include <tf2_ros/transform_broadcaster.h>         // 用于广播变换的头文件
#include <geometry_msgs/msg/transform_stamped.hpp> // geometry_msgs/TransformStamped消息类型的头文件

// 包括可视化消息类型的头文件
#include <visualization_msgs/msg/marker_array.hpp> // visualization_msgs/MarkerArray消息类型的头文件
#include <visualization_msgs/msg/marker.hpp>       // visualization_msgs/Marker消息类型的头文件

// 包括GPS和IMU传感器消息类型的头文件
#include <sensor_msgs/msg/nav_sat_fix.hpp> // sensor_msgs/NavSatFix消息类型的头文件
#include <sensor_msgs/msg/imu.hpp>         // sensor_msgs/Imu消息类型的头文件

// 包括OpenCV库的头文件
#include <opencv2/opencv.hpp>          // OpenCV库的主要头文件
#include <opencv2/highgui/highgui_c.h> // OpenCV图像显示的头文件

// 包括C++标准库的头文件
#include <fstream>    // 文件输入输出流的头文件
#include <filesystem> // 文件系统操作的头文件
#include <vector>     // 向量容器的头文件
#include <string>     // 字符串处理的头文件
#include <cstdlib>    // 通用工具的头文件

#include "ros2_kitti_publishers/visibility.h"         // 自定义的ROS 2节点的可见性宏定义的头文件
#include "ros2_kitti_publishers/WGS84toCartesian.hpp" // WGS84坐标转换函数的头文件

class KittiPublishersNode : public rclcpp::Node
{
public:
  // 发布器类型的枚举类
  enum class PublisherType
  {
    POINT_CLOUD = 0,       // 点云发布器类型
    IMAGE_LEFT_GRAY = 1,   // 左侧灰度图像发布器类型
    IMAGE_RIGHT_GRAY = 2,  // 右侧灰度图像发布器类型
    IMAGE_LEFT_COLOR = 3,  // 左侧彩色图像发布器类型
    IMAGE_RIGHT_COLOR = 4, // 右侧彩色图像发布器类型
    ODOMETRY = 5           // 里程计发布器类型
  };

  KITTI_PUBLISHERS_NODE_PUBLIC KittiPublishersNode(); // 节点的构造函数

  std::string get_path(PublisherType publisher_type);                                    // 获取指定发布器类型的路径
  std::vector<std::string> get_filenames(PublisherType publisher_type);                  // 获取指定发布器类型的文件名列表
  void set_filenames(PublisherType publisher_type, std::vector<std::string> file_names); // 设置指定发布器类型的文件名列表

private:
  void on_timer_callback(); // 定时器回调函数

  void init_file_path();                                                                                    // 初始化文件路径
  void create_publishers_data_file_names();                                                                 // 创建发布器数据文件名列表
  std::vector<std::string> parse_file_data_into_string_array(std::string file_name, std::string delimiter); // 将文件数据解析为字符串数组

  std::string mat_type2encoding(int mat_type);                                     // 将OpenCV图像类型转换为编码字符串
  void convert_image_to_msg(sensor_msgs::msg::Image &msg, const std::string path); // 将图像转换为ROS 2消息格式

  // NavSatFix 是 ROS（机器人操作系统）中的一种消息类型。它用于表示全球定位系统（GPS）接收器或类似定位系统获取的定位信息。
  void prepare_navsatfix_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::NavSatFix &msg); // 准备NavSatFix消息

  // IMU 消息是 ROS（机器人操作系统）中的一种消息类型，用于表示惯性测量单元（Inertial Measurement Unit，简称 IMU）获取的姿态和运动信息。
  void prepare_imu_msg(std::vector<std::string> &oxts_tokenized_array, sensor_msgs::msg::Imu &msg); // 准备IMU消息

  // MarkerArray 消息是 ROS（机器人操作系统）中的一种消息类型，用于在三维可视化中发布多个标记（Marker）的集合。
  // MarkerArray 消息由多个 Marker 组成，每个 Marker 表示一个可视化标记，如点、线、箭头、立方体等。每个 Marker 都可以具有不同的位置、姿态、颜色和尺寸等属性。
  void prepare_marker_array_msg(std::vector<std::string> &oxts_tokenized_array, visualization_msgs::msg::MarkerArray &msg); // 准备MarkerArray消息
  
  // PointCloud2 消息是一个灵活的数据结构，可以存储具有不同字段和属性的点云数据。每个点云数据都包含一系列点，
  // 每个点可以具有多个字段，例如位置坐标、颜色、法线等。通过使用 PointCloud2 消息，可以在 ROS 中方便地传输和处理三维点云数据。
  void convert_pcl_to_pointcloud2(sensor_msgs::msg::PointCloud2 &msg);                                                      // 将PCL点云转换为PointCloud2消息

  size_t file_index_; // 文件索引

  rclcpp::TimerBase::SharedPtr timer_; // 定时器

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_;         // Velodyne点云发布器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_gray_left_;           // 左侧矫正的灰度图像序列发布器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_gray_right_;          // 右侧矫正的灰度图像序列发布器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_color_left_;          // 左侧矫正的彩色图像序列发布器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_color_right_;         // 右侧矫正的彩色图像序列发布器
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_odometry_;                  // OXTS里程计发布器
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;                         // OXTS IMU数据发布器
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_nav_sat_fix_;           // OXTS GPS数据发布器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker_array_; // OXTS标记数组发布器

  std::vector<std::string> file_names_point_cloud_;       // 点云文件名列表
  std::vector<std::string> file_names_image_gray_left_;   // 左侧灰度图像文件名列表
  std::vector<std::string> file_names_image_gray_right_;  // 右侧灰度图像文件名列表
  std::vector<std::string> file_names_image_color_left_;  // 左侧彩色图像文件名列表
  std::vector<std::string> file_names_image_color_right_; // 右侧彩色图像文件名列表
  std::vector<std::string> file_names_oxts_;              // OXTS文件名列表

  std::string path_point_cloud_;       // 点云文件路径
  std::string path_image_gray_left_;   // 左侧灰度图像文件路径
  std::string path_image_gray_right_;  // 右侧灰度图像文件路径
  std::string path_image_color_left_;  // 左侧彩色图像文件路径
  std::string path_image_color_right_; // 右侧彩色图像文件路径
  std::string path_oxts_;              //
};

#endif // ROS2_KITTI_PUBLISHERS__KITTI_PUBLISHERS_NODE_HPP_
