import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 声明启动参数，用于设置rviz2启动文件路径
    rviz_cfg_path = os.path.join(get_package_share_directory('ros2_kitti_publishers'), 'config', 'default.rviz')
    # 启动rviz2
    rviz_runner = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        #condition=IfCondition(LaunchConfiguration('rviz'))
    )

    stata_pub_node = Node(
        package='ros2_kitti_publishers',
        executable='kitti_publishers',
        name='kitti_publishers',
        output='screen',
    )


    rviz_cfg_path = os.path.join(get_package_share_directory('ros2_kitti_publishers'), 'config', 'kitti_rqt.perspective')
    rqt_cmd = "rqt --perspective-file "+rviz_cfg_path
    rqt_node = ExecuteProcess(
        cmd=['bash', '-c', rqt_cmd],
        output='screen'
    )

    return LaunchDescription([
        stata_pub_node,
        rviz_runner,
        rqt_node,
    ])
