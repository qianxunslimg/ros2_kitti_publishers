```bash
colcon build --cmake-clean-cache
source ./install/setup.bash
ros2 launch ros2_kitti_publishers launch.py
```
the directory data should put in the same file where execute colcon build