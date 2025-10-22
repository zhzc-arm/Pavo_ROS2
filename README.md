1. Modify Starsecond LiDAR ROS1 to ROS2
2. Download File ~/pave_ws/src
3. Compile files “colcon build --packages-select pavo_ros”
4. Run LiDAR
   ros2 launch pavo_ros pavo_scan_launch.py
5. View ROS information
   ros2 topic list
   ros2 topic echo /cloud
6. Run pavo_scan.rviz file
   ros2 run rviz2 rviz2 -d /path/to/pavo_scan.rviz


