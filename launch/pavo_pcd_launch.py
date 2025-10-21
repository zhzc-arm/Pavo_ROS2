from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('enable_motor', default_value='true'),
        DeclareLaunchArgument('method', default_value='0'),

        # Launch PavoPcdNode
        Node(
            package='pavo_ros',
            executable='pavo_pcd_node',
            name='PavoPcdNode',
            output='screen',
            parameters=[{
                'frame_id': 'pcd_frame',                    # 设置Pointcloud的id, default=pcd_frame
                'cloud_topic': 'cloud',                     # 设置Pointcloud的topic, default=cloud
                'angle_min': -135.0,                        # 最小角度，单位度，范围 [-135.0,135.0]
                'angle_max': 135.0,                         # 最大角度，单位度，范围 [-135.0,135.0]
                'inverted': False,                          # 是否倒装雷达，true倒装，false正装
                'enable_motor': LaunchConfiguration('enable_motor'),  # 雷达启动/停止
                'motor_speed': 15,                          # 可配10,15,20,25Hz
                'merge_coef': 2,                            # 可配1,2,4,8点合并
                'lidar_ip': '10.10.10.6',                 # 雷达IP地址
                'lidar_port': 2368,                         # 雷达端口
                'host_ip: 10.10.10.100'
                'method': LaunchConfiguration('method'),    # 尾部滤波方法
                'switch_active_mode': False                 # 是否使用旧版本主机绑定
                # 'host_ip': '10.10.10.100',                # 可选：绑定网卡IP
                # 'host_port': 2368                         # 可选：绑定端口
            }]
        )
    ])

