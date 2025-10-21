from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('enable_motor', default_value='true'),
        DeclareLaunchArgument('method', default_value='0'),

        # Launch PavoScanNode
        Node(
            package='pavo_ros',
            executable='pavo_scan_node',
            name='PavoScanNode',
            output='screen',
            remappings=[
                ('/pavo_node/scan', '/scan')  # Topic remapping
            ],
            parameters=[{
                'frame_id': 'laser_frame',                      # 配置frame_id
                'scan_topic': 'scan',                           # 激光雷达的topic名称
                'angle_min': -135.0,                            # 最小角度，单位度
                'angle_max': 135.0,                             # 最大角度，单位度
                'range_min': 0.10,                              # 最小距离，单位米
                'range_max': 50.0,                              # 最大距离，单位米
                'inverted': False,                              # 雷达是否倒装
                'enable_motor': LaunchConfiguration('enable_motor'),  # 雷达启动/停止
                'motor_speed': 15,                              # 扫描频率
                'merge_coef': 2,                                # 点合并系数
                'lidar_ip': '10.10.10.6',                     # 雷达IP地址
                'lidar_port': 2368,                             # 雷达端口
                'method': LaunchConfiguration('method'),        # 去除拖尾方法
                'switch_active_mode': False,                     # 是否使用旧版本主机绑定
                'host_ip': '10.10.10.100',                    # 可选：绑定网卡IP
                # 'host_port': 2368                             # 可选：绑定端口
            }]
        )
    ])

