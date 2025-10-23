from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # RViz 配置文件路径
    rviz_config_file = os.path.join(
        os.environ['HOME'],
        'CSU-RM-Sentry/src/rm_perception/pointcloud_merger/config/display_point_cloud.rviz'
    )

    return LaunchDescription([
        # 点云合并节点
        Node(
            package='pointcloud_merger',
            executable='pointcloud_merger',
            name='pointcloud_merger',
            output='screen',
        ),

        # # RViz2 可视化
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_file],
        # )
    ])
