#   导入头文件
from launch import LaunchDescription
from launch_ros.actions import Node

#   定义
def generate_launch_description():
    #   创建节点描述
    transform_node = Node(
        package = "laser_to_pc2",
        executable = "laser_to_pc2_node",
        output="screen",
        parameters=[
                {"pub_name": "pointcloud2_pub"},
                {"sub_name": "scan"},
                {"frame_id": "laser_frame"},
            ]
    )

    #   launch_description
    launch_d = LaunchDescription([transform_node])

    return launch_d;
