#   导入头文件
from launch import LaunchDescription
from launch_ros.actions import Node

#   定义
def generate_launch_description():
    #   创建节点描述
    velo_pub_node = Node(
        package = "agv_pub",
        executable = "velo_pub"
    )

    serial_node = Node(
        package = "serial_test",
        executable = "serial_node",
        )

    #   launch_description
    launch_d = LaunchDescription([velo_pub_node, serial_node])

    return launch_d;
