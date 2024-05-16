from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="master_node_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="kalman_arm_controller",
                plugin="kalman_master::CanMasterNode",
                name="can_master_node",
            ),
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            container,
        ]
    )
