from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    nodes = [
        ComposableNode(
            package='mpcc_ros',         
            plugin='MPCCSim',
            name='mpcc_sim',
            namespace='',
            parameters=[],
        ),
        ComposableNode(
            package='mpcc_ros',
            plugin='MPCCOne',
            name='mpcc_one',
            namespace='',
            parameters=[],
        ),
        ComposableNode(
            package='mpcc_ros',
            plugin='MPCCTwo',
            name='mpcc_two',
            namespace='',
            parameters=[],
        ),
    ]

    container = ComposableNodeContainer(
        name='mpcc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # 멀티스레드 컨테이너
        composable_node_descriptions=nodes,
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([container])
