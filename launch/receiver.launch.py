from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

qos_names = [
    'reliable_volatile',
    'best_effort_volatile',
    'reliable_transient_local',
    'best_effort_transient_local',
    'reliable_keep_all',
    'best_effort_keep_all'
]

def create_node(context):
    enclave = LaunchConfiguration('enclave').perform(context)

    nodes = []

    for i, qos in enumerate(qos_names):
        args = []

        if enclave:
            args = ['--ros-args','--enclave', enclave]

        nodes.append(Node(
            package='ros2_bench',
            executable='receiver_node',
            name=f'receiver_{i+1}_{qos}',
            arguments=args,
            #Enumerate starts at 0 so +1 to keep the nodes within 1-6 in this case
            parameters=[{'qos_profile': qos}, {'topic_int': i+1}],
            output='screen'
        ))
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enclave',
            default_value='',
            description='Optional path to a security enclave'
        ),
        OpaqueFunction(function=create_node)
    ])