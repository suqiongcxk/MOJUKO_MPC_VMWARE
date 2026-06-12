import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_type', default_value='creeper', description='Robot type: [b1, creeper]'
        ),

        launch.actions.DeclareLaunchArgument(
            name='xmlFile',
            default_value=PathJoinSubstitution([
                get_package_share_directory('mujoco_simulator'),
                'models', LaunchConfiguration('robot_type'), 'urdf', 'robot.xml'
            ])
        ),

        launch.actions.DeclareLaunchArgument(
            name='simulatorFile',
            default_value=PathJoinSubstitution([
                get_package_share_directory('user_command'),
                'config', LaunchConfiguration('robot_type'), 'simulation.info'
            ])
        ),

        Node(
            package='mujoco_simulator',
            executable='mujoco_simulator',
            name='mujoco_simulator',
            output='screen',
            parameters=[
                {'xmlFile': launch.substitutions.LaunchConfiguration('xmlFile')},
                {'simulatorFile': launch.substitutions.LaunchConfiguration('simulatorFile')}
            ],
        ),
    ])
