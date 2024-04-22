from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
 
# Define the missing variable
def generate_launch_description():
    #Declare the launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model',
            default_value='vx300s',     #default robot model is vx300s
            description='Robot model name'
        ),
        DeclareLaunchArgument(
            'hardware_type',
            default_value='fake',       #default hardware type is fake
            description='Hardware type'
        ),

    ]
    #Include the xsarm_moveit.launch.py
    xsarm_moveit_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('interbotix_xsarm_moveit'),
                'launch',
                'xsarm_moveit.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'hardware_type': LaunchConfiguration('hardware_type'),
        }.items(),
    )
    #Create the viperx_controller_node
    viperx_controller_node = Node(
        package='viperxcontroller',
        executable='viperctrl',
        output='screen',
    )

    return LaunchDescription(
        declared_arguments +
        [
            xsarm_moveit_launch_include,
            viperx_controller_node
        ]
    )
