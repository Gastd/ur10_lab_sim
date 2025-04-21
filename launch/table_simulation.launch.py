from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = "ur10e"

    world_path = os.path.join(
        get_package_share_directory('ur10_lab_sim'),
        # 'worlds/empty.sdf')
        'worlds/table_toys.sdf')

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur10_lab_sim"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
            "world_file": world_path,
            "x": '0.2',
            "y": '0.0',
            "z": '1.02',
            "Y":'-1.57',
        }.items(),
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    nodes_to_launch = [
        ur_control_launch,
        ur_moveit_launch,
    ]

    return nodes_to_launch

def generate_launch_description():

    return LaunchDescription(
        # declared_arguments + 
        [OpaqueFunction(function=launch_setup)]
    )
