from struct import pack
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def my_function(context):
    node_num = LaunchConfiguration('numbers').perform(context)

    num_list = node_num.split()
    
    pkg_share_dir = get_package_share_directory('vessel_det')
    yaml_path = pkg_share_dir + '/configs/config.yaml'


    ld = []

    for i in num_list:
        ld.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('mbzirc_ign'),
                        'launch',
                        'spawn.launch.py'
                    ])
                ]),
                launch_arguments={
                    'name': 'suav_' + i,
                    'world': 'coast',
                    'model': 'mbzirc_quadrotor',
                    'x': str(-1490 - 2 * int(i)),
                    'y': '0',
                    'z': '4.3',
                    'R': '0',
                    'P': '0',
                    'Y': '0',
                    'slot0': 'mbzirc_hd_camera',
                    'slot0_rpy': '0 30 0',
                    'gripper': 'mbzirc_suction_gripper',
                    'flightTime': '60'
                }.items()
            )
            
        )
    
    return ld


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
        "numbers", default_value=TextSubstitution(text="1 2 3 4 5 6 7 8 9 10")
        ),
        OpaqueFunction(function=my_function)
    ])