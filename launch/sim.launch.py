import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('vacuum_robot_sim')
    urdf_file = os.path.join(pkg_path, 'urdf', 'vacuum_robot.urdf')
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_file])
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),)
    
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', urdf_file, '-name', 'vacuum_robot', '-z', '0.1'],
        output='screen')
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={os.path.join(pkg_path, "config", "bridge.yaml")}'])
    
    return LaunchDescription([
        robot_state_pub,
        gz_sim,
        spawn_robot,
        bridge])
