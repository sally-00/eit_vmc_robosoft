"""Launch ONE direct-FCI executable; no robot connection unless explicitly enabled."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def setup(context):
    share=Path(get_package_share_directory('franka_eit_vmc'))
    task=LaunchConfiguration('task').perform(context)
    executables={'joystick':'joystick_controller','ball':'ball_roll_controller','ready':'move_ready'}
    if task not in executables:raise ValueError('task must be joystick, ball, or ready')
    paths=[LaunchConfiguration('config').perform(context)]
    if task!='ready':paths.append(str(share/'config'/('joystick.yaml' if task=='joystick' else 'ball.yaml')))
    for name in ['mount','posture','overrides']:
        path=LaunchConfiguration(name).perform(context)
        if path:paths.append(path)
    enabled=LaunchConfiguration('connect_robot').perform(context).lower()=='true'
    return [Node(package='franka_eit_vmc',executable=executables[task],output='screen',parameters=paths+[{'connect_robot':enabled}])]

def generate_launch_description():
    share=Path(get_package_share_directory('franka_eit_vmc'))
    return LaunchDescription([
        DeclareLaunchArgument('task',default_value='joystick'),
        DeclareLaunchArgument('config',default_value=str(share/'config/controller.yaml')),
        DeclareLaunchArgument('mount',default_value=''),
        DeclareLaunchArgument('posture',default_value=''),
        DeclareLaunchArgument('overrides',default_value=''),
        DeclareLaunchArgument('connect_robot',default_value='false'),OpaqueFunction(function=setup)])
