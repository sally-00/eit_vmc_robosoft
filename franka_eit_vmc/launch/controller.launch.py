"""Launch ONE direct-FCI executable for the physical robot."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,RegisterEventHandler,EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def setup(context):
    task=LaunchConfiguration('task').perform(context)
    executables={'joystick':'joystick_controller','ball':'ball_roll_controller','ready':'move_ready'}
    if task not in executables:raise ValueError('task must be joystick, ball, or ready')
    config_path=Path(LaunchConfiguration('config').perform(context)).expanduser()
    paths=[('config',str(config_path))]
    if task!='ready':
        task_file='joystick.yaml' if task=='joystick' else 'ball.yaml'
        paths.append(('task defaults',str(config_path.parent/task_file)))
    for name in ['mount','posture','overrides']:
        path=LaunchConfiguration(name).perform(context)
        if path:paths.append((name,path))
    for name,path in paths:
        if not Path(path).expanduser().is_file():
            raise RuntimeError(f'{name} parameter file does not exist: {path}. '
                               'Set EIT_CONFIG_DIR to the franka_eit_vmc/config directory in this terminal.')
    parameters=[str(Path(path).expanduser()) for _,path in paths]
    controller=Node(package='franka_eit_vmc',executable=executables[task],output='screen',
                    parameters=parameters)
    with_eit=LaunchConfiguration('with_eit').perform(context).lower()
    if with_eit not in ('true','false'):
        raise ValueError('with_eit must be true or false')
    if with_eit=='false':return [controller]
    model=LaunchConfiguration('eit_model').perform(context).strip()
    sensor_config=LaunchConfiguration('eit_config').perform(context).strip()
    if not sensor_config:sensor_config=str(config_path.parent/'eit.json')
    if not Path(sensor_config).expanduser().is_file():
        raise RuntimeError('EIT configuration does not exist: '+sensor_config)
    if model and not Path(model).expanduser().is_file():
        raise RuntimeError('EIT model does not exist: '+model)
    sensor_arguments=['--config',sensor_config]
    if model:sensor_arguments.extend(['--model',model])
    sensor=Node(package='franka_eit_vmc',executable='eit_contact_node.py',output='screen',
                arguments=sensor_arguments)
    # An exit on either side tears down the shared session. Restarting the
    # launch gives both processes the same installed ROS libraries.
    handlers=[RegisterEventHandler(OnProcessExit(target_action=action,
              on_exit=[EmitEvent(event=Shutdown(reason=reason))]))
              for action,reason in ((controller,'Controller exited; stop EIT session'),
                                    (sensor,'EIT node exited; stop controller session'))]
    return [*handlers,sensor,controller]

def generate_launch_description():
    share=Path(get_package_share_directory('franka_eit_vmc'))
    return LaunchDescription([
        DeclareLaunchArgument('task',default_value='joystick'),
        DeclareLaunchArgument('config',default_value=str(share/'config/controller.yaml')),
        DeclareLaunchArgument('mount',default_value=''),
        DeclareLaunchArgument('posture',default_value=''),
        DeclareLaunchArgument('overrides',default_value=''),
        DeclareLaunchArgument('with_eit',default_value='false'),
        DeclareLaunchArgument('eit_model',default_value=''),
        DeclareLaunchArgument('eit_config',default_value=''),
        OpaqueFunction(function=setup)])
