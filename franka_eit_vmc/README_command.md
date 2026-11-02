ros2 run franka_eit_vmc eit_contact_node.py
ros2 run franka_eit_vmc eit_pad_viewer.py
ros2 launch franka_eit_vmc controller.launch.py task:=joystick   config:=$EIT_CONFIG_DIR/controller.yaml   overrides:=$EIT_CONFIG_DIR/joystick_x_minus.yaml
ros2 run franka_eit_vmc run_block.py --task joystick --block x_minus   --mechanism inward_arc --inward-arc-m 0.001 --axis x   --repeats 1 --output ~/eit_trials


python3 eit_position_toolkit/contact_position_app.py   --points 30 --repeats 1   --pins 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16   --port /dev/ttyACM0   --drive-pattern adjacent