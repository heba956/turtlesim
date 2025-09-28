from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Launch the turtlesim simulator
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim_node'
        ),

        # 2. Launch the shapeNode (for user input)
        Node(
            package='turtle_python',
            executable='shape_starter',  # This name must match entry_point in setup.py
            name='shape_node',
            output='screen',
            prefix='xterm -e'  # Forces the node to run in its own terminal for input
        ),

        # 3. Launch the turtleCommander (for movement logic)
        Node(
            package='turtle_python',
            executable='commander_starter',  # This name must match entry_point in setup.py
            name='turtle_commander',
            output='screen'
        ),
    ])
