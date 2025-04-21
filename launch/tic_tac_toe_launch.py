# launch/tic_tac_toe_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # Start draw grid node
        Node(
            package='tic_tac_toe',
            executable='draw_grid',
            name='draw_grid'
        ),
        
        # Start game logic node
        Node(
            package='tic_tac_toe',
            executable='game_logic',
            name='game_logic'
        ),
        
        # Start player controller node
        Node(
            package='tic_tac_toe',
            executable='player_controller',
            name='player_controller'
        ),
    ])
