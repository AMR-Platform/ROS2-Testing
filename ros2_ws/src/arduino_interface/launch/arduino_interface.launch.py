#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    base_width_arg = DeclareLaunchArgument(
        'base_width',
        default_value='0.3',
        description='Distance between wheels in meters'
    )
    
    ticks_per_meter_arg = DeclareLaunchArgument(
        'ticks_per_meter',
        default_value='4000',
        description='Encoder ticks per meter of wheel rotation'
    )
    
    # Create node
    arduino_node = Node(
        package='arduino_interface',
        executable='arduino_serial_node',
        name='arduino_interface',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'base_width': LaunchConfiguration('base_width'),
            'ticks_per_meter': LaunchConfiguration('ticks_per_meter'),
        }]
    )
    
    return LaunchDescription([
        port_arg,
        baud_arg,
        base_width_arg,
        ticks_per_meter_arg,
        arduino_node
    ])