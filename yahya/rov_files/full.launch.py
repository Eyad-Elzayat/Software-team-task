from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # The display is NOT here: it rewrites one terminal line, so run it in its own terminal.
    return LaunchDescription([
        Node(package='rov_link', executable='udp_bridge', name='udp_bridge'),
        Node(package='rov_link', executable='sensor_sim', name='sensor_sim'),
        Node(package='thruster_ctl', executable='joy_to_pilot', name='joy_to_pilot'),
        Node(package='thruster_ctl', executable='mode_manager', name='mode_manager', output='screen'),
        Node(package='thruster_ctl', executable='tam_mixer', name='tam_mixer'),
        Node(package='thruster_ctl', executable='slew_limiter', name='slew_limiter'),
    ])
