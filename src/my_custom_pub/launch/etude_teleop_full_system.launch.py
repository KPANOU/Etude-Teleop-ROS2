from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # --- ACCELERATOR CLEAN ---
        Node(
            package='my_custom_pub',
            executable='subscriber_accel',
            name='subscriber_accel',
            output='screen'
        ),

        # --- BRAKE CLEAN ---
        Node(
            package='my_custom_pub',
            executable='brake_subscriber',
            name='brake_subscriber',
            output='screen'
        ),

        # --- STEERING CLEAN ---
        Node(
            package='my_custom_pub',
            executable='subscriber_steering',
            name='subscriber_steering',
            output='screen'
        ),

        # --- DIRECTION CLEAN ---
        Node(
            package='my_custom_pub',
            executable='direction_subscriber',
            name='direction_subscriber',
            output='screen'
        ),

        # --- GPIO BRAKE ---
        Node(
            package='my_custom_pub',
            executable='gpio_brake',
            name='gpio_brake',
            output='screen'
        ),

        # --- GPIO DIRECTION ---
        Node(
            package='my_custom_pub',
            executable='gpio_direction',
            name='gpio_direction',
            output='screen'
        ),

        # --- STEERING TO SERIAL ---
        Node(
            package='my_custom_pub',
            executable='steering_to_serial',
            name='steering_to_serial',
            output='screen'
        ),

        # --- MOTOR_BRIDGE (si utilisé) ---
        Node(
            package='my_custom_pub',
            executable='motor_bridge',
            name='motor_bridge',
            output='screen'
        ),
    ])

