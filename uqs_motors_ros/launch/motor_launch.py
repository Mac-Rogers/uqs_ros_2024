from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    motor_control_node = Node(
        package="uqs_motors_ros",
        executable="motor_control",
        name="Motor_Control",
    )

    motor_interpreter_node = Node(
        package="uqs_motors_ros",
        executable="motor_interpreter",
        name="Motor_Interpreter",
    )


    ld.add_action(motor_control_node)
    ld.add_action(motor_interpreter_node)

    return ld

