from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Adjust these defaults if your hardware is different
    env = [
        SetEnvironmentVariable('MBOT_SERIAL_PORT', '/dev/ttyUSB0'),
        SetEnvironmentVariable('MBOT_BAUDRATE', '9600'),
        SetEnvironmentVariable('MBOT_MOVE_TOPIC', '/mbuff/move'),
        SetEnvironmentVariable('MBOT_LOG_LEVEL', 'DEBUG'),     # INFO or DEBUG
        SetEnvironmentVariable('MBOT_READBACK_MS', '0'),       # e.g. 200 if your firmware echoes
    ]

    # If web_bridge already publishes to /mbuff/move, you can drop the remapping.
    web_bridge = Node(
        package='web_bridge_node',
        executable='web_bridge',
        name='web_bridge',
        output='screen',
        # Remap 'move' -> '/mbuff/move' in case your bridge uses a plain 'move' topic
        remappings=[('move', '/mbuff/move')],
    )

    msense_hat = Node(
        package='msense_hat_node',
        executable='msense_hat',
        name='msense_hat',
        output='screen',
    )

    mbot_controller = Node(
        package='mbot_controller_node',
        executable='mbot_controller',
        name='mbot_controller',
        output='screen',
        # If you prefer ROS args instead of env for log level, uncomment:
        # arguments=['--ros-args', '--log-level', 'mbot_controller:=debug'],
    )

    return LaunchDescription(env + [msense_hat, web_bridge, mbot_controller])
