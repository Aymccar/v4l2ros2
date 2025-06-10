from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    video_writer_node = Node(
        package='v4l2ros2',
        executable='video_writer',
        name='video_writer',
        namespace='SeaVid',
        output='screen'
    )

    # Delay the script execution by 5 seconds
    delayed_script = TimerAction(
        period=5.0,  # seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c', 
                    '$(ros2 pkg prefix v4l2ros2)/lib/v4l2ros2/script.sh'
                ],
                shell=True,
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        video_writer_node,
        delayed_script
    ])

