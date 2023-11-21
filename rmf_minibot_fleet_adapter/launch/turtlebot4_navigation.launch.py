import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav2_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            '/home/siwon/turtlebot4_ws/src/turtlebot4_navigation/launch/nav2.launch.py'
        ),
        launch_arguments={'namespace': 'turtlebot4_lite_1'}.items()
    )

    localization_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            '/home/siwon/turtlebot4_ws/src/turtlebot4_navigation/launch/localization.launch.py'
        ),
        # Combine both launch_arguments into a single dictionary
        launch_arguments={
            'map': '/home/siwon/rmf_ws/src/rmf_demos/rmf_demos_maps/maps/pinklab/pinklab.yaml',
            'namespace': 'turtlebot4_lite_1'
        }.items()
    )

    return LaunchDescription([
        localization_launch,
        nav2_launch
    ])
