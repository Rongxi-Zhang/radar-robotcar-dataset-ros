# rviz2
import os
import launch
import launch_ros


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='radar_robotcar_dataset_ros2').find('radar_robotcar_dataset_ros2')
    rviz_path = os.path.join(pkg_share, 'launch/rviz2/robotcar.rviz')
    
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', rviz_path],
        )
    ])
