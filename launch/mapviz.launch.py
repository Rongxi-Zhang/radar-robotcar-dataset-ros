# mapviz for ros2: https://github.com/swri-robotics/mapviz/archive/refs/tags/2.2.0.tar.gz
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            # args: x y z yaw=-1.13 pitch roll
            arguments=["0", "0", "0", "-1.13", "0", "0", "map", "world"],
            parameters=[
                {"use_sim_time": True}
            ]
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"local_xy_frame": "map"},
                {"local_xy_origin" : "auto"},
                {"local_xy_navsatfix_topic" : "/robotcar/ins/gps"}
            ],
        ),
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
        )

    ])
