from launch import condition
from launch.actions import DeclareLaunchArgument
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false",
        description="Use SLAM mapping"
    )

    use_slam = LaunchConfiguration("use_slam")

    
    
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True"
        }.items()
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam),
        # launch_arguments={
        #     "use_sim_time": "True"
        # }.items()
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam),
        # launch_arguments={
        #     "use_sim_time": "True"
        # }.items()
    )

    safety_stop = Node(
        package="bumperbot_utils",
        executable="safety_stop.py",
        output="screen",
        
        # parameters=[{
        #     "use_sim_time": "True"
        # }],
    )

    rviz_localization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("bumperbot_localization"), 
        "rviz", 
        "odometry_motion_model.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=UnlessCondition(use_slam),
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("bumperbot_mapping"), 
        "rviz", 
        "slam.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam),
    )

    # mapper = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("bumperbot_mapping"),
    #         "launch",
    #         "mapping_with_known_poses.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "True"  
    #     }.items()
    # )
    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        safety_stop,
        localization,
        slam,
        rviz_localization,
        rviz_slam,
        # mapper,
    ])
