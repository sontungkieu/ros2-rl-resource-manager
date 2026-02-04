import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "y", "on")


def _default_nav2_params_file():
    model = os.environ.get("TURTLEBOT3_MODEL", "burger")
    return os.path.join(
        get_package_share_directory("turtlebot3_navigation2"),
        "param",
        f"{model}.yaml",
    )


def _include_nav2(context, *args, **kwargs):
    map_yaml = LaunchConfiguration("map").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = _as_bool(use_sim_time)
    params_file = LaunchConfiguration("params_file").perform(context).strip()
    if not params_file:
        params_file = _default_nav2_params_file()

    nav2_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    tb3_rviz = os.path.join(
        get_package_share_directory("turtlebot3_navigation2"),
        "rviz",
        "tb3_navigation2.rviz",
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "bringup_launch.py")),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
            }.items(),
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("start_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", tb3_rviz],
            parameters=[{"use_sim_time": use_sim_time_bool}],
            output="screen",
        ),
    ]


def _include_initial_pose_publisher(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = _as_bool(use_sim_time)

    robot_x = float(LaunchConfiguration("robot_x").perform(context))
    robot_y = float(LaunchConfiguration("robot_y").perform(context))
    robot_yaw = float(LaunchConfiguration("robot_yaw").perform(context))
    delay_s = float(LaunchConfiguration("initial_pose_delay_s").perform(context))

    return [
        Node(
            package="tb3_nav2_dynamic",
            executable="initial_pose_publisher",
            name="initial_pose_publisher",
            parameters=[
                {"use_sim_time": use_sim_time_bool},
                {"x": robot_x},
                {"y": robot_y},
                {"yaw": robot_yaw},
                {"delay_s": delay_s},
            ],
            output="screen",
        )
    ]


def _include_dynamic_obstacles(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time_bool = _as_bool(use_sim_time)
    return [
        Node(
            package="tb3_nav2_dynamic",
            executable="dynamic_obstacles",
            name="dynamic_obstacles",
            parameters=[{"use_sim_time": use_sim_time_bool}],
            output="screen",
        )
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    tb3_world_launch = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"),
        "launch",
        "turtlebot3_world.launch.py",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            # Match turtlebot3_gazebo/turtlebot3_world.launch.py defaults for visibility / consistency.
            DeclareLaunchArgument("robot_x", default_value="-2.0", description="Robot spawn X in Gazebo"),
            DeclareLaunchArgument("robot_y", default_value="-0.5", description="Robot spawn Y in Gazebo"),
            DeclareLaunchArgument("robot_yaw", default_value="0.0", description="Robot spawn yaw (radians)"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(os.environ.get("HOME", ""), "slam_rl_ws", "maps", "my_map.yaml"),
                description="Full path to map YAML to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="Optional full path to Nav2 params YAML. Empty uses turtlebot3_navigation2 defaults.",
            ),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument("start_nav2", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("auto_initial_pose", default_value="true"),
            DeclareLaunchArgument("initial_pose_delay_s", default_value="2.0"),
            DeclareLaunchArgument("start_dynamic_obstacles", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(tb3_world_launch),
                condition=IfCondition(LaunchConfiguration("start_gazebo")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "x_pose": LaunchConfiguration("robot_x"),
                    "y_pose": LaunchConfiguration("robot_y"),
                }.items(),
            ),
            OpaqueFunction(function=_include_nav2, condition=IfCondition(LaunchConfiguration("start_nav2"))),
            OpaqueFunction(
                condition=IfCondition(LaunchConfiguration("auto_initial_pose")),
                function=_include_initial_pose_publisher,
            ),
            OpaqueFunction(
                condition=IfCondition(LaunchConfiguration("start_dynamic_obstacles")),
                function=_include_dynamic_obstacles,
            ),
        ]
    )
