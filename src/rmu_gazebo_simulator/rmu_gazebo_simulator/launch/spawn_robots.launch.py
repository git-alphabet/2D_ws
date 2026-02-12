import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    pkg_pb2025_robot_description = get_package_share_directory(
        "pb2025_robot_description"
    )

    robot_xmacro_path = os.path.join(
        pkg_pb2025_robot_description,
        "resource",
        "xmacro",
        "simulation_robot.sdf.xmacro",
    )
    bridge_config_with_odom = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    bridge_config_no_odom = os.path.join(pkg_simulator, "config", "ros_gz_bridge_no_odom.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    enable_chassis_odometry_gt = LaunchConfiguration("enable_chassis_odometry_gt")
    robot_base_prefix = LaunchConfiguration("robot_base_prefix")

    # Get spawn robot init pose
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        robots = config["robots"].get(selected_world)

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "enable_chassis_odometry_gt",
            default_value="true",
            description=(
                "Whether to bridge chassis ground-truth odometry from Gazebo to ROS as '<ns>/chassis_odometry_gt'. "
                "Disable this to test navigation behavior without simulator GT odometry."),
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "robot_base_prefix",
            default_value="",
            description="Optional launch prefix for rmua19_robot_base (e.g. 'gdb -ex run -ex bt --args')",
        )
    )

    # ── Build per-robot actions ──
    # We must spawn robots one at a time (sequentially) because Ignition
    # Fortress 6 has a rendering-thread race: when two models are spawned
    # in parallel, SceneManager::CreateVisual may attempt to register the
    # same visual twice, crashing with a duplicate-name assertion.

    # Collect (spawn_node, [companion_actions]) for each robot
    robot_groups = []

    for robot in robots:
        # Generate SDF from xmacro
        xmacro.generate({"global_initial_color": robot["color"]})
        robot_xml = xmacro.to_string()

        # Generate URDF from SDF
        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        aft_replace_ros_bridge_params_with_odom = ReplaceString(
            source_file=bridge_config_with_odom,
            replacements={"<robot_name>": robot["name"]},
        )
        aft_replace_ros_bridge_params_no_odom = ReplaceString(
            source_file=bridge_config_no_odom,
            replacements={"<robot_name>": robot["name"]},
        )

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            name=f'spawn_{robot["name"].replace("-", "_")}',
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        companion_actions = [
            Node(
                package="rmoss_gz_base",
                executable="rmua19_robot_base",
                namespace=robot["name"],
                prefix=robot_base_prefix,
                parameters=[robot_config, {"robot_name": robot["name"]}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=robot["name"],
                remappings=remappings,
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": robot_urdf_xml,
                    }
                ],
            ),
            Node(
                condition=IfCondition(enable_chassis_odometry_gt),
                package="ros_gz_bridge",
                executable="parameter_bridge",
                namespace=robot["name"],
                parameters=[{"config_file": aft_replace_ros_bridge_params_with_odom}],
            ),
            Node(
                condition=UnlessCondition(enable_chassis_odometry_gt),
                package="ros_gz_bridge",
                executable="parameter_bridge",
                namespace=robot["name"],
                parameters=[{"config_file": aft_replace_ros_bridge_params_no_odom}],
            ),
            ExecuteProcess(
                cmd=[
                    "ign",
                    "service",
                    "-s",
                    "/world/default/level/set_performer",
                    "--reqtype",
                    "ignition.msgs.StringMsg",
                    "--reptype",
                    "ignition.msgs.Boolean",
                    "--timeout",
                    "2000",
                    "--req",
                    f'data: "{robot["name"]}"',
                ],
                output="screen",
            ),
        ]

        robot_groups.append((spawn_robot, companion_actions))

    # ── Chain spawns: robot[0] starts immediately; robot[N] waits for
    #    robot[N-1]'s spawn (ros_gz_sim create) to exit before beginning.
    if robot_groups:
        first_spawn, first_companions = robot_groups[0]
        ld.add_action(first_spawn)
        for action in first_companions:
            ld.add_action(action)

        prev_spawn = first_spawn
        for spawn_node, companions in robot_groups[1:]:
            # When the previous spawn process exits, start this group
            ld.add_action(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=prev_spawn,
                        on_exit=[spawn_node] + companions,
                    )
                )
            )
            prev_spawn = spawn_node

    return ld
