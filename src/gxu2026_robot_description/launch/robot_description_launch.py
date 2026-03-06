import os
import tempfile
import re

import yaml  # type: ignore

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def _make_urdf_mesh_uris_portable(urdf_xml: str) -> str:
        """Rewrite absolute file:// URIs produced on another machine into package:// URIs.

        Example from remote robot_description:
            file:///.../install/rmoss_gz_resources/share/rmoss_gz_resources/resource/... ->
            package://rmoss_gz_resources/resource/...
        """

        # Match: file:///.../install/<pkg>/share/<pkg>/
        pattern = re.compile(r"file:///(?:(?!\s).)*/install/([^/\s]+)/share/\1/")
        return pattern.sub(r"package://\1/", urdf_xml)


def launch_setup(context: LaunchContext) -> list:
    """
    NOTE: Using OpaqueFunction in order to get the context in string format...
    But it is too hacky and not recommended.
    """

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Load the robot xmacro file from the launch configuration
    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(context.launch_configurations["robot_xmacro_file"])

    # Generate SDF from xmacro
    xmacro.generate()
    robot_xml = xmacro.to_string()

    # Generate URDF from SDF
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(robot_xml)
    robot_urdf_xml = _make_urdf_mesh_uris_portable(urdf_generator.to_string())

    # Resolve params file to a concrete path. When `namespace` is empty (common
    # on the real robot), passing a LaunchConfiguration directly to ParameterFile
    # can result in parameters not being applied.
    params_file_value = (context.launch_configurations.get("params_file") or "").strip()
    if not params_file_value:
        raise RuntimeError("params_file launch argument resolved to an empty path")

    # NOTE:
    # `launch_ros` may pass an empty namespace as "__ns:=/". In that case the resolved
    # value becomes "/". Also, many stacks pass namespaces with a leading '/'.
    # nav2_common.launch.RewrittenYaml + ParameterFile may generate a temporary YAML file
    # that can be cleaned up early; if that happens, nodes fall back to default params.
    #
    # Here we avoid that failure mode by:
    # - Using the real params YAML path directly when no namespace is requested.
    # - When a namespace is requested, wrapping the YAML under that root key and writing
    #   a delete=False temporary file ourselves.
    namespace_value = (context.launch_configurations.get("namespace") or "").strip()
    normalized_root_key = namespace_value.lstrip("/")
    if normalized_root_key:
        with open(params_file_value, "r", encoding="utf-8") as f:
            raw_yaml = yaml.safe_load(f) or {}
        namespaced_yaml = {normalized_root_key: raw_yaml}
        with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".yaml") as tmp_file:
            yaml.safe_dump(namespaced_yaml, tmp_file, default_flow_style=False)
            configured_params = tmp_file.name
    else:
        configured_params = params_file_value

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    bringup_cmd_group = GroupAction(
        [
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {"use_sim_time": use_sim_time}],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                    {"use_sim_time": use_sim_time, "robot_description": robot_urdf_xml},
                ],
                arguments=["--ros-args", "--log-level", log_level],
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
            ),
        ]
    )

    return [
        stdout_linebuf_envvar,
        colorized_output_envvar,
        bringup_cmd_group,
    ]


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("gxu2026_robot_description")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="simulation_robot",
        description="The file name of the robot xmacro to be used",
    )

    declare_robot_xmacro_file_cmd = DeclareLaunchArgument(
        "robot_xmacro_file",
        default_value=[
            # Use TextSubstitution to concatenate strings
            TextSubstitution(text=os.path.join(bringup_dir, "resource", "xmacro", "")),
            LaunchConfiguration("robot_name"),
            TextSubstitution(text=".sdf.xmacro"),
        ],
        description="The file path of the robot xmacro to be used",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "robot_description.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "visualize_robot.rviz"),
        description="Full path to the RViz config file to use",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RViz"
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_xmacro_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the nodes
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
