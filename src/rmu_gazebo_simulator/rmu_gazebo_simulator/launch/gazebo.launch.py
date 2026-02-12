import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def _ign_executable_path() -> str:
    """Return the full path to the ``ign`` wrapper script."""
    path = shutil.which("ign")
    if path is None:
        raise RuntimeError("Cannot find 'ign' executable on PATH")
    return path


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    world_sdf_path = LaunchConfiguration("world_sdf_path")
    ign_config_path = LaunchConfiguration("ign_config_path")
    use_gui = LaunchConfiguration("use_gui")

    declare_world_sdf_path = DeclareLaunchArgument(
        "world_sdf_path",
        default_value=os.path.join(
            pkg_simulator, "resource", "worlds", "rmul_2024_world.sdf"
        ),
        description="Path to the world SDF file",
    )

    declare_ign_config_path = DeclareLaunchArgument(
        "ign_config_path",
        default_value=os.path.join(pkg_simulator, "resource", "ign", "gui.config"),
        description="Path to the Ignition Gazebo GUI configuration file",
    )

    declare_use_gui = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Whether to start Ignition Gazebo with the GUI",
    )

    # Set Gazebo plugin and resource path
    append_enviroment_worlds = AppendEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=os.path.join(pkg_simulator, "resource", "worlds"),
    )

    append_enviroment_models = AppendEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=os.path.join(pkg_simulator, "resource", "models"),
    )

    # ── Common rendering-engine env vars ──
    # Work around Ignition Fortress (Gazebo 6) rendering bug: OGRE2 + EGL
    # headless rendering triggers a segfault inside SceneManager::CreateVisual
    # when duplicate visual names exist across different links (e.g.
    # armor_0/light_bar_visual, armor_1/light_bar_visual).
    # Falling back to OGRE 1.x avoids the crash.
    _render_env = {
        "IGN_GAZEBO_RENDER_ENGINE_SERVER": os.environ.get(
            "IGN_GAZEBO_RENDER_ENGINE_SERVER", "ogre2"
        ),
        "IGN_GAZEBO_RENDER_ENGINE_GUI": os.environ.get(
            "IGN_GAZEBO_RENDER_ENGINE_GUI", "ogre2"
        ),
    }

    ign_exec = "ruby " + _ign_executable_path() + " gazebo"

    # Prefix command with env vars to guarantee they reach the ign gazebo
    # process, even when ros2 launch's additional_env does not propagate
    # through the shell=True path correctly.
    _env_prefix = " ".join(f"{k}={v}" for k, v in _render_env.items()) + " "

    # Launch Gazebo simulator — GUI mode
    gazebo_gui = ExecuteProcess(
        cmd=[
            _env_prefix + ign_exec,
            " ",
            world_sdf_path,
            TextSubstitution(text=" -r --gui-config "),
            ign_config_path,
            " --force-version 6",
        ],
        output="screen",
        additional_env=_render_env,
        shell=True,
        condition=IfCondition(use_gui),
    )

    # Launch Gazebo simulator — Headless mode
    gazebo_headless = ExecuteProcess(
        cmd=[
            _env_prefix + ign_exec,
            " ",
            world_sdf_path,
            TextSubstitution(text=" -r -s --headless-rendering"),
            " --force-version 6",
        ],
        output="screen",
        additional_env=_render_env,
        shell=True,
        condition=UnlessCondition(use_gui),
    )

    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_sdf_path)
    ld.add_action(declare_ign_config_path)
    ld.add_action(declare_use_gui)
    ld.add_action(append_enviroment_worlds)
    ld.add_action(append_enviroment_models)
    ld.add_action(gazebo_gui)
    ld.add_action(gazebo_headless)
    ld.add_action(robot_ign_bridge)

    return ld
