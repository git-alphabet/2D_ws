
# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
# Visualization mode configured in control_command.yaml:
#   visualization: "rviz2"     - start local RViz2
#   visualization: "foxglove" - start foxglove_bridge for remote visualization
#   Fallback: if foxglove_bridge is not installed, crashes, or times out,
#             automatically falls back to rviz2.
import os
import re
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
try:
    from ament_index_python.packages import has_package
except ImportError:
    has_package = None
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.logging

logger = launch.logging.get_logger('odin1_ros2.launch')

def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory('odin_ros_driver')

    # Declare configuration parameter
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'control_command.yaml'),
        description='Path to the control config YAML file'
    )

    # Add RViz2 configuration file parameter
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(package_dir, 'config', 'odin_ros2.rviz'),
        description='Path to RViz2 config file'
    )

    # Read visualization config from yaml (under register_keys)
    config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(config_path, 'r') as f:
        vis_config = yaml.safe_load(f).get('register_keys', {})
    visualization_mode = vis_config.get('visualization', 'rviz2')
    foxglove_port = vis_config.get('foxglove_port', 8765)
    foxglove_topics_file = vis_config.get('foxglove_topics_file', '') or \
        os.path.join(package_dir, 'config', 'foxglove_topics.txt')

    # Create main node
    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
       # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    pcd2depth_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(pcd2depth_config_path, 'r') as f:
        pcd2depth_params = yaml.safe_load(f)
    pcd2depth_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    # Cloud reprojection node
    reprojection_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(reprojection_config_path, 'r') as f:
        reprojection_params = yaml.safe_load(f)
    reprojection_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path
    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )

    # Image overlay node - overlays reprojected points on camera image
    overlay_config_path = os.path.join(package_dir, 'config', 'control_command.yaml')
    with open(overlay_config_path, 'r') as f:
        overlay_params = yaml.safe_load(f)
    image_overlay_node = Node(
        package='odin_ros_driver',
        executable='image_overlay_node',
        name='image_overlay_node',
        output='screen',
        parameters=[overlay_params]
    )

    def _load_topic_whitelist(topics_file):
        if not topics_file or not os.path.isfile(topics_file):
            return []
        topic_whitelist = []
        with open(topics_file, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#'):
                    topic_whitelist.append(f"^{re.escape(line)}$")
        return topic_whitelist

    def _package_installed(pkg_name):
        """检查 ROS 2 包是否已安装"""
        if has_package is not None:
            return has_package(pkg_name)
        try:
            get_package_share_directory(pkg_name)
            return True
        except PackageNotFoundError:
            return False

    def _make_rviz2_fallback(rviz_config_path):
        """创建 rviz2 回退节点"""
        def _launch_rviz2(context):
            config = rviz_config_path.perform(context)
            logger.warning(
                "Falling back to rviz2 (config: {})".format(config)
            )
            return [Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', config],
            )]
        return OpaqueFunction(function=_launch_rviz2)

    def _launch_visualization(context, *, mode, port, topics_file, rviz_config_arg):
        actions = []

        if mode == 'foxglove':
            # 检查 foxglove_bridge 是否安装
            if not _package_installed('foxglove_bridge'):
                logger.warning(
                    "foxglove_bridge not installed, falling back to rviz2. "
                    "Install it with: sudo apt install ros-humble-foxglove-bridge"
                )
                actions.append(_make_rviz2_fallback(rviz_config_arg))
                return actions

            # foxglove_bridge 已安装，尝试启动
            params = {'address': '0.0.0.0', 'port': port}
            topic_whitelist = _load_topic_whitelist(topics_file)
            if topic_whitelist:
                params['topic_whitelist'] = topic_whitelist

            foxglove_node = Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                name='foxglove_bridge',
                output='screen',
                parameters=[params],
            )
            actions.append(foxglove_node)

            # 注册退出事件处理器：如果 foxglove_bridge 异常退出，回退到 rviz2
            foxglove_exit_handler = RegisterEventHandler(
                OnProcessExit(
                    target_action=foxglove_node,
                    on_exit=[
                        LogInfo(msg="foxglove_bridge exited, falling back to rviz2"),
                        _make_rviz2_fallback(rviz_config_arg),
                    ],
                )
            )
            actions.append(foxglove_exit_handler)

        else:
            actions.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_arg.perform(context)],
            ))

        return actions

    visualization_cmd = OpaqueFunction(
        function=_launch_visualization,
        kwargs={
            'mode': visualization_mode,
            'port': foxglove_port,
            'topics_file': foxglove_topics_file,
            'rviz_config_arg': LaunchConfiguration('rviz_config'),
        },
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(rviz_config_arg)  # Add RViz configuration argument
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(cloud_reprojection_node)
    ld.add_action(image_overlay_node)
    ld.add_action(visualization_cmd)  # Start rviz2 or foxglove based on visualization param

    return ld
