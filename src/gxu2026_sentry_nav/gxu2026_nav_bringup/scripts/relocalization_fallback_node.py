#!/usr/bin/env python3
"""
Relocalization fallback node for odin1 Mode 2.

When odin1 relocalization takes too long, this node allows the user to manually
set an initial pose via RViz (2D Pose Estimate) to publish a map->odom TF,
unblocking Nav2 startup. When odin1 relocalization succeeds, it takes over.

Flow:
  1. Node starts in standby (no map->odom TF published)
  2. User sets initial pose in RViz -> publishes /initialpose
  3. This node computes map->odom TF from the initial pose and current odometry
  4. Nav2 warmup completes, lifecycle manager activates
  5. When odin1 relocalizes -> odin1/relocalization_success topic received
  6. This node stops publishing, odin1 takes over
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster


def quat_to_rotation_matrix(q):
    """Convert quaternion (x, y, z, w) to 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def rotation_matrix_to_quat(R):
    """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


class RelocalizationFallbackNode(Node):
    def __init__(self):
        super().__init__('relocalization_fallback')

        # State
        self._active = False  # True when publishing fallback map->odom TF
        self._odin1_relocalized = False
        self._lock = threading.Lock()

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        # Subscribers
        # /initialpose from RViz (PoseWithCovarianceStamped, map frame)
        self._initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._on_initial_pose,
            10,
        )

        # odin1 relocalization success (latched Bool)
        relocalization_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._reloc_success_sub = self.create_subscription(
            Bool,
            'odin1/relocalization_success',
            self._on_relocalization_success,
            relocalization_qos,
        )

        self.get_logger().info(
            'Relocalization fallback node started. '
            'Waiting for initial pose from RViz (2D Pose Estimate)...'
        )

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped):
        """Handle initial pose from RViz."""
        with self._lock:
            if self._odin1_relocalized:
                self.get_logger().info(
                    'odin1 already relocalized, ignoring manual initial pose.'
                )
                return

            # Extract pose in map frame
            p_map_base = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ])
            q_map_base = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            R_map_base = quat_to_rotation_matrix(q_map_base)

            # Get current odom -> base_footprint from TF
            try:
                tf_odom_base = self._tf_buffer.lookup_transform(
                    'odom', 'base_footprint', rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().warn(
                    f'Cannot lookup odom->base_footprint TF: {e}. '
                    'Make sure odin1 is publishing odometry.'
                )
                return

            p_odom_base = np.array([
                tf_odom_base.transform.translation.x,
                tf_odom_base.transform.translation.y,
                tf_odom_base.transform.translation.z,
            ])
            q_odom_base = (
                tf_odom_base.transform.rotation.x,
                tf_odom_base.transform.rotation.y,
                tf_odom_base.transform.rotation.z,
                tf_odom_base.transform.rotation.w,
            )
            R_odom_base = quat_to_rotation_matrix(q_odom_base)

            # Compute map -> odom:
            # T_map_odom = T_map_base * T_odom_base^(-1)
            R_odom_base_inv = R_odom_base.T
            R_map_odom = R_map_base @ R_odom_base_inv
            p_map_odom = p_map_base - R_map_odom @ p_odom_base
            q_map_odom = rotation_matrix_to_quat(R_map_odom)

            # Publish static map -> odom TF
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'map'
            tf_msg.child_frame_id = 'odom'
            tf_msg.transform.translation.x = float(p_map_odom[0])
            tf_msg.transform.translation.y = float(p_map_odom[1])
            tf_msg.transform.translation.z = float(p_map_odom[2])
            tf_msg.transform.rotation.x = q_map_odom[0]
            tf_msg.transform.rotation.y = q_map_odom[1]
            tf_msg.transform.rotation.z = q_map_odom[2]
            tf_msg.transform.rotation.w = q_map_odom[3]

            self._tf_broadcaster.sendTransform(tf_msg)
            self._active = True

            self.get_logger().info(
                f'Published fallback map->odom TF from initial pose: '
                f'x={p_map_odom[0]:.3f}, y={p_map_odom[1]:.3f}, '
                f'yaw={math.atan2(2*(q_map_odom[3]*q_map_odom[2]+q_map_odom[0]*q_map_odom[1]), 1-2*(q_map_odom[1]**2+q_map_odom[2]**2)):.3f}. '
                f'Nav2 should now be able to start.'
            )

    def _on_relocalization_success(self, msg: Bool):
        """Handle odin1 relocalization success."""
        if msg.data:
            with self._lock:
                self._odin1_relocalized = True
                self._active = False
            self.get_logger().info(
                'odin1 relocalization succeeded! '
                'Stopping fallback map->odom TF. odin1 takes over.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RelocalizationFallbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
