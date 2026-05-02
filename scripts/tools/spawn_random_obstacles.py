#!/usr/bin/env python3
from dataclasses import dataclass
from pathlib import Path
import math
import random
import subprocess

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


WORKSPACE_ROOT = Path(__file__).resolve().parents[2]
MODEL_SDF_PATH = (
    WORKSPACE_ROOT
    / "src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2025/model.sdf"
)
GZ_WORLD_CONFIG_PATH = (
    WORKSPACE_ROOT
    / "src/rmu_gazebo_simulator/rmu_gazebo_simulator/config/gz_world.yaml"
)
EXPECTED_WORLD_NAME = "rmuc_2025"

NUM_OBSTACLES = 3
TIMER_PERIOD = 0.1
OBSTACLE_Z = 0.5
SPAWN_MIN_DISTANCE = 1.2
WAYPOINT_REACHED_DISTANCE = 0.4
LOOKAHEAD_TIME = 1.2
# 速度提高：最小 0.5 m/s，最大 1.2 m/s
MIN_LINEAR_SPEED = 0.5
MAX_LINEAR_SPEED = 1.2
MAX_ANGULAR_SPEED = 1.5

# 直接在 RMUC 场地中间活动，避免刷到边界外或场地角落。
SPAWN_HALF_WIDTH = 4.5
SPAWN_HALF_HEIGHT = 2.8
MOVE_HALF_WIDTH = 5.5
MOVE_HALF_HEIGHT = 3.4

# 尺寸缩小：边长 0.2 x 0.2 x 0.4
BOX_SDF = """
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="{name}">
    <link name="link">
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.1</ixx> <ixy>0.0</ixy> <ixz>0.0</ixz>
          <iyy>0.1</iyy> <iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.2 0.2 0.4</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.2 0.2 0.4</size></box></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
    <plugin filename="libignition-gazebo-velocity-control-system.so" name="ignition::gazebo::systems::VelocityControl">
      <link_name>link</link_name>
      <link_type>link</link_type>
    </plugin>
  </model>
</sdf>
"""


@dataclass
class ArenaRegion:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def random_point(self):
        return (
            random.uniform(self.min_x, self.max_x),
            random.uniform(self.min_y, self.max_y),
        )

    def contains(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y

    def clamp(self, x: float, y: float):
        return (
            min(max(x, self.min_x), self.max_x),
            min(max(y, self.min_y), self.max_y),
        )


@dataclass
class ObstacleState:
    name: str
    pub: object
    x: float
    y: float
    yaw: float
    target_x: float
    target_y: float
    speed: float


class RandomObstacleNode(Node):
    def __init__(self):
        super().__init__("random_obstacle_node")
        selected_world = self._read_selected_world(GZ_WORLD_CONFIG_PATH)
        if selected_world != EXPECTED_WORLD_NAME:
            self.get_logger().warning(
                f"gz_world.yaml selects '{selected_world}', but this script is configured for "
                f"'{EXPECTED_WORLD_NAME}'."
            )

        center_x, center_y = self._read_model_center(MODEL_SDF_PATH)
        self.spawn_region = ArenaRegion(
            center_x - SPAWN_HALF_WIDTH,
            center_x + SPAWN_HALF_WIDTH,
            center_y - SPAWN_HALF_HEIGHT,
            center_y + SPAWN_HALF_HEIGHT,
        )
        self.move_region = ArenaRegion(
            center_x - MOVE_HALF_WIDTH,
            center_x + MOVE_HALF_WIDTH,
            center_y - MOVE_HALF_HEIGHT,
            center_y + MOVE_HALF_HEIGHT,
        )

        self.bridge_procs = []
        self.obstacles = []

        self.get_logger().info(
            f"RMUC center=({center_x:.2f}, {center_y:.2f}), "
            f"spawn_region=([{self.spawn_region.min_x:.2f}, {self.spawn_region.max_x:.2f}], "
            f"[{self.spawn_region.min_y:.2f}, {self.spawn_region.max_y:.2f}])"
        )

        for i in range(NUM_OBSTACLES):
            self.obstacles.append(self._spawn_obstacle(i))

        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.get_logger().info("Random obstacles are now moving inside the RMUC center area.")

    def _spawn_obstacle(self, index: int) -> ObstacleState:
        name = f"dyn_obs_{index}"
        x, y, yaw = self._random_spawn_pose()
        target_x, target_y = self._select_target(x, y)

        sdf_path = Path("/tmp") / f"{name}.sdf"
        sdf_path.write_text(BOX_SDF.format(name=name), encoding="utf-8")

        self.get_logger().info(f"Spawning {name} at {x:.2f}, {y:.2f}...")
        subprocess.run(
            [
                "ros2",
                "run",
                "ros_gz_sim",
                "create",
                "-file",
                str(sdf_path),
                "-name",
                name,
                "-x",
                str(x),
                "-y",
                str(y),
                "-z",
                str(OBSTACLE_Z),
                "-Y",
                str(yaw),
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

        topic = f"/model/{name}/cmd_vel"
        self.get_logger().info(f"Bridging {topic}...")
        proc = subprocess.Popen(
            [
                "ros2",
                "run",
                "ros_gz_bridge",
                "parameter_bridge",
                f"{topic}@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            ],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.bridge_procs.append(proc)

        return ObstacleState(
            name=name,
            pub=self.create_publisher(Twist, topic, 10),
            x=x,
            y=y,
            yaw=yaw,
            target_x=target_x,
            target_y=target_y,
            speed=random.uniform(MIN_LINEAR_SPEED, MAX_LINEAR_SPEED),
        )

    def _random_spawn_pose(self):
        for _ in range(1000):
            x, y = self.spawn_region.random_point()
            if all(
                math.hypot(x - obs.x, y - obs.y) >= SPAWN_MIN_DISTANCE
                for obs in self.obstacles
            ):
                return x, y, random.uniform(-math.pi, math.pi)
        x, y = self.spawn_region.random_point()
        return x, y, random.uniform(-math.pi, math.pi)

    def _select_target(self, current_x: float, current_y: float):
        best = None
        best_distance = -1.0
        for _ in range(100):
            target_x, target_y = self.move_region.random_point()
            distance = math.hypot(target_x - current_x, target_y - current_y)
            if distance > best_distance and distance >= 1.0:
                best = (target_x, target_y)
                best_distance = distance
        if best is not None:
            return best
        return current_x, current_y

    def timer_callback(self):
        for obs in self.obstacles:
            cmd = self._compute_command(obs)
            obs.pub.publish(cmd)

    def _compute_command(self, obs: ObstacleState) -> Twist:
        distance = math.hypot(obs.target_x - obs.x, obs.target_y - obs.y)
        if distance <= WAYPOINT_REACHED_DISTANCE:
            obs.target_x, obs.target_y = self._select_target(obs.x, obs.y)
            obs.speed = random.uniform(MIN_LINEAR_SPEED, MAX_LINEAR_SPEED)

        target_yaw = math.atan2(obs.target_y - obs.y, obs.target_x - obs.x)
        yaw_error = self._normalize_angle(target_yaw - obs.yaw)
        linear = obs.speed * max(0.25, math.cos(yaw_error))
        angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 2.0 * yaw_error))

        next_yaw = self._normalize_angle(obs.yaw + angular * TIMER_PERIOD)
        next_x = obs.x + linear * math.cos(next_yaw) * LOOKAHEAD_TIME
        next_y = obs.y + linear * math.sin(next_yaw) * LOOKAHEAD_TIME

        if not self.move_region.contains(next_x, next_y):
            obs.target_x, obs.target_y = self._select_target(obs.x, obs.y)
            target_yaw = math.atan2(obs.target_y - obs.y, obs.target_x - obs.x)
            yaw_error = self._normalize_angle(target_yaw - obs.yaw)
            linear = 0.0
            angular = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, 2.6 * yaw_error))

        obs.yaw = self._normalize_angle(obs.yaw + angular * TIMER_PERIOD)
        obs.x += linear * math.cos(obs.yaw) * TIMER_PERIOD
        obs.y += linear * math.sin(obs.yaw) * TIMER_PERIOD
        obs.x, obs.y = self.move_region.clamp(obs.x, obs.y)

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        return msg

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _read_model_center(path: Path):
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.strip()
            if not line.startswith("<pose") or "</pose>" not in line:
                continue
            pose_text = line.split(">", 1)[1].split("</pose>", 1)[0].strip()
            values = [float(value) for value in pose_text.split()]
            if len(values) >= 2:
                return values[0], values[1]
        raise RuntimeError(f"Failed to find model pose in {path}")

    @staticmethod
    def _read_selected_world(path: Path):
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            line = raw_line.split("#", 1)[0].strip()
            if line.startswith("world:"):
                return line.split(":", 1)[1].strip().strip("'\"")
        raise RuntimeError(f"Failed to find selected world in {path}")

    def cleanup(self):
        for proc in self.bridge_procs:
            proc.terminate()
        for proc in self.bridge_procs:
            try:
                proc.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                proc.kill()


def main():
    rclpy.init()
    node = RandomObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()