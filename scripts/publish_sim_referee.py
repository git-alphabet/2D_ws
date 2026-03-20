#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rm_decision_interfaces.msg import RMUL

class SimRefereeNode(Node):
    def __init__(self):
        super().__init__('sim_referee_node')
        self.pub_robot_status = self.create_publisher(RMUL, '/robot_status', 10)
        self.pub_game_status = self.create_publisher(RMUL, '/game_status', 10)
        self.pub_rfid_status = self.create_publisher(RMUL, '/rfid_status', 10)
        self.pub_robot_pos = self.create_publisher(RMUL, '/robot_position', 10)
        
        # 默认满血，比赛开始等状态
        self.msg = RMUL()
        self.msg.header.frame_id = 'map'
        self.msg.game_progress = 4
        self.msg.stage_remain_time = 180
        self.msg.current_hp = 1500
        self.msg.is_attacked = 0
        self.msg.shooter_heat = 0
        self.msg.rfid_supply_arrived = False
        self.msg.rfid_control_arrived = False
        self.msg.cmd_type = 1
        self.msg.emergency_stop = False
        self.msg.stop_gimbal_scan = False
        self.msg.chassis_spin = False
        self.msg.x = 0.0
        self.msg.y = 0.0
        self.msg.is_detect_enemy = False

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("模拟裁判系统发布器已启动。你可以修改脚本中的数值以测试低血量或其他行为。")

    def timer_callback(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        # 发布到所有话题
        self.pub_robot_status.publish(self.msg)
        self.pub_game_status.publish(self.msg)
        self.pub_rfid_status.publish(self.msg)
        self.pub_robot_pos.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimRefereeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
