#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import numpy as np
pi = np.pi

class JointCmdPublisher(Node):
    def __init__(self):
        super().__init__('joint_cmd_test')
        # Publicador en el mismo tópico que el ESP32 escucha
        self.publisher_ = self.create_publisher(JointState, 'joint_cmd', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.angle = 0.1

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["SERVO_01", "SERVO_02", "SERVO_03", "SERVO_04", "GARRA"]
        msg.position = [2.35636214377144, 1.2740463002468783, 0.8928975263095886, 0.18371153160673698, 0.0]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publicando en \joint_cmd: {[round(p, 2) for p in msg.position]} rad')

def main(args=None):
    rclpy.init(args=args)
    node = JointCmdPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
