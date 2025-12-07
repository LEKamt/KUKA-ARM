#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class TranslatorNode(Node):
    def __init__(self):
        super().__init__('translator_node')

        # --- Publishers ---
        self.pub_cmd_esp32 = self.create_publisher(Float32MultiArray, '/joint_cmd_esp32', 10)
        self.pub_angles = self.create_publisher(JointState, '/joint_angles', 10)

        # --- Subscribers ---
        self.sub_cmd = self.create_subscription(JointState, '/joint_cmd', self.cmd_callback, 10)
        self.sub_angles_esp32 = self.create_subscription(Float32MultiArray, '/joint_angles_esp32', self.angles_callback, 10)

        self.get_logger().info("Translator Node (JointState <-> Float32MultiArray) iniciado")

    # --- De JointState -> Float32MultiArray ---
    def cmd_callback(self, msg: JointState):
        array_msg = Float32MultiArray()
        array_msg.data = list(msg.position)  # Solo posiciones
        self.pub_cmd_esp32.publish(array_msg)
        self.get_logger().debug(f"-> Enviado a ESP32: {array_msg.data}")

    # --- De Float32MultiArray -> JointState ---
    def angles_callback(self, msg: Float32MultiArray):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint_{i}" for i in range(len(msg.data))]
        js.position = list(msg.data)
        self.pub_angles.publish(js)
        self.get_logger().debug(f"<- Recibido de ESP32: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = TranslatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
