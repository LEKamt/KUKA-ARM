#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeESP32(Node):
    """Nodo que reenvía directamente los JointState recibidos en /joint_cmd hacia /joint_angles."""
    def __init__(self):
        super().__init__('fake_esp32')

        self.sub_cmd = self.create_subscription(
            JointState,
            'joint_cmd',
            self.cmd_callback,
            10
        )

        self.pub_angles = self.create_publisher(JointState, 'joint_angles', 10)

        self.get_logger().info("✅ Nodo FakeESP32 iniciado. Reenviando /joint_cmd → /joint_angles")

    def cmd_callback(self, msg: JointState):
        self.pub_angles.publish(msg)
        self.get_logger().info(
            f"Echo JointState: {', '.join(f'{p:.2f}' for p in msg.position)}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FakeESP32()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
