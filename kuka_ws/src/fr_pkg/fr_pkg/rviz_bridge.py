#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class JointBridge(Node):
    def __init__(self):
        super().__init__('rviz_bridge')

        # Suscripcion al topico con 5 articulaciones
        self.subscription = self.create_subscription(JointState, 'joint_angles', self.cmd_callback, 10) # joint_angles

        # Publicador hacia robot_state_publisher
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Inicializar mensaje JointState
        self.states = JointState()
        self.states.name = [
            "SERVO_01", "SERVO_02", "SERVO_03", "SERVO_04",
            "A1", "A2", "B1", "B2", "C2", "C1"
        ]
        self.states.position = [0.0] * len(self.states.name)

        self.get_logger().info("Nodo joint_bridge iniciado, esperando en /joint_angles...")

    
    def cmd_callback(self, msg: JointState):
        if len(msg.position) < 5:
            self.get_logger().warn("El mensaje recibido tiene menos de 5 articulaciones")
            return

        a0, a1, a2, a3, a4 = msg.position[:5]

        # Recalcular segun tus reglas
        a0 = a0 - np.pi/2
        a1 = a1 - np.pi
        a3 = a3 - np.pi/2
        a4 = a4 - 25*np.pi/180  # compensacion

        self.states.header.stamp = self.get_clock().now().to_msg()
        self.states.position[0] = a0
        self.states.position[1] = a1
        self.states.position[2] = a2
        self.states.position[3] = a3
        self.states.position[4] = a4
        self.states.position[5] = -a4
        self.states.position[6] = a4
        self.states.position[7] = -a4
        self.states.position[8] = a4 * (5.0 / 6.0)
        self.states.position[9] = a4 * (-5.0 / 6.0)

        # Publicar en /joint_states
        self.publisher.publish(self.states)

        # Mensaje de depuracion
        self.get_logger().info(f"JointStates publicados: {', '.join(f'{p:.2f}' for p in self.states.position)}")


def main(args=None):
    rclpy.init(args=args)
    node = JointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
