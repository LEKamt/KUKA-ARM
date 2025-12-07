#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from cmd_pkg.action import Control
from cmd_pkg.srv import Enfoque1, Enfoque2
from simple_actions import SimpleActionServer

import numpy as np
from fr_pkg.kuka_utils import cinem_directa
import time


# ------------------------------ Clase Servidor de Control ------------------------------ 
class ControlServer(Node):

    def __init__(self):
        super().__init__('control_servidor')

        # ---- Inicializar mensajes de joints ----
        self.q_cmd = JointState()
        self.q_cmd.name = ["SERVO_01", "SERVO_02", "SERVO_03", "SERVO_04", "GARRA"]
        self.q_cmd.position = [0.0] * len(self.q_cmd.name)

        self.q_feed = JointState()
        self.q_feed.name = self.q_cmd.name
        self.q_feed.position = [0.0] * len(self.q_feed.name)

        # ---- Posición del efector final ----
        self.x_vec = np.array([0.0, -0.156, -0.023])


        # ---- Mensaje de regreso ----
        self.msg_resultado = ""

        # ---- Action Server ----
        self.action_server = SimpleActionServer(self, Control, 'control', self.do_control)

        # ---- Suscriptor y Publicador ----
        self.sub_joint = self.create_subscription(JointState, 'joint_angles', self.joint_callback, 10)
        self.pub_joint = self.create_publisher(JointState, 'joint_cmd', 10)

        # ---- Publicación periódica cada 1 segundo ----
        self.timer_pub = self.create_timer(0.1, self.timer_callback)

        # ---- Clientes de servicio ----
        self.cli_enfoque_1 = self.create_client(Enfoque1, 'enfoque_1')
        self.cli_enfoque_2 = self.create_client(Enfoque2, 'enfoque_2')

        # ---- Esperar disponibilidad de servicios ----
        while not self.cli_enfoque_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio enfoque_1...')
        while not self.cli_enfoque_2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio enfoque_2...')

        self.get_logger().info('Servidor de control listo.')

    # ------------------------------ Callback principal del Action Server ------------------------------ 
    def do_control(self, goal):
        x_des, y_des, z_des, garra, enfoque = goal.x, goal.y, goal.z, goal.g, goal.enfoque
        self.get_logger().info(f'Recibido objetivo: ({x_des:.3f}, {y_des:.3f}, {z_des:.3f}) | enfoque={enfoque}')

        # >> ---- Enfoque 1 ----
        if enfoque == 1:
            req = Enfoque1.Request()
            req.target = [x_des, y_des, z_des]

            future = self.cli_enfoque_1.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            resp = future.result()
            
            if resp:
                if resp.success:
                    self.q_cmd = resp.joint_out
                    self.q_cmd.position[4] = garra
                    self.get_logger().info('Enfoque 1 ejecutado correctamente.')
                    self.msg_resultado = "Movimiento completado correctamente"
                else:
                    self.get_logger().warn('Enfoque 1 ejecutado correctamente, pero sistema no convergio.')
                    self.get_logger().warn('No se movió el robot')
                    self.msg_resultado = "Sistema no convergió. No se movió el robot"
            else:
                self.get_logger().error('Error al obtener respuesta de enfoque_1.')
                self.msg_resultado = "Error al obtener respuesta del servidor enfoque_1."

            time.sleep(1)

        # >> ---- Enfoque 2 ----
        elif enfoque == 2:
            self.q_cmd.position[4] = garra
            error = [x_des - self.x_vec[0], y_des - self.x_vec[1], z_des - self.x_vec[2]]
            error_norm = np.linalg.norm(error)
            iter_count = 0

            while error_norm > 0.015 and iter_count < 600:
                iter_count += 1
                req = Enfoque2.Request()
                req.e = error
                joint_angles = self.q_feed.position[:4]  # angulos actuales q_prev
                req.joint_angles = joint_angles

                future = self.cli_enfoque_2.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                resp = future.result()

                if resp is None:
                    self.get_logger().error('Error en llamada a enfoque_2.')
                    self.msg_resultado = "Error al obtener respuesta del servidor enfoque_2."
                    break

                # Se actualizará automáticamente cada 0.1 s
                self.q_cmd.position[0] = resp.qk[0]
                self.q_cmd.position[1] = resp.qk[1]
                self.q_cmd.position[2] = resp.qk[2]
                self.q_cmd.position[3] = resp.qk[3]
                time.sleep(0.01)
                error = [self.x_vec[0] - x_des, self.x_vec[1] - y_des, self.x_vec[2] - z_des]
                error_norm = np.linalg.norm(error)
                self.get_logger().info(f'Iter {iter_count}: error = {error_norm:.4f}')

                if iter_count == 600:
                    self.msg_resultado = "Sistema no convergió dentro de las iteraciones máximas"
                else:
                    self.msg_resultado = "Movimiento completado correctamente"

        # ---- Respuesta final ----
        result_msg = Control.Result()
        self.x_vec = cinem_directa(self.q_feed)
        result_msg.x_final = self.x_vec[0]
        result_msg.y_final = self.x_vec[1]
        result_msg.z_final = self.x_vec[2]
        result_msg.descripcion = self.msg_resultado

        self.get_logger().info('Acción completada con éxito.')
        return result_msg

    # ------------------------------ Publicador periódico ------------------------------ 
    def timer_callback(self):
        """Publica el último comando cada 1 segundo."""
        self.q_cmd.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint.publish(self.q_cmd)
        self.get_logger().debug(f"Publicando q_cmd periódicamente: {', '.join(f'{p:.2f}' for p in self.q_cmd.position)}")

    # ------------------------------ Callback del Suscriptor ------------------------------ 
    def joint_callback(self, msg: JointState):
        self.q_feed.position = msg.position
        self.x_vec = cinem_directa(self.q_feed)


# ------------------------------ Bloque principal ------------------------------ 
def main():
    rclpy.init()
    node = ControlServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
