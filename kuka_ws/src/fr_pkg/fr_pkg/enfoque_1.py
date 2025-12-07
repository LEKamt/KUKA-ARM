#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cmd_pkg.srv import Enfoque1
from sensor_msgs.msg import JointState
from fr_pkg.kuka_utils import cinem_directa, ikine, jacobian_position

import numpy as np


class Enfoque1Server(Node):
    """
    Servidor para Enfoque1:
    Recibe un vector de 3 valores (target) y devuelve un JointState con 5 articulaciones. (4 calculadas y la 5ta por defecto)
    """
    def __init__(self):
        super().__init__('enfoque_1')

        # ---- Inicializar mensajes de joints ----
        self.q_feed = JointState()
        self.q_feed.name = ["SERVO_01", "SERVO_02", "SERVO_03", "SERVO_04", "GARRA"]
        self.q_feed.position = [0.0, 0.0, 0.0, 0.0, 0.0]

        # ---- Servidor de accion ----
        self.srv = self.create_service(Enfoque1, 'enfoque_1', self.callback)

        # ---- Suscriptor ----
        self.sub_joint = self.create_subscription(JointState, 'joint_angles', self.joint_callback, 10)

        self.get_logger().info('Servidor enfoque_1 listo.')

    def callback(self, request, response):
        # request.target es un array de 3 valores
        t = list(request.target)
        self.get_logger().info(f'Recibido target: {t}')

        # Argumentos del metodo numerico
        p_des = request.target
        q0 = JointState()
        q0.name = ["SERVO_01", "SERVO_02", "SERVO_03", "SERVO_04"]
        q0.position = self.q_feed.position[:4]

        # IK (Newton con DLS leve)
        q_sol, ok, iters, err = ikine(
            cinem_directa,
            jacobian_position,
            p_des,
            q0,
            eps= 1e-4,
            max_iter= 500,
            lam=1e-3,
            dq_clip=0.1
        )

        
        # Crear JointState de salida
        joint_out = JointState()
        joint_out.name = self.q_feed.name
        joint_out.position = [q_sol.position[0], q_sol.position[1], q_sol.position[2], q_sol.position[3], 0.0]

        # Devolvemos el mensaje en el topic "joint_out"
        response.joint_out = joint_out
        response.success = ok
        self.get_logger().info(f'Respuesta enviada: {joint_out.position}')
        self.get_logger().info(f'Calculo exitoso: {ok}')
        self.get_logger().info(f'Numero de iteraciones: {iters}')
        self.get_logger().info(f'Error en la aproximación: {err}')
        return response


    # ------------------------------ Callback del Suscriptor ------------------------------ 
    def joint_callback(self, msg: JointState):
        self.q_feed.position = msg.position


def main(args=None):
    rclpy.init(args=args)
    node = Enfoque1Server()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
