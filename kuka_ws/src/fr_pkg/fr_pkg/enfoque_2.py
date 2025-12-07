# #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cmd_pkg.srv import Enfoque2
import numpy as np
from copy import copy
from math import pi

class KinematicServer(Node):
    def __init__(self):
        super().__init__('enfoque_2')
        self.srv = self.create_service(Enfoque2, 'enfoque_2', self.kinematic_callback)
        self.get_logger().info('Servidor enfoque_2 listo.')

        # Parámetros de control
        # self.K = -0.05       # Ganancia proporcional
        # self.lam = 0.01    # Parámetro de regularización λ
        # self.dt = 0.5    # Paso de integración (s)

        self.K = -0.1       # Ganancia proporcional
        self.lam = 0.01    # Parámetro de regularización λ
        self.dt = 0.5    # Paso de integración (s)

    def dh(self, theta_deg, d, alpha_deg, a):
        theta = np.deg2rad(theta_deg)
        alpha = np.deg2rad(alpha_deg)
        cth, sth = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        return np.array([
            [cth, -ca * sth, sa * sth, a * cth],
            [sth,  ca * cth, -sa * cth, a * sth],
            [0.0,      sa,       ca,     d],
            [0.0,     0.0,      0.0,     1.0]], dtype=float)

    def fkine_kuka(self, q):
        q1 = np.rad2deg(q[0])
        q2 = np.rad2deg(q[1])
        q3 = np.rad2deg(q[2])
        q4 = np.rad2deg(q[3])

        # Parametros DH del robot
        d = [27.525, 0, 0, 0]
        theta = [q1, 180 + q2, q3, -90 - q4]
        a = [0, -86.036, -70, 95.651]
        alpha = [90, 180, 0, 0]

        # Transformacion base opcional (corrigida)
        T_base = np.array([
            [0, 1, 0, 0],
            [-1, 0, 0, 0],
            [0, 0, 1, 45],
            [0, 0, 0, 0]
        ], dtype=float)

        # Calculo de transformaciones DH
        T01 = self.dh(theta[0], d[0], alpha[0], a[0])
        T12 = self.dh(theta[1], d[1], alpha[1], a[1])
        T23 = self.dh(theta[2], d[2], alpha[2], a[2])
        T34 = self.dh(theta[3], d[3], alpha[3], a[3])

        # Cadena cinematica completa
        T = T_base @ T01 @ T12 @ T23 @ T34

        return T/1000.0

    def jacobian_position(self, q, delta=0.0001):
        q = np.asarray(q, dtype=float)
        n = q.size
        J = np.zeros((3, n))
        T = self.fkine_kuka(q)
        x = T[0:3, 3]

        for i in range(n):
            dq = copy(q)
            dq[i] += delta
            Ti = self.fkine_kuka(dq)
            xi = Ti[0:3, 3]
            J[:, i] = (xi - x) / delta
        return J

    # ======================================
    # Callback del servicio
    # ======================================
    def kinematic_callback(self, request, response):
        e = np.array(request.e).reshape((3, 1))
        q_prev = np.array(request.joint_angles).reshape((4, 1))

        # Paso 1: derivada del error
        e_dot = self.K * e

        # Paso 2: calcular Jacobiano analítico
        J = self.jacobian_position(q_prev.flatten())

        # Pseudoinversa regularizada de J
        JJt = J @ J.T
        J_pinv = J.T @ np.linalg.inv(JJt + (self.lam**2) * np.eye(JJt.shape[0]))

        # Paso 3: velocidad articular
        q_dot = J_pinv @ e_dot

        # Paso 4: integración de Euler
        q_k = q_prev + self.dt * q_dot

        # Limitar cada ángulo al rango permitido
        q_k = np.clip(q_k, 0.0, 3.14159)

        response.qk = q_k.flatten().tolist()
        self.get_logger().info(f'Nuevos ángulos Q_k: {response.qk}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = KinematicServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()