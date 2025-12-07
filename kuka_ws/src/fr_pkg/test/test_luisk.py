from fr_pkg.kuka_utils import cinem_directa, ikine, jacobian_position
from sensor_msgs.msg import JointState
import numpy as np
pi = np.pi


def dh(theta_deg, d, alpha_deg, a):
    theta = np.deg2rad(theta_deg)
    alpha = np.deg2rad(alpha_deg)
    cth, sth = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [cth, -ca * sth, sa * sth, a * cth],
        [sth,  ca * cth, -sa * cth, a * sth],
        [0.0,      sa,       ca,     d],
        [0.0,     0.0,      0.0,     1.0]], dtype=float)
        
def fkine_kuka(q):
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
    T01 = dh(theta[0], d[0], alpha[0], a[0])
    T12 = dh(theta[1], d[1], alpha[1], a[1])
    T23 = dh(theta[2], d[2], alpha[2], a[2])
    T34 = dh(theta[3], d[3], alpha[3], a[3])

    # Cadena cinematica completa
    T = T_base @ T01 @ T12 @ T23 @ T34

    return T/1000.0


def main():
    q0 = JointState()
    q0.name = ["SERVO_01", "SERVO_02", "SERVO_03", "SERVO_04"]
    q0.position = [2.357453422258683, 1.2852067750075957, 0.6795133597370732, 0.0657738137960409, 0.0]
    # print(cinem_directa(q0))

    q = [2.357453422258683, 1.2852067750075957, 0.6795133597370732, 0.0657738137960409, 0.0]
    print("Cinem. Directa de Leonardo")
    x_final = fkine_kuka(q)
    print("X final: ", end= "")
    print(x_final[0:3, 3])

    print("Cinem. Directa de Bryan")
    x_final = cinem_directa(q0)
    print("X final: ", end= "")
    print(x_final)

    # p_des = [0.04, 0.08, 0.05]
    # q0 = JointState()
    # q0.name = ["SERVO_01", "SERVO_02", "SERVO_03", "SERVO_04"]
    # q0.position = [0.0, np.pi/2, np.pi/2, 0.0]

    # x_actual = cinem_directa(q0)
    # print("X actual")
    # print(x_actual)

    # # IK (Newton con DLS leve)
    # q_sol, ok, iters, err = ikine(
    #     cinem_directa,
    #     jacobian_position,
    #     p_des,
    #     q0,
    #     eps= 1e-2,
    #     max_iter= 80,
    #     lam=1e-3,
    #     dq_clip=0.25
    # )

    # print('q_sol'), print(q_sol)
    # print('ok'), print(ok)
    # print('iters'), print(iters)
    # print('err'), print(err)

    # x_final = cinem_directa(q_sol)
    # print("X final")
    # print(x_final)


if __name__ == '__main__':
    main()