import math
import numpy as np
from sensor_msgs.msg import JointState


# ===============================
# Transformaciones homogeneas
# ===============================

def sTrasl(x, y, z):
    """Matriz de traslacion homogenea."""
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ], dtype=float)


def sTrotx(ang):
    """Rotacion alrededor del eje X (radianes)."""
    c, s = np.cos(ang), np.sin(ang)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0, 0, 1]
    ], dtype=float)


def sTroty(ang):
    """Rotacion alrededor del eje Y (radianes)."""
    c, s = np.cos(ang), np.sin(ang)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1]
    ], dtype=float)


def sTrotz(ang):
    """Rotacion alrededor del eje Z (radianes)."""
    c, s = np.cos(ang), np.sin(ang)
    return np.array([
        [ c, -s, 0, 0],
        [ s,  c, 0, 0],
        [ 0,  0, 1, 0],
        [ 0,  0, 0, 1]
    ], dtype=float)


# ===============================
# Denavit–Hartenberg numerico
# ===============================

def dh(theta_deg, d, alpha_deg, a):
    """Calculo de la matriz DH (angulos en grados, longitudes en mm)."""
    theta = np.deg2rad(theta_deg)
    alpha = np.deg2rad(alpha_deg)
    cth, sth = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [cth, -ca * sth, sa * sth, a * cth],
        [sth,  ca * cth, -sa * cth, a * sth],
        [0.0,      sa,       ca,     d],
        [0.0,     0.0,      0.0,     1.0]
    ], dtype=float)


# ===============================
# Cinematica directa
# ===============================

def cinem_directa(q):
    """Devuelve la posicion [x, y, z] del efector final (en metros)."""
    if len(q.position) < 4:
        print("No hay suficientes valores en q.position (se requieren 4).")
        return np.array([0.0, 0.0, 0.0])

    # Convertir de radianes a grados y ajustar offset mecanico
    q1 = np.rad2deg(q.position[0])
    q2 = np.rad2deg(q.position[1])
    q3 = np.rad2deg(q.position[2])
    q4 = np.rad2deg(q.position[3])

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

    # Posicion final en metros
    translation = T[0:3, 3] / 1000.0
    x_vec = np.round(translation, 3)
    return x_vec



# =============================================================
# Jacobiana numérica de posición + IK (Newton / DLS opcional)
# =============================================================
def jacobian_position(fkine_fun, q: JointState):
    """
    Jacobiana de posición Jp(q) ∈ R^{3x4} por diferencias finitas centradas.
    """
    J = np.zeros((3, 4), dtype=float)
    delta=np.deg2rad(1)

    for i in range(4):
        q_plus = JointState()
        q_minus = JointState()
        q_plus.position = list(q.position)
        q_minus.position = list(q.position)

        q_plus.position[i] += delta
        q_minus.position[i] -= delta

        p_plus = fkine_fun(q_plus)
        p_minus = fkine_fun(q_minus)

        J[:, i] = (p_plus - p_minus) / (2.0 * delta)

    return np.matrix(J)



# ===============================
# Cinematica inversa
# ===============================

def ikine(fkine_fun, jac_fun, p_target, q, eps=1.0e-4, max_iter=100, delta=1.0e-6, lam=0.0, dq_clip=0.25):
    """
    IK de posición mediante Newton (pseudoinversa) o DLS si lam>0.
    """
    q_min = np.deg2rad([0.0, 0.0, 0.0, 0.0])
    q_max = np.deg2rad([180.0, 180.0, 180.0, 180.0])

    for k in range(max_iter):
        p = fkine_fun(q)
        e = p_target - p

        # Condicion de parada
        if np.linalg.norm(e) <= eps:
            return q, True, k, float(np.linalg.norm(e))

        # Jacobiano numerico
        J = jac_fun(fkine_fun, q)
        if lam > 0.0:
            # Damped Least Squares: dq = (J^T J + λ^2 I)^{-1} J^T e
            H = J.T @ J + (lam ** 2) * np.eye(J.shape[1])
            g = np.asarray(J.T @ e).reshape(-1)

            try:
                dq = np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(J) @ e
        else:
            dq = np.linalg.pinv(J) @ e

        # Limitador de paso infinito-norma (estabilidad numérica)
        step = np.linalg.norm(dq, np.inf)
        if step > dq_clip:
            dq *= (dq_clip / step)


        for i, val in enumerate(dq):
            q.position[i] = q.position[i] + float(val)
            if q.position[i] < q_min[i]:
                q.position[i] = q_min[i]
                dq[i] = -dq[i]
            elif q.position[i] > q_max[i]:
                q.position[i] = q_max[i]
                dq[i] = -dq[i] 

    e_final = p_target - fkine_fun(q)
    return q, False, max_iter, float(np.linalg.norm(e_final))