import rclpy
from simple_actions import SimpleActionClient
from cmd_pkg.action import Control
import time
import math

# -----------------------------------------------------------
# Función de interpolación lineal (tomada del primer script)
# -----------------------------------------------------------
def interpolar_lineal(p0, pf, pasos=4):
    """Genera puntos interpolados entre p0 y pf solo para x, y, z.
       La garra g se mantiene constante."""
    x0, y0, z0, g = p0
    xf, yf, zf, g2 = pf   # g2 no se usa, g se mantiene constante
    puntos = []
    for i in range(pasos + 1):  # incluye inicio y final
        t = i / pasos
        x = x0 + t * (xf - x0)
        y = y0 + t * (yf - y0)
        z = z0 + t * (zf - z0)
        puntos.append((x, y, z, g))
    return puntos


def main():
    # Inicializar nodo
    rclpy.init()
    node = rclpy.create_node('cosecha')
    client = SimpleActionClient(node, Control, 'control')

    print("\n🤖 CLIENTE DE CONTROL DE USUARIO TRAYECTORIA COSECHA")

    # -----------------------------------------------------------
    # Selección del modo de control
    # -----------------------------------------------------------
    print("\nModos de control disponibles:")
    print("1: Cinematica Inversa + Control Posicion Articular")
    print("2: Control Cinematico + Lazo Interno")
    modo = int(input("Seleccione modo 1 o 2: ") or "1")
    print(f"\nModo de control seleccionado: {modo}")

    # -----------------------------------------------------------
    # Número de pasos de interpolación
    # -----------------------------------------------------------
    pasos = int(input("\nNúmero de pasos de interpolación entre puntos: ") or "4")
    print(f"Se usarán {pasos} pasos de interpolación entre puntos.\n")

    # -----------------------------------------------------------
    # Puntos predefinidos
    # -----------------------------------------------------------
    puntos = [
        (0.15, -0.15, 0.2, 0.0),     # HOME + GARRA CERRADA

        (0.2, 0.0, 0.12, 0.0),       # ARRIBA RECOJO
        (0.2, 0.0, 0.03, 1.57),      # ABAJO RECOJO + GARRA ABIERTA
        (0.2, 0.0, 0.03, 0.1),       # CERRAR GARRA
        (0.2, 0.0, 0.12, 0.1),       # SUBIR

        (0.0, -0.2, 0.12, 0.1),      # ARRIBA ATERRIZAJE
        (0.0, -0.2, 0.02, 0.1),      # BAJAR
        (0.0, -0.2, 0.02, 1.57),     # ABRIR GARRA
        (0.0, -0.2, 0.02, 0.1),      # CERRAR GARRA
        (0.0, -0.2, 0.12, 0.1),      # SUBIR

        (0.15, -0.15, 0.2, 0.00),   # HOME FINAL
    ]

    n = len(puntos)
    print(f"Se procesarán {n} puntos internos con interpolación.\n")

    # -----------------------------------------------------------
    # Interpolación + Envío de puntos
    # -----------------------------------------------------------
    for i in range(n - 1):
        p0 = puntos[i]
        pf = puntos[i + 1]

        print(f"\n==============================")
        print(f"Interpolando del punto {i+1} al punto {i+2}")
        print(f"==============================")

        trayectoria = interpolar_lineal(p0, pf, pasos)

        for j, (x, y, z, g) in enumerate(trayectoria, start=1):
            print(f"\nPaso {j} de {len(trayectoria)}")
            print(f"  Posición interpolada: ({x:.3f}, {y:.3f}, {z:.3f})")
            print(f"  Garra: {g:.3f}")

            goal_msg = Control.Goal()
            goal_msg.x = x
            goal_msg.y = y
            goal_msg.z = z
            goal_msg.g = g
            goal_msg.enfoque = modo

            result_code, result_msg = client(goal_msg)

            # Pequeña pausa según el modo
            if modo == 1:
                time.sleep(2)
            else:
                time.sleep(1)

            print(f"  Resultado: {result_msg.descripcion}")
            print(f"  Posición final: ({result_msg.x_final:.3f}, {result_msg.y_final:.3f}, {result_msg.z_final:.3f})")

    print("\n✔ Trayectoria con interpolación completada.")
    print("=" * 50)

    rclpy.shutdown()


if __name__ == '__main__':
    main()



# import rclpy
# from simple_actions import SimpleActionClient
# from cmd_pkg.action import Control
# import time

# def main():
#     # Inicializar nodo
#     rclpy.init()
#     node = rclpy.create_node('cosecha')
#     client = SimpleActionClient(node, Control, 'control')

#     print("\n🤖 CLIENTE DE CONTROL DE USUARIO (TRAYECTORIA COSECHA)")

#     # ------------------------------------------------------------------
#     # Modo de control definido internamente
#     # ------------------------------------------------------------------
#     print("\nModos de control disponibles:")
#     print("1: Cinematica Inversa + Control Posicion Articular")
#     print("2: Control Cinematico + Lazo Interno")
#     modo = int(input("Seleccione modo (1 o 2): ") or "1")
#     print(f"\nModo de control seleccionado: {modo}")

#     # ------------------------------------------------------------------
#     # Lista de puntos predefinida (x, y, z, g)
#     # ------------------------------------------------------------------
#     puntos = [
#         (0.15, -0.15, 0.2, 0.0),   # HOME + GARRA CERRADA

#         (0.2, 0.0, 0.12, 0.0),     # ARRIBA RECOJO + GARRA CERRADA
#         (0.2, 0.0, 0.03, 1.57),     # ABAJO RECOJO + GARRA ABIERTA
#         (0.2, 0.0, 0.03, 0.1),      # ABAJO RECOJO + GARRA CERRADA
#         (0.2, 0.0, 0.12, 0.1),      # ARRIBA RECOJO + GARRA CERRADA

#         (0.0, -0.2, 0.12, 0.1),     # ARRIBA ATERRIZAJE + GARRA CERRADA
#         (0.0, -0.2, 0.02, 0.1),     # ABAJO ATERRIZAJE + GARRA CERRADA
#         (0.0, -0.2, 0.02, 1.57),    # ABAJO ATERRIZAJE + GARRA ABIERTA
#         (0.0, -0.2, 0.02, 0.1),     # ABAJO ATERRIZAJE + GARRA CERRADA
#         (0.0, -0.2, 0.12, 0.1),     # ARRIBA ATERRIZAJE + GARRA CERRADA

#         (0.15, -0.15, 0.12, 0.00),  # HOME + GARRA CERRADA 
#     ]

#     n = len(puntos)
#     print(f"\nSe enviarán {n} puntos definidos internamente.\n")

#     # ------------------------------------------------------------------
#     # Envío de puntos
#     # ------------------------------------------------------------------
#     for i, (x, y, z, g) in enumerate(puntos, start=1):
#         print(f"\n======== Enviando punto {i} de {n} ========")
#         print(f"  Posición: ({x}, {y}, {z})")
#         print(f"  Garra: {g}")

#         goal_msg = Control.Goal()
#         goal_msg.x = x
#         goal_msg.y = y
#         goal_msg.z = z
#         goal_msg.g = g
#         goal_msg.enfoque = modo

#         # Enviar y esperar respuesta
#         result_code, result_msg = client(goal_msg)
#         if (modo == 1):
#             time.sleep(2)
#         else:
#             time.sleep(1)
        
#         print("\n  >> RESULTADO DEL SERVIDOR <<")
#         print(f"     Código: {result_code.name}")
#         print(f"     Mensaje: {result_msg.descripcion}")
#         print(f"     Posición alcanzada: ({result_msg.x_final:.3f}, {result_msg.y_final:.3f}, {result_msg.z_final:.3f})")
#         print("  -------------------------------")
        
#     print("\n✔ Trayectoria enviada completamente.")
#     print("="*50)

#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()


