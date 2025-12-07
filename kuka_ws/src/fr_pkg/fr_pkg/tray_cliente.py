import rclpy
from simple_actions import SimpleActionClient
from cmd_pkg.action import Control
import math
import time

def interpolar_lineal(p0, pf, pasos=4):
    """Genera puntos interpolados entre p0 y pf solo para x, y, z.
       La garra g se mantiene constante."""
    x0, y0, z0, g = p0
    xf, yf, zf, _ = pf
    puntos = []

    for i in range(pasos + 1):  # incluye inicio y final
        t = i / pasos
        x = x0 + t * (xf - x0)
        y = y0 + t * (yf - y0)
        z = z0 + t * (zf - z0)
        puntos.append((x, y, z, g))  # g constante
    return puntos

def leer_valor(prompt, default="0.0"):
    """Lee un valor del usuario, permitiendo expresiones tipo pi/7."""
    entrada = input(prompt) or default
    return eval(entrada, {"__builtins__": None}, {"pi": math.pi})

def main():
    rclpy.init()
    node = rclpy.create_node('tray_cliente')
    client = SimpleActionClient(node, Control, 'control')

    print("\n🤖 CLIENTE DE TRAYECTORIA MULTIPLE CON GARRA FIJA\n")

    # Leer número de puntos de la trayectoria
    num_puntos = int(input("Número de puntos de la trayectoria: ") or "2")
    
    # Lista para almacenar los puntos
    puntos = []
    
    # Leer todos los puntos
    for i in range(num_puntos):
        print(f"\nIngrese PUNTO {i+1} (x, y, z):")
        x = leer_valor("  x: ")
        y = leer_valor("  y: ")
        z = leer_valor("  z: ")
        if i == 0:
            # Solo en el primer punto se pide la garra, será constante
            g = leer_valor("  Ángulo de garra (constante): ")
        puntos.append((x, y, z, g))
    
    # Número de pasos entre cada par de puntos
    pasos = int(input("\nNúmero de pasos de interpolación entre puntos (ej: 4): ") or "4")

    # Modo de control
    modo = int(input("\nModo de control (1 o 2): ") or "1")

    # Generar y enviar la trayectoria completa
    for i in range(len(puntos)-1):
        p0 = puntos[i]
        pf = puntos[i+1]
        trayectoria = interpolar_lineal(p0, pf, pasos)
        
        print(f"\nInterpolando del punto {i+1} al punto {i+2}:")

        for j, (x, y, z, g) in enumerate(trayectoria):
            print(f" Enviando paso {j+1}/{len(trayectoria)}: ({x:.3f}, {y:.3f}, {z:.3f}, {g:.3f})")
            goal_msg = Control.Goal()
            goal_msg.x = x
            goal_msg.y = y
            goal_msg.z = z
            goal_msg.g = g
            goal_msg.enfoque = modo

            result_code, result_msg = client(goal_msg)

            print(f"  {result_msg.descripcion}")
            print(f"  Pos final: ({result_msg.x_final:.3f}, {result_msg.y_final:.3f}, {result_msg.z_final:.3f})")
            if hasattr(result_msg, 'g_final'):
                print(f"  Garra final: {result_msg.g_final:.3f}")

    print("\n" + "="*50)
    print("Trayectoria múltiple completada con garra constante.")
    print("="*50)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


