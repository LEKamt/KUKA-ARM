import rclpy
from simple_actions import SimpleActionClient
from cmd_pkg.action import Control

def main():
    # Definir nodo
    rclpy.init()
    node = rclpy.create_node('user_cliente')
    client = SimpleActionClient(node, Control, 'control')

    # Pedir datos al usuario
    print("\n 🤖 CLIENTE DE CONTROL DE USUARIO")
    print("Ingrese la posicion deseada y modo de control:")

    # Se definieron valores estandar en caso no sea ingresado ningun dato 

    x = float(input("Posicion X deseada: ") or "1.5")
    y = float(input("Posicion Y deseada: ") or "0.8") 
    z = float(input("Posicion Z deseada: ") or "0.3")
    g = float(input("Angulo garra deseada: ") or "0.0")

    print("\nModos de control disponibles:")
    print("1: Cinematica Inversa + Control Posicion Articular")
    print("2: Control Cinematico + Lazo Interno")
    modo = int(input("Seleccione modo (1 o 2): ") or "1")

    # Crear objetivo
    goal_msg = Control.Goal()
    goal_msg.x = x
    goal_msg.y = y
    goal_msg.z = z
    goal_msg.g = g
    goal_msg.enfoque = modo

    print(f"\n Enviando objetivo:")
    print(f"   Posicion: ({x}, {y}, {z})")
    print(f"   Garra: ({g})")
    print(f"   Modo control: {modo}")

    # Enviar objetivo y recibir resultado
    result_code, result_msg = client(goal_msg)

    # Mostrar resultados
    print("\n" + "="*50)
    print("RESULTADO FINAL:")
    print(f"   Codigo: {result_code.name}")  # SUCCEEDED, ABORTED, etc.
    print(f"   Mensaje: {result_msg.descripcion}")
    print(f"   Posicion alcanzada: ({result_msg.x_final:.3f}, {result_msg.y_final:.3f}, {result_msg.z_final:.3f})")
    # print(f"   Error final: {result_msg.error_final:.4f} m")
    print("="*50)
	
	
if __name__ == '__main__':
    main()