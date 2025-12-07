---

# Guía detallada de uso del proyecto

**Creado por Luis Kamt - 6/12/2025 - Proyecto FR**

## Requisitos recomendados

* **Windows 11** con **WSL2** y distribución **Ubuntu 22.04**
* **ROS 2 Humble** instalado dentro de WSL
* **VS Code** con extensiones Remote-WSL
* Conexión física al robot y al **ESP32-S3 DevKit**
* Fuente de alimentación **5V** externa para el robot (obligatoria si se controla el robot real)

---

# Paso 1: Programar el ESP32-S3 DevKit

1. Descargar la carpeta que contiene el archivo **KUKA_ESP32_v12.ino**.
2. Abrir el archivo en Arduino IDE o en PlatformIO.
3. Cargar el firmware al ESP32-S3.
4. Colocar el ESP32 en el **zócalo correspondiente de la PCB** del sistema.

**Nota:**
El robot requiere además una **alimentación independiente de 5 V**. El ESP32 conectado únicamente por USB **no es suficiente para alimentar el robot**.

---

# Paso 2: Conectar el robot (solo necesario si usas Windows 11 + WSL)

Estos comandos permiten “exportar” el puerto USB del ESP32 hacia WSL.

1. Ver dispositivos disponibles:

   ```
   usbipd list
   ```
2. Conectar el ESP32 hacia la distribución Ubuntu-22.04 en WSL:

   ```
   usbipd attach --busid 1-3 --wsl Ubuntu-22.04
   ```

**Importante:**
Este paso se usa únicamente en **Windows 11 con WSL**.
En Linux nativo no es necesario.

---

# Paso 3: Activar micro-ROS

Dentro del espacio de trabajo del micro-agente:

1. ```
   cd microros_ws/
   ```
2. Compilar:

   ```
   colcon build
   ```
3. Cargar el entorno:

   ```
   source install/setup.bash
   ```
4. Ejecutar el agente micro-ROS a través del puerto serial del ESP32:

   ```
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6
   ```

**Recordatorio:** `/dev/ttyACM0` puede variar. Verifica con:

```
ls /dev/ttyACM*
```

---

# Paso 4: Activar los nodos ROS 2 del proyecto

1. ```
   cd kuka_ws/
   ```
2. Compilar:

   ```
   colcon build
   ```
3. Cargar el entorno:

   ```
   source install/setup.bash
   ```

Ahora se pueden lanzar los nodos:

1. ```
   ros2 run fr_pkg enfoque_1
   ```
2. ```
   ros2 run fr_pkg enfoque_2
   ```
3. ```
   ros2 run fr_pkg control_servidor
   ```
4. ```
   ros2 run fr_pkg translator_node
   ```
5. Lanzar el modelo URDF del robot:

   ```
   ros2 launch kuka_description urdf.launch.py
   ```
6. Ejecutar el cliente del usuario:

   ```
   ros2 run fr_pkg user_cliente
   ```

---

# Uso alternativo sin robot físico

Si no se dispone del robot real, puede usarse el nodo simulado:

```
ros2 run fr_pkg fake_esp32
```

Este nodo reemplaza la comunicación serial y permite probar todo el flujo de ROS 2 sin hardware real.

---
