Creado por Luis Kamt el 4/11/2025 para el proyecto de FR



###### **Conectar el robot**

1. usbipd list
2. usbipd attach --busid 1-3 --wsl Ubuntu-22.04



###### **Activar micro ROS**

1. cd microros\_ws/
2. colcon build
3. source install/setup.bash
4. ros2 run micro\_ros\_agent micro\_ros\_agent serial --dev /dev/ttyACM0 -v6





###### **Activar los nodos**

1. cd kuka\_ws/
2. colcon build
3. source install/setup.bash



1. ros2 run fr\_pkg enfoque\_1
2. ros2 run fr\_pkg enfoque\_2
3. ros2 run fr\_pkg control\_servidor
4. ros2 run fr\_pkg translator\_node
5. ros2 launch kuka\_description urdf.launch.py
6. ros2 run fr\_pkg user\_cliente
