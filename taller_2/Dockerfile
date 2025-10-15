# Imagen base con ROS 2 Jazzy Desktop
FROM osrf/ros:jazzy-desktop

# Definir el entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Instalar dependencias necesarias
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-matplotlib \
    nano \
    git \
    net-tools \
    iproute2 \
    iputils-ping \
    tree \
    && rm -rf /var/lib/apt/lists/*

# Crear workspace
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Crear el paquete ROS2 dentro del workspace
WORKDIR /root/ros2_ws/src
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && ros2 pkg create --build-type ament_python sensor_program --license MIT"

#  Copiar tu paquete y scripts Python
COPY rospy/sensor_node.py /root/ros2_ws/src/sensor_program/sensor_program/sensor_node.py
COPY rospy/reader_node.py /root/ros2_ws/src/sensor_program/sensor_program/reader_node.py
COPY rospy/plotter_node.py /root/ros2_ws/src/sensor_program/sensor_program/plotter_node.py
COPY rospy/setup.py /root/ros2_ws/src/sensor_program/setup.py

# Compilar el workspace
WORKDIR /root/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build"

# Crear carpeta donde se guardarán los gráficos
RUN mkdir -p /datos

# Comando por defecto al iniciar el contenedor
CMD ["bash"]
