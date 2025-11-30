#!/bin/bash

# Este script debe ejecutarse desde el directorio ~/Robotica/ros2_ws

rm -rf build/car_pkg install/car_pkg
rm -rf build/webots_ros2 install/webots_ros2

echo "1. Fuenteando el entorno base de ROS 2 (Jazzy)..."
source /opt/ros/jazzy/setup.bash

echo "2. Compilando el paquete car_pkg (modo symlink)..."
colcon build --symlink-install

if [ $? -ne 0 ]; then
    echo "ERROR: La compilación de car_pkg ha fallado. Abortando."
    exit 1
fi

echo "3. Fuenteando el entorno del workspace..."
source install/setup.bash

echo "4. Exportando PYTHONPATH para Webots R2025a..."
export WEBOTS_HOME="/home/usuario/Robotica/webots-R2025a-x86-64/webots"
export PYTHONPATH=$WEBOTS_HOME/lib/controller:$PYTHONPATH
echo "PYTHONPATH final incluye: $WEBOTS_HOME/lib/controller"

export LD_LIBRARY_PATH=$WEBOTS_HOME/lib/controller:$WEBOTS_HOME/lib:$LD_LIBRARY_PATH


echo "6. Lanzando car_pkg/car_launch.py..."
ros2 launch car_pkg car_launch.py
