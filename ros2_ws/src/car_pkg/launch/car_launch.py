from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
import os

def generate_launch_description():
    package_name = 'car_pkg'

    # 1. Ruta del Mundo Webots
    world = os.path.join(
        get_package_share_directory(package_name),
        'world',
        'city_traffic.wbt'
    )

    # 2. Ruta de la Descripción del Robot (IMPORTANTE: Verifica si tienes este archivo)
    # Si usas un URDF u otro archivo de descripción de robot, debe estar en 'resource'.
    robot_description = os.path.join(
        get_package_share_directory(package_name),
        'resource',
        'car.urdf' # <--- ASEGÚRATE DE QUE ESTA RUTA ES CORRECTA
    )

    # 3. NODO DEL DRIVER DE WEBOTS (EL CONTROLADOR EXTERNO)
    my_robot_driver = WebotsController(
        robot_name='car',
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([

        # 4. Lanzar Webots
        ExecuteProcess(
            cmd=[
                '/home/usuario/Robotica/webots-R2025a-x86-64/webots/webots',
                world
            ],
            output='screen'
        ),

        # 5. Lanzar el Driver (este se conecta al puerto 1234)
        my_robot_driver
    ])
