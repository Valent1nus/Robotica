import os
from glob import glob
from setuptools import setup

package_name = 'car_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Copia los archivos de launch
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.py'))),
        
        # Copia los archivos del mundo .wbt
        (os.path.join('share', package_name, 'world'), 
            glob(os.path.join('world', '*.wbt'))),
            
        # Copia los archivos de la red SUMO
        (os.path.join('share', package_name, 'world', 'city_traffic_net'), 
            glob(os.path.join('world', 'city_traffic_net', '*'))),
            
        # Copia las imágenes de señales de tráfico
        (os.path.join('share', package_name, 'signs'), 
            glob(os.path.join('signs', '*.png'))),
            
        # Asegúrate de copiar el URDF a la carpeta 'resource'
        (os.path.join('share', package_name, 'resource'), 
            glob(os.path.join('resource', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usuario',
    maintainer_email='usuario@example.com',
    description='Car autonomous navigation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    # Se eliminan todos los entry_points de los controladores de Webots/robot
    entry_points={
    'webots_ros2_driver': [
        'vehicle_controller =       car_pkg.vehicle_controller:VehicleController'
    ],
}
,
)
