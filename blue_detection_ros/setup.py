from setuptools import setup

package_name = 'blue_detection_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ingenierojorgemtz',
    maintainer_email='ingenierojorgemtz@todo.todo',
    description='Paquete ROS2 que publica el feed de iRium y detecta el color azul.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'irium_publisher_node = blue_detection_ros.irium_publisher_node:main',
            'blue_detection_node = blue_detection_ros.blue_detection_node:main',
        ],
    },
)

