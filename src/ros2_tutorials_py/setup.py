from setuptools import find_packages, setup

# Import libraries needed for launch files to be visible to ROS2
import os
from glob import glob

package_name = 'ros2_tutorials_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add path to launch folder
        (os.path.join('share', package_name, 'launch/'),
         glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yanwen',
    maintainer_email='yanwen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Executables are set here: name of executable, followed by filepath and start point for execution
            # In this case, execution point is main in ros2_tutorials_py/talker_node
            "talker_node = ros2_tutorials_py.talker_node:main",
            "listener_node = ros2_tutorials_py.listener_node:main",
            "service_node = ros2_tutorials_py.service_node:main",
            "client_node = ros2_tutorials_py.client_node:main",
            "listener_client_node = ros2_tutorials_py.listener_client_node:main",
        ],
    },
)
