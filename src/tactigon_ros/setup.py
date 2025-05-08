from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'tactigon_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Correct message path (package root msg/ directory)
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='banjo',
    maintainer_email='banjo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
            'tactigon_turtlesim_controller = tactigon_ros.tactigon_turtlesim_controller:main',
            'tactigon_data = tactigon_ros.tactigon_data:main',
            'tactigon_logger = tactigon_ros.tactigon_logger:main',
            'braccio_position = tactigon_ros.braccio_position:main',
            'braccio_control = tactigon_ros.braccio_control:main',
        ],
    },

)