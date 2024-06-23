from setuptools import setup
import os
from glob import glob

package_name = 'simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hamza OKHADIR ',
    maintainer_email='hamza.okhadir2018@gmail.com',
    description='ROS2 simulator package for our RC Car project.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gym_bridge = simulator.gym_bridge:main'
        ],
    },
)
