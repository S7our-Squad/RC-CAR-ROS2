from setuptools import find_packages, setup

package_name = 'ld_ctrl'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/ld_ctrl.launch.py',           # YDLiDAR driver
            'launch/lane_detection.launch.py',    # Lane detection
            'launch/slam_async.launch.py',        # SLAM
            'launch/base_link.launch.py',         # TF frames
        ]),
        ('share/' + package_name + '/config', [
            'config/ydlidar.yaml',
            'config/slam_toolbox_async.yaml',
            'config/lane_detection_config.yaml',  # Lane detection config
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nitish',
    maintainer_email='nitish@example.com',
    description='Lane Detection + YDLiDAR Control for RC-CAR-ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector_node = ld_ctrl.lane_detector:main',
            'lane_controller_node = ld_ctrl.lane_controller:main',
        ],
    },
)

