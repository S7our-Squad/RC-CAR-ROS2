from setuptools import setup

package_name = 'manual_ctrl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/joy_raw.launch.py',
            'launch/joy_drive.launch.py',
            'launch/motor_signals.launch.py',
            'launch/manual_ctrl.launch.py',  # Include your combined launch file if present
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Manual control nodes for RC Car in ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_raw = manual_ctrl.joy_raw:main',
            'joy_drive = manual_ctrl.joy_drive:main',
            'motor_signals = manual_ctrl.motor_signals:main',
            #'ld_ctrl_manual = manual_ctrl.ld_ctrl_manual:main',
        ],
    },
)
