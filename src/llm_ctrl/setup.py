from setuptools import setup, find_packages

package_name = 'llm_ctrl'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/controller.launch.py',
            'launch/json_to_ackermann.launch.py',
            'launch/llm_ctrl.launch.py',
            'launch/motor_signals.launch.py',
            'launch/input.launch.py', # Include your combined launch file if present
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='driftpilot',
    maintainer_email='driftpilot@example.com',
    description='LLM based controller for RC car',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = llm_ctrl.controller:main',
            'json_to_ackermann = llm_ctrl.json_to_ackermann:main',
            'motor_signals = llm_ctrl.motor_signals:main',
            'input = llm_ctrl.input:main',
        ],
    },
)

