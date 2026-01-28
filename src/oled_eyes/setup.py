from setuptools import setup

package_name = 'oled_eyes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/oled_eyes.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='OLED eyes node for displaying emotions or direction',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oled_eyes = oled_eyes.oled_eyes:main'
        ],
    },
)
