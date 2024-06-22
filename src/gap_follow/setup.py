from setuptools import setup

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OKHADIR Hamza',
    maintainer_email='hamza.okhadir2018@gmail.com',
    description='My solution for Lab 4: Follow the Gap for University of Pennsylvania course F1TENTH',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_node = gap_follow.reactive_node:main',
        ],
    },
)
