from setuptools import setup

package_name = 'safety_node'

setup(
    name=package_name,
    packages=[package_name],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'safety_node=safety_node.safety_node:main',
        ],
    },
)
