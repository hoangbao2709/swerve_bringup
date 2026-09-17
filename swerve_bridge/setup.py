from setuptools import setup
from glob import glob
import os

package_name = 'swerve_bridge'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'websocket-client'],
    zip_safe=True,
    entry_points={'console_scripts': ['swerve_bridge_node = swerve_bridge.bridge_node:main']},
)
