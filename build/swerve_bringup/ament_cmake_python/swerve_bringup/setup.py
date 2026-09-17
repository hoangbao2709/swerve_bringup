from setuptools import find_packages
from setuptools import setup

setup(
    name='swerve_bringup',
    version='0.1.0',
    packages=find_packages(
        include=('swerve_bringup', 'swerve_bringup.*')),
)
