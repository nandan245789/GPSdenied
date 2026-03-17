from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_denied_bringup'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='GPS Denied Dev', maintainer_email='dev@gpsdenied.dev',
    description='Launch files and system config.',
    license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'telemetry_aggregator = gps_denied_bringup.telemetry_aggregator:main',
    ]},
)
