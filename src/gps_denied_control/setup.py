from setuptools import find_packages, setup
package_name = 'gps_denied_control'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='GPS Denied Dev', maintainer_email='dev@gpsdenied.dev',
    description='Trajectory tracking and PX4 offboard control bridge.',
    license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'trajectory_tracker = gps_denied_control.trajectory_tracker:main',
        'px4_commander = gps_denied_control.px4_commander:main',
    ]},
)
