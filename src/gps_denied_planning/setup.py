from setuptools import find_packages, setup
package_name = 'gps_denied_planning'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GPS Denied Dev',
    maintainer_email='dev@gpsdenied.dev',
    description='Waypoint manager and local path planner.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_manager = gps_denied_planning.waypoint_manager:main',
            'local_planner = gps_denied_planning.local_planner:main',
        ],
    },
)
