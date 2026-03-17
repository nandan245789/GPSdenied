from setuptools import find_packages, setup
package_name = 'gps_denied_safety'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='GPS Denied Dev', maintainer_email='dev@gpsdenied.dev',
    description='Mission supervisor, fail-safe state machine, and safety monitor.',
    license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mission_supervisor = gps_denied_safety.mission_supervisor:main',
        'safety_monitor = gps_denied_safety.safety_monitor:main',
    ]},
)
