from setuptools import find_packages, setup
package_name = 'gps_denied_state_estimation'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='GPS Denied Dev', maintainer_email='dev@gpsdenied.dev',
    description='VIO and EKF fusion for GPS-denied pose estimation.',
    license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'vio_node = gps_denied_state_estimation.vio_node:main',
        'px4_vision_bridge = gps_denied_state_estimation.px4_vision_bridge:main',
    ]},
)
