from setuptools import find_packages, setup

package_name = 'gps_denied_perception'

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
    description='Camera drivers, feature extraction, depth processing, and point cloud generation.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'feature_extractor = gps_denied_perception.feature_extractor:main',
            'depth_processor = gps_denied_perception.depth_processor:main',
            'obstacle_detector = gps_denied_perception.obstacle_detector:main',
            'surveillance_manager = gps_denied_perception.surveillance_manager:main',
        ],
    },
)
