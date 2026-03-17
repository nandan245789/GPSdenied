from setuptools import find_packages, setup
package_name = 'gps_denied_mapping'
setup(
    name=package_name, version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='GPS Denied Dev', maintainer_email='dev@gpsdenied.dev',
    description='OctoMap-based local 3D mapping.',
    license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'octomap_builder = gps_denied_mapping.octomap_builder:main',
        'map_manager = gps_denied_mapping.map_manager:main',
    ]},
)
