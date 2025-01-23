from setuptools import find_packages, setup

package_name = 'pickn_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['share/aruco_config.yaml']),
        ('share/' + package_name, ['share/aruco_calibration.yaml']),
        ('share/' + package_name, ['share/arucoID_match.yaml']),
        ('share/' + package_name, ['share/mount_offsets.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erds',
    maintainer_email='erdie@qltyss.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_perception = pickn_place.aruco_perception:main',
            'perception_transform = pickn_place.perception_transform:main',
            'calibrate_aruco = pickn_place.calibrate_aruco:main',
            'calibrate_tool_mount = pickn_place.calibrate_tool_mount:main',
            'mount_frame_generator = pickn_place.mount_frame_generator:main',
            'marker_calibration_node = pickn_place.marker_calibration_node:main'
        ],
    },
)
