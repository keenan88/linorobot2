from setuptools import find_packages, setup

package_name = 'linorobot2_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry = linorobot2_localization.wheel_odometry:main',
            'wheel_unraveller = linorobot2_localization.sim_wheel_rotation_unraveller:main',
            'amcl_visualizer = linorobot2_localization.amcl_visualizer:main',
            'path_orientation_updater = linorobot2_localization.path_orientation_updater:main',
            'slam_image_recorder = linorobot2_localization.slam_image_recorder:main',
            'slam_recording_publisher = linorobot2_localization.slam_recording_publisher:main',
            'detected_dynamic_obstacles_publisher = linorobot2_localization.detected_dynamic_obstacles_publisher:main'
        ],
    },
)
