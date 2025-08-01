from setuptools import setup, find_packages

package_name = 'boltbot_obstacle'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # launch files
        ('share/' + package_name + '/launch', ['launch/obstacle_yolo_launch.py']),
        # ── 여기에 모델 파일 포함 ──
        ('share/' + package_name + '/models', ['models/best.pt']),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='jhchoman0823@gmail.com',
    description='ROS 2 Python nodes for safety bubble and YOLO socket detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_node = boltbot_obstacle.obstacle_node:main',
            'yolo_node     = boltbot_obstacle.yolo_socket_node:main',
        ],
    },
)
