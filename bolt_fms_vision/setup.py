from setuptools import find_packages, setup

package_name = 'bolt_fms_vision'

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
    maintainer='kgyucheol',
    maintainer_email='kdjin11@naver.com',
    description='AprilTag Based Vision Pose Publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_tracker_node = bolt_fms_vision.tag_tracker_node:main',
        ],
    },
)