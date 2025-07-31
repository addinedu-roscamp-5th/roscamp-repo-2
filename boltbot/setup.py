from setuptools import find_packages, setup

package_name = 'auto_navigation_pkg'

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
    maintainer='addinedu',
    maintainer_email='woo10000k@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'auto_navigation_node = auto_navigation_pkg.auto_navigation_node:main',
        'test_pose_publisher = auto_navigation_pkg.test_pose_publisher:main',
        ],
    },
)
