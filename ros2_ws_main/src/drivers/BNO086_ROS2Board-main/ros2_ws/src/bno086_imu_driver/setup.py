from glob import glob

from setuptools import find_packages, setup

package_name = 'bno086_imu_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='fTomo-robot',
    maintainer_email='zhiguitengyuan@gmail.com',
    description='ROS 2 driver for the BNO086_ROS2Board IMU board',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = bno086_imu_driver.imu_node:main',
            'serial_monitor = bno086_imu_driver.serial_monitor:main',
            'imu_viz = bno086_imu_driver.imu_viz:main',
        ],
    },
)
