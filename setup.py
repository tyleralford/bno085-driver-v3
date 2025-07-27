from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'bno085_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'examples'), glob('examples/*')),
        (os.path.join('share', package_name, 'firmware'), glob('src/*') + glob('include/*') + ['platformio.ini']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tyler Alford',
    maintainer_email='alford.tyler94@gmail.com',
    description='A micro-ROS driver for the BNO085 9-DOF IMU sensor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
