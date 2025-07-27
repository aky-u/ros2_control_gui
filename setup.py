from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros2_control_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include example YAML configs
        (os.path.join('share', package_name, 'config'), glob('*.yaml')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='akiyoshi',
    maintainer_email='uchida.akiyoshi.s3@dc.tohoku.ac.jp',
    description='ROS 2 Control GUI for joint controller management',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller_gui = ros2_control_gui.main:main',
        ],
    },
)
