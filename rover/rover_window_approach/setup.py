from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_window_approach'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='Approach the red window with a camera (mecanum rover).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'window_approach_node = rover_window_approach.window_approach_node:main',
            'window_check = rover_window_approach.check:main',
            'lens_calib = rover_window_approach.lens_calib:main',
            'window_webcam_server = rover_window_approach.webcam_server:main',
            'window_sim = rover_window_approach.sim:main',
        ],
    },
)
