from setuptools import setup
import os
from glob import glob

package_name = 'drone_detector_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all files from the 'launch' folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all files from the 'worlds' folder
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # Include all files from the 'rviz' folder
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AntonioLuisGH',
    maintainer_email='user@todo.todo',
    description='Simulation for detecting a drone with a lidar.',
    license='Apache-2.0',
    
    # --- FIX ---
    # 'tests_require' is deprecated. Use 'extras_require' instead.
    extras_require={
        'test': ['pytest']
    },
    # --- END FIX ---
    
    entry_points={
        'console_scripts': [
            'ground_truth_bbox_node = drone_detector_sim.ground_truth_bbox_node:main',
            'drone_oscillator_node = drone_detector_sim.drone_oscillator_node:main',
        ],
    },
)