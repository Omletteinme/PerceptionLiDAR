import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'sewer_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='account2',
    maintainer_email='sindhurareddyyy@gmail.com',
    description='Simulated inspection arm/gripper: pick, carry, release with evidence-based verification',
    license='MIT',
    entry_points={
        'console_scripts': [
            'manipulation_node = sewer_manipulation.manipulation_node:main',
            'scan_self_filter_node = sewer_manipulation.scan_self_filter_node:main',
        ],
    },
)
