from setuptools import find_packages, setup

package_name = 'sewer_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeevi',
    maintainer_email='jeeviteshmishra26@gmail.com',
    description='LiDAR blockage detection; object-level debris and structural defect inspection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'blockage_detector_node = sewer_perception.blockage_detector_node:main',
            'inspection_node = sewer_perception.inspection_node:main',
            'dataset_recorder_node = sewer_perception.dataset_recorder_node:main',
        ],
    },
)
