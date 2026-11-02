from setuptools import find_packages, setup

package_name = 'sewer_localization'

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
    description='Baseline and reliability-aware particle filters',
    license='MIT',
    entry_points={
        'console_scripts': [
            'localizer_node = sewer_localization.localizer_node:main',
            'scan_disturbance_node = sewer_localization.scan_disturbance_node:main',
            'odom_disturbance_node = sewer_localization.odom_disturbance_node:main',
        ],
    },
)
