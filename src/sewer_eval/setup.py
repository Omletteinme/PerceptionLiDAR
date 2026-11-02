from setuptools import find_packages, setup

package_name = 'sewer_eval'

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
    description='Evaluation tooling: collision counting from Gazebo contact sensors, shared metrics',
    license='MIT',
    entry_points={
        'console_scripts': [
            'collision_counter_node = sewer_eval.collision_counter_node:main',
        ],
    },
)
