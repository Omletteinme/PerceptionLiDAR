from setuptools import find_packages, setup

package_name = 'sewer_planning'

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
    description='Uncertainty-aware route planning on the sewer graph',
    license='MIT',
    entry_points={
        'console_scripts': [
            'route_planner_node = sewer_planning.route_planner_node:main',
            'route_follower_node = sewer_planning.route_follower_node:main',
            'exploration_node = sewer_planning.exploration_node:main',
        ],
    },
)
