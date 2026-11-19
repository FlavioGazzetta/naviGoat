from setuptools import find_packages, setup

package_name = 'guide_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoni',
    maintainer_email='antoni@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # command name = python.module.path:function
            # "ros2 run guide_nodes localizer" runs main() in localizer_node.py
            'localizer = guide_nodes.localizer_node:main',
            'route_follower = guide_nodes.route_follower_node:main',
            'goal_sender = guide_nodes.goal_sender_node:main',
            'route_planner = guide_nodes.route_planner_node:main',
            'fetch_osm_graph = guide_nodes.fetch_osm_graph:main',
            'plan_route = guide_nodes.plan_route_cli:main',
            'mission_manager = guide_nodes.mission_manager_node:main',
            'geocoder = guide_nodes.geocoder_node:main',
            'gps_bridge = guide_nodes.gps_bridge_node:main',
        ],
    },
)
