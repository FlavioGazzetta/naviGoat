from setuptools import setup


package_name = 'navigation_nodes'

setup(
    name=package_name,
    version='0.0.1',
    py_modules=['places_node', 'people_node', 'feedback_node'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation.launch.py', 'launch/navigation_sim.launch.py',
                                                   'launch/navigation_robot.launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='nihil',
    maintainer_email='nihil@users.noreply.github.com',
    description='Drives the robot with Nav2 to places, objects and points, and puts people on the map',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'places_node = places_node:main',
            'people_node = people_node:main',
            'feedback_node = feedback_node:main',
        ],
    },
)
