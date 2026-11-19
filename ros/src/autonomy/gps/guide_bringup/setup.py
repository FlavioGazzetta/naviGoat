from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'guide_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Data files installed into share/guide_bringup/<folder>, where launch
        # files find them with FindPackageShare. glob() runs at BUILD time: a
        # file added later is only installed after the next colcon build.
        (os.path.join('share', package_name, 'routes'), glob('routes/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Step 7: curated destinations (mission manager), the cached OSM walk
        # graph (route planner; the .graphml is gitignored, re-fetch it with
        # fetch_osm_graph on a new machine) and the RViz configurations.
        (os.path.join('share', package_name, 'destinations'), glob('destinations/*.yaml')),
        (os.path.join('share', package_name, 'osm'),
            glob('osm/*.graphml') + glob('osm/*.meta.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'sim'), glob('sim/*.yaml')),
        # Step 10: the parameter file (origin, topic names, frames, Nav2 action).
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
        ],
    },
)
