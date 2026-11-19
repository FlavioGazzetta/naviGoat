from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'guide_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Step 10: canned Nominatim answers for fake_nominatim (guide.launch.py
        # sim:=true), copied from guide_core/test/data. glob() runs at build time.
        (os.path.join('share', package_name, 'nominatim'),
            glob('nominatim/*.json') + glob('nominatim/*.md')),
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
            'sim_robot = guide_sim.sim_robot_node:main',
            'fake_nominatim = guide_sim.fake_nominatim:main',
            'run_evaluator = guide_sim.run_evaluator:main',
        ],
    },
)
