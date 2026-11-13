from setuptools import setup


package_name = 'wristband_nodes'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/wristband_launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='maxryan4',
    maintainer_email='161385834+maxryan4@users.noreply.github.com',
    description='ROS nodes for the vibrating wristband',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'ble_bridge = wristband_nodes.ble_bridge:main',
        ],
    },
)
