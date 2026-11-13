from setuptools import setup


package_name = 'bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='maxryan4',
    maintainer_email='161385834+maxryan4@users.noreply.github.com',
    description='Launch naviGoat nodes',
    license='Proprietary',
)
