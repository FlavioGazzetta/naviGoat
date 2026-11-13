from setuptools import setup


package_name = 'voice_nodes'

setup(
    name=package_name,
    version='0.0.1',
    py_modules=['voice', 'listen_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/voice.launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='evangeliakommata',
    maintainer_email='evangeliakommata@users.noreply.github.com',
    description='Speech to text and spoken replies',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'voice_node = voice:main',
            'listen_controller = listen_controller:main',
        ],
    },
)
