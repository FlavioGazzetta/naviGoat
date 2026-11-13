from setuptools import setup


package_name = 'middleman'

setup(
    name=package_name,
    version='0.0.1',
    py_modules=['middleman', 'middleman_node', 'catalog'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/middleman.launch.py']),
    ],
    install_requires=['setuptools'],
    maintainer='evangeliakommata',
    maintainer_email='evangeliakommata@users.noreply.github.com',
    description='Decides what a spoken request means and tells perception and navigation',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'middleman_node = middleman_node:main',
        ],
    },
)
