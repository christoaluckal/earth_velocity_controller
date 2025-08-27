from setuptools import find_packages, setup
from glob import glob

package_name = 'earth_velocity_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/csv', glob('csv/*.csv'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christoa',
    maintainer_email='christoa@buffalo.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_controller = earth_velocity_controller.controller:main',
            'velocity_tracker = earth_velocity_controller.tracker:main'
        ],
    },
)
