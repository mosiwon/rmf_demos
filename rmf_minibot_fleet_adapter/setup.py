import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'rmf_minibot_fleet_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name,['config.yaml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yadunund',
    maintainer_email='yadunund@openrobotics.org',
    description='A template for an RMF fleet adapter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=rmf_minibot_fleet_adapter.fleet_adapter:main',
            'fleet_manager=rmf_minibot_fleet_adapter.fleet_manager:main'
        ],
    },
)
