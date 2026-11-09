import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bebop_guidance'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max Gardenswartz',
    maintainer_email='mgardenswartz@ufl.edu',
    description='Example off-board RISE trajectory-tracking controller for a single Bebop 2',
    license='MPL-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bebop_guidance_node = bebop_guidance.bebop_guidance_node:main',
        ],
    },
)
