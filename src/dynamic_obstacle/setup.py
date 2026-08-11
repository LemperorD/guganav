import os

from glob import glob

from setuptools import setup


package_name = 'dynamic_obstacle'


setup(
    name=package_name,

    version='0.0.1',

    packages=[package_name],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'models'
            ),
            glob('models/*.sdf')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*.yaml')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,

    maintainer='breeze',

    maintainer_email='breeze@example.com',

    description=(
        'Dynamic obstacle manager for '
        'Ignition Gazebo Fortress'
    ),

    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'obstacle_manager = '
            'dynamic_obstacle.obstacle_manager:main',
        ],
    },
)
