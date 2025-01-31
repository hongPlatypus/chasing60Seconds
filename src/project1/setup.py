from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'project1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeonho',
    maintainer_email='yeonho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            #detection
            'following = project1.following:main',
            'object_detection= project1.object_detection:main',

            #amr_control
            
            'init_pose = project1.init_pose:main',
            'police_office = project1.police_office:main',
            'waypoint_follower = project1.waypoint_follower:main',
        ],
    },
)
