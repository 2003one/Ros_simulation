import os
from glob import glob
from setuptools import setup

package_name = 'spider_bot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
	(os.path.join('share', package_name, 'config'),
       	    glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anup',
    maintainer_email='abhishekc.dev@gmail.com',
    description='URDF description for a 12-DOF quadruped spider bot',
    license='MIT',
    entry_points={
        'console_scripts': [
                'pub          = pubsub.publisher:main',
                'sub          = pubsub.subscriber:main',
                'walk_forward = spider_bot_description.walk_forward:main',
                
            ],
    },
)
