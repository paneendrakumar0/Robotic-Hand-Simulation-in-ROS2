import os
from glob import glob
from setuptools import setup

package_name = 'dexhand_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
      
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='DexHand Advanced Control',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
       
            'advanced_tracker = dexhand_control.advanced_tracker:main',
        ],
    },
)
