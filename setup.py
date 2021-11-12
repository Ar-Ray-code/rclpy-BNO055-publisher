
from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'bno055_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.launch.py')),
    ],

    entry_points={
        'console_scripts': [
            'scripts_main = '+ package_name +'.main:ros_main',
        ],
    }
)