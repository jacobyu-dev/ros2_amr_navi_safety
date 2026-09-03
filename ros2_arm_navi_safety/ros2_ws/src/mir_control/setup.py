from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mir_control'

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
    maintainer='mox',
    maintainer_email='mox@todo.todo',
    description='Custom control package for MiR mobile robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mir_controller = mir_control.mir_controller:main'
        ],
    },
)
