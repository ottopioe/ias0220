from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ias0220_243627'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/differential_robot_rviz_task7.rviz')),
        (os.path.join('share', package_name, 'config'), ['config/simple_control_v2.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ottoalex',
    maintainer_email='otkare@taltech.ee',
    description='Task7',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = ias0220_243627.image_publisher:main',
            'camera_calibration = ias0220_243627.camera_calibration:main',
            'control_node = ias0220_243627.simple_control:main'

        ],
    },

)
