from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'aric_camera_calibration'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdullah AlShateri',
    maintainer_email='abdullah.alshateri@ku.ac.ae',
    description='Camera-robot hand-eye calibration for Doosan M1013 (ROS 2)',
    license='BSD',
    entry_points={
        'console_scripts': [
            'collect_and_calibrate = aric_camera_calibration.data_collection_routine:main',
            'charuco_check         = aric_camera_calibration.charuco_check:main',
            'publish_tcp_T_cam_tf  = aric_camera_calibration.publish_tcp_T_cam_tf:main',
            'pixel_picker          = aric_camera_calibration.pixel_picker:main',
        ],
    },
)
