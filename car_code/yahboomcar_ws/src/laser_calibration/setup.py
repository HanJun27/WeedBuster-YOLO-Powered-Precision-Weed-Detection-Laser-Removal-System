from setuptools import setup
import os
from glob import glob

package_name = 'laser_calibration'

setup(
    name=package_name,
    version='3.10.12',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description=(
        '双目多光谱感知与闭环激光打击系统 —— Phase1标定 + Phase2真NDVI + Phase3视觉伺服'
    ),
    license='MIT',
    entry_points={
        'console_scripts': [
            'stereo_camera = laser_calibration.stereo_camera:main',
            'calib_camera  = laser_calibration.calib_camera_align:main',
            'calib_laser   = laser_calibration.calib_laser_offset:main',
            'calib_refl    = laser_calibration.calib_reflectance:main',
            'show_calib    = laser_calibration.show_calib:main',
            'calib_diffuse = laser_calibration.calib_diffuse:main',
            'ndvi_node     = laser_calibration.ndvi_node:main',
            'ndvi_monitor  = laser_calibration.ndvi_monitor:main',
            'vision_servo  = laser_calibration.vision_servo:main',
            'yolo_detector = laser_calibration.yolo_detector:main',
            'strike_planner = laser_calibration.strike_planner:main',
            'chassis_controller = laser_calibration.chassis_controller:main',
        ],
    },
)
