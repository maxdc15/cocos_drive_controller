from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cocos_drive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test'], include=[package_name, 'lib', 'lib.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eduardohufg',
    maintainer_email='eduardochavezmartin10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detection = cocos_drive_controller.color_detection:main',
            'cmd_robot = cocos_drive_controller.cmd_robot:main',
            'line_detection = cocos_drive_controller.line_detection:main',
            'main_controller = cocos_drive_controller.main_controller:main',
        ],
    },
)
