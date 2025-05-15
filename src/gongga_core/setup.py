from setuptools import find_packages, setup
from glob import glob

package_name = 'gongga_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chengdu Humanoid Innovation Center',
    maintainer_email='zhangxianglin17@outlook.com',
    description='ray_robot core ros package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_aruco_markers = gongga_core.detect_aruco_markers:main',
            'gongga_driver = gongga_core.gongga_driver:main'
        ],
    },
)
