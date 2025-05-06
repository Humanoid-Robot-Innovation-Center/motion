import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'motion_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',  # 基本依赖
        'curobo',      # 添加curobo作为Python依赖
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='zhangxianglin17@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stable_motion_generator = motion_generator.stable_motion_generator:main'
        ],
    },
)
