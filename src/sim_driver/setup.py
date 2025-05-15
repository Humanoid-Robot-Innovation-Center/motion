from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sim_driver'
    
setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    	'setuptools',
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='jpy1197916908@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'sim_driver = sim_driver.sim_driver:main'
        ],
    },
)
