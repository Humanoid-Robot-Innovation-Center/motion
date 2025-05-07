from setuptools import setup

package_name = 'camera_to_robot_calibration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='你的名字',
    maintainer_email='你的邮箱',
    description='Camera to Robot Calibration Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = camera_to_robot_calibration.calibration:main',
        ],
    },
)