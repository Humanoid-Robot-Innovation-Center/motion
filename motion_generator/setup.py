import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'motion_generator'

data_files = []
data_folder = 'data'
# 遍历 data 文件夹及其子文件夹
for root, dirs, files in os.walk(data_folder):
    # 构建目标安装路径
    install_dir = os.path.join('share', package_name, root)
    # 构建源文件路径列表
    file_paths = [os.path.join(root, f) for f in files]
    # 将源文件路径和目标安装路径添加到 data_files 列表
    data_files.append((install_dir, file_paths))
    
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
    ],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='peiyan_j@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stable_motion_generator = motion_generator.stable_motion_generator:main'
        ],
    },
)
