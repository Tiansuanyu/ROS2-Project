import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. 安装 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. 安装 urdf 文件
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        
        # 3. 安装 rviz 配置文件
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # 把 worlds 文件夹下的所有 .world 文件拷贝到安装目录
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),

        # 把 maps 文件夹下的所有文件 (.yaml, .pgm) 拷贝到安装目录
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),

        # 👇👇👇 关键检查：必须有这一行！把 config 文件夹拷过去 👇👇👇
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='My Final Project Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_master = my_robot_bringup.mission_master:main',
            'spawn_ball = my_robot_bringup.spawn_ball:main',
            'ball_detector = my_robot_bringup.ball_detector:main',
        ],
    },
)