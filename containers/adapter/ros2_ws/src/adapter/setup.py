from setuptools import setup
import os
from glob import glob

package_name = 'adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rm',
    maintainer_email='rm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'initialpose = adapter.initialpose:main',
            # 'scanner_bridge = adapter.scanner_bridge_node:main',
            'odom_to_tf = adapter.odom_to_tf_node:main',
            'goal_pose_to_nav = adapter.goal_pose_to_nav:main'
        ],
    },
)
