from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'URDF'), glob('URDF/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hkumar456',
    maintainer_email='hkm@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # node_name = pkg_name.file_name:function_name
            'joint_space_cubic_trajectory_node = robot_arm.Joint_Space_Cubic_Trajectory:main'
            # 'task_space_cubic_trajectory_node = robot_arm.Task_Space_Cubic_Trajectory:main'
        ],
    },
)
