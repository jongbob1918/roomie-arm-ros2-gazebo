# setup.py

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roomie_ac'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        # [핵심 수정] data 폴더의 상대 경로를 정확히 지정합니다.
        (os.path.join('share', package_name, 'data'), glob('roomie_ac/data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mac',
    maintainer_email='mac@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'ac_node = roomie_ac.ac_node:main',
        ],
    },
)