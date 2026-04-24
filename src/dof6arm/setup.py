from setuptools import setup
from glob import glob
import os


package_name = 'dof6arm'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
    ('share/ament_index/resource_index/packages',
    ['resource/' + package_name]),

    ('share/' + package_name, ['package.xml']),

    (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),

    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),

    (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='TODO:',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
        ],
    },
)
