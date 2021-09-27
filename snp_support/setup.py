import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
from setuptools import setup

package_name = 'snp_support'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes/hc10/visual'), glob('meshes/hc10/visual/*')),
        (os.path.join('share', package_name, 'meshes/hc10/collision'), glob('meshes/hc10/collision/*')),
        (os.path.join('share', package_name, 'meshes/pushcorp'), glob('meshes/pushcorp/*')),
        (os.path.join('share', package_name), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rchristopher',
    maintainer_email='rchristopher@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
