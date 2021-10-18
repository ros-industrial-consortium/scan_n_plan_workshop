from setuptools import setup

package_name = 'ros_to_moto'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='Colin Lewis',
 maintainer_email='colin.lewi@swri.org',
 description='Motoman Motion Executon Package for Roscon 2021',
 license='BSD',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'motion_execution = ros_to_moto.ros_to_moto:main'
     ],
   },
)
