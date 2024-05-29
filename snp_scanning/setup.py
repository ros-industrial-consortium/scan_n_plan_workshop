from setuptools import setup

package_name = 'snp_scanning'

setup(
    name=package_name,
    version='4.6.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    scripts=[
        'scripts/reconstruction_sim_node',
        'scripts/scan_motion_plan_from_file_node',
    ],
)
