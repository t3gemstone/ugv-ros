from glob import glob

from setuptools import setup

package_name = 'ugv_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='T3 Gemstone',
    maintainer_email='support@t3gemstone.org',
    description='slam_toolbox + Nav2 configuration and map saving service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_manager_node = ugv_navigation.map_manager_node:main',
        ],
    },
)
