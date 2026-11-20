from setuptools import setup

package_name = 'ugv_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='T3 Gemstone',
    maintainer_email='support@t3gemstone.org',
    description='Drive mode management, /cmd_vel multiplexer, emergency stop and lidar brake',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_manager_node = ugv_control.mode_manager_node:main',
        ],
    },
)
