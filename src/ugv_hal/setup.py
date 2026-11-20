from setuptools import setup

package_name = 'ugv_hal'

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
    description='GPIO (libgpiod v1/v2) and sysfs PWM abstraction layer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_manager_node = ugv_hal.lidar_manager_node:main',
        ],
    },
)
