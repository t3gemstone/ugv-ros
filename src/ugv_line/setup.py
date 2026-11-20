from setuptools import setup

package_name = 'ugv_line'

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
    description='Deneyap / QTR-RC / digital IR line sensor driver and PD line following',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_sensor_node = ugv_line.line_sensor_node:main',
            'line_follower_node = ugv_line.line_follower_node:main',
            'line_check = ugv_line.line_check:main',
        ],
    },
)
