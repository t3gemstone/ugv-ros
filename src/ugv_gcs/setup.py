from glob import glob

from setuptools import setup

package_name = 'ugv_gcs'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/web', glob('web/*.*')),
        ('share/' + package_name + '/web/vendor', glob('web/vendor/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='T3 Gemstone',
    maintainer_email='support@t3gemstone.org',
    description='Browser-based host control UI (rosbridge)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_pose_node = ugv_gcs.robot_pose_node:main',
        ],
    },
)
