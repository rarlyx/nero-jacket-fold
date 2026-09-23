from setuptools import find_packages, setup

package_name = 'agx_jacket_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liyun',
    maintainer_email='you@example.com',
    description='Scripted jacket folding sequence via FollowJointTrajectory, start/stop triggered',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jacket_sequence_node = agx_jacket_demo.jacket_sequence_node:main',
        ],
    },
)
