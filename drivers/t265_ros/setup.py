from setuptools import setup

package_name = 't265_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/t265.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wayne',
    maintainer_email='you@example.com',
    description='T265 pose publisher',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            't265_node = t265_ros.t265_node:main',
        ],
    },
)
