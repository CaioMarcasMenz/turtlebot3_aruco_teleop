from setuptools import setup

package_name = 'turtlebot3_aruco_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/marker_teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dein Name',
    maintainer_email='dein.name@example.com',
    description='TurtleBot3 teleop using RealSense + ArUco',
    license='MIT',
    entry_points={
        'console_scripts': [
            'marker_teleop = turtlebot3_aruco_teleop.marker_teleop:main',
        ],
    },
)
