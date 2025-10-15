from setuptools import find_packages, setup

package_name = 'edge_rtc'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/server.yaml']),
        ('share/' + package_name + '/launch', ['launch/webrtc_video_server.launch.py', 'launch/webrtc_ros2_client.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asus',
    maintainer_email='sachinkumar.ar97@gmail.com',
    description='This package is about using webrtc with ros2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webrtc_video_server = edge_rtc.webrtc_video_server:main',
            'webrtc_ros2_client = edge_rtc.webrtc_ros2_client:main',
        ],
    },
)
