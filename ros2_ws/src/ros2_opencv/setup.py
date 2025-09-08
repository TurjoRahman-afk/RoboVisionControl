from setuptools import find_packages, setup

package_name = 'ros2_opencv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rahman',
    maintainer_email='rahman@todo.todo',
    description='ROS2 OpenCV Camera Nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = ros2_opencv.cameraPublisher:main',
            'image_subscriber = ros2_opencv.subscriberImage:main',
        ],
},
)
