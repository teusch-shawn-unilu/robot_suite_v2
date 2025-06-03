from setuptools import find_packages, setup

package_name = 'spot_object_recognition'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools', 
        'ultralytics', 
        'opencv-python-headless',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Shawn Teusch',
    maintainer_email='shawn.teusch.001@student.uni.lu',
    description='ROS 2 node for YOLO v8-based object recognition on Spot\'s RealSense feed, logging unique lavels to CSV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spot_object_recognition_node = spot_object_recognition.spot_object_recognition_node:main',
        ],
    },
)
