from setuptools import find_packages, setup
import glob, os

package_name = 'camera_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asilia',
    maintainer_email='keummingi0131@gmail.com',
    description='Package for calibration & publisher imgs',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration = camera_ros.calibration:main',
            'publisher = camera_ros.publisher:main',
            'publisher2 = camera_ros.publisher2:main',
            'test = camera_ros.topic_test:main',
            'test2 = camera_ros.topic_test2:main',
        ],
    },
)
