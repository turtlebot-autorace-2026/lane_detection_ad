import glob

from setuptools import find_packages, setup

package_name = 'lane_detection_ad'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/models', glob.glob('models/*.pt') + glob.glob('models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='turtlebot-autorace-2026',
    maintainer_email='eureka01020304@gmail.com',
    description='Lane following (YOLO-seg lane model + near/far controller) for TurtleBot3 AutoRace 2026',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = lane_detection_ad.detection:main',
            'sim_camera_bridge = lane_detection_ad.sim_camera_bridge:main',
            'section_judge = lane_detection_ad.section_judge:main',
        ],
    },
)
