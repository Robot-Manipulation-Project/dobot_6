from setuptools import find_packages, setup

package_name = 'dobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_dir={'dobot.resource': 'resource'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         ("share/" + package_name + "/launch", ["launch/dobot_launch.xml"]),
         ("share/" + package_name + "/resource", ["resource/dobot_server.py", "resource/dobot_driver.py"]),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YasiruRF',
    maintainer_email='dyrfernando@gmail.com',
    description='Basic ROS2 interface for Dobot Magician robotic arm',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "homing_node = dobot.homing_node:main",
            "suction_cup_node = dobot.suction_cup_node:main",
            "joint_state_node = dobot.joint_state_node:main",
            "joint_ptp_node = dobot.joint_ptp_node:main",
            "cartesian_ptp_node = dobot.cartesian_ptp_node:main",
            "pick_and_place_node = dobot.pick_and_place_node:main",
        ],
    },
)
