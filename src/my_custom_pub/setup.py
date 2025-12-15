from setuptools import setup
from glob import glob
import os

package_name = 'my_custom_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', 
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kpanou-pi',
    maintainer_email='kpanou@example.com',
    description='Custom ROS2 publisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'custom_pub = my_custom_pub.custom_pub:main',
            'subscriber_accel = my_custom_pub.subscriber_accel:main',
            'subscriber_steering = my_custom_pub.subscriber_steering:main',
            'brake_subscriber = my_custom_pub.brake_subscriber:main',
            'gpio_brake = my_custom_pub.gpio_brake:main',
            'motor_bridge = my_custom_pub.motor_bridge:main',
            'steering_to_serial = my_custom_pub.steering_to_serial:main',
            'gpio_direction = my_custom_pub.gpio_direction:main',
            'direction_subscriber = my_custom_pub.direction_subscriber:main',
            'esp32_bridge = my_custom_pub.esp32_bridge:main',
        ],
    },
)

