from setuptools import setup
from glob import glob
import os

package_name = 'follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('worlds/*.world'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Navaneeth',
    maintainer_email='navaneeth@ufpr.br',
    description='Have a differential drive robot follow a Robotrace track by using a camera.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower_node = follower.follower_node:main',
            'tester = follower.test_topic:main',
            'fast_pub = follower.fast_pub:main',
            'slider = follower.slider_sub:main',
            'detector = follower.line_detector:main',

        ],
    },
)
