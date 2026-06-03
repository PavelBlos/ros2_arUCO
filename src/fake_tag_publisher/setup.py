from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fake_tag_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.mp4')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='panik_bel',
    maintainer_email='panik_bel@todo.todo',
    description='Fake AprilTag localization project',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'fake_tag_node = fake_tag_publisher.fake_tag_node:main',
            'localization_node = fake_tag_publisher.localization_node:main',
            'static_tf_node = fake_tag_publisher.static_tf_node:main',
            'camera_calibrator = fake_tag_publisher.camera_calibrator:main',
            'video_tag_detector = fake_tag_publisher.video_tag_detector:main',
        ],
    },
)
