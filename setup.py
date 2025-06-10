import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'v4l2ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('lib', package_name), ['scripts/script.sh', 'scripts/reset.sh']),
    ],
    install_requires=['setuptools', 'av'],
    zip_safe=True,
    maintainer='aymeric',
    maintainer_email='aymericardot@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_writer = v4l2ros2.video_writer:main',
        ],
    },
)

