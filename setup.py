from setuptools import find_packages, setup
import os

package_name = 'auto_bag'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        (os.path.join('share', package_name, 'config'), ['config/topics.yaml']),
        (os.path.join('share', package_name, 'launch'), ['launch/auto_bag_launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emilien Ghazal',
    maintainer_email='emilienghazal@gmail.com',
    description='Node to trigger bag recording via service call',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record_topics = auto_bag.data_record:main',
        ],
    },
)

