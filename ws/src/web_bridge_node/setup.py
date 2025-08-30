# robot-mBuff/ws/src/web_bridge_node/setup.py
from setuptools import setup, find_packages

package_name = 'web_bridge_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ecm',
    maintainer_email='n/a',
    description='Web bridge (split into move/camera/sense modules)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_bridge = web_bridge_node.web_bridge:main',
        ],
    },
)
