from setuptools import setup

package_name = 'web_bridge_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='Web ↔ ROS2 bridge (stub).',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_bridge = web_bridge_node.web_bridge:main',
        ],
    },
)
