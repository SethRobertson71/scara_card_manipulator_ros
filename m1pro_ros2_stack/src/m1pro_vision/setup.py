from setuptools import setup, find_packages

package_name = 'm1pro_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.dsv']),
        ('share/' + package_name + '/launch', ['launch/vision.launch.py']),
        ('share/' + package_name + '/config', ['config/vision_params.yaml']),
        ('share/' + package_name + '/environment', [
            'environment/ament_prefix_path.dsv',
            'environment/ament_prefix_path.sh',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='YOLO vision node for M1 Pro pick and place',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vision_node = m1pro_vision.vision_node:main',
        ],
    },
)
