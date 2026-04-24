from setuptools import find_packages, setup

package_name = 'm1pro_skipbo_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.dsv']),
        (
            'share/' + package_name + '/launch',
            ['launch/skipbo_vision.launch.py', 'launch/template_capture.launch.py'],
        ),
        ('share/' + package_name + '/config', ['config/skipbo_params.yaml']),
        ('share/' + package_name + '/environment', [
            'environment/ament_prefix_path.dsv',
            'environment/ament_prefix_path.sh',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Classical CV Skip-Bo vision node for M1 Pro',
    license='MIT',
    entry_points={
        'console_scripts': [
            'skipbo_vision_node = m1pro_skipbo_vision.skipbo_vision_node:main',
            'template_capture_node = m1pro_skipbo_vision.template_capture_node:main',
            'card_world_publisher = m1pro_skipbo_vision.card_world_publisher:main',
        ],
    },
)
