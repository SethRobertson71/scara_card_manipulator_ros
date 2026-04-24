from setuptools import find_packages, setup

package_name = 'm1pro_motion'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=['Dobot_Color_Sorting'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/color_sorting.launch.py', 'launch/test_color_sorter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Motion node for Dobot M1 Pro color sorting',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dobot_color_sorting_node = m1pro_motion.dobot_color_sorting_node:main',
            'test_color_sorter = m1pro_motion.test_color_sorter:main',
        ],
    },
)
