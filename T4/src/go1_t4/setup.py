from setuptools import setup

setup(
    name='go1_t4',
    version='0.1.0',
    packages=['go1_t4'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/go1_t4']),
        ('share/go1_t4', ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['odometry = go1_t4.odom_node:main']},
)
