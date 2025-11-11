from setuptools import find_packages, setup

package_name = 'ros2_bench'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
        install_requires=['setuptools', 'rclpy', 'builtin_interfaces', 'rosidl_runtime_py'],
    zip_safe=True,
    maintainer='rdros2',
    maintainer_email='rdros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sender_node = ros2_bench.sender_node:main',
            'receiver_node = ros2_bench.receiver_node:main',
        ],
    },
)
