from setuptools import find_packages, setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aklile',
    maintainer_email='aklile@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_node = my_robot_controller.Topics.my_first_node:main',
            'draw_circle = my_robot_controller.Topics.draw_circle:main',
            'pose_subscriber = my_robot_controller.Topics.pose_subscriber:main',
            'bundle = my_robot_controller.Topics.bundle:main',
            'add_two_ints_server = my_robot_controller.Services.add_two_ints_server:main',
            'add_two_ints_client = my_robot_controller.Services.add_two_ints_client_no_oop:main',
            'hw_status_publisher = my_robot_controller.MsgTester.hw_status_publisher:main',
            'number_publisher_W_Param = my_robot_controller.ParameterDemo.number_publisher_W_Param:main',
        ],
    },
)
