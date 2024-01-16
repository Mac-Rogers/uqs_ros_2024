from setuptools import find_packages, setup

package_name = 'uqs_motors_ros'

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
    maintainer='neye2204',
    maintainer_email='neye2204@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motor_control = uqs_motors_ros.Motor_Control:main",
            "motor_interpreter = uqs_motors_ros.Motor_Interpreter:main"
        ],
    },
)
