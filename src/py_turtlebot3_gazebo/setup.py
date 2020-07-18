from setuptools import setup

package_name = 'py_turtlebot3_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haruki',
    maintainer_email='haruki@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'py_turtlebot3_drive = py_turtlebot3_gazebo.py_turtlebot3_drive:main',
            'py_turtlebot3_sub = py_turtlebot3_gazebo.py_turtlebot3_sub:main',
        ],
    },
)
