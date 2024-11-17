from setuptools import find_packages, setup

package_name = 'robotics_assignment1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch_turtlesim.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emmett Fitzharris',
    maintainer_email='emmett.fitzharris@mycit.ie',
    description='Robotics and Autonomous Systems Assignment 1',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'spawnClient = robotics_assignment1.SpawnClient:main',
        'spawnTurtlesServer = robotics_assignment1.SpawnTurtlesServer:main',
        'spawnTurtlesServerClient = robotics_assignment1.SpawnTurtlesServerClient:main',
        'moveTurtle1ActionServer = robotics_assignment1.MoveTurtle1ActionServer:main',
        'runSimulationActionServer = robotics_assignment1.RunSimulationActionServer:main',
        'runSimulationActionClient = robotics_assignment1.RunSimulationActionClient:main',
        ],
    },
)
