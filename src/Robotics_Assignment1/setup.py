from setuptools import find_packages, setup

package_name = 'Robotics_Assignment1'

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
    maintainer='Emmett Fitzharris',
    maintainer_email='emmett.fitzharris@mycit.ie',
    description='Robotics and Autonomous Systems Assignment 1',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'service = Robotics_Assignment1.service_member_function:main',
		'client = Robotics_Assignment1.client_member_function:main',
		'spawnClient = Robotics_Assignment1.SpawnClient:main',
        'spawnService = Robotics_Assignment1.SpawnService:main',
        'spawnTurtlesServer = Robotics_Assignment1.SpawnTurtlesServer:main',
        ],
    },
)
