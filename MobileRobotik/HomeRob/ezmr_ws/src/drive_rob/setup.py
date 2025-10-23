from setuptools import find_packages, setup

package_name = 'drive_rob'

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
    maintainer='philnoc',
    maintainer_email='philipp.noecker@stud.htwk-leipzig.de',
    description='Simple Paket with Node that is driving a Heart in Turtlesim',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'turtle_heart = drive_rob.robot_heart:main',
		'turtle_square = drive_rob.robot_square1:main',
        ],
    },
)
