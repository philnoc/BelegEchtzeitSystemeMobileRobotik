from setuptools import find_packages, setup

package_name = 'phil_turtlebot'

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
    maintainer='leobots',
    maintainer_email='nico.beyer@stud.htwk-leipzig.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'callAction=phil_turtlebot.callAction:main',
        'Spinn45=phil_turtlebot.spinn:main',
        'oneIMG=phil_turtlebot.show_one_img:main',
        'musictest=phil_turtlebot.playmusic:main',
        'musicDetect=phil_turtlebot.movedisco:main',
        ],
    },
)
