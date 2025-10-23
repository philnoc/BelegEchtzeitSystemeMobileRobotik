from setuptools import find_packages, setup

package_name = 'sensors_add'

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
    description='Package for University purpose, adding sensors to esp32 project',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'ultra_display = sensors_add.ultra_disp:main',
        'circle_warn = sensors_add.steal_warn:main',
        ],
    },
)
