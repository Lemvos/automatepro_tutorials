import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'automatepro_python_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='balachandra',
    maintainer_email='balachandra.bhat@lemvos.com',
    description='Tutorials and Example for AutomatePro Usage',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Sensor Nodes
            'gnss_position_node = automatepro_python_tutorials.sensors.gnss_position:main',
            'gnss_heading_node = automatepro_python_tutorials.sensors.gnss_heading:main',
            'imu_node = automatepro_python_tutorials.sensor.imu:main',

            # IO Nodes
            'analog_in_node = automatepro_python_tutorials.io.analog_in:main',
            'digital_in_node = automatepro_python_tutorials.io.digital_in:main',    
            'digital_out_node = automatepro_python_tutorials.io.digital_out:main',
            'digital_drive_out_node = automatepro_python_tutorials.io.digital_drive_out:main',
            'warning_light_node = automatepro_python_tutorials.io.warning_light:main',

            # Diagnostic Nodes
            'io_controller_diagnostic_node = automatepro_python_tutorials.diagnostics.io_controller:main',
        ],
    },
)
