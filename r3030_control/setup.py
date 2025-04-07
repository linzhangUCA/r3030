import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'r3030_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "rviz"), glob(os.path.join("rviz", "*.rviz"))),
        (os.path.join("share", package_name, "configs"), glob(os.path.join("configs", "*.yaml"))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='r2bot',
    maintainer_email='lzhang12@uca.edu',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_interface = r3030_control.driver_interface:main'
        ],
    },
)
