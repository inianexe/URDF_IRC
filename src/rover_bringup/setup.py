from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_bringup'
setup(
    name=package_name, version='0.2.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        *[('share/' + package_name + '/' + folder, glob(folder + '/*'))
          for folder in ['config', 'launch', 'rviz', 'worlds', 'maps']],
    ],
    install_requires=['setuptools'], zip_safe=True,
    tests_require=['pytest'],
    maintainer='Inian', maintainer_email='inian@example.com',
    description='IRC rover Cartographer and gamepad simulation',
    license='LicenseRef-Proprietary',
    entry_points={'console_scripts': [
        'gamepad_teleop = rover_bringup.gamepad_teleop:main',
        'save_carto_map = rover_bringup.save_map:main',
    ]},
)
