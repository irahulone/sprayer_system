from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'prescription_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zen',
    maintainer_email='zyamao@scu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "line_nav = gps_nav.gps_line_nav:main ",
            "rover_sim = gps_nav.sim_rover:main",
            "dfr_pub = gps_nav.dfr_pub:main"
        ],
    },
)
