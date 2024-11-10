from setuptools import find_packages, setup
import os 
from glob import glob 
package_name = 'link_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'),glob('launch/*')), 
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),  # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meet',
    maintainer_email='meet@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ur5controller_fk = link_test.ur5controller_fk:main",
            "ur5control = link_test.ur5control:main",
            "ur5controller_ik = link_test.ur5controller_ik:main",
            "ur5control_ik = link_test.ur5control_ik:main",
        ],
    },
)
