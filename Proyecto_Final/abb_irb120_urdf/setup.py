import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'abb_irb120_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes/visual'), glob('meshes/visual/*.stl')),
        (os.path.join('share', package_name, 'meshes/collision'), glob('meshes/collision/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mily',
    maintainer_email='mily@todo.todo',
    description='Gemelo Digital del ABB IRB 120',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cinematica_directa = abb_irb120_description.cinematica_directa:main',
        ],
    },
)
