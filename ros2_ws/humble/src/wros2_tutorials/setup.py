
import os
from glob import glob
from setuptools import setup

package_name = 'wros2_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.rviz'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'wrs/0000_examples/objects'),
         glob(os.path.join('../wrs/0000_examples/objects', '*.stl'))),
        (os.path.join('share', package_name, 'wrs/robot_sim/end_effectors/gripper/robotiqhe/meshes'),
         glob(os.path.join('../wrs/robot_sim/end_effectors/gripper/robotiqhe/meshes', '*.stl'))),
        (os.path.join('share', package_name, 'wrs/robot_sim/end_effectors/gripper/robotiq85/meshes'),
         glob(os.path.join('../wrs/robot_sim/end_effectors/gripper/robotiq85/meshes', '*.stl'))),
        (os.path.join('share', package_name, 'wrs/robot_sim/end_effectors/gripper/robotiq140/meshes'),
         glob(os.path.join('../wrs/robot_sim/end_effectors/gripper/robotiq140/meshes', '*.stl'))),
        (os.path.join('share', package_name, 'wrs/robot_sim/end_effectors/single_contact/suction/sandmmbs/mbsegg'),
         glob(os.path.join('../wrs/robot_sim/end_effectors/single_contact/suction/sandmmbs/mbsegg', '*.egg'))),
        (os.path.join('share', package_name, 'wrs/robot_sim/end_effectors/single_contact/suction/sandmmbs/mbs'),
         glob(os.path.join('../wrs/robot_sim/end_effectors/single_contact/suction/sandmmbs/mbs', '*.stl'))),
        (os.path.join('share', package_name, 'wrs/robot_sim/end_effectors/single_contact/suction/sgb30'),
         glob(os.path.join('../wrs/robot_sim/end_effectors/single_contact/suction/sgb30', '*.stl'))),
        (os.path.join('share', package_name, 'wrs/robot_sim/end_effectors/single_contact/suction/sgb30'),
         glob(os.path.join('../wrs/robot_sim/end_effectors/single_contact/suction/sgb30', '*.egg'))),
        (os.path.join('share', package_name, 'wrs/pyhiro/suction/geomprim'),
         glob(os.path.join('../wrs/pyhiro/suction/geomprim', '*.egg'))),
        (os.path.join('share', package_name, 'wrs/pyhiro/suction/objects'),
         glob(os.path.join('../wrs/pyhiro/suction/objects', '*.stl'))),
        (os.path.join('share', package_name, 'wrs/trimesh/templates'),
         glob(os.path.join('../wrs/trimesh/templates', '*.template'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Takuya Kiyokawa',
    author_email='taku8926@gmail.com',
    maintainer='Takuya Kiyokawa',
    maintainer_email='taku8926@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='WROS2 tutorials.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'grasp_planning_service = '
            ' wros2_tutorials.grasp_planning_service:main',
        ],
    },
    include_package_data=True,
)
