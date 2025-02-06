from setuptools import find_packages, setup

package_name = 'ros_connector_manager'

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
    maintainer='nam',
    maintainer_email='cknam0708@sju.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
<<<<<<< HEAD:src/ros_connector_manager/setup.py
            'ros_connector_manager = ros_connector_manager.ros_connector_manager:main',
=======
             'ROS_connector = ros_bridge_manager.ros_bridge_manager:main',
>>>>>>> 209eb5f (로컬 변경 사항 커밋):src/ros_bridge_manager/setup.py
        ],
    },
)
