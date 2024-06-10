from setuptools import find_packages, setup
import os
import glob


package_name = 'ct_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name],),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jin',
    maintainer_email='sna145815@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_tower = ct_package.control_tower:main',
            'service_client = ct_package.service_client:main',
            'drobot_control = ct_package.drobot_control:main',
            'kiosk_manager = ct_package.KioskManager:main',
            'store_manager = ct_package.StoreManager:main',
            'robot_manager = ct_package.RobotManager:main',
            'db_manager = ct_package.db_manager:main',
            'pathDict = ct_package.pathDict:main',
            'taskManager = ct_package.taskManager:main',
        ],
    },
)
