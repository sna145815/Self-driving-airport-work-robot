from setuptools import find_packages, setup

package_name = 'ct_package'

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
    maintainer='jin',
    maintainer_email='sna145815@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_tower = ct_package.control_tower:main',
            'service_client = ct_package.service_client:main',
            'robot_control = ct_package.robot_control:main',
        ],
    },
)
