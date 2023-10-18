from setuptools import find_packages, setup

package_name = 'simple_truck'

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
    maintainer='rj',
    maintainer_email='arjan.vonk@outlook.com',
    description='Simple truck simulator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_truck_node = simple_truck.simple_truck_node:main'
        ],
    },
)
