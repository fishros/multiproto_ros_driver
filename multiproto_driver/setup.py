from setuptools import find_packages, setup

package_name = 'multiproto_driver'

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
    maintainer='fishros',
    maintainer_email='fishros@foxmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multiproto_driver_udp = multiproto_driver.multiproto_driver_udp:main',
            'multiproto_driver_serial = multiproto_driver.multiproto_driver_serial:main',
        ],
    },
)
