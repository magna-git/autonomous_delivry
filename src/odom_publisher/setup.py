from setuptools import setup

package_name = 'odom_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khadija',
    maintainer_email='khadija@example.com',
    description='Publie les données d’odométrie via serial',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_pub = odom_publisher.odom_node:main',
            'lidar_stop = odom_publisher.lidar_stop_node:main',
        ],
    },
)
