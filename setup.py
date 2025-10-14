from setuptools import find_packages, setup

package_name = 'csce452_proj3'

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
    maintainer='Lance Wiley',
    maintainer_email='lance.wiley09@tamu.edu',
    description='Track people moving through LIDAR scans',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'marker = csce452_proj3.marker:main',
            'lidar = csce452_proj3.lidar_inter:main'
        ],
    },
)
