from setuptools import find_packages, setup

package_name = 'geo_pipeline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/geo_pipeline.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hp',
    maintainer_email='hp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'geo_compute_node = geo_pipeline.geo_compute_node:main',
        'result_publisher_node = geo_pipeline.result_publisher_node:main',
        'coord_cli = geo_pipeline.coord_cli:main',
        ],
    },
)
