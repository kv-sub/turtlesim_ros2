from setuptools import find_packages, setup

package_name = 'my_turtle_sim_catch'

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
    maintainer='kv',
    maintainer_email='kvsubramanian03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtlecontroller = my_turtle_sim_catch.turtle_controller:main",
            "turtlespawnner = my_turtle_sim_catch.turtle_spawnner:main"
        ],
    },
)
