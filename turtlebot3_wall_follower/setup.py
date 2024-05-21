from setuptools import find_packages, setup

package_name = 'turtlebot3_wall_follower'

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
    maintainer='mesum',
    maintainer_email='mesum@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'wall_follower = turtlebot3_wall_follower.wall_follower:main',
    ],
	},
)
