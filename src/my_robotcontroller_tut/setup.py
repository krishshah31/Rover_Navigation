from setuptools import find_packages, setup

package_name = 'my_robotcontroller_tut'

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
    maintainer='krish',
    maintainer_email='krish@todo.todo',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robotcontroller_tut.test_node:main",
            "turtle_controller = my_robotcontroller_tut.turtle_controller:main"
        ],
    },
)
