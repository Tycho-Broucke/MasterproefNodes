from setuptools import setup

package_name = 'counter_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rcply'],
    zip_safe=True,
    maintainer='tycho',
    maintainer_email='tycho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'counter_publisher = counter_publisher.counter_publisher:main',
        ],
    },
)
