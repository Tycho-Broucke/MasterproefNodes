from setuptools import setup

package_name = 'masterproef_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pandas',
        'watchdog',
        'torch',
        'opencv-python'
    ],
    zip_safe=True,
    maintainer='tycho',
    maintainer_email='tycho@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # Entry points for CSV subscriber and watcher nodes
            'csv_subscriber = masterproef_nodes.csv_subscriber:main',  
            'csv_watcher = masterproef_nodes.csv_watcher:main',
            'target_selector_node = masterproef_nodes.target_selector_node:main',
            'yolo_coordinate_publisher = masterproef_nodes.yolo_coordinate_publisher:main',     
        ],
    },
)
