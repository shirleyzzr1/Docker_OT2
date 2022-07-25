from setuptools import setup

package_name = 'ot2_controller'
protocol_handler = "ot2_controller/protocol_handler"
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,protocol_handler],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'OT2_client =  ot2_controller.OT2_node:main',
        ],
    },
)
