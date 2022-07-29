from setuptools import setup

package_name = 'demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kjw',
    maintainer_email='kojowelbeck2021@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_robot = demo.demo_robot:main',
            'action_server = demo.action_server:main',
            'action_client = demo.action_client:main'
        ],
    },
)
