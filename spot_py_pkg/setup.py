from setuptools import find_packages, setup

package_name = 'spot_py_pkg'

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
    maintainer='spot',
    maintainer_email='raphbenhams@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_input = spot_py_pkg.controller_input:main',
            'spot_controller = spot_py_pkg.spot_controller:main'
        ],
    },
    scripts= [
        'scripts/launch_spot.sh'
    ]
)
