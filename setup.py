from setuptools import find_packages, setup

package_name = 'rc_controll'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy','cssrlib'],
    zip_safe=True,
    maintainer='shogotakiza',
    maintainer_email='shogotakiza@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'Kalman_Node = rc_controll.kalman_node:main',
        'Ecef2Enu_Node = rc_controll.ecef2enu:main',
        ],
    },
)
