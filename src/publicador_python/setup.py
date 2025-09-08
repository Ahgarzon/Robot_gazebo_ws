from setuptools import find_packages, setup

package_name = 'publicador_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='angel',
    maintainer_email='Ahgarzon@github.com',
    description='Publicador de mensajes Twist en /publicador y suscriptor de /wheel/odometry',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'publicador_cmd = publicador_python.publicador_cmd:main',
        'subscriber_odom = publicador_python.subscriber_odom:main',
        'autonomia_wander = publicador_python.autonomia_wander:main',
    ],
},

)

