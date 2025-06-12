from setuptools import setup

package_name = 'publish_sensor_info'

setup(
    name='publish_sensor_info',
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian and Mike',
    maintainer_email='you@example.com',
    description='ROS2 Übung 1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = publish_sensor_info.publisher:main',
            'sub = publish_sensor_info.subscriber:main',
        ],
    },
)