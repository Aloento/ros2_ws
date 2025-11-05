from setuptools import find_packages, setup

package_name = 'turtle_topics_demo'

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
    maintainer='ubuntu',
    maintainer_email='11802769+Aloento@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vel_pub     = turtle_topics_demo.vel_publisher:main',
            'pose_sub    = turtle_topics_demo.pose_subscriber:main',
            'cmdvel_echo = turtle_topics_demo.cmdvel_subscriber:main',
        ],
    },
)
