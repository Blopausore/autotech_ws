from setuptools import setup

package_name = 'high_level'

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
    maintainer='smaug',
    maintainer_email='augustin_650@hotmail.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "teleop_node = monthlery.teleop:main",
            "com_node = monthlery.com:main",
            "lidar_to_ai = monthlery.lidar_to_ai:main",
            "ai_node = autotech_ros2_pkg.ai_node:main"
        ],
    },
)
