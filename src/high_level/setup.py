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
            "teleop_node = high_level.teleop:main",
            "com_node = high_level.com:main",
            "lidar_to_ai = high_level.lidar_to_ai:main",
            "ai_node = high_level.ai_node:main"
        ],
    },
)
