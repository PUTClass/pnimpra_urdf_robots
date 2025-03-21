from setuptools import setup
from glob import glob
package_name = 'urdf_robots'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/description', glob('description/*')),
        ('share/' + package_name + '/launch', glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='rafal.staszak@put.poznan.pl',
    description='Package with example robotic arms and joint manipulation routine',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'show_joint_states = urdf_robots.show_joint_states:main',
                'simulate_joint_states = urdf_robots.simulate_joint_states:main'
        ],
    },
)
