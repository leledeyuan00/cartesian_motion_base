from setuptools import find_packages, setup

package_name = 'cartesian_motion_tools'

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
    maintainer='Dayuan',
    maintainer_email='leledeyuan@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gravity_pose_record = cartesian_motion_tools.gravity_pose_record:main',
            'gravity_compute = cartesian_motion_tools.gravity_compute:gravity_compute',
        ],
    },
)
