from setuptools import find_packages, setup

package_name = 'mi_robot_control'

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
    maintainer='maria',
    maintainer_email='maria.rojas10@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mux_aeb = mi_robot_control.mux_aeb:main',
            'control_ps = mi_robot_control.control_ps:main',
            'seguidor_pared = mi_robot_control.seguidor_pared:main',
        ],
    },
)
