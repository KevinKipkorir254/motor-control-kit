from setuptools import find_packages, setup

package_name = 'encoded_dc_motor_kit_response_analyzer'

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
    maintainer='kevo',
    maintainer_email='kipkorir.magut254@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'step_response_analyzer = encoded_dc_motor_kit_response_analyzer.step_response_analyzer:main',
        ],
    },
)
