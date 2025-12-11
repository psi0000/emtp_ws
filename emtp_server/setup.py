from setuptools import find_packages, setup
from glob import glob
package_name = 'emtp_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/env/experiment',
            glob('common/env/experiment/*.yaml') + glob('common/env/experiment/*.csv')),
        ('share/' + package_name + '/env/college',
            glob('common/env/college/*.yaml') + glob('common/env/college/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='psi',
    maintainer_email='qkrwk0921@naver.com',
    description='Experiment server package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = emtp_server.server:main',
            'emtp = EMTP.script.main_ros:main',
            
        ],
    },
)
