from setuptools import setup
import os
from glob import glob

package_name = 'pathbuilder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.py')),
        (os.path.join('share', package_name, "cfg"), glob('cfg/*.yaml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='worbots',
    maintainer_email='boring.someone@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pathbuilder = pathbuilder.pathbuilder:main'
        ],
    },
)
