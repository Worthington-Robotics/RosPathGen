from setuptools import setup
from glob import glob
import os

package_name = 'rospathgen'

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
    maintainer='taz',
    maintainer_email='tzupfer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pathservices = rospathgen.pathservices:main',
            'pathtester = rospathgen.pathtester:main',
            'pathgetter = rospathgen.pathgetter:main',
            'pathbuilder = rospathgen.pathbuilder:main'
        ],
    },
)
