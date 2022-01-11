from setuptools import setup

package_name = 'rospathgen'

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
    maintainer='taz',
    maintainer_email='tzupfer@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pathgenerator = rospathgen.pathgenerator:main',
            'pathtester = rospathgen.pathtester:main'
        ],
    },
)
