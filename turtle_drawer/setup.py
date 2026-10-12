from setuptools import find_packages, setup

package_name = 'turtle_drawer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhuxinyang',
    maintainer_email='zhuxinyang@example.com',
    description='控制 turtlesim 小海龟绘制学号最后两位数字',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_drawer = turtle_drawer.turtle_drawer:main',
        ],
    },
)
