from setuptools import setup
from glob import glob
package_name = 'assignment2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics23',
    maintainer_email='robotics23@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task1=assignment2.task1:main',
            'task2=assignment2.task2:main',
            'task3=assignment2.task3:main',
            'bonus=assignment2.bonus:main',
            'right=assignment2.follow_right:main',
            'solver=assignment2.maze_solver:main'
        ],
    },
)
