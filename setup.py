from setuptools import find_packages, setup

package_name = 'tic_tac_toe'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Tic Tac Toe game with ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_logic = tic_tac_toe.game_logic_node:main',
            'draw_grid = tic_tac_toe.draw_grid:main',
        ],
    },
)
