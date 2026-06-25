from setuptools import find_packages, setup

package_name = 'robot_control_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # Ship the dashboard's static web assets (HTML/CSS/JS + the Three.js vendor
    # bundle for the 3D viewer) inside the package so control_test_node.py can
    # resolve them via Path(__file__).parent / "static" whether installed
    # normally or via `colcon build --symlink-install`.
    package_data={
        package_name: [
            'static/*.html', 'static/*.css', 'static/*.js',
            'static/vendor/*.js',
            'static/vendor/addons/controls/*.js',
            'static/vendor/addons/loaders/*.js',
        ],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/control_test.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Interactive web bench with a 3D canvas to test a robot with '
                'each of its ros2_control controllers (joint-trajectory, '
                'forward-position, and the FZI Cartesian motion / compliance / '
                'force controllers) and verify it moves correctly.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_test_node = '
            'robot_control_test.control_test_node:main',
        ],
    },
)
