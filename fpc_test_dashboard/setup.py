from setuptools import find_packages, setup

package_name = 'fpc_test_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # Ship the static web assets (HTML/CSS/JS) inside the Python package so
    # dashboard_node.py can resolve them via Path(__file__).parent / "static"
    # whether installed normally or via `colcon build --symlink-install`.
    package_data={
        package_name: ['static/*.html', 'static/*.css', 'static/*.js'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/dashboard.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Web dashboard to test a robot\'s forward-position-controller '
                'capability (smooth / stair / smoothed tracking) before '
                'deploying host-side admittance control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard_node = fpc_test_dashboard.dashboard_node:main',
        ],
    },
)
