from setuptools import find_packages, setup

package_name = 'aux_frame_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # Ship the optional dashboard's static web assets (HTML/CSS/JS + the
    # Three.js vendor bundle for the 3D viewer) inside the package so
    # dashboard_node.py can resolve them via Path(__file__).parent / "static"
    # whether installed normally or via `colcon build --symlink-install`.
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
         ['launch/aux_frame_manager.launch.py',
          'launch/cartesian_urdf_source.launch.py']),
        ('share/' + package_name + '/config',
         ['config/fzi_topic_test.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='yizhongzhang',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='Single-writer owner of the canonical augmented robot_description '
                '(auxiliary FT/compliance/operation frames) published on a latched '
                'topic for the FZI Cartesian controllers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aux_frame_manager = aux_frame_manager.aux_frame_manager_node:main',
            'aux_frame_guard = aux_frame_manager.aux_frame_guard:main',
            'aux_frame_dashboard = aux_frame_manager.dashboard_node:main',
        ],
    },
)
