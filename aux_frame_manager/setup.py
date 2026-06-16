from setuptools import find_packages, setup

package_name = 'aux_frame_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
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
        ],
    },
)
