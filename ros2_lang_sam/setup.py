from setuptools import find_packages, setup

package_name = 'ros2_lang_sam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/server.launch.py',
            'launch/segmentation_demo.launch.py',
            'launch/segmentation_rviz_demo.launch.py',
            'launch/segmentation_rqt_demo.launch.py',
            'launch/test_segmentation.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/segmentation_visualization.rviz',
            'config/segment_visualization.perspective'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS 2 Developer',
    maintainer_email='maintainer@example.com',
    description='ROS 2 wrapper for lang-segment-anything',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lang_sam_server_node = ros2_lang_sam.lang_sam_server_node:main',
            'lang_sam_client_node = ros2_lang_sam.lang_sam_client_node:main',
            'segmentation_node = ros2_lang_sam.segmentation_node:main',
            'image_publisher_node = ros2_lang_sam.image_publisher_node:main',
            'result_visualizer_node = ros2_lang_sam.result_visualizer_node:main',
            'request_publisher_node = ros2_lang_sam.request_publisher_node:main',
        ],
    },
)
