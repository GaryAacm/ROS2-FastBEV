from setuptools import setup

package_name = 'camera_sim_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@localhost',
    description='Publish 6 camera images from CUDA-FastBEV example-data to ROS2 topics.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'multi_cam_pub = camera_sim_pub.multi_cam_pub:main',
        ],
    },
)
