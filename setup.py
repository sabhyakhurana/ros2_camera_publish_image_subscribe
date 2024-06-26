from setuptools import setup

package_name = 'ros2_camera_publish'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['settings.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ANI717',
    maintainer_email='animesh.ani@live.com',
    description='Publish Camera Data',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read = ros2_camera_publish.image_subscribe_function:main',
        ],
    },
)
