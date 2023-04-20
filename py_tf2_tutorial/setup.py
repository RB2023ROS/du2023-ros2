from setuptools import setup

package_name = 'py_tf2_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf2_broadcaster = py_tf2_tutorial.tf2_broadcast:main',
            'tf2_listener = py_tf2_tutorial.tf2_listen:main',
            'tf2_add_frame = py_tf2_tutorial.tf2_add_frame:main',
        ],
    },
)
