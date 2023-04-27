from setuptools import setup

package_name = 'py_pcl_tutorial'

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
            'pcl_sub_node    = py_pcl_tutorial.pcl_sub:main',
            'pcl_cluster_pub = py_pcl_tutorial.pcl_cluster_pub:main',
        ],
    },
)
