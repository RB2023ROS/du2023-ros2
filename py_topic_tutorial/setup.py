from setuptools import setup

package_name = 'py_topic_tutorial'

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
            'topic_pub_node = py_topic_tutorial.topic_example_1_publisher:main',
            'topic_sub_node = py_topic_tutorial.topic_example_2_subscriber:main',
            'parking_node   = py_topic_tutorial.topic_example_3_pub_and_sub:main',
            'mimic_node     = py_topic_tutorial.topic_example_4_mimic:main',
        ],
    },
)
