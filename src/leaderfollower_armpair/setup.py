from setuptools import find_packages, setup

package_name = 'leaderfollower_armpair'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='1994524450@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower = leaderfollower_armpair.robo_follower_node:main',
            "demo = leaderfollower_armpair.demo:main",
            "loop_demo = leaderfollower_armpair.loop_demo:main",
            "leader_demo = leaderfollower_armpair.leader_demo:main",
        ],
    },
)
