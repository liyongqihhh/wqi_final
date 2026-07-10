from setuptools import find_packages, setup

package_name = 'ugvcar_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/delivery_task.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='UGV',
    maintainer_email='87068644+UGV@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init_robot_pose=ugvcar_application.init_robot_pose:main',
            'delivery_task=ugvcar_application.delivery_task_manager:main',
        ],
    },
)
