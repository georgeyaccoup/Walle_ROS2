from setuptools import setup

package_name = 'walle_ROS2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'sounddevice',
        'numpy',
        'scipy',
        'openai'
    ],
    zip_safe=True,
    maintainer='George Yaccoup',
    maintainer_email='george@example.com',
    description='Walle ROS2 semi-humanoid robot with AI control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'main_controller_1 = walle_ROS2.main_controller_1:main',
            'main_controller_2 = walle_ROS2.main_controller_2:main',
            'voice_listener = walle_ROS2.voice_listener:main',
            'chatgpt_processor = walle_ROS2.chatgpt_processor:main',
        ],
    },
)
