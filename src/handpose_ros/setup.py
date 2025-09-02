from setuptools import find_packages, setup

package_name = 'handpose_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index 등록
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치 (있다면)
        ('share/' + package_name + '/launch', ['launch/handpose_launch.py']),
        # configuration files
        ('share/' + package_name + '/config', ['config/rviz_config.rviz']),
    ],
    install_requires=['setuptools',
                      'mediapipe',
                      'pyrealsense2'],
    zip_safe=True,
    maintainer='DaeyunJang',
    maintainer_email='bigyun9375@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 두 노드 엔트리 포인트
            'mediapipe_hands_node = handpose_ros.mediapipe_hands_node:main',
            'handpose_tf_broadcaster   = handpose_ros.handpose_tf_broadcaster:main',
        ],
    },
)
