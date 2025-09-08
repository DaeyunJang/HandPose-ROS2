from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    handpose_share = get_package_share_directory('handpose_ros')
    rviz_config = os.path.join(handpose_share, 'config', 'rviz_config.rviz')
    
    image_width = 1280
    image_height = 720
    cameara_fps = 30
    scale = 1/image_width
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
                launch_arguments={
                'rgb_camera.color_profile': f'{image_width},{image_height},{cameara_fps}',
                'depth_module.depth_profile': f'{image_width},{image_height},{cameara_fps}',
                # default
                # 'rgb_camera.color_profile': '1280,720,30',
                # 'depth_module.depth_profile': '1280,720,30',
                
                # 'rgb_camera.enable_auto_exposure': 'false',
                # 'rgb_camera.exposure': '80',
                # 'rgb_camera.profile': '640,480,30',
                # 'depth_module.profile': '640,480,30',
                }.items()
        ),
        Node(
            package='handpose_ros',
            executable='mediapipe_hands_node',
            name='mediapipe_hands_node',
            output='screen',
            parameters=[{
                'image_topic': '/camera/camera/color/image_raw',
                'max_num_hands': 2,
                'min_detection_confidence': 0.95,
                'min_tracking_confidence': 0.95,
                # 'draw': False, # Not use
                'flip_image': True,
            }]
        ),
        Node(
            package='handpose_ros',
            executable='handpose_tf_broadcaster',
            name='handpose_tf_broadcaster',
            output='screen',
            parameters=[{
                'hands_topic': 'hands/detections',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'camera_frame': 'camera_color_optical_frame',   # For TF name
                'tf.norm.enable': False,
                'tf.canonical.enable': False,
                'tf.canonical_norm.enable': True,
                'tf.canonical_norm.scale': scale, # 1/1280
                
                # TBD
                'tf.user_defined.enable': False,
                'tf.user_defined.scale': 0.3/0.5,
                'tf.user_defined.suffix': 'gripper_scale',
                # Not use
                # 'use_depth': True,  # True면 아래 두 토픽도 필요
                # 'depth_topic': '/camera/aligned_depth_to_color/image_raw',
                # 'camera_info_topic': '/camera/color/camera_info',
            }]
        ),
        # === RViz2 추가 ===

        Node(
            package='rviz2',
            executable='rviz2',
            name='handpose_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            # env={'LD_LIBRARY_PATH': ''},
        ),
    ])
