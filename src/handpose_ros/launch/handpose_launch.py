from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch arguments (override 가능)
    image_width   = LaunchConfiguration('image_width')
    image_height  = LaunchConfiguration('image_height')
    camera_fps    = LaunchConfiguration('camera_fps')

    # scale = 1.0 / image_width  (float 타입 보장)
    scale_value = ParameterValue(
        PythonExpression(['1.0 / ', image_width]),
        value_type=float
    )

    # Realsense launch 포함 (권장 경로 스타일)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'rgb_camera.color_profile':  [image_width, ',', image_height, ',', camera_fps],
            'depth_module.depth_profile': [image_width, ',', image_height, ',', camera_fps],
        }.items()
    )

    # RViz config 경로 (권장 경로 스타일)
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('handpose_ros'),
        'config',
        'rviz_config.rviz'
    ])

    mediapipe_node = Node(
        package='handpose_ros',
        executable='mediapipe_hands_node',
        name='mediapipe_hands_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera/camera/color/image_raw',
            'max_num_hands': 2,
            'min_detection_confidence': 0.7,
            'min_tracking_confidence': 0.7,
            # 'draw': False,
            'flip_image': True,
        }]
    )

    tf_broadcaster_node = Node(
        package='handpose_ros',
        executable='handpose_tf_broadcaster',
        name='handpose_tf_broadcaster',
        output='screen',
        parameters=[{
            'hands_topic': 'hands/detections',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'camera_frame': 'camera_color_optical_frame',
            'tf.norm.enable': False,
            'tf.canonical.enable': False,
            'tf.canonical_norm.enable': False,
            'tf.canonical_norm.scale': scale_value,            # 1 / image_width
            'tf.world_absolute_scale.enable': True,
            'tf.world_absolute_scale.target_length': 0.06,     # meters
            'tf.world_absolute_scale.finger_name': 'index',
            'tf.world_absolute_scale.joint_name': 'mcp',
            'tf.world_absolute_scale.eps': 1e-6,
            'tf.world_absolute_scale.EMA_smooth_alpha': 0.3,   # percentage
            'tf.world_absolute_scale.suffix': 'world_abs',
            'tf.world_absolute_scale.max_scale_step': 0.0,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='handpose_rviz',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        # 인자 선언 (기본값: 기존 코드와 동일)
        DeclareLaunchArgument('image_width',  default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('camera_fps',   default_value='30'),

        realsense_launch,
        mediapipe_node,
        tf_broadcaster_node,
        rviz_node,
    ])
