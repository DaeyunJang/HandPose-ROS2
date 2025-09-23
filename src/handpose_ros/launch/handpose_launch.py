from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- Launch arguments (override-able) ---
    image_width   = LaunchConfiguration('image_width')
    image_height  = LaunchConfiguration('image_height')
    camera_fps    = LaunchConfiguration('camera_fps')
    params_file   = LaunchConfiguration('params_file')

    # scale = 1.0 / image_width  (ensure float)
    scale_value = ParameterValue(
        PythonExpression(['1.0 / ', image_width]),
        value_type=float
    )

    # Realsense launch
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

    # RViz config path
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('handpose_ros'),
        'config',
        'rviz_config.rviz'
    ])

    # params.yaml (default path under this package)
    default_params_path = PathJoinSubstitution([
        FindPackageShare('handpose_ros'),
        'config',
        'params.yaml'
    ])

    mediapipe_node = Node(
        package='handpose_ros',
        executable='mediapipe_hands_node',
        name='mediapipe_hands_node',
        output='screen',
        # 1) load params.yaml
        # 2) (옵션) 개별 오버라이드가 필요하면 dict 추가 가능
        parameters=[params_file]
    )

    tf_broadcaster_node = Node(
        package='handpose_ros',
        executable='handpose_tf_broadcaster',
        name='handpose_tf_broadcaster',
        output='screen',
        # 1) load params.yaml
        # 2) override tf.canonical_norm.scale with 1.0 / image_width
        parameters=[
            params_file,
            {
                'tf.canonical_norm.scale': scale_value
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='handpose_rviz',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        # Declare args (defaults preserve prior behavior)
        DeclareLaunchArgument('image_width',  default_value='1280'),
        DeclareLaunchArgument('image_height', default_value='720'),
        DeclareLaunchArgument('camera_fps',   default_value='30'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_path,
            description='Full path to the ROS 2 parameters file to use'
        ),

        realsense_launch,
        mediapipe_node,
        tf_broadcaster_node,
        rviz_node,
    ])
