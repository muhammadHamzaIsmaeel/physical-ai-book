from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    robot_description_pkg = LaunchConfiguration('robot_description_pkg')
    robot_description_file = LaunchConfiguration('robot_description_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    vla_model = LaunchConfiguration('vla_model')
    whisper_model = LaunchConfiguration('whisper_model')

    declare_robot_description_pkg_cmd = DeclareLaunchArgument(
        'robot_description_pkg',
        default_value='capstone_description',
        description='Package containing robot description (URDF/XACRO)'
    )
    declare_robot_description_file_cmd = DeclareLaunchArgument(
        'robot_description_file',
        default_value='robot.urdf.xacro',
        description='Robot description file (e.g., robot.urdf.xacro)'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Isaac Sim) clock if true'
    )
    declare_vla_model_cmd = DeclareLaunchArgument(
        'vla_model',
        default_value='openvla-7b',
        description='VLA model to use (e.g., openvla-7b)'
    )
    declare_whisper_model_cmd = DeclareLaunchArgument(
        'whisper_model',
        default_value='base.en',
        description='Whisper model to use for ASR (e.g., base.en)'
    )

    # Robot description launch (robot_state_publisher)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(robot_description_pkg),
                'launch',
                'description.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_description_file': robot_description_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Navigation stack launch (conceptual - replace with actual Nav2 launch)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('capstone_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Perception stack launch
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('capstone_perception'),
                'launch',
                'perception.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Manipulation stack launch
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('capstone_manipulation'),
                'launch',
                'manipulation.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # VLA Integration launch
    vla_integration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('capstone_vla_integration'),
                'launch',
                'vla_integration.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'vla_model': vla_model
        }.items()
    )

    # Voice Handler launch
    voice_handler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('capstone_voice_handler'),
                'launch',
                'voice_handler.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'whisper_model': whisper_model
        }.items()
    )

    # Task Planner launch
    task_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('capstone_task_planner'),
                'launch',
                'task_planner.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        declare_robot_description_pkg_cmd,
        declare_robot_description_file_cmd,
        declare_use_sim_time_cmd,
        declare_vla_model_cmd,
        declare_whisper_model_cmd,
        robot_description_launch,
        navigation_launch,
        perception_launch,
        manipulation_launch,
        vla_integration_launch,
        voice_handler_launch,
        task_planner_launch
    ])
