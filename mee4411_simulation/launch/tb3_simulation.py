from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    model = LaunchConfiguration('model')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_a = LaunchConfiguration('initial_pose_a')
    joint_noise_std = LaunchConfiguration('joint_noise_std')
    sim_namespace = LaunchConfiguration('sim_namespace')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=EnvironmentVariable('TURTLEBOT3_MODEL'),
        description='Model type [burger, waffle, waffle_pi].'
    )

    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial x position of the robot.'
    )

    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial y position of the robot.'
    )

    initial_pose_a_arg = DeclareLaunchArgument(
        'initial_pose_a',
        default_value='0.0',
        description='Initial angle of the robot.'
    )

    joint_noise_std_arg = DeclareLaunchArgument(
        'joint_noise_std',
        default_value='0.0',
        description='Standard deviation of wheel noise in deg per second.'
    )

    sim_namespace_arg = DeclareLaunchArgument(
        'sim_namespace',
        default_value='',
        description='Namespace for `real` simulated robot.'
    )

    # Robot description file
    urdf_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_description'),
        'urdf',
        ['turtlebot3_', model, '.urdf']
    ])
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path,
                 " ", "namespace:=", sim_namespace]),
        value_type=str
    )

    # Nodes
    # Robot transformations
    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        exec_name='robot_state_publisher',
        namespace=sim_namespace,
        output='screen',
        parameters=[
            {'publish_frequency': 50.0},
            {'robot_description': robot_description}
        ],
        # remappings=[
        #     ('/tf','tf'),
        #     ('/tf_static','tf_static')
        # ]
    )

    # Odometry
    odometry = Node(
        package='turtlebot3_fake_node',
        executable='turtlebot3_fake_node',
        exec_name='turtlebot3_fake_node',
        namespace=sim_namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('mee4411_simulation'),
                    'param',
                    [model, '.yaml']]
            ),
            {'joint_states_frame': [sim_namespace, 'base_footprint']},
            {'odom_frame': [sim_namespace, 'odom']},
            {'base_frame': [sim_namespace, 'base_footprint']},
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ]
    )

    # Localization
    localization = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        exec_name='map_odom_tf',
        namespace=sim_namespace,
        output='screen',
        arguments=['--x', initial_pose_x,
                   '--y', initial_pose_y,
                   '--yaw', initial_pose_a,
                   '--frame-id', '/map',
                   '--child-frame-id', [sim_namespace, 'odom']]
    )

    # Laser scanner
    laser = Node(
        package='laser_simulator',
        executable='laser_sim_node',
        exec_name='laser_sim_node',
        namespace=sim_namespace,
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('laser_simulator'),
                    'models',
                    'LDS_01.yaml']
            ),
            {'frame_id': [sim_namespace, 'base_scan']},
            {'depth': 1.0},
            {'offset/x': 0.0},
            {'offset/y': 0.0},
            {'offset/z': 0.0},
        ],
        remappings=[
            ('map', '/map'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ]
    )

    # Sensor data republisher
    sensor_republisher = Node(
        package='mee4411_simulation',
        executable='sensor_republisher',
        exec_name='sensor_republisher',
        namespace=sim_namespace,
        output='screen',
        parameters=[
            {'noise_std': PythonExpression([joint_noise_std, '/pi'])},
            {'namespace_remove': sim_namespace}
        ],
        remappings=[
            ('~/scan_out', '/scan'),
            ('~/joint_states_out', '/joint_states'),
        ]
    )

    return LaunchDescription([
        model_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_a_arg,
        joint_noise_std_arg,
        sim_namespace_arg,
        robot_state,
        localization,
        odometry,
        laser,
        sensor_republisher
    ])
