# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import TimerAction

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""

    if_rviz = True
    if_sim = False

    robot_bringup_path = get_package_share_directory("img_process")
    robot_bt_decision_maker_path = get_package_share_directory("robot_bt_decision_maker")
    yaml_path = os.path.join(robot_bringup_path, "config", "params.yaml")

    param_yaml_path = LaunchConfiguration("params_file", default=yaml_path)
    declare_yaml_path = DeclareLaunchArgument(
        "params_file",
        default_value=param_yaml_path,
        description="Full path to the configuration file to load",
    )
    param_launch_rviz = LaunchConfiguration("launch_rviz", default=if_rviz)
    declare_launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value=param_launch_rviz,
        description="Whether to run rviz",
    )
    param_rviz_config_dir = LaunchConfiguration(
        "rviz_config_dir",
        default=os.path.join(robot_bringup_path,"config","rviz_config.rviz"), #(nav2_bringup_dir, "rviz", "nav2_default_view.rviz"),
    )
    declare_rviz_config_dir = DeclareLaunchArgument(
        "rviz_config_dir",
        default_value=param_rviz_config_dir,
        description="Full path to the rviz config file to load",
    )
    param_launch_gazebo = LaunchConfiguration("launch_gazebo", default=if_sim)
    declare_launch_gazebo = DeclareLaunchArgument(
        "launch_gazebo",
        default_value=param_launch_gazebo,
        description="Whether to run gazebo",
    )
    # img_process_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [img_process_path, "/launch", "/img_process_node.launch.py"]
    #     ),
    #     launch_arguments={
    #         "params_file": param_yaml_path,
    #     }.items(),
    # )
    img_process_node = Node(
        package="img_process",
        executable="img_process_node",
        name="img_process_node",
        output="screen",
        parameters=[param_yaml_path],
    )
    robot_serial_node = Node(
        package="robot_serial",
        executable="robot_serial",
        output="screen",
        name="robot_serial_node",
        parameters=[param_yaml_path],
        respawn=True # 重启
    )
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_bringup_path, "/launch", "/bringup_launch.py"]
        ),
        launch_arguments={
            "params_file": param_yaml_path,
            "use_sim_time": param_launch_gazebo,
        }.items(),
    )
    bt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [robot_bt_decision_maker_path, "/launch", "/robot_bt_decision_maker.launch.py"]
        ),
    )
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom']
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", param_rviz_config_dir],
        parameters=[{"use_sim_time": param_launch_gazebo}],
        output="screen",
        condition=IfCondition(param_launch_rviz),
    )

    # 创建LaunchDescription对象launch_description,用于描述launch文件
    # 我尝试下来，在lio启动时，系统不能负载太高，因此，选择在lio启动后再启动其他节点，这个时间可以根据实际情况调整
    list = [declare_launch_gazebo, 
            declare_yaml_path, 
            declare_launch_rviz, 
            declare_rviz_config_dir, 
            tf_node,
            bt_launch,
            img_process_node,
            navigation_launch,
            robot_serial_node,
            # rviz_node
            ]


    # 返回让ROS2根据launch描述执行节点
    return LaunchDescription(list)
