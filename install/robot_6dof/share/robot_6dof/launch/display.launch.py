import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_name = 'robot_6dof'
    urdf_file_name = 'robot_6dof.urdf'
    rviz_file_name = 'urdf.rviz'  # <-- Tên file config bạn vừa lưu ở Bước 1

    pkg_share = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    
    # Tạo đường dẫn tới file config RViz
    rviz_config_path = os.path.join(pkg_share, 'rviz', rviz_file_name)

    # Đọc file URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Gói nội dung URDF
    robot_description_param = ParameterValue(robot_desc, value_type=str)

    return LaunchDescription([
        # Node 1: Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_param}]
        ),

        # Node 2: Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Node 3: RViz2 (Đã thêm tham số nạp file config)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path] 
        )
    ])