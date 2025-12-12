import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

# Hàm tiện ích để load file YAML
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # --- 1. KHAI BÁO TÊN GÓI VÀ ROBOT ---
    # Bạn phải thay đổi tên này khớp với package của bạn
    MY_PACKAGE = "robot_6dof_moveit_config1" 
    
    # --- 2. LOAD CÁC FILE CẤU HÌNH (Phần quan trọng nhất) ---

    # A. Robot Description (URDF/Xacro)
    # File xacro gốc thường nằm trong gói config hoặc gói mô tả robot gốc
    xacro_file = os.path.join(get_package_share_directory(MY_PACKAGE), 'config', 'robot_6dof.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # B. Robot Description Semantic (SRDF)
    srdf_file = os.path.join(get_package_share_directory(MY_PACKAGE), 'config', 'robot_6dof.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # C. Kinematics (Động học)
    kinematics_yaml = load_yaml(MY_PACKAGE, 'config/kinematics.yaml')

    # D. Joint Limits (Giới hạn khớp)
    # MoveIt 2 cần load file này để biết giới hạn tốc độ/gia tốc
    joint_limits_yaml = load_yaml(MY_PACKAGE, 'config/joint_limits.yaml')
    
    # E. MoveIt Planning (OMPL)
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(MY_PACKAGE, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # F. Trajectory Execution (Cấu hình thực thi quỹ đạo)
    # Để chạy demo giả lập, ta cần cấu hình fake controller manager
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # G. MoveIt Controllers (File định nghĩa controller)
    moveit_controllers = load_yaml(MY_PACKAGE, 'config/moveit_controllers.yaml')

    # H. Cấu hình hiển thị RViz
    rviz_config_file = os.path.join(get_package_share_directory(MY_PACKAGE), 'config', 'moveit.rviz')

    # --- 3. KHỞI TẠO CÁC NODE ---

    # Node 1: Robot State Publisher
    # Nhiệm vụ: Công bố TF của robot dựa trên URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Node 2: Virtual Joint Broadcaster (Static TF)
    # Nhiệm vụ: Nối 'world' với 'base_link' của robot nếu robot cố định
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )

    # Node 3: Move Group Node (TRÁI TIM CỦA MOVEIT)
    # Đây là node nhận lệnh plan và tính toán
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            {'use_sim_time': True}, # Quan trọng nếu dùng mô phỏng
        ],
    )

    # Node 4: RViz
    # Giao diện hiển thị
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )
    
    # Node 5: Fake Joint Driver (Thay thế cho ros2_control trong bản Demo đơn giản)
    # Để có thể kéo thả robot mà không cần phần cứng thật hay Gazebo,
    # ta dùng Joint State Publisher GUI để giả lập các khớp.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'source_list': ['/move_group/fake_controller_joint_states']}] 
    )

    # Trả về danh sách các Node cần chạy
    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        rviz_node,
        # Nếu muốn dùng bảng điều khiển khớp thủ công thì uncomment dòng dưới:
        # joint_state_publisher_gui 
    ])