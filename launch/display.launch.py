import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'final_project'
    pkg_share = get_package_share_directory(pkg_name)
    
    # URDF 파일 경로 설정
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm1_URDF_plus.urdf')
    
    # URDF 파일 읽기
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Robot State Publisher 노드 (TF 변환)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 2. Joint State Publisher GUI 노드 (관절을 마우스로 조종할 수 있는 창)
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 3. RViz2 노드 (3D 시각화)
    # rviz 폴더 안에 기본 설정 파일이 없다면 그냥 빈 화면으로 켜집니다.
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'display.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])