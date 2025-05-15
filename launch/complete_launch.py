from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    node1 = Node(package='cocos_drive_controller',
                          executable='line_detection',
                            name="line_detection",
                          )
        
    node2 = Node(package='cocos_drive_controller',
                       executable='color_detection',
                       name="color_detection",
                       )
    
    node3 = Node(package='cocos_drive_controller',
                       executable='main_controller',
                       name="main_controller",
                       )
    
    node4 = Node(package='cocos_drive_controller',
                        executable='cmd_robot',
                        name="cmd_robot",
                        )
    
    l_d = LaunchDescription([node1, node2, node3, node4])

    return l_d