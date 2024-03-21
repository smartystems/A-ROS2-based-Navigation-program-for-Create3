from launch import LaunchDescription
from launch_ros.actions import Node

name =input("Robot's namespace: ")

# Set default namespace if nothing is given
if name == "":
    name = "/S2R6"

def generate_launch_description():

    sub_cmd = Node(
        package='navigator',
        executable='navigation',
        parameters=[{
            'use_sim_time': False
        }],
        remappings=[
        ('cmd_vel', name + '/cmd_vel'),
        ('cmd_lightring', name + '/cmd_lightring'),
        ('ir_intensity', name + '/ir_intensity')
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(sub_cmd)

    return ld
