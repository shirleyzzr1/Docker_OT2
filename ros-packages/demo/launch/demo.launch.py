from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    Nodelist = [
        # Node(
        #     package = 'image_tools',
        #     namespace = '',
        #     executable = 'cam2image',
        #     name = 'cam2image',
        #     arguments=['--ros-args','--log-level','WARN'],
        #     parameters=[
        #     {"device_id": 4}]
        # ),
        Node(
            package = 'demo',
            namespace = os.getenv('robot_name'),
            executable = 'action_server',
            name = 'action_server'
        )
    ]
    return LaunchDescription(Nodelist)