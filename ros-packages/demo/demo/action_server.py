from distutils.log import info
import time
import subprocess
import yaml
import os


import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from demo_interfaces.action import OT2Job



class DemoActionServer(Node):
    """

    """
    def __init__(self):
        """"
        
        """
        super().__init__('demo_action_server')
        self._action_server = ActionServer(
            self, OT2Job, 
            'OT2', 
            self.action_callback
        )

        self.get_logger().info("OT2 Action Server running!")
        self.get_logger().info("Awaiting robot and protocol configuration from action client ...")
        

    def action_callback(self, goal_handle):
        """

        """
        
        python_file_path = "/root/ot2_driver/ot2_driver/ot2_driver_ssh.py"
        rc_document_path = "/root/config/temp/rc_document.yaml"
        pc_document_path = "/root/config/temp/pc_document.yaml"

        job = goal_handle.request.job
        robot_config = job.robot_config
        protocol_config = job.protocol_config

        ## Comment out to test hard-coded job_path 
        pc_path = job.pc_path
        rc_path = job.rc_path
        simulate = job.simulate

        ## Uncomment to test out hard_coded job_path
        # simulate = True
        # rc_path = "/root/config/test_config.yaml"
        # pc_path = "/root/ot2_driver/ot2_driver/protopiler/example_configs/basic_config.yaml"
        
        
        ## Log/Print the recieved Job
        self.get_logger().info("Recieved New Job Request...")
        if job.rc_path != None:
            self.get_logger().info(".. robot config path: {}".format(job.rc_path))
        if job.pc_path != None:
            self.get_logger().info(".. protocol config path: {}".format(job.pc_path))


        path = '/root/config/temp'
        if not os.path.exists(path):
            os.mkdir(path)

        ## Checking if path or raw configs were sent in the job message
        ## Current job message definition allows either option
        if job.rc_path=='None':
            rc_path = rc_document_path
            self.get_logger().info("Writing robot config to {} ...".format(rc_document_path))
            with open(rc_document_path, 'w',encoding = "utf-8") as rc_file:
                rc_file.write(job.robot_config)

        if job.pc_path=='None':
            pc_path = pc_document_path
            self.get_logger().info("Writing protocol config to {} ...".format(pc_document_path))
            with open(pc_document_path,'w',encoding = "utf-8") as pc_file:
                pc_file.write(job.protocol_config)
        
        ## Calling the script that compiles, transfers and execute OT2 instructions
        success = False
        cmd = ["python3", python_file_path, "-rc", rc_path, "-pc", pc_path, "-v"]
        if simulate:
            cmd.append("-s")

        completed_process = subprocess.run(cmd, capture_output=True, text=True)  #check=True

        ## Setting the goal state to acknowledge the action client
        goal_handle.succeed()
        
        ## Print/Log output from script call
        self.get_logger().info("########## Completed Process #########")

        self.get_logger().info("STDERR")
        self.get_logger().info(completed_process.stderr)

        self.get_logger().info("STDOUT")
        self.get_logger().info(completed_process.stdout)

        self.get_logger().info("################ end ################")

        ## Formulating response to the action client
        result = OT2Job.Result()
        result.error_msg = completed_process.stdout

        if not completed_process.returncode:
            success = True
        result.success = success

        return result

def main(args=None):
    """

    """
    rclpy.init(args=args)
    action_server = DemoActionServer()

    try:
        rclpy.spin(action_server)
        action_server.logger().info("OT2 Action Server running!")
        action_server.logger().info("Awaiting robot and protocol configuration from action client ...")
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()