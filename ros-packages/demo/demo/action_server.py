from distutils.log import info
import time
import subprocess
from unittest import result
import yaml
import os

import rclpy
from rclpy.action import ActionServer, GoalRepsonse
from rclpy.node import Node

from demo_interfaces.action import OT2Job
from demo_interfaces.msg import EmergencyAlert
from demo_interfaces.msg import Heartbeat
from demo_interfaces.srv import RaiseEmergency


from std_msgs.msg import String

from ot2_driver.ot2_driver_http import OT2_Driver, OT2_Config



class DemoActionServer(Node):
    """

    """
    def __init__(self):
        """
        Subscribe to emergency alerts
        Create client to report emergencies
        Publish Heartbeat on a timer
        Create action server awaiting OT2 goals/jobs, and reporting back from OT2
        TODO Reject goals on emergency or accept and not execute
        """
        super().__init__('demo_action_server')
        
        ## Action server to be namespaced with action server 
        self._action_server = ActionServer(
            self, OT2Job, 
            'OT2', 
            execute_callback=self.action_execution_callback
        )

        ### TODO - Namespaces are taken care off automatically: No need to concatenate strings
        ### TODO - Need to remove all references
        self.name = self.get_namespace()[1:]

        ## Set up Emergency tracking and service proxy (client)
        self.emergency_sub = self.create_subscription(EmergencyAlert,'/emergency',self.emergency_callback,10)
        self.emergency_flag = False  # TODO maybe should default to True
        # self.emergency_client = self.create_client(RaiseEmergency, "/raise_emergency")
        
        ## Set up Heartbeat publisher/publishing on a timer
        heartbeat_timer_period = 2 #seconds TODO
        self.heartbeat_timer = self.create_timer(heartbeat_timer_period, self.heartbeat_timer_callback)
        self.heartbeat_publisher = self.create_publisher(Heartbeat, '{}/heartbeat'.format(self.get_name()), 10)
        self.heartbeat_msg = Heartbeat()
        self.heartbeat_msg.header.src = self.get_fully_qualified_name()
        self._heartbeat_state = Heartbeat.IDLE  # TODO maybe should default to BUSY or ERROR

        ## Alert that the Action Server has been created
        self.get_logger().info("OT2 ActionServer {} running!".format(self.get_fully_qualified_name()))
        self.get_logger().info("Awaiting robot and protocol configuration from OT2 ActionClient ...")


    def emergency_callback(self,msg):
        """
        Update private emergency status 
        """

        if msg.isEmergency and not self.emergency_flag:
            self.emergency_flag = True
            self.get_logger().warn("{} action received an emergency alert: {}".format(self.get_fully_qualified_name(),msg.message)) 
            ## TODO: Should include the source of error in the warn

        if not msg.isEmergency and self.emergency_flag:
            self.emergency_flag = False
            self.get_logger().info("Emergency alert(s) cleared.")

    def heartbeat_timer_callback(self):
        """
        Update the heartbeat's header timestamp and heartbeat state
        and publish
        """
        self.heartbeat_msg.state = self._heartbeat_state
        self.heartbeat_msg.header.stamp = self.get_clock().now().to_msg()
        self.heartbeat_publisher.publish(self.heartbeat_msg)

    def action_goal_callback(self, requested_goal):
        """
        TODO: Log new; check for emrgency state then accept(GoalResponse.ACCEPT) or reject (GoalResponse.)
        """
        pass

    def goal_accepted_callback(self, goal_handle):
        """
        TODO: Unpack and Save yaml 
        """
        pass

    def action_execute_callback(self, goal_handle):
        """
            TO DO
            Required aspects == All the checks implemented
        """
        ## Configurations/Settings
        feedback_sleep_time = 0.5
        rc_document_path = "/root/config/temp/rc_document.yaml"
        pc_document_path = "/root/config/temp/pc_document.yaml"
        success = False

        ## Setup Feedback and Result msgs
        result_msg = OT2Job.Result()
        result_msg.header.src = self.get_fully_qualified_name()
        feedback_msg = OT2Job.Feedback()

        ## Validating recvd goal request
        valid, validation = self.validate_goal_request(goal_handle)
        if valid:
            path = '/root/config/temp'
            if not os.path.exists(path):
                os.mkdir(path)
            self.get_logger().info("Writing protocol config to {} ...".format(pc_document_path))
            with open(pc_document_path,'w',encoding = "utf-8") as pc_file:
                pc_file.write(validation["protocol_config"])        
        
        else: 
            self.get_logger().info(validation)
            result_msg.error_msg = validation

        ## Create ot2_driver object 
        self.ot2 = OT2_Driver(OT2_Config(ip=self.robot_ip))
            
        ## Compile, transfer then execute OT2 protocol

        ## OLD SUBPROCESSES VERSION
        # cmd = ["python3", python_file_path, "-rc", rc_path, "-pc", pc_path, "-v"]
        # if simulate:
        #     cmd.append("-s")
        # if not self.emergency_flag:
        #     completed_process = subprocess.run(cmd, capture_output=True, text=True)  #check=True

        ## NEW OT2 DRIVER MODULE
        ## TODO might have to not make these variables instance variables; event of new goal will override
        ##      Alternatively could make this node only take one action at a time via thread locks 
            
        self.get_logger().error("1")
        self.protocol_file_path, self.resource_file_path = self.ot2.compile_protocol(pc_document_path)
        self.get_logger().error("2 {}, {}".format(self.protocol_file_path, self.resource_file_path))

        self.protocol_id, self.run_id = self.ot2.transfer(self.protocol_file_path)
        self.get_logger().error("3")
        self.get_logger().warn("run_id: {}".format(self.run_id))
        self.get_logger().error("4")
        
        if not self.emergency_flag:
            execute_resp = None
            if validation["simulate"]:
                
                # TODO subprocess call to simulate
                # execute_resp = "" ## TODO stdout from subprocess
                
                # ## TODO handle error and success
                # if execute_resp.returncode == 0:
                #     success = True
                #     goal_handle.succeed()

                # result_msg.error_msg = execute_resp.stdout  ## TODO
                # return result_msg 
                pass 

            else: 
                
                execute_resp = self.ot2.execute(self.run)
                self.get_logger().error("5")
                self.get_logger().warn("execute_resp: {}".format(execute_resp))
                self.get_logger().error("6")

            ## Setting the goal state to acknowledge to the ActionClient TODO remove
            ## Update Heartbeat State, Log response from OT2/OT2_driver
            ## TO-DO: Verify/Validate response from OT2/OT2_driver
            if execute_resp: ##TODO require something more specific from resp

                self.get_logger().info(execute_resp)
                self._heartbeat_state = Heartbeat.BUSY
                self.get_logger().info("IDLE --> BUSY") #placeholder
                
            else:  ##TODO require something more specific from resp

                self._heartbeat_state = Heartbeat.ERROR
                self.get_logger().info("IDLE --> ERROR") #placeholder
                # Must alert the error type and propagate to ActionClient


            ## Getting Feedback through to ActionClient

            ## TODO: adapt to the specifics of how OT2 responds to the status request
            ##      - what to do when running and when done and when something is reported

            run_status = self.ot2.get_run(self.run_id)
            while run_status == "in-progress": ## TODO
                feedback_msg.progress_msg = run_status
                feedback_msg.header.stamp = self.get_clock().now().to_msg()
                goal_handle.publish_feedback(feedback_msg)
                run_status = self.ot2.get_run(self.run_id)
                ## need to exit when done

                if run_status == "error": ## TODO
                    self._heartbeat_state = Heartbeat.ERROR
                    result_msg.error_msg = "" ## TODO Error from the OT2

                elif run_status == "finished": ## TODO
                    self._heartbeat_state = Heartbeat.FINISHED  # TODO but according to terminate set in the goal request
                    self.get_logger().info("BUSY --> {}".format(validation["termination_state"]))
                    success = True

                time.sleep(feedback_sleep_time)

        else:
            result_msg.error_msg = "[ERROR] Cannot execute protocols during an Emergency"
            self.get_logger().error(result_msg.error_msg)
            
        ## Formulating and returning response to the ActionClient
        if success:
            goal_handle.succeed() 
        result_msg.success = success
        return result_msg



    def validate_goal_request(self, _goal_handle):
        """
        Check for:
         - required robot_ip (possible regex check for ip address, if not validated upstream with workflow)
         - required protocol_config (yaml content as string; assumed yaml validate upstream with workflow)
        """

        job = _goal_handle.request.job
        protocol_config = None
        robot_ip = None
        
        ## Check that an ip address was sent
        ## [To-Do?]: Validate IP address signature by regex

        if job.robot_ip:
            self.get_logger().info("OT2JobActionClient:{} provided a robot IP address: {}".format(job.header.src, job.ip_address))
            robot_ip = job.ip_address
        else:
            response = "OT2JobActionClient:{} did not provide the required robot_ip".format(job.header.src)
            self.get_logger().error(response) 
            return False,  response

        
        ## Check that protocol configuration was sent
        ## Validation of yaml content should be completed upstream

        if job.protocol_config:
            self.get_logger().info("OT2JobActionClient:{} provided a protocol condfiguration".format(job.header.src))
            protocol_config = job.protocol_config
        else:
            response = "OT2JobActionClient:{} did not provide the required protocol_config".format(job.header.src)
            self.get_logger().error(response) 
            return False,  response

        return True, {  "robot_ip": robot_ip, 
                        "protocol_config": protocol_config, 
                        "termination_state": job.termination_state,
                        "simulate": job.simulate}

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

## Plan to Handle Pauses
## onEmercency callback: 
    ## if heartbeat_status is busy, pause and set status to Emergency
    ## if idle, error do nothing
## offEmergecny callback
    ## if heartbeat_status is emergency, recover to busy and  resume

## Heartbeat consideration:
## Intent behind action server termination state should be offloaded to action client
    ## Action server should always resets to IDLE
    ## Action client terminates on FINISHED or IDLE depending on if needed again 
## Action server hearbeat needed for
    ## client checking aliveness before sending goal
    ## pause and resume states