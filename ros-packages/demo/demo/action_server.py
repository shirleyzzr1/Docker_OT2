import time
# import subprocess
# import yaml
import os

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from demo_interfaces.action import OT2Job
from demo_interfaces.msg import EmergencyAlert
from demo_interfaces.msg import Heartbeat
from demo_interfaces.srv import RaiseEmergency


from std_msgs.msg import String

from ot2_driver.ot2_driver_http import OT2_Driver, OT2_Config
from opentrons.simulate import simulate, format_runlog



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
            goal_callback=self.action_goal_callback,
            handle_accepted_callback=self.goal_accepted_callback,
            execute_callback=self.action_execute_callback,
            cancel_callback=self.cancel_goal_callback
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
        self._heartbeat_info = ""

        ## Alert that the Action Server has been created
        self.get_logger().info("OT2 ActionServer {} running!".format(self.get_fully_qualified_name()))
        self.get_logger().info("Awaiting robot and protocol configuration from OT2 ActionClient ...")

        ## To Store Recieved configs as yaml files
        self.rc_document_path = "/root/config/temp/rc_document.yaml"
        self.pc_document_path = "/root/config/temp/pc_document.yaml"

        ## Timer(s) Settings
        # self.wait_out_emergency_timer_period = 1
        self._exec_cb = "original"

    def get_fully_qualified_name(self) -> str:
        return "{}/{}".format(self.get_namespace(), self.get_name())

    def emergency_callback(self,msg):
        """
        Update private emergency_flag status and Heartbeat state 
        BUSY + emergency alert ==> pause, EMERGENCY
        EMERGENCY + cleared alerts ==> resume, BUSY
        """


        if msg.is_emergency and not self.emergency_flag:
            self.emergency_flag = True

            ## TODO: Should include the source of error in the warn
            self.get_logger().warn("{} action received an emergency alert: {}".format(self.get_fully_qualified_name(),msg.message)) 
            
            if self._heartbeat_state == Heartbeat.BUSY:
                self._heartbeat_state = Heartbeat.EMERGENCY
                self.get_logger().info("BUSY ---> EMERGENCY")
                self.get_logger().warn("Pausing OT2...")
                pause_response = self.ot2.pause(self.run_id)
                ## TODO: verify pause response
               

        elif not msg.is_emergency and self.emergency_flag:
            self.emergency_flag = False
            self.get_logger().info("Emergency alert(s) cleared.")
          
            if self._heartbeat_state == Heartbeat.EMERGENCY:
                self._heartbeat_state = Heartbeat.BUSY
                self.get_logger().info("EMERGENCY ---> BUSY")
                ## TODO Include a resume call if state is on emergency
                resume_respone = self.ot2.execute(self.run_id)
                ## TODO: verify resume response

    def heartbeat_timer_callback(self):
        """
        Update the heartbeat's header timestamp and heartbeat state
        and publish
        """
        self.heartbeat_msg.state = self._heartbeat_state
        self.heartbeat_msg.message = self._heartbeat_info
        self.heartbeat_msg.header.stamp = self.get_clock().now().to_msg()
        self.heartbeat_publisher.publish(self.heartbeat_msg)

    def action_goal_callback(self, requested_goal):
        """
        Log new request; 
        Check for required components of the request 
        Accept(GoalResponse.ACCEPT) or reject (GoalResponse.REJECT)
        """

        self.get_logger().info("Recieved new goal request")

        ## Validating recvd goal request
        ## If not valid, Reject Goal
        ## TODO: How to Propogate the error message upward

        valid, self.validation = self.validate_goal_request(requested_goal.job)
        if valid:
            path = '/root/config/temp'
            if not os.path.exists(path):
                os.mkdir(path)
            self.get_logger().info("Writing protocol config to {} ...".format(self.pc_document_path))
            with open(self.pc_document_path,'w',encoding = "utf-8") as pc_file:
                pc_file.write(self.validation["protocol_config"])   
          
        
        else: 
            self.get_logger().info(self.validation)
            return GoalResponse.REJECT

        ## Could check for emergency here, but cannot defer to a timer to 
        ## return the GoalReponse.ACCEPT
        ## Instead will check for emergency in the goal_accepted_callback, 
        ## which can defer the execute() to a timer and hence delay the 
        ## succeed() terminal state requrired 
        ## This timer is only reqd if the goal is to wait out the emergency without a reissue

        # if self.emergency_flag:
        #     return GoalResponse.REJECT 
         
        return GoalResponse.ACCEPT

    def goal_accepted_callback(self, goal_handle):
        """
        Compiles protocol
        Transfers to OT2
        Constructs the base feedback message
        Launchs execution timer for timer-based async feedback  
        TODO: status of ot2 when just loaded with protocol
        """

        self._goal_handle = goal_handle

        self.ot2 = OT2_Driver(OT2_Config(ip=self.validation["robot_ip"]))
        self.protocol_file_path, self.resource_file_path = self.ot2.compile_protocol(self.pc_document_path)
        self.feedback_msg = OT2Job.Feedback()
        self.feedback_msg.progress.header.src = self.get_fully_qualified_name()
        
        if self.validation["simulate"] or (os.getenv('simulate') and os.getenv('simulate').lower() == 'true'):
            self.get_logger().warn("Running protocol in SIMULATION mode...")
            self.i = 0
            self.simulate_timer = self.create_timer(1, self.simulate_timer_callback)
        else:       
            self.execute_protocol_timer = self.create_timer(1, self.execute_protocol_timer_callback)

    def execute_protocol_timer_callback(self):
        """
        Feedback on changes only
        Updates to Heartbeat state, and feedback messages
        If IDLE + onEmergency ==> Cannot execute during emergency, stay IDLE
        If IDLE + onClearedEmergecny ==> execute, go BUSY if successful, else terminate

        When running
         - if running --> report "running"
         - if finishing --> report "finishing"
         - if completed --> report "success", IDLE, cancel timer
        """

        if self._goal_handle.is_cancel_requested:
            self._goal_handle.canceled()
            self.get_logger().info("canceled")
            self.execute_protocol_timer.cancel()
            return OT2Job.Result()

        ## Starting the Protocol only if ActionServer is primed to run and no emergency reported
        ## Effectively waits for emergencies to clear, then runs 
        
        if self._heartbeat_state == Heartbeat.IDLE:
            
            if self.emergency_flag:
                self._heartbeat_info = "[ERROR] Cannot execute protocols during an Emergency"
            else:
                self.protocol_id, self.run_id = self.ot2.transfer(self.protocol_file_path)
                execute_resp = self.ot2.execute(self.run_id)

                ## Response to a successful Run initiated
                if execute_resp["data"]["actionType"] == "play": 

                    self._heartbeat_info = "Run ({}) created at {}".format(execute_resp["data"]["id"], execute_resp["data"]["createdAt"])
                    self._heartbeat_state = Heartbeat.BUSY
                    self.get_logger().info("IDLE ---> BUSY") 
                    self.get_logger().info(self._heartbeat_info)
                    
                else:  
                ## Response to error in run initiation
                    self._heartbeat_info = "[OT2 Error] {}: {}".format(execute_resp["errors"][0]["id"], execute_resp["errors"][0]["detail"])
                    self._heartbeat_state = Heartbeat.ERROR
                    self.get_logger().info("IDLE ---> ERROR") 
                    self.get_logger().error(self._heartbeat_info)
                    ## TODO: verify how appropos this is
                    self.execute_protocol_timer.cancel()
                    self._goal_handle.abort()
        
        else:

            run_status = self.ot2.get_run(self.run_id)

            if run_status["data"]["status"] == "running":
                self._heartbeat_info = "OT2 Running"

                ## TODO Possibly Elapsed Time, Progress/Fraction
                ## TODO Reporting running should be removed and reporting should be 
                ## defered to the back and forth of 
                ## zmq request --> trigger to vision service --> zmq response
                ## TODO defered to 

                
            elif run_status["data"]["status"] == "paused":
                self._heartbeat_info = "OT2 Paused"

            elif run_status["data"]["status"] == "finishing":
                self._heartbeat_info = "OT2 Finishing"

            elif run_status["data"]["status"] == "succeeded":
                self._heartbeat_state = Heartbeat.IDLE
                self.get_logger().info("OT2 Success!")
                self.get_logger().info("BUSY --> IDLE")
                self._heartbeat_info = "OT2 completed job {} at {}".format(run_status["data"]["id"], run_status["data"]["completedAt"])
                
                self.feedback_msg.progress.header.stamp = self.get_clock().now().to_msg()
                self.feedback_msg.progress.progress_msg = self._heartbeat_info
                self._goal_handle.publish_feedback(self.feedback_msg)

                self.execute_protocol_timer.cancel()
                if self._exec_cb == "simulate":
                    self._action_server.register_execute_callback(self.action_execute_callback)
                    self._exec_cb = "original"
                self._goal_handle.execute()
            
        if self._heartbeat_info != self.feedback_msg.progress.progress_msg:
            ## TODO: Uncomment when feedback messages are responding with job progression updates
            # self.feedback_msg = OT2Job.Feedback()
            # self.feedback_msg.progress.header.src = self.get_fully_qualified_name()
            ## ALTERNATIVELY, store progress metrics as instance variables ie self.xyz

            self.feedback_msg.progress.header.stamp = self.get_clock().now().to_msg()
            self.feedback_msg.progress.progress_msg = self._heartbeat_info
            self._goal_handle.publish_feedback(self.feedback_msg)


    def simulate_timer_callback(self):

        response_message = ""

        if self.emergency_flag:

            response_message = "Simulation Paused"
                
        elif self.i <= 10:

            response_message = "Simulation Running"

            if self._goal_handle.is_cancel_requested:
                self._goal_handle.canceled()
                response_message = "Simualtion Canceled"
                self.simulate_timer.cancel()
                return OT2Job.Result()
            # self.get_logger().warn(str(self.i))
            
            feedback_msg = OT2Job.Feedback()
            feedback_msg.progress.progress_msg = str(self.i)
            self._goal_handle.publish_feedback(feedback_msg)
            self.i += 1

        else:

            if self._exec_cb != "simulate":
                self._action_server.register_execute_callback(self.simulate_only_execute_callback)
                self._exec_cb = "simulate"
            self.simulate_timer.cancel()
            self._goal_handle.execute() 

        if response_message != self._heartbeat_info:
            self._heartbeat_info = response_message
            self.get_logger().info(self._heartbeat_info)    


    def action_execute_callback(self, goal_handle):
        """
        Signal succesful job outcome to ActionClient  
        """
        
        ## Formulating and returning response to the ActionClient
        goal_handle.succeed()
        result_msg = OT2Job.Result()
        result_msg.success = True
        result_msg.error_msg = self._heartbeat_info
        
        return result_msg


    def simulate_only_execute_callback(self, goal_handle):
        
        resp, _bundle = simulate(open(self.protocol_file_path))

        goal_handle.succeed()
        result = OT2Job.Result()
        result.success = True
        result.error_msg = format_runlog(resp)

        self.get_logger().info("Simulation Output:")
        self.get_logger().info(result.error_msg)
        response_message = "Simulation Finished"
        self.get_logger().info(response_message)
        
        return result


    def cancel_goal_callback(self, cancel_request):
        self.get_logger().info("Recieved cancel request. Cancelling...")
        return CancelResponse.ACCEPT


    def validate_goal_request(self, job):
        """
        Check for:
         - required robot_ip (possible regex check for ip address, if not validated upstream with workflow)
         - required protocol_config (yaml content as string; assumed yaml validate upstream with workflow)
        """

        protocol_config = None
        robot_ip = None
        
        ## Check that an ip address was sent
        ## [To-Do?]: Validate IP address signature by regex

        if job.robot_ip:
            self.get_logger().info("OT2JobActionClient:{} provided a robot IP address: {}".format(job.header.src, job.robot_ip))
            robot_ip = job.robot_ip
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

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    # executor = MultiThreadedExecutor()

    try:
        rclpy.spin(action_server) #,executor=executor)
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
