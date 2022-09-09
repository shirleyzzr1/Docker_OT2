import time
import os

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node

from demo_interfaces.action import OT2Job
from demo_interfaces.msg import EmergencyAlert
from demo_interfaces.msg import Heartbeat
from demo_interfaces.srv import RaiseEmergency

from ot2_driver.ot2_driver_http import OT2_Driver, OT2_Config
from opentrons.simulate import simulate, format_runlog



class DemoActionServer(Node):
    """
    A ros node to interface directly with the OT2, serving as the central 
    process of the robot terminal connected to the OT2.
    This node provides a ros action server, through which action clients can 
    send goal requests to instruct the OT2 to execute it core capabilities 
    of picking pipettes, aspirate and dispensing substrates and dropping off 
    pipettes for disposal.     

    ActionServer:
     - ~/OT2 [demo_interfaces/action/OT2Job] -- for instructing OT2 run defined by protocol configurations 

    Publishers:
     - ~/action_server/heartbeat [demo_interfaces/msg/Heartbeat] -- to alert subscribers to OT2(+node) state


    Subscribers:
     - /emergency [demo_interfaces/msg/EmergencyAlert]  -- for global emergency tracking


    Service Clients:
     - /raise_emergency [demo_interfaces/srv/RaiseEmergency] -- As part of global emergency system to alert system to OT2 emergencies [currently deactivated]
     - /clear_emergency [demo_interfaces/srv/RaiseEmergency] -- As part of global emergency system to alert system to resolved OT2 emergency [currently deactivated]
     



    """
    def __init__(self):
        """
        Subscribe and respond to Emergency Alerts system
        Create service client to report emergencies to Emergency Alerts system
        Publish Heartbeat on a timer
        Create action server awaiting OT2 goals/jobs, and reporting back from OT2
        Accept goals during emergency event but do not execute until no remaining event
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
        

        ## Set up global Emergency tracking and service proxy (client)
        ## Initialized as True for safety; waiting for clearance
        ## No use case here for raising emergency so client is commented out
        self.emergency_sub = self.create_subscription(EmergencyAlert,'/emergency',self.emergency_callback,10)
        self.emergency_flag = True      
        # self.emergency_flagging_client = self.create_client(RaiseEmergency, "/raise_emergency") 
        # self.emergency_deflaggin_client = self.create_client(RaiseEmergency, "/clear_emergency") 
        

        ## Set up Heartbeat publisher/publishing on a timer
        heartbeat_timer_period = 2 #seconds 
        self.heartbeat_timer = self.create_timer(heartbeat_timer_period, self.heartbeat_timer_callback)
        self.heartbeat_publisher = self.create_publisher(Heartbeat, '{}/heartbeat'.format(self.get_name()), 10)
        self.heartbeat_msg = Heartbeat()
        self.heartbeat_msg.header.src = self.get_fully_qualified_name()
        self._heartbeat_state = Heartbeat.IDLE 
        self._heartbeat_info = ""


        ## Alert that the Action Server has been created
        self.get_logger().info("OT2 ActionServer {} running!".format(self.get_fully_qualified_name()))
        self.get_logger().info("Awaiting robot and protocol configuration from OT2 ActionClient ...")


        ## To Store Recieved configs as yaml files
        self.rc_document_path = "/root/config/temp/rc_document.yaml"
        self.pc_document_path = "/root/config/temp/pc_document.yaml"


        ## Flag for toggling between "orginal" and "simulate" action execute callback
        self._exec_cb = "original"



    def get_fully_qualified_name(self) -> str:
        """
        Return a concatenation of node namespace and node name
        """
        return "{}/{}".format(self.get_namespace(), self.get_name())



    def emergency_callback(self ,msg):
        """
        Update emergency_flag status and Heartbeat state 
        BUSY state + emergency alert ==> trigger pause, EMERGENCY state
        EMERGENCY state + cleared alert ==> trigger resume, BUSY state
        """

        if msg.is_emergency and not self.emergency_flag:

            self.emergency_flag = True
            self.get_logger().warn("Received an emergency alert: {}".format(msg.message)) 
            self.get_logger().warn("Emergency reported by {} at {}".format(msg.header.src, msg.header.stamp))  ## TODO Verify correcting printing
            
            if self._heartbeat_state == Heartbeat.BUSY:
                self._heartbeat_state = Heartbeat.EMERGENCY
                self.get_logger().warn("BUSY ---> EMERGENCY")
                self.get_logger().warn("Pausing OT2 run...")
                pause_response = self.ot2.pause(self.run_id)
                ## TODO Update heartbeat info?
                
                ## Verifying pause
                try: 
                    if pause_response["data"]["actionType"] == "pause":
                        self.get_logger().warn("OT2 is paused")
                except KeyError:
                    self.get_logger().error("The OT2 robot failed to pause. REQUIRES MANUAL INTERRUPT.")
                    ## TODO Log reported OT2 Error
                    ## TODO Update heartbeat state and info for timer-based publisher

        elif not msg.is_emergency and self.emergency_flag:
            self.emergency_flag = False
            self.get_logger().info("Emergency alert(s) cleared.")
          
            if self._heartbeat_state == Heartbeat.EMERGENCY:
                self._heartbeat_state = Heartbeat.BUSY
                self.get_logger().info("EMERGENCY ---> BUSY")
                self.get_logger().info("Resuming OT2 run...")
                resume_response = self.ot2.execute(self.run_id)

                ## Verifying resumption
                try: 
                    if resume_response["data"]["actionType"] == "play":
                        self.get_logger().info("OT2 resumed run")
                except KeyError:
                    self.get_logger().error("The OT2 robot failed to resume. Resume operation reported an error. REQUIRES MANUAL INTERRUPT.")
                    ## TODO Log reported OT2 Error
               

    def heartbeat_timer_callback(self):
        """
        Update the heartbeat's header timestamp, heartbeat state,
        heartbeat info and publish
        """
        self.heartbeat_msg.state = self._heartbeat_state
        self.heartbeat_msg.message = self._heartbeat_info
        self.heartbeat_msg.header.stamp = self.get_clock().now().to_msg()
        self.heartbeat_publisher.publish(self.heartbeat_msg)


    def action_goal_callback(self, requested_goal):
        """
        Log new request; 
        Check for required components of the request
        Accept or reject goal request by returing GoalResponse.ACCEPT GoalResponse.REJECT
        """

        self.get_logger().info("Recieved new goal request. Checking if request is valid...")

        ## Validating recvd goal request
        ## If not valid, Reject Goal and propagate error signal upwards

        valid, self.validation = self.validate_goal_request(requested_goal.job)
        
        if valid:

            ## Resetting heartbeat state and info incase previous goal request was errored
            self._heartbeat_state = Heartbeat.ERROR
            self._heartbeat_info = ""

            self.get_logger().info("Request is valid.")
            path = '/root/config/temp'
            if not os.path.exists(path):
                os.mkdir(path)
            self.get_logger().info("Writing protocol config to {} ...".format(self.pc_document_path))
            with open(self.pc_document_path,'w',encoding = "utf-8") as pc_file:
                pc_file.write(self.validation["protocol_config"])   
            self.get_logger().info("Writing complete.".format(self.pc_document_path))

            return GoalResponse.ACCEPT
        
        self.get_logger().error(self.validation)
        self._heartbeat_state = Heartbeat.ERROR
        self._heartbeat_info = self.validation
        
        return GoalResponse.REJECT

    def goal_accepted_callback(self, goal_handle):
        """
        Compiles protocol from yaml to py
        Constructs base of feedback message
        Launches execution timer for timer-based async feedback
        The particular execution timer depends on whether running simulation or not  
        NOTE: Protocol transfer and execution 
        """

        self._goal_handle = goal_handle

        self.ot2 = OT2_Driver(OT2_Config(ip=self.validation["robot_ip"]))
        self.protocol_file_path, self.resource_file_path = self.ot2.compile_protocol(self.pc_document_path)
        self.get_logger().info('New protocol compiled')
        
        self.feedback_msg = OT2Job.Feedback()
        self.feedback_msg.progress.header.src = self.get_fully_qualified_name()
        
        if self.validation["simulate"] or (os.getenv('simulate', 'false') and os.getenv('simulate').lower() == 'true'):
            self.get_logger().warn("Running protocol in SIMULATION mode...")
            self.i = 0
            self.simulate_timer = self.create_timer(1, self.simulate_timer_callback)
        else:       
            self.execute_protocol_timer = self.create_timer(1, self.execute_protocol_timer_callback)

    def execute_protocol_timer_callback(self):
        """
        Check for and execute goal cancellation request
        Transfers and triggers execution of compiled instruction set
        Currently only publishes feedback messages when heartbeat info changes
        Updates to Heartbeat state, and feedback messages
        If IDLE + onEmergency ==> Cannot execute during emergency, stay IDLE
        If IDLE + onClearedEmergecny ==> execute, go BUSY if successful, 
                                                                else terminate

        When running
         - if running --> report "running"
         - if finishing --> report "finishing"
         - if completed --> report "success", IDLE, cancel timer

        TODO Update feedback to include time_elapsed and other entries per 
        JobProgress.msg definition
        TODO Completion progress can/should be defered to the zmq back and 
        forth between protocol and ros system  
        """

        if self._goal_handle.is_cancel_requested:
            self.get_logger().info("Recieved request to cancel goal")
            self._goal_handle.canceled()
            self.get_logger().info("Goal cancelled")
            self.execute_protocol_timer.cancel()
            return OT2Job.Result()

        ## Starting the protocol transfer and execution only if ActionServer 
        ## is primed to run (ie IDLE) and no emergency reported
        ## Effectively waits for emergencies to clear, then runs 
        
        if self._heartbeat_state == Heartbeat.IDLE:
            
            if self.emergency_flag:
                self._heartbeat_info = "[ERROR] Cannot execute protocols during an Emergency"
            else:
                self.protocol_id, self.run_id = self.ot2.transfer(self.protocol_file_path)
                execute_resp = self.ot2.execute(self.run_id)

                try:
                    ## Response to a successfully initiated run
                    if execute_resp["data"]["actionType"] == "play": 

                        self._heartbeat_info = "Run ({}) created at {}".format(execute_resp["data"]["id"], execute_resp["data"]["createdAt"])
                        self._heartbeat_state = Heartbeat.BUSY
                        self.get_logger().info("IDLE ---> BUSY") 
                        self.get_logger().info(self._heartbeat_info)
                
                except KeyError:
                    pass
                    
                try: 
                    ## Response to error in run initiation
                    self._heartbeat_info = "[OT2 Error] {}: {}".format(execute_resp["errors"][0]["id"], execute_resp["errors"][0]["detail"])
                    self._heartbeat_state = Heartbeat.ERROR
                    self.get_logger().error("IDLE ---> ERROR") 
                    self.get_logger().error(self._heartbeat_info)
                    self.execute_protocol_timer.cancel()
                    
                    ## TODO: verify how appropos below is
                    self._goal_handle.abort()
                
                except KeyError:
                    pass
        
        else:

            run_status = self.ot2.get_run(self.run_id)

            if run_status["data"]["status"] == "running":
                self._heartbeat_info = "OT2 Running"
                
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

                ## Toggle execute_callback to "original" then execute
                if self._exec_cb == "simulate":
                    self._action_server.register_execute_callback(self.action_execute_callback)
                    self._exec_cb = "original"
                self._goal_handle.execute()
            
        if self._heartbeat_info != self.feedback_msg.progress.progress_msg:

            self.feedback_msg.progress.header.stamp = self.get_clock().now().to_msg()
            self.feedback_msg.progress.progress_msg = self._heartbeat_info
            self._goal_handle.publish_feedback(self.feedback_msg)


    def simulate_timer_callback(self):
        """
        Checks for and executes goal cancellation request
        Counts up to 10 before triggering simulation of run
        Pauses on emergency
        Currently publishes feedback when 
        """

        response_message = ""

        if self.emergency_flag:
            response_message = "Simulation Paused"
                
        elif self.i <= 10:
            response_message = "Simulation Running"

            if self._goal_handle.is_cancel_requested:
                self.get_logger().info("Recieved request to cancel goal")
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

            ## Toggle execute_callback to "simulate" then execute
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
        """
        Effect the compiled protocol in simulation mode
        """
        
        resp, _ = simulate(open(self.protocol_file_path))

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
        return CancelResponse.ACCEPT


    def validate_goal_request(self, job):
        """
        Check for:
         - required robot_ip (TODO: possible regex check for ip address, if not validated upstream with workflow)
         - required protocol_config (yaml content as string; assumed yaml validate upstream with workflow)
        """

        protocol_config = None
        robot_ip = None
        
        ## Check that an ip address was sent
        ## TODO: Validate IP address signature by regex

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

    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
