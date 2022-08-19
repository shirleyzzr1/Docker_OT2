import rclpy
from rclpy.node import Node

from demo_interfaces.srv import RaiseEmergency
from demo_interfaces.msg import EmergencyAlert
from std_msgs.msg import String


class ErrorHandler(Node):
    def __init__(self):
        super().__init__('error_handler')

        self.raise_emergency_srv = self.create_service(RaiseEmergency,'/raise_emergency',lambda req,resp:self.emergency_callback(req,resp))
        self.clear_emergency_srv = self.create_service(RaiseEmergency,'/clear_emergency',lambda req,resp:self.emergency_callback(req,resp))
        self.emergency_msg = ""
        self.emergency_flag = False
        self.emergency_publisher = self.create_publisher(EmergencyAlert, '/emergency', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = EmergencyAlert()
        msg.message = self.emergency_msg
        msg.is_emergency = self.emergency_flag
        self.emergency_publisher.publish(msg)

    def emergency_callback(self,request,response):
        self.emergency_msg = request.alert.message
        self.emergency_flag = request.alert.is_emergency
        response.success= True
        response.error_msg = "None"
        return response



def main(args=None):
    rclpy.init(args=args)
    error_handler = ErrorHandler()
    rclpy.spin(error_handler)

if __name__ == '__main__':
    main()

