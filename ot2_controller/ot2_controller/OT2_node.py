import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .protocol_handler.protocol_handling_client import send_message_to_OT2
class OT2Client(Node):
	def __init__(self):
		super().__init__('OT2Client')
		self.publisher_ = self.create_publisher(String, 'OT2State',10)
		timer_perood = 0.5	#seconds
		send_message_to_OT2("send message to OT2","root","127.0.0.1","8085")
		self.timer = self.create_timer(timer_period, self.timer_callback)
	def timer_callback(self):
		msg = String()
		msg.data = 'Hello'
		self.publisher_.publish(msg)
		self.get_logger().info('Publishing: "%s"' % msg.data)
def main(args = None):
 	rclpy.init(args = args)
 	OT2_Client = OT2Client()
 	rclpy.spin(OT2_Client)

if __name__=='__main__':
    main()
