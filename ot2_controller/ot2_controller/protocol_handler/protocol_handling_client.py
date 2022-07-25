import zmq
import time
import sys

from .protocol_transfer import transfer
from .protocol_parser import * 

def handler(Protocol_path, user, ip, port): 
    #print("Protocol saved into " + path + "directory")
    #status = transfer(path, user, ip) # Gives it IP and username to send script over
    #if(status == 1): # error 
    #   return "", "", 1

    #print(protocol)
    msg_error, msg_output, msg_errorcode = send_message_to_OT2("python3 "+ "/data/" + Protocol_path.split("/")[-1], user, ip, port)
    
    return msg_output, msg_error, msg_errorcode

def send_message_to_OT2(message, user, ip, port):

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.connect("tcp://"+ip+":"+port) # TODO: instead of 10.193.254.91 put your IP  TODO: don't hard code the IP, make it a ROS parameter

    #print("Starting protocol handling client...")
    while True:
        sock.send_string(message)
        #print("Sent string: %s ..." % message)
        time.sleep(1)
        msg = sock.recv_string()
        msg = msg.split('@')
        msg_output, msg_error, msg_errorcode = msg[0], msg[1], msg[2]
        if msg_output != None:
            #print("Client recived the output message from the completed protocol. Sending message to the ROS Master")
            return msg_error, msg_output, int(msg_errorcode)
    sock.close()

def main_null():
    print("This is not meant to have a main function")

if __name__ == "__main__":
    filename = "../ot2_example.py"
    handler(filename,"root","127.0.0.1","8085")
