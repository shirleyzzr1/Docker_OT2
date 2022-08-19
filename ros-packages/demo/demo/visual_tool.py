import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from demo_interfaces.srv import ExecuteJob

from demo_interfaces.msg import EmergencyAlert
from demo_interfaces.msg import Heartbeat
from std_msgs.msg import String

import yaml
import enum
import os
import curses
from curses.textpad import Textbox, rectangle
from curses import wrapper

# Using enum class create enumerations
class States(enum.Enum):
   IDLE = 0
   BUSY = 1
   FINISHED = 2
   ERROR = 3
   EMERGENCY =4

class VisualTool(Node):
    def __init__(self):
        super().__init__('visual_tool')
        self.workcell_data = None
        self.machines = self.parse_machines()
        self.machine_states = self.create_states()
        self.steps = self.workcell_data['actions']

        self.create_subs()
        self.emergency_flag = False
        self.emergency = self.create_subscription(EmergencyAlert,'/emergency',self.emergency_callback,10)


    def parse_machines(self):
        user_path = "/root/example_ws.yml"
        self.workcell_data = yaml.safe_load(open(user_path))
        machines= self.workcell_data['modules']
        names = []
        for m in machines:
            names.append([m['name'],m['type']])
        return names

    def create_states(self):
        dicts = {}
        for machine,type in self.machines:
            dicts[machine]="ERROR"
        return dicts

    def common_callback(self,msg):
        # self.get_logger().info('I heard: "%s"'%msg.data)
        machine = msg.header.src
        state = States(msg.state).name
        self.machine_states[machine]=state
        # self.get_logger().info(machine+" is now "+state)
        info = msg.message
        if self.machine_states[machine]=="ERROR" and state=="IDLE":
            # self.get_logger().info(machine+" is now IDLE!")
            self.machine_states[machine]="IDLE"
        elif self.machine_states[machine]=="ERROR":
            # self.get_logger().info(machine + info)
            self.machine_states[machine]="ERROR"

    def create_subs(self):
        for name,type in self.machines:
            setattr(self,"sub"+name, self.create_subscription(Heartbeat,"/{}/action_client/heartbeat".format(name),lambda msg:self.common_callback(msg),10))
   
    def emergency_callback(self,msg):
        if msg.is_emergency==True:
            self.emergency_flag=True
        else:
            self.emergency_flag = False
            # self.get_logger().info("client_manager received an emergency alert: " + msg.message)

class DrawCurses():
    def __init__(self,machines) -> None:
        self.stdscr = curses.initscr()
        self.curses_init()
        self.machines = machines
        self.wins = []
        self.draw_init()


    def curses_init(self):
        curses.noecho()
        curses.cbreak()
        curses.start_color()
        curses.curs_set(0)
        self.stdscr.keypad(True)
        self.stdscr.nodelay(True)
        self.stdscr.clear()

        curses.init_pair(1,curses.COLOR_GREEN,curses.COLOR_BLACK)
        curses.init_pair(2,curses.COLOR_RED,curses.COLOR_BLACK)
        self.GREEN_AND_BLACK = curses.color_pair(1)
        self.RED_AND_BLACK = curses.color_pair(2)

    def draw_init(self):
        for i in range(len(self.machines)):
            rectangle(self.stdscr,2,10+20*i,10,25+20*i)
            self.stdscr.addstr(2,15+20*i,self.machines[i][0])
            type = "type: {}".format(self.machines[i][1])
            self.stdscr.addstr(4,11+20*i,type)
            self.stdscr.addstr(5,11+20*i,"State:ERROR",self.RED_AND_BLACK)
            wins = curses.newwin(2,14,5,11+20*i)
            self.wins.append(wins)


    def update_state(self,states,emergency_flag):
        for i in range(len(self.wins)):
            state = "State: {} ".format(states[self.machines[i][0]])
            if states[self.machines[i][0]]=="ERROR":
                self.wins[i].addstr(0,0,state,self.RED_AND_BLACK)
            else:
                self.wins[i].addstr(0,0,state,self.GREEN_AND_BLACK)
            if emergency_flag:
                self.wins[i].addstr(1,0,"EMERGENCY!",curses.A_BLINK | self.RED_AND_BLACK)
            else:
                self.wins[i].addstr(1,0,"          ",curses.A_BLINK | self.RED_AND_BLACK)
            self.wins[i].refresh() 

def main():
    rclpy.init()
    vis = VisualTool()

    drawcurses = DrawCurses(vis.machines)

    x,y = 0,0
    k = 0
    while True:
        try:
            
            k = drawcurses.stdscr.getch()
        except:
            k = None

        if k ==curses.KEY_LEFT:
            x -=1
        elif k==curses.KEY_RIGHT:
            x+=1
        elif k==curses.KEY_UP:
            y-=1
        elif k==curses.KEY_DOWN:
            y+=1
        elif k==ord('q'):
            break

        drawcurses.update_state(vis.machine_states,vis.emergency_flag)
        rclpy.spin_once(vis,timeout_sec=0)

    rclpy.shutdown()
    curses.endwin()


if __name__ == '__main__':
    wrapper(main())
