#!/usr/bin/env python3

import threading

import rospy

from espdrone_msgs.msg import Hover
from espdrone_msgs.srv import Takeoff, TakeoffRequest
from espdrone_msgs.srv import Land, LandRequest
from std_srvs.srv import SetBool, SetBoolRequest
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
        w           u
    a   s   d   h   j    k
Description:
w : up (+z)
s : down (-z)
a : left yaw
d : right yaw
u : forward
j : backward
h : left
k : right

Speed Bindings:
q/z : increase/decrease only linear speed by 10% (i, j, k, l)
o/p : increase/decrease only angular speed by 10%

Special Bindings:
t : takeoff
l : land
e : emergency
CTRL-C to quit and emergency
"""

moveBindings = {
        'u':(1,0,0,0),
        'h':(0,1,0,0),
        'j':(-1,0,0,0),
        'k':(0,-1,0,0),
        'a':(0,0,0,-1),
        's':(0,0,-1,0),
        'd':(0,0,0,1),
        'w':(0,0,1,0),
    }

speedBindings={
        'q':(1.1, 1),
        'z':(.9, 1),
        'o':(1,1.1),
        'p':(1,.9),
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_hover', Hover, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.height = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None
        self.start()

    def set_height(self, height):
        self.height = height

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.set_height(0)
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        hover = Hover()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            if self.done:
                break
            # Copy state into Hover message.
            hover.vx = self.x * self.speed
            hover.vy = self.y * self.speed
            hover.yawrate = self.th * self.turn
            self.height += self.z * 0.05
            hover.zDistance = self.height

            self.condition.release()
            # Publish.
            self.publisher.publish(hover)
            hover = Hover()

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.1)  # speed in m/s
    turn = rospy.get_param("~turn", 5.0)  # turn in degree/s
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.05)
    drone_name = rospy.get_param("~drone_name", "")
    takeoff_height = rospy.get_param("~takeoff_height", 0.2)
    emergency_service = rospy.ServiceProxy(f"/{drone_name}/emergency", SetBool)
    takeoff_service = rospy.ServiceProxy(f"/{drone_name}/takeoff", Takeoff)
    land_service = rospy.ServiceProxy(f"/{drone_name}/land", Land)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        takeoff = False
        emergency = False
        print(msg)
        while(1):
            key = getKey(key_timeout)
            if (key == 't') and not (takeoff or emergency):
                takeoff_request= TakeoffRequest()
                takeoff_request.height = takeoff_height
                takeoff_request.duration = rospy.Duration(3)
                takeoff_service.call(takeoff_request)
                pub_thread.set_height(takeoff_height)
                rospy.sleep(3)
                takeoff = True
            if (key == 'e'):
                emergency = not emergency
                emergency_request = SetBoolRequest()
                emergency_request.data = emergency
                emergency_service.call(emergency_request)
                if emergency:
                    takeoff = False
            if (key == '\x03'): # Ctrl + C
                if takeoff and not emergency:
                    emergency_request = SetBoolRequest()
                    emergency_request.data = emergency
                    emergency_service.call(emergency_request)
                break
            if takeoff:
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                    z = moveBindings[key][2]
                    th = moveBindings[key][3]
                elif key in speedBindings.keys():
                    speed = speed * speedBindings[key][0]
                    turn = turn * speedBindings[key][1]

                    print(vels(speed,turn))
                    if (status == 14):
                        print(msg)
                    status = (status + 1) % 15
                else:
                    x = 0
                    y = 0
                    z = 0
                    th = 0
                pub_thread.update(x, y, z, th, speed, turn)    
                if (key == 'l'):
                    land_request= LandRequest()
                    land_request.height = 0.2
                    land_request.duration = rospy.Duration(5)
                    land_service.call(land_request)
                    pub_thread.set_height(0)
                    rospy.sleep(5)
                    takeoff = False
                    
                

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)