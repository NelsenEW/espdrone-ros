#!/usr/bin/python3
import numpy as np
import sys
import tty
import termios
from ast import literal_eval
from threading import Thread, Event
import rospy
from geometry_msgs.msg import PoseStamped
from espdrone_msgs.srv import GoTo, GoToRequest
from espdrone_msgs.srv import Takeoff, TakeoffRequest
from std_srvs.srv import SetBool, SetBoolRequest


class WaypointSender:
    def __init__(self, drone_name, transition_duration, goal_threshold):
        self._drone_name = drone_name
        self._transition_duration = transition_duration
        self._goal_threshold = goal_threshold
        self._current_pos = [0]*3

        # Thread for requesting emergency.
        self._emg_request_thread = Thread(target=self.__check_emg_request, daemon=True)
        self._emg_request_event = Event()

        rospy.Subscriber(f"/{self._drone_name}/pose", PoseStamped, self.__pose_callback)
        self._goto_service = rospy.ServiceProxy(f"/{self._drone_name}/go_to", GoTo)
        self._takeoff_service = rospy.ServiceProxy(f"/{self._drone_name}/takeoff", Takeoff)
        self._emergency_service = rospy.ServiceProxy(f"/{self._drone_name}/emergency", SetBool)
        
        rospy.loginfo(f"[{self._drone_name}] Waiting for services...")
        self._goto_service.wait_for_service()
        self._takeoff_service.wait_for_service()
        self._emergency_service.wait_for_service()
        rospy.loginfo(f"[{self._drone_name}] All services ready!")

        self._emg_request_thread.start()

    
    def send_waypoints(self, waypoints_list):
        rospy.loginfo(f"[{self._drone_name}] Waypoints to follow: {waypoints_list}")

        takeoff_request= TakeoffRequest()
        takeoff_request.height = 0.3
        takeoff_request.duration = rospy.Duration(3)
        self._takeoff_service.call(takeoff_request)
        rospy.sleep(3)

        goto_request = GoToRequest()
        goto_request.duration = rospy.Duration(self._transition_duration)
        goto_request.relative = False
        
        while waypoints_list and not self._emg_request_event.is_set():
            current_waypoint = waypoints_list[0]
            goto_request.goal.x = current_waypoint[0]
            goto_request.goal.y = current_waypoint[1]
            goto_request.goal.z = current_waypoint[2]
            self._goto_service.call(goto_request)
            rospy.loginfo(f"[{self._drone_name}] go_to requested to {current_waypoint}")

            # Wait until goal is reached.
            while np.linalg.norm(np.array(self._current_pos)-np.array(current_waypoint[:3])) > self._goal_threshold:
                if self._emg_request_event.is_set():
                    break
            
            waypoint_delay = current_waypoint[3]
            rospy.loginfo(f"[{self._drone_name}] Waypoint reached, delaying for {waypoint_delay} seconds")
            rospy.sleep(waypoint_delay)
            waypoints_list.pop(0)
        
        # TODO: send land request!


    def __pose_callback(self, data: PoseStamped):
        self._current_pos[0] = data.pose.position.x
        self._current_pos[1] = data.pose.position.y
        self._current_pos[2] = data.pose.position.z

    def __check_emg_request(self):
        emergency_request = SetBoolRequest(True)
        self.__wait_for_key('e')
        self._emergency_service.call(emergency_request)
        self._emg_request_event.set()
        rospy.logerr(f"[{self._drone_name}] Emergency requested!")
    
    def __wait_for_key(self, key):
        filedescriptors = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin)
        while True:
            if sys.stdin.read(1) == key:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN,filedescriptors)
                return


def main():
    rospy.init_node('waypoint_sender')

    drone_name = rospy.get_param('~drone_name', default="espdrone")
    waypoints_list = rospy.get_param('~waypoints_list', default=[])
    waypoints_list = literal_eval(waypoints_list)
    transition_duration = rospy.get_param('~transition_duration', default=4)
    goal_threshold = rospy.get_param('~goal_threshold', default=0.05)

    waypoint_sender = WaypointSender(drone_name, transition_duration, goal_threshold)
    waypoint_sender.send_waypoints(waypoints_list)


if __name__ == "__main__":
    main()