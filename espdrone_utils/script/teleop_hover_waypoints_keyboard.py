#!/usr/bin/env python3

import threading

import rospy
import math
from ast import literal_eval
from espdrone_msgs.msg import Hover
from espdrone_msgs.srv import Takeoff, TakeoffRequest
from espdrone_msgs.srv import Land, LandRequest
from espdrone_msgs.srv import GoTo, GoToRequest
from std_srvs.srv import SetBool, SetBoolRequest
from geometry_msgs.msg import PoseStamped
import sys, select, termios, tty
import numpy as np
from tf.transformations import euler_from_quaternion


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
m : run mission with waypoints
SPACE : pause/play mission
Emergency Bindings:
e : emergency
CTRL-C to quit and emergency
"""

moveBindings = {
    "u": (1, 0, 0, 0),
    "h": (0, 1, 0, 0),
    "j": (-1, 0, 0, 0),
    "k": (0, -1, 0, 0),
    "a": (0, 0, 0, -1),
    "s": (0, 0, -1, 0),
    "d": (0, 0, 0, 1),
    "w": (0, 0, 1, 0),
}

speedBindings = {
    "q": (1.1, 1),
    "z": (0.9, 1),
    "o": (1, 1.1),
    "p": (1, 0.9),
}
emergencyBindings = ["e", "\x03"]


class WaypointThread(threading.Thread):
    def __init__(
        self, drone_name: str, transition_duration: float, goal_threshold: float
    ):
        super(WaypointThread, self).__init__()
        self._drone_name = drone_name
        self._transition_duration = transition_duration
        self._goal_threshold = goal_threshold
        self._current_pos = [0] * 3
        self._orientation = [0] * 4
        self._waypoints_list = []
        self._takeoff_height = rospy.get_param("~takeoff_height", 0.7)
        self.condition = threading.Condition()
        self.done = False
        self.play = threading.Event()
        self.takeoff = threading.Event()

        rospy.Subscriber(f"/{self._drone_name}/pose", PoseStamped, self.__pose_callback)
        self._goto_service = rospy.ServiceProxy(f"/{self._drone_name}/go_to", GoTo)
        self._takeoff_service = rospy.ServiceProxy(f"/{drone_name}/takeoff", Takeoff)
        self._land_service = rospy.ServiceProxy(f"/{drone_name}/land", Land)
        self._takeoff_service.wait_for_service()
        self._land_service.wait_for_service()
        self._goto_service.wait_for_service()
        self.start()

    def update(self, waypoints_list):
        self.condition.acquire()
        if waypoints_list:
            rospy.loginfo(f"[{self._drone_name}] Waypoints to follow: {waypoints_list}")
        self._waypoints_list = waypoints_list
        self.play.set()
        # Notify service thread that we have a new waypoints.
        self.condition.notify()
        self.condition.release()

    def toggle_play(self):
        if self._waypoints_list:
            self.play.clear() if self.play.is_set() else self.play.set()

    def stop(self):
        self.done = True
        self.update(list())
        self.play.clear()
        self.join(0)

    def get_height(self):
        return self._current_pos[2]

    def sleep(self, duration):
        start_time = rospy.get_time()
        while (
            rospy.get_time() - start_time < duration
            and not self.done
            and self.play.is_set()
        ):
            pass

    def run(self):
        goto_request = GoToRequest()
        goto_request.duration = rospy.Duration(self._transition_duration)
        goto_request.relative = False
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(None)
            if self.done:
                break
            self.condition.release()
            if not self.takeoff.is_set():
                takeoff_request = TakeoffRequest()
                takeoff_request.height = 0.3
                takeoff_request.duration = rospy.Duration(3)
                self._takeoff_service.call(takeoff_request)
                self.takeoff.set()
                self.sleep(3)
            if self.done:
                break
            while self._waypoints_list:
                if not self.done and self.play.is_set():
                    current_waypoint = self._waypoints_list[0]
                    goto_request.goal.x = current_waypoint[0]
                    goto_request.goal.y = current_waypoint[1]
                    goto_request.goal.z = current_waypoint[2]
                    goto_request.yaw = current_waypoint[3]
                    self._goto_service.call(goto_request)
                    rospy.loginfo(
                        f"[{self._drone_name}] go_to requested to {current_waypoint}"
                    )
                    start_time = rospy.get_time()
                    # Wait until goal is reached.
                    yaw = math.degrees(euler_from_quaternion(self._orientation)[-1])
                    yaw_different = abs(yaw - current_waypoint[3])
                    while (
                        (
                            (
                                np.linalg.norm(
                                    np.array(self._current_pos)
                                    - np.array(current_waypoint[:3])
                                )
                                > self._goal_threshold
                            )
                            or (yaw_different > 5)
                        )
                        and rospy.get_time() - start_time < self._transition_duration
                        and not self.done
                        and self.play.is_set()
                    ):
                        pass
                    if not self.done and self.play.is_set():
                        waypoint_delay = current_waypoint[-1]
                        rospy.loginfo(
                            f"[{self._drone_name}] Waypoint reached, delaying for {waypoint_delay} seconds"
                        )
                        self.sleep(waypoint_delay)
                        if self.done:
                            break
                        self._waypoints_list.pop(0)
                elif self.done:
                    break
            if not self.done and self.takeoff.is_set():
                rospy.loginfo(f"[{self._drone_name}]: Finish waypoint list")
                land_request = LandRequest()
                land_request.height = 0.10
                land_request.duration = rospy.Duration(5)
                self._land_service.call(land_request)
                self.play.clear()
                self.takeoff.clear()
                self.sleep(5)

    def __pose_callback(self, data: PoseStamped):
        self._current_pos[0] = data.pose.position.x
        self._current_pos[1] = data.pose.position.y
        self._current_pos[2] = data.pose.position.z
        self._orientation[0] = data.pose.orientation.x
        self._orientation[1] = data.pose.orientation.y
        self._orientation[2] = data.pose.orientation.z
        self._orientation[3] = data.pose.orientation.w


class PublishThread(threading.Thread):
    def __init__(self, drone_name, rate):
        super(PublishThread, self).__init__()
        self.drone_name = drone_name
        self.publisher = rospy.Publisher(
            f"/{self.drone_name}/cmd_hover", Hover, queue_size=1
        )
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
                print(
                    "Waiting for subscriber to connect to {}".format(
                        self.publisher.name
                    )
                )
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
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def timer_completion(flag: threading.Event):
    flag.clear()


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("teleop_twist_keyboard")

    speed = rospy.get_param("~speed", 0.1)  # speed in m/s
    turn = rospy.get_param("~turn", 5.0)  # turn in degree/s
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.05)
    drone_name = rospy.get_param("~drone_name", "")
    takeoff_height = rospy.get_param("~takeoff_height", 0.7)
    emergency_service = rospy.ServiceProxy(f"/{drone_name}/emergency", SetBool)
    takeoff_service = rospy.ServiceProxy(f"/{drone_name}/takeoff", Takeoff)
    land_service = rospy.ServiceProxy(f"/{drone_name}/land", Land)
    # waypoints
    waypoints_list: list = rospy.get_param("~waypoints_list", default=[])
    waypoints_list = literal_eval(waypoints_list)
    transition_duration = rospy.get_param("~transition_duration", default=4)
    goal_threshold = rospy.get_param("~goal_threshold", default=0.05)

    waypoint_thread = WaypointThread(drone_name, transition_duration, goal_threshold)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(drone_name, repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        emergency = False
        takeoff = waypoint_thread.takeoff
        blocking = threading.Event()
        print(msg)
        while 1:
            key = getKey(key_timeout)
            if key in emergencyBindings:
                if key == "e":
                    emergency = not emergency
                    emergency_request = SetBoolRequest()
                    emergency_request.data = emergency
                    emergency_service.call(emergency_request)
                    if emergency:
                        takeoff.clear()
                        waypoint_thread.play.clear()
                elif key == "\x03":  # Ctrl + C
                    if takeoff.is_set() and not emergency:
                        emergency_request = SetBoolRequest()
                        emergency_request.data = emergency
                        emergency_service.call(emergency_request)
                        takeoff.clear()
                    break
            elif not blocking.is_set():
                if (key == "t") and not (takeoff.is_set() or emergency):
                    takeoff_request = TakeoffRequest()
                    takeoff_request.height = takeoff_height
                    takeoff_request.duration = rospy.Duration(3)
                    takeoff_service.call(takeoff_request)
                    pub_thread.set_height(takeoff_height)
                    takeoff.set()
                    threading.Timer(
                        3, args=(blocking,), function=timer_completion
                    ).start()
                    blocking.set()
                elif key == "m":
                    waypoint_thread.update(waypoints_list.copy())
                elif takeoff.is_set():
                    if key == "l":
                        waypoint_thread.update(list())
                        land_request = LandRequest()
                        land_request.height = 0.15
                        land_request.duration = rospy.Duration(5)
                        land_service.call(land_request)
                        pub_thread.set_height(waypoint_thread.get_height())
                        takeoff.clear()
                        threading.Timer(
                            5, args=(blocking,), function=timer_completion
                        ).start()
                        blocking.set()
                    elif key == " ":
                        waypoint_thread.toggle_play()
                    elif not waypoint_thread.play.is_set():
                        if key in moveBindings.keys():
                            x = moveBindings[key][0]
                            y = moveBindings[key][1]
                            z = moveBindings[key][2]
                            th = moveBindings[key][3]
                        elif key in speedBindings.keys():
                            speed = speed * speedBindings[key][0]
                            turn = turn * speedBindings[key][1]

                            print(vels(speed, turn))
                            if status == 14:
                                print(msg)
                            status = (status + 1) % 15
                        else:
                            x = 0
                            y = 0
                            z = 0
                            th = 0
                        pub_thread.update(x, y, z, th, speed, turn)
                    else:
                        if (
                            key in moveBindings.keys() or key in speedBindings.keys()
                        ):  # Pause the mission
                            waypoint_thread.toggle_play()
                        pub_thread.set_height(waypoint_thread.get_height())
                # rospy.loginfo_throttle(
                #     1, f"{blocking.is_set()}, {takeoff.is_set()}, {emergency}"
                # )

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        waypoint_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
