#!/usr/bin/env python3
import numpy as np
import math
import termios, tty, select, sys
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations 


class WaypointRecorder():
    def __init__(self):
        rospy.init_node('waypoint_recorder', anonymous=False)
        
        self._drone_name = rospy.get_param("~drone_name")
        self._pose_topic = rospy.get_param("~pose_topic")
        self._drone_config_dir = rospy.get_param("~drone_config_dir")
        self._num_samples = rospy.get_param("~num_samples", default=20)  # Number of pose samples to collect for averaging.
        
        self._instructions = \
            """
            ------------------ Waypoint Recorder ------------------
            
            1. Press SPACE to record a new waypoint.
            2. Press 'd' to change delay value for subsequent waypoints.
            3. Press BACKSPACE to remove last recorded waypoint.
            4. Press ENTER to save the recorded waypoints and quit.
            5. Press ESCAPE to quit without saving.
            """
        # Remove tabs/spaces from the instruction docstring.
        self._instructions = "\n".join([ line.strip() for line in self._instructions.splitlines() ])
        
        self._pose_raw = []  # List to temporarily store pose data before averaging.
        self._recorded_waypoints = []
        self._waypoint_delay = 0

        self._stdin_settings = termios.tcgetattr(sys.stdin)

        self.__collect_waypoints()

    def __collect_waypoints(self):
        print(self._instructions)
        self._waypoint_delay = float(input("Input waypoint delay in seconds: "))
        print("Ready to record waypoints!")
        
        try:
            while True:
                key = self.__get_keypress()

                if key == ' ':  # Record a new waypoint.
                    # Use the output of aruco_map_pose_tracker instead of drone's pose (output of Kalman filter) as the
                    # Kalman filter's yaw estimate is erroneous when the drone is not in flight. 
                    print("\nRecording waypoint...")
                    pose_sub = rospy.Subscriber(f"/{self._drone_name}/{self._pose_topic}", PoseStamped, self.__pose_callback)
                    while len(self._pose_raw) < self._num_samples:
                        pass
                    pose_sub.unregister()

                    # Average all pose samples and round to 3 decimal places.
                    pose_avg = np.mean(self._pose_raw, axis=0)
                    new_waypoint = list( np.round([*pose_avg, self._waypoint_delay], decimals=3) )
                    self._recorded_waypoints.append(new_waypoint)
                    self._pose_raw.clear()           
                    print(f"Waypoint recorded: {len(self._recorded_waypoints)}. {pose_avg}")

                elif key == 'd':  # Change delay value.
                    self._waypoint_delay = float(input("\nInput new waypoint delay in seconds: "))

                elif key == '\x7f':  # Backspace (delete); Remove last recorded waypoint.
                    if len(self._recorded_waypoints):
                        last_waypoint = self._recorded_waypoints.pop()
                        print(f"\nRemoved {last_waypoint}, {len(self._recorded_waypoints)} waypoint(s) left.")
                    else:
                        print("\nThere are no waypoints to remove.")

                elif key == '\x0d':  # Enter (carriage return); Save waypoints and quit.
                    if not len(self._recorded_waypoints):
                        print("\nThere are no waypoints to save, quit without saving.")
                        break
                    
                    print(f"\nSaving {len(self._recorded_waypoints)} waypoints to {self._drone_name}.yaml...")
                    self.__save_to_config_file()
                    print("Done!")
                    break  # Quit the while loop

                elif key == '\x1b':  # Escape; quit without saving.
                    print("\nQuit without saving.")
                    break

        except Exception as e:
            print(e)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)

    def __save_to_config_file(self):
        wp_config_string = f"waypoints_list  : {str(self._recorded_waypoints)}\n"
        with open(f"{self._drone_config_dir}/{self._drone_name}.yaml", 'r+') as file:
            config_file_content = file.readlines()
            wp_list_found = False

            for line_num, line in enumerate(config_file_content):
                # Use 2 conditions in case comments lines inside the file contain 'waypoints_list'.
                if "waypoints_list" in line and "[[" in line:
                    config_file_content[line_num] = wp_config_string
                    wp_list_found = True
                    break

            if not wp_list_found:
                config_file_content.append(wp_config_string)

            file.seek(0)
            file.truncate()
            file.writelines(config_file_content)
    
    def __pose_callback(self, data):
        x_raw = data.pose.position.x
        y_raw = data.pose.position.y
        z_raw = data.pose.position.z

        quat = data.pose.orientation
        rpy_raw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        yaw_raw = rpy_raw[2]*180/math.pi

        self._pose_raw.append([x_raw, y_raw, z_raw, yaw_raw])

    def __get_keypress(self, key_timeout = 0.05):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._stdin_settings)
        return key


if __name__ == "__main__" :
    waypoint_recorder = WaypointRecorder()
    