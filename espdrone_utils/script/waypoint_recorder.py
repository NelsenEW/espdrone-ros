#!/usr/bin/env python3
import numpy as np
import math
import termios, tty, select, sys
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations 


class WaypointRecorder():
    def __init__(self, num_samples, drone_name, pose_topic, drone_config_dir):
        # rospy.init_node('waypoint_recorder', anonymous=False)
        
        self._drone_name = drone_name
        self._pose_topic = pose_topic
        self._drone_config_dir = drone_config_dir
        self._num_samples = num_samples
        
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
        self.recorded_waypoints = []
        self.waypoint_delay = 0

        self._stdin_settings = termios.tcgetattr(sys.stdin)

        print(self._instructions)
        self.waypoint_delay = float(input("Input waypoint delay in seconds: "))
        print("Ready to record waypoints!")

        # self.collect_waypoints()

    def collect_waypoints(self):
        
        # if key == ' ':  # Record a new waypoint.
        # # Use the output of aruco_map_pose_tracker instead of drone's pose (output of Kalman filter) as the
        # # Kalman filter's yaw estimate is erroneous when the drone is not in flight. 
        print("\nRecording waypoint...")
        pose_sub = rospy.Subscriber(self._pose_topic, PoseStamped, self.__pose_callback)
        while len(self._pose_raw) < self._num_samples:
            pass
        pose_sub.unregister()

        # Average all pose samples and round to 3 decimal places.
        pose_avg = np.mean(self._pose_raw, axis=0)
        new_waypoint = list( np.round([*pose_avg, self.waypoint_delay], decimals=3) )
        self.recorded_waypoints.append(new_waypoint)
        self._pose_raw.clear()           
        print(f"Waypoint recorded: {len(self.recorded_waypoints)}. {pose_avg}")


    def delete_waypoint(self):       
        if len(self.recorded_waypoints):
            last_waypoint = self.recorded_waypoints.pop()
            print(f"\nRemoved {last_waypoint}, {len(self.recorded_waypoints)} waypoint(s) left.")
        else:
            print("\nThere are no waypoints to remove.")

    def save_waypoint(self):
        if not len(self.recorded_waypoints):
            print("\nThere are no waypoint to save, quit without saving")
        
        print(f"\nSaving {len(self.recorded_waypoints)} waypoints to {self._drone_name}.yaml...")
        self.__save_to_config_file()
        print("Done!")

    def __save_to_config_file(self):
        wp_config_string = f"waypoints_list  : {str(self.recorded_waypoints)}\n"
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
        yaw_raw = rpy_raw[2]*180/math.pi + 90

        self._pose_raw.append([x_raw, y_raw, z_raw, yaw_raw])

if __name__ == "__main__" :
    waypoint_recorder = WaypointRecorder()
    