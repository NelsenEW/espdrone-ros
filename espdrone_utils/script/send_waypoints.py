#!/usr/bin/python3
import numpy
import rospy
from geometry_msgs.msg import PoseStamped
from espdrone_msgs.srv import GoTo, GoToRequest

def main():
    rospy.init_node('waypoint_sender')

    drone_name = rospy.get_namespace().split('/')[-2]
    waypoints_list = rospy.get_param('~waypoints_list', default=[])
    waypoints_list = list(waypoints_list)
    transition_duration = rospy.get_param('~transition_duration', default=4)
    goal_threshold = rospy.get_param('~goal_threshold', default=0.05)
    current_pos = [0]*3

    rospy.loginfo(f"[{drone_name}] Waypoints to follow: {waypoints_list}")

    rospy.Subscriber('pose', PoseStamped, pose_callback, callback_args=current_pos)
    goto_service = rospy.ServiceProxy('go_to', GoTo)

    goto_request = GoToRequest()
    goto_request.duration = transition_duration
    goto_request.relative = False

    while not rospy.is_shutdown() and waypoints_list:
        current_waypoint = waypoints_list[0]
        goto_request.goal.x = current_waypoint[0]
        goto_request.goal.y = current_waypoint[1]
        goto_request.goal.z = current_waypoint[2]
        goto_service.call(goto_request)
        rospy.loginfo(f"[{drone_name}] go_to requested to {current_waypoint}")

        # Wait until goal is reached.
        while numpy.linalg.norm(current_pos[:]-current_waypoint[:3]) > goal_threshold:
            pass
        
        waypoint_delay = current_waypoint[3]
        rospy.loginfo(f"[{drone_name}] Waypoint reached, delaying for {waypoint_delay} seconds")
        rospy.sleep(waypoint_delay)
        waypoints_list.pop(0)


def pose_callback(data: PoseStamped, drone_pos):
    drone_pos[0] = data.pose.position.x
    drone_pos[1] = data.pose.position.y
    drone_pos[2] = data.pose.position.z


if __name__ == "__main__":
    main()