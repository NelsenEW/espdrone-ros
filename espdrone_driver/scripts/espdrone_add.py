#!/usr/bin/python3
import rospy
from espdrone_msgs.srv import AddEspdrone, AddEspdroneRequest
from espdrone_msgs.msg import LogBlock


def main():
    rospy.init_node("espdrone_add", anonymous=True)
    uri = rospy.get_param("~uri")
    tf_prefix = rospy.get_param("~tf_prefix")
    roll_trim = rospy.get_param("~roll_trim", 0.0)
    pitch_trim = rospy.get_param("~pitch_trim", 0.0)
    use_ros_time = rospy.get_param("~use_ros_time", True)
    enable_logging = rospy.get_param("~enable_logging", False)
    enable_parameters = rospy.get_param("~enable_parameters", False)
    enable_logging_imu = rospy.get_param("~enable_logging_imu", False)
    enable_logging_image = rospy.get_param("~enable_logging_image", False)
    enable_logging_battery = rospy.get_param("~enable_logging_battery", False)
    enable_logging_motor = rospy.get_param("~enable_logging_motor", False)
    enable_logging_zranger = rospy.get_param("~enable_logging_zranger", False)
    enable_logging_pose = rospy.get_param("~enable_logging_pose", False)
    enable_pose_tf_publisher = rospy.get_param("~enable_pose_tf_publisher", False)
    enable_logging_setpoint = rospy.get_param("~enable_logging_setpoint", False)
    enable_custom_log_file = rospy.get_param("~enable_custom_log_file", False)
    camera_info_file = rospy.get_param("~camera_info_file", "")
    params_file = rospy.get_param("~custom_params_file", "")

    rospy.loginfo("wait_for_service /add_espdrone")
    add_espdrone_service = rospy.ServiceProxy("/add_espdrone", AddEspdrone)
    add_espdrone_service.wait_for_service()
    rospy.loginfo("found /add_espdrone")
    add_espdrone_request = AddEspdroneRequest()
    add_espdrone_request.uri = uri
    add_espdrone_request.tf_prefix = tf_prefix
    add_espdrone_request.roll_trim = roll_trim
    add_espdrone_request.pitch_trim = pitch_trim
    add_espdrone_request.enable_logging = enable_logging
    add_espdrone_request.enable_parameters = enable_parameters
    add_espdrone_request.use_ros_time = use_ros_time
    add_espdrone_request.enable_logging_image = enable_logging_image
    add_espdrone_request.enable_logging_imu = enable_logging_imu
    add_espdrone_request.enable_logging_battery = enable_logging_battery
    add_espdrone_request.enable_logging_motor = enable_logging_motor
    add_espdrone_request.enable_logging_zranger = enable_logging_zranger
    add_espdrone_request.enable_logging_pose = enable_logging_pose
    add_espdrone_request.enable_pose_tf_publisher = enable_pose_tf_publisher
    add_espdrone_request.enable_logging_setpoint = enable_logging_setpoint
    add_espdrone_request.camera_info_file = camera_info_file
    add_espdrone_request.params_file = params_file
    generic_log_topics = rospy.get_param("~genericLogTopics", list())
    generic_log_topic_frequencies = rospy.get_param(
        "~genericLogTopicFrequencies", list()
    )

    if enable_custom_log_file:
        if len(generic_log_topics) == len(generic_log_topic_frequencies):
            for i, topic in enumerate(generic_log_topics):
                log_block = LogBlock()
                log_block.topic_name = topic
                log_block.frequency = generic_log_topic_frequencies[i]
                log_block.variables = rospy.get_param(
                    "~genericLogTopic_" + topic + "_Variables"
                )
                add_espdrone_request.log_blocks.append(log_block)
        else:
            rospy.logerr(
                "Cardinality of genericLogTopics and genericLogTopicFrequencies does not match!"
            )
    add_espdrone_service.call(add_espdrone_request)


if __name__ == "__main__":
    main()
