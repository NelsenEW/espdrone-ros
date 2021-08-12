#!/usr/bin/env python3

import cv2
import numpy as np
# from urllib2 import urlopen
from urllib.request import urlopen
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml


CAMERA_BUFFER_SIZE=4096
bts=b''

rospy.init_node('camera_streamer', anonymous=True)
drone_name = rospy.get_namespace().split('/')[-2]
drone_ip_addr = rospy.get_param('~drone_ip_addr', default="192.168.43.42")

camera_img_pub = rospy.Publisher("camera_stream", Image, queue_size=1)

try:
    drone_camera_info_file = rospy.get_param('~drone_camera_info_file')
    camera_info_is_available = True
    camera_info_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=1)
except KeyError:
    rospy.logwarn(f"[{drone_name}] Camera calibration file not specified in 'drone_camera_info_file' param, will not publish camera_info")
    camera_info_is_available = False

stream = urlopen(f"http://{drone_ip_addr}/stream.jpg")
bridge = CvBridge()

# Load camera calibration info if it is available
if camera_info_is_available:
    with open(drone_camera_info_file, 'r') as camera_info:
        drone_camera_info = yaml.safe_load(camera_info)
    camera_info_msg = CameraInfo(height            = drone_camera_info['image_height'],
                                width              = drone_camera_info['image_width'],
                                distortion_model   = drone_camera_info['distortion_model'],
                                D                  = drone_camera_info['distortion_coefficients']['data'],
                                K                  = drone_camera_info['camera_matrix']['data'],
                                R                  = drone_camera_info['rectification_matrix']['data'],
                                P                  = drone_camera_info['projection_matrix']['data'])
    camera_info_msg.header.frame_id = drone_name 

rospy.loginfo(f"[{drone_name}] Starting camera stream")
while (not rospy.is_shutdown()):    
    try:
        bts+=stream.read(CAMERA_BUFFER_SIZE)
        jpghead=bts.find(b'\xff\xd8')
        jpgend=bts.find(b'\xff\xd9')
        if jpghead>-1 and jpgend>-1:
            jpg=bts[jpghead:jpgend+2]
            bts=bts[jpgend+2:]
            img=cv2.imdecode(np.frombuffer(jpg,dtype=np.uint8),cv2.IMREAD_UNCHANGED)
            
            curr_time = rospy.get_rostime()
            camera_img = bridge.cv2_to_imgmsg(img, 'bgr8')
            # camera_img.header.stamp = curr_time
            camera_img.header.frame_id = drone_name
            camera_img_pub.publish(camera_img)

            if camera_info_is_available:
                # camera_info_msg.header.stamp = curr_time  # CameraInfo timestamp must match raw_image's timestamp for image_proc to work
                camera_info_pub.publish(camera_info_msg)
    
    except Exception as e:
        rospy.logerr(f"[{drone_name}] " + str(e))
        bts=b''
        stream = urlopen(f"http://{drone_ip_addr}/stream.jpg")
        continue