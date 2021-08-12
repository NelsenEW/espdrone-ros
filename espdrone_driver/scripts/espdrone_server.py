#!/usr/bin/python3

# Edlib
from os import link
from threading import Thread
import yaml
from edlib.espdrone.localization import Localization
from edlib.espdrone.log import LogConfig
from edlib.espdrone.mem import MemoryElement, TrajectoryMemory
from edlib.espdrone import Espdrone
import edlib.crtp

# Utility
from dataclasses import dataclass
from typing import Dict, List
import math


# ROS
import rospy
import tf2_ros
import tf_conversions

# Espdrone srvs
from espdrone_msgs.srv import AddEspdrone, AddEspdroneRequest, AddEspdroneResponse
from espdrone_msgs.srv import (
    RemoveEspdrone,
    RemoveEspdroneRequest,
    RemoveEspdroneResponse,
)
from espdrone_msgs.srv import GoTo, GoToRequest, GoToResponse
from espdrone_msgs.srv import Takeoff, TakeoffRequest, TakeoffResponse
from espdrone_msgs.srv import Land, LandRequest, LandResponse
from espdrone_msgs.srv import SetGroupMask, SetGroupMaskRequest, SetGroupMaskResponse
from espdrone_msgs.srv import (
    StartTrajectory,
    StartTrajectoryRequest,
    StartTrajectoryResponse,
)
from espdrone_msgs.srv import (
    UploadTrajectory,
    UploadTrajectoryRequest,
    UploadTrajectoryResponse,
)
from espdrone_msgs.srv import Stop, StopRequest, StopResponse
from espdrone_msgs.srv import (
    UpdateParams,
    UpdateParamsRequest,
    UploadTrajectoryResponse,
)
from espdrone_msgs.srv import Motors, MotorsRequest, MotorsResponse

# Espdrone msgs
from espdrone_msgs.msg import LogBlock
from espdrone_msgs.msg import GenericLogData
from espdrone_msgs.msg import FullState
from espdrone_msgs.msg import Hover
from espdrone_msgs.msg import Position
from espdrone_msgs.msg import VelocityWorld
from espdrone_msgs.msg import BatteryStatus
from espdrone_msgs.msg import MotorStatus
from espdrone_msgs.msg import TofMeasurement

# Common Msgs & Srvs
from std_msgs.msg import Empty
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

class EspdroneROS:
    @dataclass
    class LogImuData:
        acc_x: float
        acc_y: float
        acc_z: float
        gyro_x: float
        gyro_y: float
        gyro_z: float
        #zrange: int

    @dataclass
    class LogPowerData:
        vbat: float
        state: int
        m1: int
        m2: int
        m3: int
        m4: int

    @dataclass
    class LogPoseData:
        x: float
        y: float
        z: float
        quat_compressed: int

    @dataclass
    class LogSetpointData:
        x: float
        y: float
        z: float
        yaw: float

    def __init__(
        self,
        link_uri: str,
        tf_prefix: str,
        roll_trim: float,
        pitch_trim: float,
        enable_logging: bool,
        enable_parameters: bool,
        log_blocks: List[LogBlock],
        use_ros_time: bool,
        enable_logging_image: bool,
        enable_logging_imu: bool,
        enable_logging_battery: bool,
        enable_logging_motor: bool,
        enable_logging_zranger: bool,
        enable_logging_pose: bool,
        enable_pose_tf_publisher: bool,
        enable_logging_setpoint: bool,
        camera_info_file: str
    ):
        self._tf_prefix = tf_prefix
        self._is_emergency = False
        self._is_flying = False
        self._is_connected = False
        self._roll_trim = roll_trim
        self._pitch_trim = pitch_trim
        self._enable_logging = enable_logging
        self._enable_parameters = enable_parameters
        self._log_blocks = log_blocks
        self._use_ros_time = use_ros_time
        self._enable_logging_image = enable_logging_image
        self._enable_logging_imu = enable_logging_imu
        self._enable_logging_battery = enable_logging_battery
        self._enable_logging_motor = enable_logging_motor
        self._enable_logging_zranger = enable_logging_zranger
        self._enable_logging_pose = enable_logging_pose
        self._enable_pose_tf_publisher = enable_pose_tf_publisher
        self._enable_logging_setpoint = enable_logging_setpoint
        self._camera_info_file = camera_info_file
        self._global_frame = "world"
        self._camera_info = None

        # Drone name should not be empty, the default is the last two digit of the ip address
        if not self._tf_prefix:
            self._tf_prefix = f"espdrone_{link_uri[-2:]}"
        
        # Setup espdrone
        start = rospy.Time.now()
        self._ed = Espdrone(name=self._tf_prefix, rw_cache="./cache")

        # Connect to drone
        self._ed.connected.add_callback(self._connect)
        self._ed.console.receivedChar.add_callback(self.on_console)
        self._ed.connection_failed.add_callback(self._connection_failed)
        self._ed.connection_lost.add_callback(self._connection_lost)
        self._ed.disconnected.add_callback(self._disconnect)

        self.__static_message_buffer = str()
        self._ed.open_link(link_uri)

        while not self._is_connected and not rospy.is_shutdown():
            rospy.loginfo_throttle(
                1, f"[{self._tf_prefix}] Espdrone is not connected, waiting..."
            )
        self._ext_pos_handler = self._ed.extpos
        self._commander = self._ed.commander
        self._high_level_commander = self._ed.high_level_commander
        self._localization_handler = self._ed.loc
        self._log_handler = self._ed.log
        self._param_handler = self._ed.param
        self._memory_handler = self._ed.mem
        self._camera = self._ed.camera

        self._global_frame = rospy.get_param("~global_frame", "map")

        self._load_camera_info()
        self._initialize_subscribers()
        self._initialize_publishers()
        self._initialize_services()
        if self._enable_parameters:
            self._initialize_param()
        if self._enable_logging:
            self._initialize_log()

        rospy.loginfo(f"[{self._tf_prefix}]: Ready to start!")
        rospy.loginfo(f"[{self._tf_prefix}]: Elapsed: {(rospy.Time.now() - start).to_sec()}")

    def _initialize_subscribers(self):
        self._sub_cmd_vel = rospy.Subscriber(
            self._tf_prefix + "/cmd_vel",
            Twist,
            queue_size=1,
            callback=self.cmd_vel_changed,
        )
        
        self._sub_cmd_full_state = rospy.Subscriber(
            self._tf_prefix + "/cmd_full_state",
            FullState,
            queue_size=1,
            callback=self.cmd_full_state_setpoint,
        )
        self._sub_cmd_velocity_world = rospy.Subscriber(
            self._tf_prefix + "/cmd_velocity_world",
            VelocityWorld,
            queue_size=1,
            callback=self.cmd_velocity_world_setpoint,
        )
        self._sub_external_position = rospy.Subscriber(
            self._tf_prefix + "/external_position",
            PointStamped,
            queue_size=1,
            callback=self.position_measurement_changed,
        )
        self._sub_external_pose = rospy.Subscriber(
            self._tf_prefix + "/external_pose",
            PoseStamped,
            queue_size=1,
            callback=self.pose_measurement_changed,
        )
        self._sub_cmd_hover = rospy.Subscriber(
            self._tf_prefix + "/cmd_hover",
            Hover,
            queue_size=1,
            callback=self.cmd_hover_setpoint,
        )
        self._sub_cmd_stop = rospy.Subscriber(
            self._tf_prefix + "/cmd_stop",
            Empty,
            queue_size=1,
            callback=self.cmd_stop,
        )
        self._sub_cmd_position = rospy.Subscriber(
            self._tf_prefix + "/cmd_position",
            Position,
            queue_size=1,
            callback=self.cmd_position_setpoint,
        )

    def _initialize_publishers(self):
        if self._enable_logging_image:
            self._pub_image = rospy.Publisher(
                self._tf_prefix + "/camera_stream", Image, queue_size=1
            )
            self._pub_camera_info = rospy.Publisher(
                self._tf_prefix + "/camera_info", CameraInfo, queue_size=1
            )

        if self._enable_logging_imu:
            self._pub_imu = rospy.Publisher(
                self._tf_prefix + "/imu", Imu, queue_size=1
            )

        if self._enable_logging_battery:
            self._pub_battery = rospy.Publisher(
                self._tf_prefix + "/battery", BatteryStatus, queue_size=1
            )
        if self._enable_logging_motor:
            self._pub_motor = rospy.Publisher(
                self._tf_prefix + "/motor", MotorStatus, queue_size=1
            )
        if self._enable_logging_zranger:
            self._pub_zranger = rospy.Publisher(
                self._tf_prefix + "/zrange", TofMeasurement, queue_size=1
            )
        if self._enable_logging_pose:
            self._pub_pose = rospy.Publisher(
                self._tf_prefix + "/pose", PoseStamped, queue_size=1
            )

        if self._enable_logging_setpoint:
            self._pub_setpoint = rospy.Publisher(
                self._tf_prefix + "/setpoint", PoseStamped, queue_size=1
            )

        self._pub_log_data_generic: Dict[str, rospy.Publisher] = dict()
        for i in range(len(self._log_blocks)):
            self._pub_log_data_generic["Generic_" + str(i)] = rospy.Publisher(
                self._tf_prefix + "/" + self._log_blocks[i].topic_name,
                GenericLogData,
                queue_size=1,
            )
    
    def _load_camera_info(self):
        if not self._camera_info_file:
            rospy.logwarn(f"[{self._tf_prefix}]: Camera info file is not provided, will not\
                            publish camera info")
        else:
            with open(self._camera_info_file, 'r') as camera_info_stream:
                camera_info = yaml.safe_load(camera_info_stream)
            self._camera_info = CameraInfo( height = camera_info['image_height'],
                                            width = camera_info['image_width'],
                                            distortion_model = camera_info['distortion_model'],
                                            D = camera_info['distortion_coefficients']['data'],
                                            K = camera_info['camera_matrix']['data'],
                                            R = camera_info['rectification_matrix']['data'],
                                            P = camera_info['projection_matrix']['data']
                                        )
            self._camera_info.header.frame_id = self._tf_prefix
    
    def _initialize_services(self):
        # High-level setpoints
        self._service_set_group_mask = rospy.Service(
            self._tf_prefix + "/set_group_mask", SetGroupMask, self.set_group_mask
        )
        self._service_takeoff = rospy.Service(
            self._tf_prefix + "/takeoff", Takeoff, self.takeoff
        )
        self._service_land = rospy.Service(self._tf_prefix + "/land", Land, self.land)
        self._service_is_flying = rospy.Service(
            self._tf_prefix + "/is_flying", Trigger, self.is_flying
        )
        self._service_is_emergency = rospy.Service(
            self._tf_prefix + "/is_emergency", Trigger, self.is_emergency
        )
        self._service_stop = rospy.Service(self._tf_prefix + "/stop", Stop, self.stop)
        self._service_go_to = rospy.Service(
            self._tf_prefix + "/go_to", GoTo, self.go_to
        )
        self._service_upload_trajectory = rospy.Service(
            self._tf_prefix + "/upload_trajectory",
            UploadTrajectory,
            self.upload_trajectory,
        )
        self._service_start_trajectory = rospy.Service(
            self._tf_prefix + "/start_trajectory",
            StartTrajectory,
            self.start_trajectory,
        )
        self._service_emergency = rospy.Service(
            self._tf_prefix + "/emergency", SetBool, self.emergency
        )
        self._service_motor_set = rospy.Service(
            self._tf_prefix + "/motor_set", Motors, self.motor_set
        )

    def _initialize_param(self):
        # Wait until all the parameters have been updated
        while not self._param_handler.is_updated:
            rospy.loginfo("Waiting for param update")
            rospy.sleep(1)
        rospy.loginfo(f"[{self._tf_prefix}] Updating parameters...")
        for group in self._param_handler.values.keys():
            for name in self._param_handler.values[group].keys():
                param_name = "/" + self._tf_prefix + "/" + group + "/" + name
                rospy.set_param(param_name, self._param_handler.values[group][name])
        self._service_update_params = rospy.Service(
            self._tf_prefix + "/update_params",
            UpdateParams,
            self.update_params,
        )

    def _initialize_log(self):
        log_block_generic: List[LogConfig] = list()

        if self._enable_logging_image:
            self._camera.image_received_cb.add_callback(self.on_camera_data)
            self._camera.start()
                 
        if self._enable_logging_imu:
            log_block_imu = LogConfig("IMU", 20)
            log_block_imu.add_variable("acc.x")
            log_block_imu.add_variable("acc.y")
            log_block_imu.add_variable("acc.z")
            log_block_imu.add_variable("gyro.x")
            log_block_imu.add_variable("gyro.y")
            log_block_imu.add_variable("gyro.z")
            self._log_handler.add_config(log_block_imu)
            log_block_imu.data_received_cb.add_callback(self.on_imu_data)
            log_block_imu.start()

        if (
            self._enable_logging_battery
            or self._enable_logging_motor
            or self._enable_logging_zranger
        ):
            log_block_power = LogConfig("Power", 500)
            log_block_power.add_variable("pm.vbat")
            log_block_power.add_variable("pm.state")
            log_block_power.add_variable("motor.m1")
            log_block_power.add_variable("motor.m2")
            log_block_power.add_variable("motor.m3")
            log_block_power.add_variable("motor.m4")
            # log_block_power.add_variable("range.zrange")
            self._log_handler.add_config(log_block_power)
            log_block_power.data_received_cb.add_callback(
                self.on_power_data
            )
            log_block_power.start()

        if self._enable_logging_pose or self._enable_logging_battery:
            log_block_pose = LogConfig("Pose", 10)
            log_block_pose.add_variable("stateEstimate.x")
            log_block_pose.add_variable("stateEstimate.y")
            log_block_pose.add_variable("stateEstimate.z")
            log_block_pose.add_variable("stateEstimateZ.quat")
            self._log_handler.add_config(log_block_pose)
            log_block_pose.data_received_cb.add_callback(self.on_pose_data)
            log_block_pose.start()

        if self._enable_logging_setpoint:
            log_block_setpoint = LogConfig("Setpoint", 10)
            log_block_setpoint.add_variable("ctrltarget.x")
            log_block_setpoint.add_variable("ctrltarget.y")
            log_block_setpoint.add_variable("ctrltarget.z")
            log_block_setpoint.add_variable("ctrltarget.yaw")
            self._log_handler.add_config(log_block_setpoint)
            log_block_setpoint.data_received_cb.add_callback(
                self.on_setpoint_data
            )
            log_block_setpoint.start()

        for i in range(len(self._log_blocks)):
            log_block_generic.append(
                LogConfig(
                    "Generic_" + str(i), self._log_blocks[i].frequency
                )
            )
            for variable in self._log_blocks[i].variables:
                log_block_generic[i].add_variable(variable)
            self._log_handler.add_config(log_block_generic[i])
            log_block_generic[i].data_received_cb.add_callback(
                self.on_log_custom
            )
            log_block_generic[i].start()

    def _connect(self, link_uri):
        rospy.loginfo(f"[{self._tf_prefix}]: Connected to {link_uri}")
        self._is_connected = True

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Espdrone
        at the specified address)"""
        rospy.logerr(f"[{self._tf_prefix}]: Connection to {link_uri} failed: {msg}")
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Espdrone moves out of range)"""
        rospy.logerr(f"[{self._tf_prefix}]: Connection to {link_uri} lost: {msg}")
        self.is_connected = False


    def _disconnect(self, link_uri):
        if self._is_connected:
            rospy.loginfo(f"[{self._tf_prefix}]: Disconnected from {link_uri}")
            self._is_connected = False
            self._ed.close_link()
            self.__unregister_subscribers()
            self.__unregister_publishers()
            self.__unregister_services()
            if self._enable_parameters:
                self.__unregister_parameters()

    def __unregister_subscribers(self):
        self._sub_cmd_full_state.unregister()
        self._sub_cmd_hover.unregister()
        self._sub_cmd_position.unregister()
        self._sub_cmd_stop.unregister()
        self._sub_cmd_vel.unregister()
        self._sub_cmd_velocity_world.unregister()
        self._sub_external_pose.unregister()
        self._sub_external_position.unregister()

    def __unregister_publishers(self):
        if self._enable_logging_image:
            self._pub_image.unregister()
        if self._enable_logging_battery:
            self._pub_battery.unregister()
        if self._enable_logging_motor:
            self._pub_motor.unregister()
        if self._enable_logging_imu:
            self._pub_imu.unregister()
        if self._enable_logging_zranger:
            self._pub_zranger.unregister()
        if self._enable_logging_pose:
            self._pub_imu.unregister()
        if self._enable_logging_setpoint:
            self._pub_setpoint.unregister()
        if self._pub_log_data_generic:
            for key in self._pub_log_data_generic.keys():
                self._pub_log_data_generic[key].unregister()
            self._pub_log_data_generic = {}
        
    def __unregister_services(self):
        self._service_set_group_mask.shutdown()
        self._service_takeoff.shutdown()
        self._service_land.shutdown()
        self._service_is_flying.shutdown()
        self._service_is_emergency.shutdown()
        self._service_stop.shutdown()
        self._service_go_to.shutdown()
        self._service_upload_trajectory.shutdown()
        self._service_start_trajectory.shutdown()
        self._service_emergency.shutdown()
        self._service_motor_set.shutdown()
        if self._enable_parameters:
            self._service_update_params.shutdown()

    def __unregister_parameters(self):
        rospy.delete_param("/" + self._tf_prefix)

    def disconnect(self):
        return self._disconnect(self._ed.link_uri)

    def emergency(self, req: SetBoolRequest):
        if not self._is_emergency and req.data:
            rospy.logerr(f"[{self._tf_prefix}]: Emergency requested!")
            self._localization_handler.send_emergency_stop()
            rospy.set_param("/" + self._tf_prefix + "/stabilizer/stop", True)
            self._is_emergency = True
            self._is_flying = False
        elif self._is_emergency and not req.data:
            # send zero setpoint for thrust-lock and in case
            # the previous zero setpoint after emergency failed
            for _ in range(50):
                self._commander.send_setpoint(0, 0, 0, 0)
            # Reset Espdrone stop
            self._localization_handler.send_emergency_reset()
            rospy.set_param("/" + self._tf_prefix + "/stabilizer/stop", False)
            rospy.logerr(f"[{self._tf_prefix}]: Emergency reset")
            self._is_emergency = False
        return SetBoolResponse(success = self._is_emergency)

    def update_param(self, group: str, name: str):
        value = rospy.get_param("/" + self._tf_prefix + "/" + group + "/" + name)
        self._param_handler.set_value(group + "." + name, value)

    def update_params(self, req: UpdateParamsRequest):
        rospy.loginfo(f"[{self._tf_prefix}]: Update parameters")
        p: str
        for p in req.params:
            group, name = p.split("/")

            if not self._param_handler.toc.get_element(group, name):
                rospy.logerr(
                    f"[{self._tf_prefix}]: Could not find param {group}/{name}"
                )
                return True

            self.update_param(group, name)
        return True

    def cmd_hover_setpoint(self, msg: Hover):
        vx = msg.vx
        vy = msg.vy
        yawrate = msg.yawrate
        zDistance = msg.zDistance
        self._commander.send_hover_setpoint(vx, vy, yawrate, zDistance)

    def cmd_stop(self, msg: Empty):
        self._commander.send_stop_setpoint()

    def cmd_position_setpoint(self, msg: Position):
        x = msg.x
        y = msg.y
        z = msg.z
        yaw = msg.yaw
        self._commander.send_position_setpoint(x, y, z, yaw)

    def cmd_vel_changed(self, msg: Twist):
        roll = int(msg.linear.y + self._roll_trim)
        pitch = int(msg.linear.x + self._pitch_trim)
        yawrate = int(msg.angular.z)
        thrust = int(min(max(msg.linear.z, 0.0), 60000))
        self._commander.send_setpoint(roll, pitch, yawrate, thrust)

    def cmd_full_state_setpoint(self, msg: FullState):
        self._commander.send_full_state_setpoint(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.acc.x,
            msg.acc.y,
            msg.acc.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
        )

    def cmd_velocity_world_setpoint(self, msg: VelocityWorld):
        self._commander.send_velocity_world_setpoint(
            msg.vel.x, msg.vel.y, msg.vel.z, msg.yawRate
        )

    def position_measurement_changed(self, msg: PointStamped):
        self._ext_pos_handler.send_extpos(msg.point.x, msg.point.y, msg.point.z)

    def pose_measurement_changed(self, msg: PoseStamped):
        self._ext_pos_handler.send_extpose(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

    def on_camera_data(self, image, fps):
        bridge = CvBridge()
        msg = bridge.cv2_to_imgmsg(image, 'bgr8')
        msg.header.frame_id = self._tf_prefix + "/base_link"
        self._pub_image.publish(msg)
        
        if self._camera_info:
            # CameraInfo timestamp must match image's timestamp
            # self._camera_info.header.stamp = rospy.Time.now()
            self._pub_camera_info.publish(self._camera_info)

    def on_imu_data(self, timestamp_ms: int, data: dict(), *_):
        if self._enable_logging_imu:
            imu_data = EspdroneROS.LogImuData(*data.values())
            msg = Imu()
            msg.header.stamp = (
                rospy.Time.now()
                if self._use_ros_time
                else rospy.Time(timestamp_ms / 1000.0)
            )
            msg.header.frame_id = self._tf_prefix + "/base_link"
            msg.orientation_covariance[0] = -1

            # measured in mG; need to convert ot m/s^2
            msg.linear_acceleration.x = imu_data.acc_x * 9.81
            msg.linear_acceleration.y = imu_data.acc_y * 9.81
            msg.linear_acceleration.z = imu_data.acc_z * 9.81

            # measured in deg/s; need to convert to rad/s
            msg.angular_velocity.x = math.radians(imu_data.acc_x)
            msg.angular_velocity.y = math.radians(imu_data.acc_y)
            msg.angular_velocity.z = math.radians(imu_data.acc_z)

            self._pub_imu.publish(msg)

    def on_power_data(self, timestamp_ms: int, data: Dict[str, int], *_):
        power_data = EspdroneROS.LogPowerData(*data.values())
        if self._enable_logging_battery:
            msg = BatteryStatus()
            msg.header.stamp = (
                rospy.Time.now()
                if self._use_ros_time
                else rospy.Time(timestamp_ms / 1000.0)
            )
            msg.header.frame_id = self._tf_prefix + "/base_link"
            msg.voltage = power_data.vbat
            msg.state = power_data.state

            self._pub_battery.publish(msg)
        if self._enable_logging_motor:
            msg = MotorStatus()
            msg.header.stamp = (
                rospy.Time.now()
                if self._use_ros_time
                else rospy.Time(timestamp_ms / 1000.0)
            )
            msg.header.frame_id = self._tf_prefix + "/base_link"
            msg.m1 = power_data.m1
            msg.m2 = power_data.m2
            msg.m3 = power_data.m3
            msg.m4 = power_data.m4
            self._pub_motor.publish(msg)

        if self._enable_logging_zranger:
            msg = TofMeasurement()
            msg.header.stamp = (
                rospy.Time.now()
                if self._use_ros_time
                else rospy.Time(timestamp_ms / 1000.0)
            )
            msg.header.frame_id = self._tf_prefix + "/base_link"
            msg.zrange = power_data.zrange

            self._pub_zranger.publish(msg)

    def on_pose_data(self, timestamp_ms: int, data: Dict[str, float], *_):
        if self._enable_logging_pose:
            pose_data = EspdroneROS.LogPoseData(*data.values())
            msg = PoseStamped()
            msg.header.stamp = (
                rospy.Time.now()
                if self._use_ros_time
                else rospy.Time(timestamp_ms / 1000.0)
            )
            msg.header.frame_id = self._global_frame

            msg.pose.position.x = pose_data.x
            msg.pose.position.y = pose_data.y
            msg.pose.position.z = pose_data.z

            q = Localization.quatdecompress(pose_data.quat_compressed)
            msg.pose.orientation.x = q[0]
            msg.pose.orientation.y = q[1]
            msg.pose.orientation.z = q[2]
            msg.pose.orientation.w = q[3]

            self._pub_pose.publish(msg)

            if self._enable_pose_tf_publisher:
                br = tf2_ros.StaticTransformBroadcaster()
                pose_transform = TransformStamped()
                pose_transform.header.stamp = (
                    rospy.Time.now()
                    if self._use_ros_time
                    else rospy.Time(timestamp_ms / 1000.0)
                )
                pose_transform.header.frame_id = self._global_frame
                pose_transform.child_frame_id = self._tf_prefix + "/base_link"
                pose_transform.transform.translation.x = msg.pose.position.x
                pose_transform.transform.translation.y = msg.pose.position.y
                pose_transform.transform.translation.z = msg.pose.position.z
                pose_transform.transform.rotation.x = msg.pose.orientation.x
                pose_transform.transform.rotation.y = msg.pose.orientation.y
                pose_transform.transform.rotation.z = msg.pose.orientation.z
                pose_transform.transform.rotation.w = msg.pose.orientation.w

                br.sendTransform(pose_transform)

    def on_setpoint_data(self, timestamp_ms: int, data: Dict[str, float], *_):
        if self._enable_logging_setpoint:
            setpoint_data = EspdroneROS.LogSetpointData(*data.values())
            msg = PoseStamped()
            msg.header.stamp = (
                rospy.Time.now()
                if self._use_ros_time
                else rospy.Time(timestamp_ms / 1000.0)
            )
            msg.header.frame_id = self._global_frame

            msg.pose.position.x = setpoint_data.x
            msg.pose.position.y = setpoint_data.y
            msg.pose.position.z = setpoint_data.z
            q = Quaternion(
                *tf_conversions.transformations.quaternion_from_euler(
                    0, 0, setpoint_data.yaw
                )
            )

            msg.pose.orientation.x = q.x
            msg.pose.orientation.y = q.y
            msg.pose.orientation.z = q.z
            msg.pose.orientation.w = q.w

            self._pub_setpoint.publish(msg)

    def on_log_custom(
        self, timestamp_ms: int, data: Dict[str, float], log_config: LogConfig
    ):
        msg = GenericLogData()
        msg.header.stamp = (
            rospy.Time.now()
            if self._use_ros_time
            else rospy.Time(timestamp_ms / 1000.0)
        )
        msg.header.frame_id = self._tf_prefix + "/base_link"
        msg.values.extend(data.values())
        self._pub_log_data_generic[log_config.name].publish(msg)

    def on_console(self, msg: str):
        self.__static_message_buffer += msg
        # print(msg)
        if "\n" in self.__static_message_buffer:
            msgs = self.__static_message_buffer.splitlines()
            self.__static_message_buffer = msg.pop()
            for msg in msgs:
                rospy.loginfo(f"[{self._tf_prefix}]: ED Console: {msg}")

    def set_group_mask(self, req: SetGroupMaskRequest):
        rospy.loginfo(f"[{self._tf_prefix}]: SetGroupMask requested")
        self._high_level_commander.set_group_mask(req.groupMask)
        return SetGroupMaskResponse()

    def takeoff(self, req: TakeoffRequest):
        if not self._is_emergency or not self._is_flying:
            rospy.loginfo(f"[{self._tf_prefix}]: Takeoff requested")
            self._high_level_commander.takeoff(
                req.height, req.duration.to_sec(), req.groupMask
            )
            self._is_flying = True
        else:
            rospy.logerr(
                f"[{self._tf_prefix}]: Drone is already flying, unable to service takeoff request"
            )
        return TakeoffResponse()

    def land(self, req: LandRequest):
        if self._is_flying:
            rospy.loginfo(f"[{self._tf_prefix}]: Land requested")
            self._high_level_commander.land(
                req.height, req.duration.to_sec(), req.groupMask
            )
            self._is_flying = False
        else:
            rospy.logerr(
                f"[{self._tf_prefix}]: Drone is not flying, unable to service land request"
            )
        return LandResponse()

    def stop(self, req: StopRequest):
        if self._is_flying:
            rospy.loginfo(f"[{self._tf_prefix}]: Stop requested")
            self._high_level_commander.stop(req.groupMask)
        else:
            rospy.logerr(
                f"[{self._tf_prefix}]: Drone is not flying, unable to service stop request"
            )
        return Stop()

    def go_to(self, req: GoToRequest):
        # if self._is_flying:
        rospy.loginfo(f"[{self._tf_prefix}]: GoTo requested")
        self._high_level_commander.go_to(
            req.goal.x,
            req.goal.y,
            req.goal.z,
            req.yaw,
            req.duration.to_sec(),
            req.relative,
            req.groupMask,
        )
        # else:
        #     rospy.logerr(
        #         f"[{self._tf_prefix}]: Drone is not flying, unable to service go_to request"
        #     )
        return GoToResponse()

    def motor_set(self, req: MotorsRequest):
        if not self._is_flying:
            rospy.loginfo(f"[{self._tf_prefix}]: Motor setting requested")
            param_m1 = "/" + self._tf_prefix + "/motorPowerSet/m1"
            param_m2 = "/" + self._tf_prefix + "/motorPowerSet/m2"
            param_m3 = "/" + self._tf_prefix + "/motorPowerSet/m3"
            param_m4 = "/" + self._tf_prefix + "/motorPowerSet/m4"
            param_enable = "/" + self._tf_prefix + "/motorPowerSet/enable"

            rospy.set_param(param_m1, req.m1)
            rospy.set_param(param_m2, req.m2)
            rospy.set_param(param_m3, req.m3)
            rospy.set_param(param_m4, req.m4)
            rospy.set_param(param_enable, req.enable)

            self.update_param("motorPowerSet", "m1")
            self.update_param("motorPowerSet", "m2")
            self.update_param("motorPowerSet", "m3")
            self.update_param("motorPowerSet", "m4")
            self.update_param("motorPowerSet", "enable")

        else:
            rospy.logerr(
                f"[{self._tf_prefix}]: Drone is flying, unable to service motor_set request"
            )

        return MotorsResponse()

    # Warning not yet implemented in the ESPDRONE FIRMWARE -> SPIFFS SHOULD BE USED
    def upload_trajectory(self, req: UploadTrajectoryRequest):

        rospy.loginfo(f"[{self._tf_prefix}]: UploadTrajectory requested")
        mems = self._memory_handler.get_mems(MemoryElement.TYPE_TRAJ)

        if len(mems) != 1:
            raise RuntimeError("Invalid number of trajectory memories found")

        traj_mem: TrajectoryMemory = mems[0]

        traj_mem.poly4Ds = [0] * len(req.pieces)
        for i in range(len(traj_mem.poly4Ds)):
            if (
                (len(req.pieces[i].poly_x) != 8)
                or (len(req.pieces[i].poly_y) != 8)
                or (len(req.pieces[i].poly_y) != 8)
            ):
                rospy.logfatal(f"[{self._tf_prefix}]: Wrong number of pieces")
                return False
            traj_mem.poly4Ds[i].duration = req.pieces[i].duration.to_sec()
            for j in range(8):
                traj_mem.poly4Ds[i].x[j] = req.pieces[i].poly_x[j]
                traj_mem.poly4Ds[i].y[j] = req.pieces[i].poly_y[j]
                traj_mem.poly4Ds[i].z[j] = req.pieces[i].poly_z[j]
                traj_mem.poly4Ds[i].yaw[j] = req.pieces[i].poly_yaw[j]

        traj_mem.write_data(None)
        rospy.loginfo(f"[{self._tf_prefix}]: Upload completed")
        return UploadTrajectoryResponse()

    def start_trajectory(self, req: StartTrajectoryRequest):
        rospy.loginfo(f"[{self._tf_prefix}]: StartTrajectory requested")
        self._high_level_commander.start_trajectory(
            req.trajectoryId, req.timescale, req.relative, req.reversed, req.groupMask
        )
        return StartTrajectoryResponse()

    def is_flying(self, req: TriggerRequest):
        res = TriggerResponse()
        res.success = self._is_flying
        return res

    def is_emergency(self, req: TriggerRequest):
        res = TriggerResponse()
        res.success = self._is_emergency
        return res

class EspdroneServer:
    def __init__(self):
        self._espdrones: Dict[str, EspdroneROS] = {}

    def run(self):
        rospy.Service("add_espdrone", AddEspdrone, self.add_espdrone)
        rospy.Service("remove_espdrone", RemoveEspdrone, self.remove_espdrone)
        rospy.spin()

    def add_espdrone(self, req: AddEspdroneRequest):
        if req.uri in self._espdrones:
            rospy.logerr(f"Cannot add {req.uri}, already added")
            return AddEspdroneResponse(success = False)

        rospy.loginfo(
            """Adding drone with ip {} as {} with trim({}, {}). 
            Parameters: {}, Logging: {}, Pose tf Publisher: {}, Use ROS time: {}""".format(
                req.uri,
                req.tf_prefix,
                req.roll_trim,
                req.pitch_trim,
                req.enable_parameters,
                req.enable_logging,
                req.enable_pose_tf_publisher,
                req.use_ros_time,
            )
        )

        if req.enable_pose_tf_publisher and (
            not req.enable_logging or not req.enable_logging_pose
        ):
            rospy.logwarn(
                f"Pose tf publisher enabled for {req.tf_prefix} but pose logging is disabled."
                "No transform will be published."
            )

        ed = EspdroneROS(
            req.uri,
            req.tf_prefix,
            req.roll_trim,
            req.pitch_trim,
            req.enable_logging,
            req.enable_parameters,
            req.log_blocks,
            req.use_ros_time,
            req.enable_logging_image,
            req.enable_logging_imu,
            req.enable_logging_battery,
            req.enable_logging_motor,
            req.enable_logging_zranger,
            req.enable_logging_pose,
            req.enable_pose_tf_publisher,
            req.enable_logging_setpoint,
            req.camera_info_file
        )

        self._espdrones[req.uri] = ed
        return AddEspdroneResponse(success=False)

    def remove_espdrone(self, req: RemoveEspdroneRequest):
        if req.uri not in self._espdrones:
            rospy.logerr(f"Cannot remove ip {req.uri}, not connected")
            return RemoveEspdroneResponse(success=False)
        rospy.loginfo(f"Removing Espdrone with ip {req.uri}")
        self._espdrones[req.uri].disconnect()
        del self._espdrones[req.uri]

        rospy.loginfo(f"Espdrone ip {req.uri} removed")
        return RemoveEspdroneResponse(success=True)

def main():
    rospy.init_node("espdrone_server")
    espdrone_server = EspdroneServer()
    espdrone_server.run()


if __name__ == "__main__":
    edlib.crtp.init_drivers(enable_debug_driver=True)
    main()
