#!/usr/bin/env python

import rospy
from drone_operation.srv import Arm, Takeoff, Land, Disarm, CenterObject, ArmResponse, TakeoffResponse, LandResponse, DisarmResponse, CenterObjectResponse
from pymavlink import mavutil
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse

import sys
sys.path.append('/home/yeonarom/catkin_ws/src/drone_operation/src')

from object_detection import ObjectDetector
from object_tracking import ObjectTracker
from object_reidentification import ObjectReidentifier
from visualize import Visualization

class DroneController:
    def __init__(self, connection_str='udp:127.0.0.1:14550'):
        self.connection = mavutil.mavlink_connection(connection_str)
        self.wait_for_connection()
        #self.set_mode('OFFBOARD')

    def wait_for_connection(self):
        print('Waiting for the drone to connect...')
        self.connection.wait_heartbeat()
        print('Drone is connected!')

    def set_mode(self, mode):
        if mode not in self.connection.mode_mapping():
            print(f"Unknown mode: {mode}")
            return

        mode_id = self.connection.mode_mapping()[mode][1]
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            0, mode_id, 0, 0, 0, 0, 0
        )

        while True:
            ack_msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True)
            if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                print(f"Mode changed to {mode}")
                break

    def set_position(self, x, y, z, vx=0, vy=0, vz=0, yaw=0, yaw_rate=0):
        self.connection.mav.set_position_target_local_ned_send(
            0,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000),
            x, y, z,
            vx, vy, vz,
            0, 0, 0,
            yaw, yaw_rate
        )

    def get_current_altitude(self):
        while True:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_alt = msg.relative_alt / 1000.0
                return current_alt
            
    def get_position(self):
        while True:
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            if msg:
                return (msg.x, msg.y, msg.z)

    def arm(self):
        print('Waiting for the drone to arm...')
        self.connection.arducopter_arm()
        self.connection.motors_armed_wait()
        print("Armed!")

    def takeoff(self):
        current_altitude = self.get_current_altitude()
        if current_altitude is not None:
            print(f"Current altitude: {current_altitude} meters")
            target_altitude = max(current_altitude + 10, 10)
        else:
            target_altitude = 10

        print(f"Taking off to {target_altitude} meters")
        target_position = (0, 0, -target_altitude)

        while True:
            self.set_position(target_position[0], target_position[1], target_position[2])
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            if msg and abs(msg.z + target_altitude) < 0.1:
                print("Reached target altitude")
                break
            time.sleep(1)

    def land(self):
        print("Landing")
        current_altitude = self.get_current_altitude()
        if current_altitude is None:
            print("Unable to get current altitude.")
            return

        target_altitude = 0.1

        while current_altitude > target_altitude:
            self.set_position(0, 0, -current_altitude)  # Move to the current lower altitude
            msg = self.connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            if msg:
                current_altitude = -msg.z
                print(f"Current altitude: {current_altitude:.2f} meters")
            time.sleep(1)

        print("Landed")


    def disarm(self):
        print('Waiting for the drone to disarm...')
        self.connection.arducopter_disarm()
        self.connection.motors_disarmed_wait()
        print("Disarmed!")

    def close(self):
        self.connection.close()

class MainNode:
    def __init__(self):
        rospy.init_node('drone_main_node', anonymous=True)

        self.bridge = CvBridge()
        
        self.detector = ObjectDetector()
        self.tracker = ObjectTracker()
        self.reidentifier = ObjectReidentifier()
        self.visualizer = Visualization()
        self.drone_controller = DroneController()
        
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        self.detection_enabled = True
        self.detection_service = rospy.Service('/set_detection', SetBool, self.handle_set_detection)

        self.arm_service = rospy.Service('/arm', Arm, self.handle_arm)
        self.takeoff_service = rospy.Service('/takeoff', Takeoff, self.handle_takeoff)
        self.land_service = rospy.Service('/land', Land, self.handle_land)
        self.disarm_service = rospy.Service('/disarm', Disarm, self.handle_disarm)
        self.center_object_service = rospy.Service('/center_object', CenterObject, self.handle_center_object)


    def handle_set_detection(self, req):
        self.detection_enabled = req.data
        if self.detection_enabled:
            rospy.loginfo("Object detection enabled")
        else:
            rospy.loginfo("Object detection disabled")
        return SetBoolResponse(success=True, message="Object detection set to "+ str(self.detection_enabled))
        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        if self.detection_enabled:
            # Perform object detection
            detected_objects = self.detector(cv_image)

            if detected_objects is not None:
                # Perform object tracking
                tracked_objects = self.tracker(detected_objects)
                
                # Perform object reidentification
                reidentified_objects = self.reidentifier(cv_image, tracked_objects)
                
                # Visualize the results
                self.visualizer(cv_image, tracked_objects, reidentified_objects)

                self.tracked_objects = tracked_objects
                self.reidentified_objects = reidentified_objects

    def handle_arm(self, req):
        try:
            self.drone_controller.arm()
            return ArmResponse(success=True, message="Drone armed successfully.")
        except Exception as e:
            return ArmResponse(success=False, message=str(e))

    def handle_takeoff(self, req):
        try:
            self.drone_controller.takeoff()
            return TakeoffResponse(success=True, message="Takeoff successful.")
        except Exception as e:
            return TakeoffResponse(success=False, message=str(e))

    def handle_land(self, req):
        try:
            self.drone_controller.land()
            return LandResponse(success=True, message="Landing successful.")
        except Exception as e:
            return LandResponse(success=False, message=str(e))

    def handle_disarm(self, req):
        try:
            self.drone_controller.disarm()
            return DisarmResponse(success=True, message="Drone disarmed successfully.")
        except Exception as e:
            return DisarmResponse(success=False, message=str(e))
        
    def handle_center_object(self, req):
        if self.tracked_objects is None:
            return CenterObjectResponse(False)
        
        object_idx = self.reidentified_objects.index(min(list(map(int, self.reidentified_objects))))
        lowest_id_object = self.tracked_objects[object_idx]

        image_width = 640
        image_height = 480
        center_x = image_width / 2
        center_y = image_height / 2

        error_x = (lowest_id_object[2] - lowest_id_object[0]) / 2 - center_x
        error_y = (lowest_id_object[3] - lowest_id_object[1]) / 2 - center_y

        kp = 0.1
        adjust_x = -kp * error_x
        adjust_y = -kp * error_y

        current_altitude = self.drone_controller.get_current_altitude()
        target_position = (adjust_x, adjust_y, current_altitude)

        threshold = 0.05
        while True:
            self.drone_controller.set_position(adjust_x, adjust_y, -current_altitude)
            current_position = self.drone_controller.get_position()

            # 현재 위치와 목표 위치 간의 오차 계산
            position_error_x = current_position[0] - target_position[0]
            position_error_y = current_position[1] - target_position[1]

            # 목표 위치에 도달했는지 확인
            if abs(position_error_x) <= threshold and abs(position_error_y) <= threshold:
                print("Reached target position")
                break
            rospy.sleep(0.1)

        return CenterObjectResponse(True)
        
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    service_node = MainNode()
    service_node.run()
