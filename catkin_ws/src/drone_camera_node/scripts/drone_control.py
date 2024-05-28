#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
from pymavlink import mavutil
import threading

class DroneController:
    def __init__(self, connection_str='udp:127.0.0.1:14550'):
        self.connection = mavutil.mavlink_connection(connection_str)
        self.wait_for_connection()

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

    def get_current_altitude(self):
        while True:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_alt = msg.relative_alt / 1000.0
                return current_alt

    def arm_and_takeoff(self, target_altitude):
        print('Waiting for the drone to arm...')
        self.connection.arducopter_arm()
        self.connection.motors_armed_wait()
        print("Armed!")

        print(f"Taking off to {target_altitude} meters")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, target_altitude
        )
        
        while True:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg and msg.relative_alt / 1000 >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
        
    def land(self):
        print("Landing")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        while True:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg and msg.relative_alt / 1000 <= 0.1:
                print("Landed")
                break
            time.sleep(1)

    def disarm(self):
        print('Waiting for the drone to disarm...')
        self.connection.arducopter_disarm()
        self.connection.motors_disarmed_wait()
        print("Disarmed!")

    def close(self):
        self.connection.close()

class ImageSubscriber:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('image_subscriber', anonymous=True)
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

    def start(self):
        rospy.spin()

class MainApp:
    def __init__(self):
        self.drone_controller = DroneController()
        self.image_subscriber = ImageSubscriber()

    def run(self):
        image_thread = threading.Thread(target=self.image_subscriber.start)
        image_thread.start()

        self.drone_controller.set_mode('OFFBOARD')

        current_altitude = self.drone_controller.get_current_altitude()
        if current_altitude is not None:
            print(f"Current altitude: {current_altitude} meters")
            target_altitude = max(current_altitude + 10, 10)
        else:
            target_altitude = 10

        self.drone_controller.arm_and_takeoff(target_altitude)

        time.sleep(10)

        self.drone_controller.land()
        '''
        self.drone_controller.disarm()
        self.drone_controller.close()
        '''
        
if __name__ == '__main__':
    app = MainApp()
    app.run()
