#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import torch
from ultralytics import YOLO
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations as tf_trans
import bbox_visualizer as bbv

class ObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.marker_pub = rospy.Publisher("/object_markers", MarkerArray, queue_size=10)
        
        self.model = YOLO('./yolov8n.pt')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            detected_objects = self.detect_objects(cv_image)
            marker_array_msg = self.create_marker_message(detected_objects)
            self.marker_pub.publish(marker_array_msg)

            bboxes = []
            labels = []
            for object in detected_objects:
                bbox = (object[0], object[1], object[0]+object[2], object[1]+object[3])
                bboxes.append(bbox)
                labels.append(object[4])

            img = bbv.draw_multiple_rectangles(cv_image, bboxes)
            img = bbv.add_multiple_labels(img, labels, bboxes)
            cv2.imshow("Image", img)
            cv2.waitKey(1)
            
            self.publish_camera_tf()
        except CvBridgeError as e:
            print(e)

    def detect_objects(self, image):
        results = self.model(image)
        detected_objects = []
        
        for result in results[0].boxes:
            x1, y1, x2, y2 = result.xyxy[0].tolist()
            class_label = self.model.names[int(result.cls)]
            confidence = result.conf
            x, y, w, h = (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1
            detected_objects.append((x, y, w, h, class_label, confidence))
        
        return detected_objects

    def create_marker_message(self, detected_objects):
        marker_array_msg = MarkerArray()
        
        for i, obj in enumerate(detected_objects):
            x, y, w, h, class_label, confidence = obj
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x / 100
            marker.pose.position.y = y / 100
            marker.pose.position.z = 0.5
            marker.scale.x = w / 100
            marker.scale.y = h / 100
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # Quaternion 초기화
            quat = tf_trans.quaternion_from_euler(0, 0, 0)
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

            marker_array_msg.markers.append(marker)
        
        return marker_array_msg

    def publish_camera_tf(self):
        # map -> camera_link 변환
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0  # 카메라 높이를 적절히 설정하십시오.
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        
        # camera_link -> camera_frame 변환 (카메라 내부에서 사용하는 프레임)
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "camera_link"
        t.child_frame_id = "camera_frame"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main():
    rospy.init_node('object_detection_node', anonymous=True)
    object_detector = ObjectDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
