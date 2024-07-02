#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations as tf_trans
from std_msgs.msg import ColorRGBA

class Visualization:
    def __init__(self):
        self.marker_pub = rospy.Publisher("/object_markers", MarkerArray, queue_size=10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def __call__(self, cv_image, tracked_objects, reidentified_objects):
        if reidentified_objects is not None:
            marker_objs = self.create_marker_message(tracked_objects, reidentified_objects)
            self.marker_pub.publish(marker_objs)
            
            self.visualization(cv_image, tracked_objects, reidentified_objects)
            
            self.publish_camera_tf()
    
    def visualization(self, image, tracked_objects, reidentified_objects):
        for obj, reid_id in zip(tracked_objects, reidentified_objects):
            x1, y1, x2, y2, track_id = obj
            cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(image, str(int(track_id)) + ":" + str(int(reid_id)), (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        cv2.imshow("Image", image)
        cv2.waitKey(1)

    def create_marker_message(self, tracked_objects, reidentified_objects):
        marker_objs = MarkerArray()
        
        for i, (obj, reid_id) in enumerate(zip(tracked_objects, reidentified_objects)):
            x1, y1, x2, y2, track_id = obj
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x1 / 100
            marker.pose.position.y = y1 / 100
            marker.pose.position.z = 0.5
            marker.scale.x = (x2 - x1) / 100
            marker.scale.y = (y2 - y1) / 100
            marker.scale.z = 0.1
            marker.color = self.get_color(track_id)
            marker.text = str(int(track_id)) + ":" + str(int(reid_id))

            # Quaternion 초기화
            quat = tf_trans.quaternion_from_euler(0, 0, 0)
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

            marker_objs.markers.append(marker)
        
        return marker_objs
    
    def get_color(self, track_id):
        colors = [
            ColorRGBA(1.0, 0.0, 0.0, 1.0),
            ColorRGBA(1.0, 1.0, 0.0, 1.0),
            ColorRGBA(0.0, 1.0, 0.0, 1.0),
            ColorRGBA(0.0, 0.0, 1.0, 1.0),
        ]

        return colors[int(track_id % len(colors))]

    def publish_camera_tf(self):
        # map -> camera_link 변환
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.0
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