#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import torch
import numpy as np
from ultralytics import YOLO
from scipy.optimize import linear_sum_assignment
from geometry_msgs.msg import Point
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations as tf_trans


class STrack:
    def __init__(self, tlwh, score, cls, track_id):
        self.tlwh = np.array(tlwh)
        self.score = score
        self.cls = cls
        self.track_id = track_id
        self.is_activated = False
        self.trace = [tlwh[:2]]

    def predict(self):
        pass

    def update(self, new_tlwh, score):
        self.tlwh = np.array(new_tlwh)
        self.score = score
        self.trace.append(new_tlwh[:2])
        self.is_activated = True


class BYTETracker:
    def __init__(self):
        self.tracks = []
        self.track_id_count = 0

    def update(self, detections):
        if len(self.tracks) == 0:
            self.tracks = [STrack(det[:4], det[4], det[5], self.track_id_count + i) for i, det in enumerate(detections)]
            self.track_id_count += len(detections)
        else:
            iou_matrix = np.zeros((len(self.tracks), len(detections)), dtype=np.float32)
            for t, track in enumerate(self.tracks):
                for d, det in enumerate(detections):
                    iou_matrix[t, d] = self.iou(track.tlwh, det[:4])

            row_indices, col_indices = linear_sum_assignment(-iou_matrix)

            for row, col in zip(row_indices, col_indices):
                if iou_matrix[row, col] >= 0.3:
                    self.tracks[row].update(detections[col][:4], detections[col][4])
                    self.tracks[row].is_activated = True

            unmatched_detections = set(range(len(detections))) - set(col_indices)
            for i in unmatched_detections:
                self.tracks.append(STrack(detections[i][:4], detections[i][4], detections[i][5], self.track_id_count))
                self.track_id_count += 1

        self.tracks = [t for t in self.tracks if t.is_activated]

        return self.tracks

    @staticmethod
    def iou(bbox1, bbox2):
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2

        xi1 = max(x1, x2)
        yi1 = max(y1, y2)
        xi2 = min(x1 + w1, x2 + w2)
        yi2 = min(y1 + h1, y2 + h2)

        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)
        bbox1_area = w1 * h1
        bbox2_area = w2 * h2
        union_area = bbox1_area + bbox2_area - inter_area

        return inter_area / union_area


class ObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.marker_pub = rospy.Publisher("/object_markers", MarkerArray, queue_size=10)
        
        self.model = YOLO('./yolov8n.pt')
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tracker = BYTETracker()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            detected_objects = self.detect_objects(cv_image)
            tracked_objects = self.track_objects(detected_objects)
            marker_array_msg = self.create_marker_message(tracked_objects)
            self.marker_pub.publish(marker_array_msg)
            
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
            detected_objects.append((x, y, w, h, confidence, class_label))
        
        return detected_objects

    def track_objects(self, detected_objects):
        detections = [np.array([x - w/2, y - h/2, x + w/2, y + h/2, s, c])
                      for x, y, w, h, s, c in detected_objects]
        tracked_objects = self.tracker.update(detections)
        return tracked_objects

    def create_marker_message(self, tracked_objects):
        marker_array_msg = MarkerArray()
        
        for i, obj in enumerate(tracked_objects):
            x1, y1, x2, y2 = obj.tlwh
            w, h = x2 - x1, y2 - y1
            x, y = (x1 + x2) / 2, (y1 + y2) / 2

            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "objects"
            marker.id = int(obj.track_id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x / 100  # 좌표 조정
            marker.pose.position.y = y / 100  # 좌표 조정
            marker.pose.position.z = 0.5  # Z 축 위치 조정
            marker.scale.x = w / 100  # 크기 조정
            marker.scale.y = h / 100  # 크기 조정
            marker.scale.z = 0.1  # 크기 조정
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            quat = tf_trans.quaternion_from_euler(0, 0, 0)
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

            marker_array_msg.markers.append(marker)

            # Create a line strip marker for the object's trace
            line_marker = Marker()
            line_marker.header.frame_id = "camera_link"
            line_marker.header.stamp = rospy.Time.now()
            line_marker.ns = "trace"
            line_marker.id = int(obj.track_id) + 1000  # Unique ID for trace
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05
            line_marker.color.a = 1.0
            line_marker.color.r = 1.0
            line_marker.color.g = 0.0
            line_marker.color.b = 0.0

            for pos in obj.trace:
                p = Point()
                p.x = pos[0] / 100
                p.y = pos[1] / 100
                p.z = 0.5
                line_marker.points.append(p)

            marker_array_msg.markers.append(line_marker)
        
        return marker_array_msg

    def publish_camera_tf(self):
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


if __name__ == '__main__':
    rospy.init_node('object_detector')
    od = ObjectDetector()
    rospy.spin()
