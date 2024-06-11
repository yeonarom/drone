#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import torch
import torchvision
from ultralytics import YOLO
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf.transformations as tf_trans
from sort import Sort
import numpy as np
from std_msgs.msg import ColorRGBA
import os
import torchreid

class ReidModel:
    def __init__(self, model_path):
        self.model = torchreid.models.build_model(name='resnet50', num_classes=751, pretrained=True)
        torchreid.utils.load_pretrained_weights(self.model, model_path)
        self.model.eval()

    def extract_features(self, image):
        transform = torchvision.transforms.Compose([
            torchvision.transforms.ToPILImage(),
            torchvision.transforms.Resize((128, 256)),
            torchvision.transforms.ToTensor(),
        ])
        image = transform(image).unsqueeze(0)
        with torch.no_grad():
            features = self.model(image)
        return features

    def compare_features(self, features1, features2):
        return torch.nn.functional.cosine_similarity(features1, features2).item()

class ObjectDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.marker_pub = rospy.Publisher("/object_markers", MarkerArray, queue_size=10)
        
        self.model = YOLO('./src/drone_camera_node/scripts/yolov8n.pt')
        self.tracker = Sort()
        self.reid_model = ReidModel('./src/drone_camera_node/scripts/market.pth')

        self.object_features = {}
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            detected_objects, detected_objects_2_track = self.detect_objects(cv_image)
            tracked_objects = self.track_objects(detected_objects_2_track)
            
            if tracked_objects is not None:
                labels = np.array(detected_objects).T[4]
                reidentified_objects = self.reidentify_objects(cv_image, tracked_objects, labels)
                marker_objs = self.create_marker_message(tracked_objects, reidentified_objects)
                self.marker_pub.publish(marker_objs)
                
                self.visualization(cv_image, tracked_objects, reidentified_objects)
                
                self.publish_camera_tf()
        except CvBridgeError as e:
            print(e)

    def detect_objects(self, image):
        results = self.model(image)
        detected_objects = []
        detected_objects_2_track = []
        
        for result in results[0].boxes:
            x1, y1, x2, y2 = result.xyxy[0].tolist()
            class_label = self.model.names[int(result.cls)]
            confidence = result.conf.item()
            x, y, w, h = (x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1
            detected_objects.append((x, y, w, h, class_label, confidence))
            detected_objects_2_track.append([x1, y1, x2, y2, confidence])
        
        return detected_objects, np.array(detected_objects_2_track, dtype=np.float32)
    
    def track_objects(self, detected_objects):
        tracked_objects = self.tracker.update(detected_objects)
        return tracked_objects
    
    def reidentify_objects(self, image, tracked_objects, labels):
        reidentified_objects = []
        for obj, label in zip(tracked_objects, labels):
            x1, y1, x2, y2, track_id = obj
            cropped_image = image[int(y1):int(y2), int(x1):int(x2)]
            features = self.reid_model.extract_features(cropped_image)
            if track_id in self.object_features:
                similarity = self.reid_model.compare_features(self.object_features[track_id], features)
                if similarity > 0.5:
                    reidentified_objects.append(label)
                else:
                    reidentified_objects.append(f"{label}_new")
            else:
                self.object_features[track_id] = features
                reidentified_objects.append(label)
        return reidentified_objects
    
    def visualization(self, image, tracked_objects, reidentified_objects):
        for obj, label in zip(tracked_objects, reidentified_objects):
            x1, y1, x2, y2, track_id = obj
            cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            cv2.putText(image, str(int(track_id)) + ":" + label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

        cv2.imshow("Image", image)
        cv2.waitKey(1)

    def create_marker_message(self, tracked_objects, reidentified_objects):
        marker_objs = MarkerArray()
        
        for i, (obj, label) in enumerate(zip(tracked_objects, reidentified_objects)):
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
            marker.text = str(int(track_id)) + ":" + label

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

def main():
    rospy.init_node('object_detection_node', anonymous=True)
    object_detector = ObjectDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
