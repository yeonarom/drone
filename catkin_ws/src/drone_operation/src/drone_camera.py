import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse

import sys
sys.path.append('/home/yeonarom/catkin_ws/src/drone_operation/src')

from object_detection import ObjectDetector
from object_tracking import ObjectTracker
from object_reidentification import ObjectReidentifier
from visualize import Visualization

class MainNode:
    def __init__(self):
        rospy.init_node('drone_image_processing_node', anonymous=True)        
        
        self.bridge = CvBridge()
        
        self.detector = ObjectDetector()
        self.tracker = ObjectTracker()
        self.reidentifier = ObjectReidentifier()
        self.visualizer = Visualization()
        
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        self.detection_enabled = True
        self.detection_service = rospy.Service('/set_detection', SetBool, self.handle_set_detection)

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
            
            # Perform object tracking
            tracked_objects = self.tracker(detected_objects)
            
            # Perform object reidentification
            reidentified_objects = self.reidentifier(cv_image, tracked_objects)
            
            # Visualize the results
            self.visualizer(cv_image, tracked_objects, reidentified_objects)
        else:
            rospy.loginfo("Object detection is diabled.")
        
    def run(self):
        rospy.spin()
        
if __name__ == '__main__':
    try:
        main_node = MainNode()
        main_node.run()
    except rospy.ROSInterruptException:
        pass
