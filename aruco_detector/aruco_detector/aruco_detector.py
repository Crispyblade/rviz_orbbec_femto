import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import cv2
import cv2.aruco as aruco
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_link"

        
        # Parameters
        self.declare_parameter('marker_size', 0.05)  # 5cm markers
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_dict', '4X4_50')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
            
        self.markers_pub = self.create_publisher(MarkerArray, 'detected_markers', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters()
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            if ids is not None:
                markers = MarkerArray()
                for i, marker_id in enumerate(ids):
                    # Publish marker pose transform
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = self.get_parameter('camera_frame').value
                    t.child_frame_id = f'marker_{marker_id[0]}'
                    
                    # Replace with actual pose estimation using camera intrinsics
                    # This is simplified positional approximation
                    t.transform.translation.x = corners[i][0][0][0] / 100.0
                    t.transform.translation.y = corners[i][0][0][1] / 100.0
                    t.transform.translation.z = 0.5  # Placeholder depth
                    t.transform.rotation.w = 1.0
                    
                    self.tf_broadcaster.sendTransform(t)
                    # Add to image_callback after TF broadcast
                    if ids is not None:
                        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
                        self.odom_msg.pose.pose.position.x = t.transform.translation.x  # From marker detection
                        self.odom_msg.pose.pose.orientation.w = 1.0  # Default orientation
                        self.odom_pub.publish(self.odom_msg)
                
                # Publish marker array for visualization
                markers.markers = self.create_marker_messages(ids, corners)
                self.markers_pub.publish(markers)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def create_marker_messages(self, ids, corners):
        # Implementation for creating Marker messages
        pass

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()
