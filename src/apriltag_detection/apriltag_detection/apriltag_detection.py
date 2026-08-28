import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from pupil_apriltags import Detector
import math

traffic_signs = [
    "No Entry",    # 0
    "Dead End",    # 1
    "Right",       # 2
    "Left",        # 3
    "Forward",     # 4
    "Stop"         # 5
]

class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_detection')
        self.bridge = CvBridge()
        self.detector = Detector(families='tag36h11')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/rs2_cam_main/color/image_raw',
            self.image_callback,
            1
        )
        
        self.pub_sign_marker_id = self.create_publisher(Int32, '/sign/marker/id', 1)
        self.pub_marker_image = self.create_publisher(Image, '/sign/marker/image', 1)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        tag_id = self.detect_nearest_apriltag(frame)
        self.pub_sign_marker_id.publish(Int32(data=tag_id))
        # self.get_logger().info(f"Published Tag ID: {tag_id if tag_id != -1 else 'None'}")

    def detect_nearest_apriltag(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        equalized = cv2.equalizeHist(gray)
        tags = self.detector.detect(equalized)

        if not tags:
            return -1  

        best_tag = None
        best_score = float('-inf')

        for tag in tags:
            tag_id = tag.tag_id
            if not (0 <= tag_id < len(traffic_signs)):
                continue

            edge_lengths = [
                math.dist(tag.corners[i], tag.corners[(i + 1) % 4])
                for i in range(4)
            ]
            # avg_edge = sum(edge_lengths) / 4.0
            sum_edge = sum(edge_lengths)

            score = sum_edge
            # score = avg_edge - 0.5 * distance_to_center

            if score > best_score:
                best_score = score
                best_tag = tag

        if best_tag:
            final_tag_id = best_tag.tag_id

            # # Draw tag
            # for i in range(4):
            #     pt1 = tuple(map(int, best_tag.corners[i]))
            #     pt2 = tuple(map(int, best_tag.corners[(i + 1) % 4]))
            #     cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            # center = tuple(map(int, best_tag.center))
            # cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # top_left = tuple(map(int, best_tag.corners[0]))
            # cv2.putText(frame, traffic_signs[final_tag_id], (top_left[0], top_left[1] - 10),
            #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            if frame is not None and frame.shape[0] > 0 and frame.shape[1] > 0:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.pub_marker_image.publish(img_msg)
            else:
                self.get_logger().warn("Frame is empty, not publishing.")

            return final_tag_id

        return -1


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
