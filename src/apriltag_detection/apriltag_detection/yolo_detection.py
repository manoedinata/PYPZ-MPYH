import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO # Import YOLO
import numpy as np
import torch
import os

os.environ['OMP_NUM_THREADS'] = '1'
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['MKL_NUM_THREADS'] = '1'
os.environ['NUMEXPR_NUM_THREADS'] = '1'
cv2.setNumThreads(1)
torch.set_num_threads(1)

TRAFFIC_SIGNS = [
    "No Entry",    # 0
    "Dead End",    # 1
    "Right",       # 2
    "Left",        # 3
    "Forward",     # 4
    "Stop"         # 5
]

TARGET_ID = [1, 4, 3, 0, 2, 5]
class YOLOSignDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_sign_detection')

        # Parameter untuk membatasi thread
        self.declare_parameter('max_threads', 1)
        max_threads = self.get_parameter('max_threads').get_parameter_value().integer_value
        
        # Set thread limits
        torch.set_num_threads(max_threads)
        cv2.setNumThreads(max_threads)

        self.declare_parameter('yolo_model_path', '/home/iris/best.pt')
        yolo_model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value

        # Lower this if you're missing detections, raise if you have too many false positives.
        self.declare_parameter('confidence_threshold', 0.5)
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        # Lower this if you have multiple overlapping boxes for the same sign.
        self.declare_parameter('iou_threshold', 0.5)
        self.iou_threshold = self.get_parameter('iou_threshold').get_parameter_value().double_value

        self.declare_parameter('camera_string', '/dev/v4l/by-id/usb-e-con_systems_See3CAM_CU55_14205401-video-index0')
        camera_string = self.get_parameter('camera_string').get_parameter_value().string_value

        self.bridge = CvBridge()
        
        try:
            self.get_logger().info(f"Loading YOLO model from: {yolo_model_path}")
            self.model = YOLO(yolo_model_path)
            self.get_logger().info("YOLO model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            self.get_logger().error("Ensure the path is correct.")
            rclpy.shutdown()
            return

        self.ov_model = YOLO("/home/iris/best_openvino_model/")

        # self.subscription = self.create_subscription(
        #     Image,
        #     '/vision/color_image',
        #     self.image_callback,
        #     1 
        # )

        self.cap = cv2.VideoCapture(camera_string)
        
        self.pub_sign_image = self.create_publisher(Image, '/sign/image', 1)
        self.pub_sign_picture_id = self.create_publisher(Int32, '/sign/picture/id', 1)
        self.timer = self.create_timer(0.1, self.image_routine)

        # # Optional: For visualization of detections 
        self.declare_parameter('enable_debug_display', True)
        self.enable_debug_display = self.get_parameter('enable_debug_display').get_parameter_value().bool_value
        if self.enable_debug_display:
            self.get_logger().info("Debug display enabled. Showing detection window.")

        
        
    def verify_arrow_direction(self, frame, box):
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        
        # Add margin to focus on the center (arrow)
        margin_x = int((x2 - x1) * 0.1)
        margin_y = int((y2 - y1) * 0.1)
        x1m = max(x1 + margin_x, 0)
        x2m = max(x2 - margin_x, x1m + 1)
        y1m = max(y1 + margin_y, 0)
        y2m = max(y2 - margin_y, y1m + 1)
        crop = frame[y1m:y2m, x1m:x2m]

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # Morphological operations to remove noise
        kernel = np.ones((3, 3), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # Find largest contour 
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros_like(thresh)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            cv2.drawContours(mask, [largest], -1, 255, -1)
            thresh = cv2.bitwise_and(thresh, mask)
            
        h, w = thresh.shape
        left_half = thresh[:, :w // 2]
        right_half = thresh[:, w // 2:]
        left_white = cv2.countNonZero(left_half)
        right_white = cv2.countNonZero(right_half)

        if right_white > left_white * 1.2:
            return "right"
        elif left_white > right_white * 1.2:
            return "left"
        else:
            return None

    def image_routine(self):
        start_time = self.get_clock().now()
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera stream.")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from camera.")
            return
        
        # Resize frame untuk konsistensi
        frame = cv2.resize(frame, (640, 480))

        # Gunakan OpenVINO model untuk inference
        try:
            results = self.ov_model(frame, conf=self.conf_threshold, iou=self.iou_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f"OpenVINO inference error: {e}")
            return

        best_confidence = -1.0
        best_id = -1

        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue
                
            # r.boxes contains all bounding box detections
            for box in r.boxes:
                model_id = int(box.cls.item())
                confidence = box.conf.item()
                
                if 0 <= model_id < len(TARGET_ID):
                    class_id = TARGET_ID[model_id]

                    if class_id in [2, 3]:  # Right or Left
                        direction = self.verify_arrow_direction(frame, box)
                        if direction == "right":
                            class_id = 2
                        elif direction == "left":
                            class_id = 3
                        else:
                            self.get_logger().warn("Arrow direction unclear, skipping.")
                            continue  # Skip uncertain arrows
                    
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_id = class_id
                        
                    # self.get_logger().info(f"Detected: {TRAFFIC_SIGNS[class_id]} (ID: {class_id}, conf: {confidence:.2f})")
                
                # else:
                    # self.get_logger().warn(f"Detected unknown class ID: {model_id} with confidence {confidence:.2f}")

        msg = Int32()
        if best_id != -1: 
            msg.data = best_id
            # self.get_logger().info(f"Publishing sign ID: {best_id} ({TRAFFIC_SIGNS[best_id]})")
        else:
            msg.data = -1 
        
        # Put text on the frame
        if best_id != -1:
            cv2.line(frame,  (0, 15), (999, 15), (255,255,255), 100, -1);
            cv2.putText(frame, f"Sign ID: {best_id} ({TRAFFIC_SIGNS[best_id]})", (10, 34), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4)
            
        self.pub_sign_picture_id.publish(msg)
        self.pub_sign_image.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        
        end_time = self.get_clock().now()
        elapsed_time = (end_time - start_time).nanoseconds / 1e6
        # self.get_logger().info(f"OpenVINO detection completed in {elapsed_time:.2f} ms")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return
        
        # input_yolo = frame.copy()
        
        # # change brighness & saturation
        # input_yolo = cv2.cvtColor(input_yolo, cv2.COLOR_BGR2HSV)
        # h, s, v = cv2.split(input_yolo)
        # s = cv2.add(s, 10)  # Increase saturation
        # # v = cv2.add(v, 50)  # Increase brightness
        # input_yolo = cv2.merge((h, s, v))
        # input_yolo = cv2.cvtColor(input_yolo, cv2.COLOR_HSV2BGR)
        # input_yolo = cv2.convertScaleAbs(input_yolo, alpha=1.3, beta=20)  # Increase brightness and contrast

       
        results = self.model(frame, conf=self.conf_threshold, iou=self.iou_threshold, verbose=False)

        best_confidence = -1.0
        best_id = -1
        # class_id = -1

        for r in results:
            # r.boxes contains all bounding box detections
            for box in r.boxes:
                model_id = int(box.cls.item())
                confidence = box.conf.item()
                
                if 0 <= model_id < len(TARGET_ID):
                    class_id = TARGET_ID[model_id]

                    if class_id in [2, 3]:  # Right or Left
                        direction = self.verify_arrow_direction(frame, box)
                        if direction == "right":
                            class_id = 2
                        elif direction == "left":
                            class_id = 3
                        else:
                            self.get_logger().warn("Arrow direction unclear, skipping.")
                            continue  # Skip uncertain arrows
                
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_id = class_id     

                else:
                    self.get_logger().warn(f"Detected unknown class ID: {class_id} with confidence {confidence:.2f}")

        msg = Int32()
        if best_id != -1: 
            msg.data = best_id
        else:
            msg.data = -1 
            
        self.pub_sign_picture_id.publish(msg)
        self.pub_sign_image.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        # # --- Optional: Debug Display ---
        # if self.enable_debug_display:
        #     # The 'plot()' method draws bounding boxes and labels on the frame
        #     annotated_frame = results[0].plot()
        #     cv2.imshow("YOLOv8 Sign Detection", annotated_frame)
        #     cv2.waitKey(1) 

def main(args=None):
    rclpy.init(args=args)
    node = YOLOSignDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly.")
    finally:
        node.destroy_node()
        # if node.enable_debug_display:
        #     cv2.destroyAllWindows() 
        rclpy.shutdown()

if __name__ == '__main__':
    main()