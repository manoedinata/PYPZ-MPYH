import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime
import time

TRAFFIC_SIGNS = [
    "No Entry",    # 0
    "Dead End",    # 1
    "Right",       # 2
    "Left",        # 3
    "Forward",     # 4
    "Stop"         # 5
]

TARGET_ID = [1, 4, 3, 0, 2, 5]
INPUT_SIZE = 640  

class ONNXSignDetectionNode(Node):
    def __init__(self):
        super().__init__('onnx_sign_detection')
        self.bridge = CvBridge()

        self.declare_parameter('onnx_model_path', '/home/iris/best.onnx')
        model_path = self.get_parameter('onnx_model_path').get_parameter_value().string_value

        self.declare_parameter('confidence_threshold', 0.5)
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value

        self.declare_parameter('enable_debug_display', False)
        self.enable_debug_display = self.get_parameter('enable_debug_display').get_parameter_value().bool_value

        self.get_logger().info(f"Loading ONNX model: {model_path}")
        self.session = onnxruntime.InferenceSession(model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name

        self.subscription = self.create_subscription(
            Image,
            '/camera/rs2_cam_main/color/image_raw',
            self.image_callback,
            1
        )

        self.pub_sign_image = self.create_publisher(Image, '/sign/image', 1)
        self.pub_sign_picture_id = self.create_publisher(Int32, '/sign/picture/id', 1)

    def preprocess(self, frame):
        resized = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        normalized = rgb.astype(np.float32) / 255.0
        nchw = np.transpose(normalized, (2, 0, 1))
        return np.expand_dims(nchw, axis=0)

    def verify_arrow_direction(self, frame, x1, y1, x2, y2):
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
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)

        # Find largest contour
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask = np.zeros_like(thresh)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            cv2.drawContours(mask, [largest], -1, 255, -1)
            thresh = cv2.bitwise_and(thresh, mask)

        h, w = thresh.shape
        left_white = cv2.countNonZero(thresh[:, :w // 2])
        right_white = cv2.countNonZero(thresh[:, w // 2:])
        if right_white > left_white * 1.2:
            return "right"
        elif left_white > right_white * 1.2:
            return "left"
        return None
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        height, width = frame.shape[:2]
        input_tensor = self.preprocess(frame)

        # start_time = time.time()
        outputs = self.session.run(None, {self.input_name: input_tensor})
        detections = outputs[0][0]  # (1, N, 6) -> (N, 6)
        # inference_time = time.time() - start_time

        best_confidence = -1.0
        best_id = -1
        # best_box = None

        for det in detections:
            if len(det) < 6:
                continue
            x1, y1, x2, y2, confidence, cls = det[:6]
            if confidence < self.conf_threshold:
                continue

            model_id = int(cls)
            if model_id >= len(TARGET_ID):
                continue
            class_id = TARGET_ID[model_id]
            if 0 <= model_id < len(TARGET_ID):
                class_id = TARGET_ID[model_id]

                # Rescale to original image size
                x1 = int(x1 / INPUT_SIZE * width)
                x2 = int(x2 / INPUT_SIZE * width)
                y1 = int(y1 / INPUT_SIZE * height)
                y2 = int(y2 / INPUT_SIZE * height)

                # if class_id in [2, 3]:
                #     direction = self.verify_arrow_direction(frame, x1, y1, x2, y2)
                #     if direction == "right":
                #         class_id = 2
                #     elif direction == "left":
                #         class_id = 3
                #     else:
                #         continue 

                if confidence > best_confidence:
                    best_confidence = confidence
                    best_id = class_id     

            else:
                self.get_logger().warn(f"Detected unknown class ID: {class_id} with confidence {confidence:.2f}")
                # best_box = (x1, y1, x2, y2)

        msg = Int32()
        msg.data = best_id if best_id != -1 else -1
        self.pub_sign_picture_id.publish(msg)

        # if best_box:
        #     x1, y1, x2, y2 = best_box
        #     label = f"{TRAFFIC_SIGNS[best_id]} {best_conf:.2f}"
        #     cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #     cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.pub_sign_image.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        if self.enable_debug_display:
            cv2.imshow("ONNX Sign Detection", frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ONNXSignDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        if node.enable_debug_display:
            cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
