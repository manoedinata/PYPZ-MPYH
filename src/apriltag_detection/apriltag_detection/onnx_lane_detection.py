#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import onnxruntime as ort
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from std_msgs.msg import Int32
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped, Point


class ONNXLaneDetection(Node):
    def __init__(self):
        super().__init__('onnx_lane_following_node')
        
        self.bridge = CvBridge()
        self.session = None
        self.session_ready = False

        # Parameters
        self.declare_parameter("onnx_model_path", "/home/iris/best224.onnx")
        onnx_model_path = self.get_parameter("onnx_model_path").get_parameter_value().string_value
        onnx_model_path = os.path.expanduser(onnx_model_path)

        self.look_ahead_distance = self.declare_parameter('look_ahead_distance', 400).value
        self.conf_thres = self.declare_parameter('conf_threshold', 0.5).value
        self.min_box_area = self.declare_parameter('min_box_area', 1000).value

        try:
            providers = ['OpenVINOExecutionProvider', 'CPUExecutionProvider']
            self.session = ort.InferenceSession(onnx_model_path, providers=providers)
            self.session_ready = True
            active_provider = self.session.get_providers()[0]
            self.get_logger().info(f'Loaded ONNX model from {onnx_model_path} using {active_provider}')
        except Exception as e:
            self.get_logger().error(f'Failed to load ONNX model: {e}')

        # Publishers
        self.debug_blurred_pub = self.create_publisher(Image, '/vision/debug_blur', 1)
        self.debug_canny_pub = self.create_publisher(Image, '/vision/debug_canny', 1)
                                                     
        self.dashed_lines_pub = self.create_publisher(Image, '/vision/dashed_lines', 1)
        self.processed_frame_pub = self.create_publisher(Image, '/vision/processed_frame', 1)
        
        self.intersection_point_pub = self.create_publisher(Float32, '/vision/intersection_point', 1)
        self.intersection_found_pub = self.create_publisher(Bool, '/vision/intersection_found', 1)

        self.filtered_points_pub = self.create_publisher(PointCloud2, '/vision/filtered_points', 1)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rs2_cam_main/color/image_raw',
            self.image_callback,
            1
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/rs2_cam_main/aligned_depth_to_color/image_raw',
            self.depth_callback,
            1
        )

        self.pcl_sub = self.create_subscription(
            PointCloud2,
            '/camera/rs2_cam_main/depth/color/points',
            self.pcl_callback,
            1
        )

        self.frame_count = 0
        self.get_logger().info('ONNX Lane Following Node initialized successfully')

    def lane_finder_flood(self, binary_image):
        try:
            mask = np.zeros((binary_image.shape[0] + 2, binary_image.shape[1] + 2), dtype=np.uint8)
            start_points = [
                (binary_image.shape[1] // 2 - 100, binary_image.shape[0] - 1),
                (binary_image.shape[1] // 2 + 100, binary_image.shape[0] - 1)
            ]
            for pt in start_points:
                if binary_image[pt[1], pt[0]] == 0:
                    cv2.floodFill(binary_image, mask, pt, 255,
                                  flags=cv2.FLOODFILL_MASK_ONLY | (255 << 8))
                    cv2.circle(mask, pt, 1, 255, -1)
            return mask[1:-1, 1:-1]
        except Exception as e:
            self.get_logger().error(f"Error in lane_finder_flood: {e}")
            return np.zeros(binary_image.shape, dtype=np.uint8)

    def interpolate_line_points_numpy(self, start_point, end_point, num_points=10):
        try:
            start = np.array(start_point, dtype=np.float32).flatten()
            end = np.array(end_point, dtype=np.float32).flatten()
            distance = np.linalg.norm(end - start)
            adaptive_num_points = max(5, int(distance / 2))
            t_values = np.linspace(0, 1, adaptive_num_points)
            return start[np.newaxis, :] + t_values[:, np.newaxis] * (end - start)[np.newaxis, :]
        except Exception as e:
            self.get_logger().error(f"Error in interpolate_line_points_numpy: {e}")
            return np.array([])
        
    def find_point_cloud(self, mask, center_cam_x, center_cam_y, is_rising=True):
    # """
    # Find point cloud by searching along rays from center for first transition.
    
    # Args:
    #     mask: Binary image (numpy array)
    #     center_cam_x: X coordinate of center
    #     center_cam_y: Y coordinate of center
    #     is_rising: If True, search for 0->255 transition, else 255->0
    
    # Returns:
    #     List of points [(x, y), ...] relative to center
    # """
        line_points = []
        debug_points = []

        # Precompute cos/sin for all angles to avoid repeated computation
        angle_step = 1.5  # degrees
        num_angles = int(180.0 / angle_step)
        
        # Precompute trigonometric values
        angles_rad = np.arange(num_angles) * angle_step * np.math.pi / 180.0
        cos_vals = np.cos(angles_rad)
        sin_vals = np.sin(angles_rad)
        
        # Get mask dimensions
        rows, cols = mask.shape
        
        # For each angle, search along the ray for the first transition
        for i in range(num_angles):
            cos_a = cos_vals[i]
            sin_a = sin_vals[i]
            last_x, last_y = -1, -1
            
            # Search along the ray
            max_magnitude = cols * 0.6
            for magnitude in np.arange(0, max_magnitude, 1.0):
                pixel_x = int(center_cam_x + magnitude * cos_a + 0.5)
                pixel_y = int(center_cam_y - magnitude * sin_a + 0.5) 
                prev_pixel_x = 0
                prev_pixel_y = 0
                if magnitude == 0:
                    prev_pixel_x = center_cam_x
                    prev_pixel_y = center_cam_y
                else:
                    prev_pixel_x = int(center_cam_x + (magnitude - 1) * cos_a + 0.5)
                    prev_pixel_y = int(center_cam_y - (magnitude - 1) * sin_a + 0.5)

                debug_points.append((pixel_x, pixel_y))
                
                # Check bounds
                if pixel_x < 0 or pixel_x >= cols or pixel_y < 0 or pixel_y >= rows:
                    break

                # Skip duplicate points
                if pixel_x == last_x and pixel_y == last_y:
                    continue
                last_x, last_y = pixel_x, pixel_y
                
                # Get pixel value
                pixel_value = mask[pixel_y, pixel_x]
                prev_pixel_value = mask[prev_pixel_y, prev_pixel_x] if (0 <= prev_pixel_x < cols and 0 <= prev_pixel_y < rows) else mask[pixel_y, pixel_x]
                # print(pixel_value - prev_pixel_value)
                
                # Check for transition
                if is_rising:
                    if pixel_value - prev_pixel_value > 0:
                        self.get_logger().info(f"Found point at angle {i * angle_step:.1f} degrees: ({pixel_x}, {pixel_y}) with value {pixel_value} and previous value {prev_pixel_value}")
                        # Convert to relative coordinates
                        rel_x = pixel_x - center_cam_x
                        rel_y = center_cam_y - pixel_y
                        line_points.append((rel_x, rel_y))
                        break
                else:
                    if pixel_value - prev_pixel_value < 0:
                        # self.get_logger().info(f"Found point at angle {i * angle_step:.1f} degrees: ({pixel_x}, {pixel_y}) with value {pixel_value} and previous value {prev_pixel_value}")
                        # Convert to relative coordinates
                        rel_x = pixel_x - center_cam_x
                        rel_y = center_cam_y - pixel_y
                        line_points.append((rel_x, rel_y))
                        break

        return line_points
        # return debug_points


    def compute_angle_degrees(self, dx, dy):
        """
        Menghitung sudut dalam derajat dari dx dan dy dengan validasi numerik.
        """
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0
        angle_rad = np.arctan2(dy, dx)
        angle_deg = np.degrees(angle_rad)
        if np.isnan(angle_deg) or np.isinf(angle_deg):
            return 90.0
        return angle_deg

    def draw_curvature_analysis(self, frame, sorted_centroids, scale_x, scale_y):
        """
        Menghitung dan menggambar gradien sudut antar centroid serta garis koneksi,
        garis ekstrapolasi, dan vektor normal terhadap arah koneksi.
        """
        curvature_gradient_n_to_o = []
        dashed_lines = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)

        # Hitung sudut antar centroid dan tampilkan
        if len(sorted_centroids) >= 2:
            for i in range(len(sorted_centroids) - 1):
                x1, y1 = sorted_centroids[i][0] * scale_x, sorted_centroids[i][1] * scale_y
                x2, y2 = sorted_centroids[i + 1][0] * scale_x, sorted_centroids[i + 1][1] * scale_y

                dx, dy = x2 - x1, y2 - y1
                angle_deg = self.compute_angle_degrees(dx, dy)
                curvature_gradient_n_to_o.append(angle_deg)

                # Gambar sudut
                cv2.putText(frame, f"{angle_deg:.1f}", (int(x1), int(y1 - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Hitung dan gambar vektor normal ke dua arah dengan jarak 100 px
                vector = np.array([dx, dy], dtype=np.float32)
                norm = np.linalg.norm(vector)
                if norm > 1e-6:
                    unit_vector = vector / norm
                    normal_vector = np.array([-unit_vector[1], unit_vector[0]]) * 100  # jarak 100 px
                    start_point = np.array([x1, y1])
                    
                    # Gambar vektor normal ke arah pertama (kiri)
                    end_point1 = start_point + normal_vector
                    cv2.arrowedLine(frame, tuple(start_point.astype(int)), tuple(end_point1.astype(int)), (255, 0, 0), 2, tipLength=0.3)
                    
                    # Gambar vektor normal ke arah kedua (kanan)
                    end_point2 = start_point - normal_vector
                    cv2.arrowedLine(frame, tuple(start_point.astype(int)), tuple(end_point2.astype(int)), (255, 0, 0), 2, tipLength=0.3)

        # Ekstrapolasi sudut berdasarkan selisih
        if len(curvature_gradient_n_to_o) >= 2:
            curvature_diff = curvature_gradient_n_to_o[1] - curvature_gradient_n_to_o[0]
            extrapolated_curvature = curvature_gradient_n_to_o[-1] + curvature_diff
            curvature_gradient_n_to_o.append(extrapolated_curvature)

            x0, y0 = sorted_centroids[0][0] * scale_x, sorted_centroids[0][1] * scale_y
            cv2.putText(frame, f"Extrap: {extrapolated_curvature:.1f}", (int(x0), int(y0 + 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        # Gambar garis antar centroid
        for i in range(len(sorted_centroids) - 1):
            x1, y1 = sorted_centroids[i][0] * scale_x, sorted_centroids[i][1] * scale_y
            x2, y2 = sorted_centroids[i + 1][0] * scale_x, sorted_centroids[i + 1][1] * scale_y
            cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
            cv2.line(dashed_lines, (int(x1), int(y1)), (int(x2), int(y2)), (255), 2, cv2.LINE_AA)

        # Gambar garis ekstrapolasi dari ujung awal (dua centroid pertama)
        if len(sorted_centroids) >= 2:
            first = np.array(sorted_centroids[0]) * np.array([scale_x, scale_y])
            second = np.array(sorted_centroids[1]) * np.array([scale_x, scale_y])
            direction_start = first - second
            norm_start = np.linalg.norm(direction_start)

            if norm_start > 1e-6:
                direction_start /= norm_start
                cv2.line(frame, tuple(first.astype(int)), (frame.shape[1]//2, frame.shape[0]), (0, 128, 255), 2)  # Warna orange untuk ujung awal
                cv2.line(dashed_lines, tuple(first.astype(int)), (frame.shape[1]//2, frame.shape[0]), (255), 2, cv2.LINE_AA)

        # Gambar garis ekstrapolasi dari ujung akhir (dua centroid terakhir)
        if len(sorted_centroids) >= 2:
            last = np.array(sorted_centroids[-1]) * np.array([scale_x, scale_y])
            second_last = np.array(sorted_centroids[-2]) * np.array([scale_x, scale_y])
            direction_end = last - second_last
            norm_end = np.linalg.norm(direction_end)

            if norm_end > 1e-6:
                direction_end /= norm_end
                extrap_end = last + direction_end * 50  # panjang maksimum 400 piksel
                cv2.line(frame, tuple(last.astype(int)), tuple(extrap_end.astype(int)), (0, 0, 255), 2)  # Warna merah untuk ujung akhir
                cv2.line(dashed_lines, tuple(last.astype(int)), tuple(extrap_end.astype(int)), (255), 2, cv2.LINE_AA)

        return curvature_gradient_n_to_o, dashed_lines
    
    def depth_callback(self, msg):
        """
        Callback untuk menerima pesan gambar dari kamera depth.
        Data depth dari RealSense camera dalam format 16-bit.
        """
        try:
            # RealSense depth data is 16-bit, use 'mono16' or 'passthrough'
            depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # self.get_logger().info(f"Received depth image of size: {depth_frame.shape}, dtype: {depth_frame.dtype}")
            
            # Optional: Convert to visualization format (8-bit for display)
            # depth_normalized = cv2.convertScaleAbs(depth_frame, alpha=255.0/depth_frame.max())
            
        except Exception as e:
            self.get_logger().error(f"cv_bridge error for depth data: {e}")

    def pcl_callback(self, msg):
        """
        Callback untuk menerima pesan PointCloud2 dari kamera RealSense.
        Menggunakan sensor_msgs/PointCloud2 untuk mendapatkan data point cloud.
        """
        try:            
            # Convert PointCloud2 message to list of points
            points_list = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            self.get_logger().info(f"Received point cloud with {len(points_list)} points")

            # Get transform from camera frame to base_link
            try:
                if not hasattr(self, 'tf_buffer'):
                    self.tf_buffer = Buffer()
                    self.tf_listener = TransformListener(self.tf_buffer, self)
                
                # Get transform from camera frame to base_link
                transform = self.tf_buffer.lookup_transform(
                    'base_link',  # target frame
                    msg.header.frame_id,  # source frame (camera frame)
                    msg.header.stamp,  # time
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                
                self.get_logger().info(f"Transform from {msg.header.frame_id} to base_link received")
                
                # Transform each point to base_link frame
                transformed_points = []
                for point in points_list[:100]:  # Limit to first 100 points for performance
                    # Create PointStamped message
                    point_stamped = PointStamped()
                    point_stamped.header = msg.header
                    point_stamped.point.x = float(point[0])
                    point_stamped.point.y = float(point[1]) 
                    point_stamped.point.z = float(point[2])
                    
                    # Transform point to base_link frame
                    transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                    transformed_points.append([
                        transformed_point.point.x,
                        transformed_point.point.y, 
                        transformed_point.point.z
                    ])
                
                self.get_logger().info(f"Transformed {len(transformed_points)} points to base_link frame")

                # remove points with z > 0.1 and z < -0.1
                filtered_points = [p for p in transformed_points if -0.1 < p[2] < 0.1]
                self.get_logger().info(f"Filtered points count: {len(filtered_points)}")
                # Create PointCloud2 message for filtered points
                filtered_pcl_msg = pc2.create_cloud_xyz32(
                    msg.header,
                    transformed_points
                )
                self.filtered_points_pub.publish(filtered_pcl_msg)
                
            except Exception as tf_error:
                self.get_logger().error(f"Transform error: {tf_error}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud data: {e}")
    
    def image_callback(self, msg):
        """
        Callback untuk menerima pesan gambar dari kamera.
        Memproses gambar untuk mendeteksi jalur dan mengirimkan hasilnya.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # self.get_logger().info(f"Received image of size: {frame.shape}, dtype: {frame.dtype}")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return
        try:
            start_time = time.time()
            
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            #? ==================================================
            #?                  Process Image 
            #? ==================================================
            blur_frame = cv2.GaussianBlur(frame, (31, 31), 0)
            gray_frame = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2GRAY)
            canny_frame = cv2.Canny(gray_frame, 50, 70, 3)
            canny_frame = cv2.dilate(canny_frame, np.ones((3, 3), np.uint8), iterations=3)

             # Hitung dan tampilkan FPS
            end_time = time.time()
            fps = 1.0 / (end_time - start_time)
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            self.debug_blurred_pub.publish(self.bridge.cv2_to_imgmsg(blur_frame, "bgr8"))
            self.debug_canny_pub.publish(self.bridge.cv2_to_imgmsg(canny_frame, "mono8"))
            # self.dashed_lines_pub.publish(self.bridge.cv2_to_imgmsg(lane_mask, "mono8"))
            self.processed_frame_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


    def image_callback_backup(self, msg):
        self.frame_count += 1
        self.get_logger().debug(f"[Frame {self.frame_count}] Callback triggered.")

        if not self.session_ready:
            self.get_logger().warn("ONNX model not loaded. Skipping frame.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if frame is None or frame.size == 0:
                self.get_logger().warn("Empty frame received.")
                return
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        try:
            start_time = time.time()
            h, w = frame.shape[:2]
            scale_x = w / 224
            scale_y = h / 224

            blurred = cv2.GaussianBlur(frame, (31, 31), 0)
            canny = cv2.Canny(blurred, 50, 70, apertureSize=3)
            canny = cv2.dilate(canny, np.ones((9, 9), np.uint8), 3)
            lane_mask = self.lane_finder_flood(canny)
            lane_mask = cv2.bitwise_not(lane_mask)

            contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) < 25000:
                    cv2.drawContours(lane_mask, [cnt], -1, 0, thickness=cv2.FILLED)

            lane_mask = cv2.bitwise_not(lane_mask)
            masked_frame = cv2.bitwise_and(frame, frame, mask=lane_mask)

            img_resized = cv2.resize(masked_frame, (224, 224))
            img_input = img_resized.transpose(2, 0, 1)[None].astype(np.float32) / 255.0

            ##########################################
            # outputs = self.session.run(None, {self.session.get_inputs()[0].name: img_input})
            # output = outputs[0].transpose()
            ##########################################
            canny_h, canny_w = canny.shape[:2]
            detected_points = self.find_point_cloud(canny, canny_w // 2 - 1, canny_h - 1, is_rising=True)
            print(f"[Frame {self.frame_count}] Detected points: {len(detected_points)}")
            for point in detected_points:
                cv2.circle(frame, (int(point[1]), int(point[0])), 10, (0, 255, 0), -1)
            ##########################################
            output = np.zeros((0, 6), dtype=np.float32)  # Inisialisasi output dengan 6 kolom untuk angle
            blur_hernanda = cv2.GaussianBlur(masked_frame, (15, 15), 0)
            grey_hernanda = cv2.cvtColor(blur_hernanda, cv2.COLOR_BGR2GRAY)
            threshold_hernanda = cv2.threshold(grey_hernanda, 100, 255, cv2.THRESH_BINARY)[1]

            background_hernanda = cv2.threshold(grey_hernanda, 1, 255, cv2.THRESH_BINARY)[1]
            threshold_hernanda = cv2.bitwise_and(threshold_hernanda, background_hernanda)

            # threshold_hernanda = cv2.dilate(threshold_hernanda, np.ones((9, 9), np.uint8), iterations=5)
            # threshold_hernanda = cv2.erode(threshold_hernanda, np.ones((3, 3), np.uint8), iterations=5)

            # Cari contours pada threshold_hernanda kemudian cari semua area contours dan print
            contours_hernanda, _ = cv2.findContours(threshold_hernanda, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            threshold_hernanda = np.zeros((threshold_hernanda.shape[0], threshold_hernanda.shape[1]), dtype=np.uint8)
            # cluster menjadi noise, object, dan background sesuai ukuran area

            # Check rectangle dengan approxPolyDP jika memiliki 4 sisi maka pertahankan cnt selain itu hapus
            for cnt in contours_hernanda:
                area = cv2.contourArea(cnt)
                if area < 500:
                    continue
                
                # Parameter tuning untuk deteksi bentuk
                epsilon_factor = 0.02  # Tingkatkan untuk bentuk lebih kasar, turunkan untuk lebih presisi
                epsilon = epsilon_factor * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                
                # Toleransi untuk bentuk kotak/trapezium (4-6 titik)
                min_vertices = 4  # Minimum untuk kotak
                max_vertices = 6  # Maximum untuk trapezium/kotak yang tidak sempurna
                
                if min_vertices <= len(approx) <= max_vertices:
                    # Validasi tambahan untuk aspek ratio dan convexity
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = w / h
                    
                    # Toleransi aspect ratio (0.3 - 3.0 untuk berbagai bentuk)
                    min_aspect_ratio = 0.3
                    max_aspect_ratio = 3.0
                    
                    # Check convexity (opsional untuk bentuk yang lebih fleksibel)
                    is_convex = cv2.isContourConvex(approx)
                    
                    if (min_aspect_ratio <= aspect_ratio <= max_aspect_ratio):
                        cv2.drawContours(threshold_hernanda, [approx], -1, 255, thickness=cv2.FILLED)
                        
                        # Debug info (opsional)
                        # print(f"Vertices: {len(approx)}, AR: {aspect_ratio:.2f}, Convex: {is_convex}")
            # Hapus contours yang terlalu besar
            contours_hernanda, _ = cv2.findContours(threshold_hernanda, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_hernanda:
                area = cv2.contourArea(cnt)
                if area > 12000:
                    cv2.drawContours(threshold_hernanda, [cnt], -1, 0, thickness=cv2.FILLED)

            contours_hernanda, _ = cv2.findContours(threshold_hernanda, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours_hernanda:
                # Get minimum area rectangle for the contour (handles rotation and skew)
                rect = cv2.minAreaRect(cnt)
                center_x, center_y = rect[0]
                width, height = rect[1]
                angle = rect[2]  # Rotation angle in degrees
                
                # Get the 4 corner points of the rotated rectangle
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                
                # Draw the rotated rectangle on the frame for visualization
                cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
                
                # Normalize coordinates to model input size (224x224)
                norm_center_x = center_x / scale_x
                norm_center_y = center_y / scale_y
                norm_width = width / scale_x
                norm_height = height / scale_y
                norm_angle = angle  # Keep angle as is or normalize if needed
                
                # Add to output with angle information
                output = np.append(output, np.array([[norm_center_x, norm_center_y, norm_width, norm_height, 1.0, norm_angle]]), axis=0)

            # canny_hernanda = cv2.Canny(blur_hernanda, 50, 95)
            # canny_hernanda = cv2.dilate(canny_hernanda, np.ones((3, 3), np.uint8), iterations=5)

            # lane_mask = cv2.erode(lane_mask, np.ones((3, 3), np.uint8), iterations=8)
            # canny_hernanda = cv2.bitwise_and(canny_hernanda, lane_mask)

            # hough line detetc short lines
            # lines = cv2.HoughLinesP(canny_hernanda, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=10)
            # if lines is not None:
            #     for line in lines:
            #         x1, y1, x2, y2 = line[0]
            #         cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            ##########################################
            # output = np.array([[112, 112, 50, 50, 0.9],
            #         [150, 150, 30, 30, 0.8],
            #         [180, 180, 40, 40, 0.7]])
            ##########################################

            dashed_line_mask = np.zeros_like(masked_frame, dtype=np.uint8)
            detected_centroids = []

            # Proses output model
            for box in output:
                x_center, y_center, box_w, box_h, conf = box[:5]
                if conf < self.conf_thres:
                    continue

                x_center = float(x_center)
                y_center = float(y_center)
                box_w = float(box_w)
                box_h = float(box_h)

                x1 = int((x_center - box_w / 2) * scale_x)
                y1 = int((y_center - box_h / 2) * scale_y)
                x2 = int((x_center + box_w / 2) * scale_x)
                y2 = int((y_center + box_h / 2) * scale_y)

                box_area = (x2 - x1) * (y2 - y1)
                if box_area >= self.min_box_area:
                    cv2.rectangle(dashed_line_mask, (x1, y1), (x2, y2), 255, thickness=cv2.FILLED)

            # Gabungkan mask dengan frame asli
            frame = cv2.addWeighted(frame, 0.5, dashed_line_mask, 0.5, 0)

            # Konversi ke grayscale untuk findContours
            dashed_line_mask_gray = cv2.cvtColor(dashed_line_mask, cv2.COLOR_BGR2GRAY)
            contours, _ = cv2.findContours(dashed_line_mask_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt) < 1000:
                    continue

                x, y, w_box, h_box = cv2.boundingRect(cnt)
                x_center = (x + w_box / 2) / scale_x
                y_center = (y + h_box / 2) / scale_y
                detected_centroids.append((x_center, y_center))
            
            # Urutkan centroid berdasarkan jarak dan sudut
            sorted_centroids = []
            if detected_centroids:
                remaining_centroids = detected_centroids.copy()
                max_y_idx = np.argmax([centroid[1] for centroid in remaining_centroids])
                current_centroid = remaining_centroids.pop(max_y_idx)
                sorted_centroids.append(current_centroid)

                while remaining_centroids:
                    last_centroid = sorted_centroids[-1]
                    last_pixel = (last_centroid[0] * scale_x, last_centroid[1] * scale_y)

                    best_score = float('inf')
                    best_centroid = None

                    for centroid in remaining_centroids:
                        pixel = (centroid[0] * scale_x, centroid[1] * scale_y)
                        distance = np.linalg.norm(np.array(pixel) - np.array(last_pixel))
                        score = distance

                        if score < best_score:
                            best_score = score
                            best_centroid = centroid

                    if best_centroid:
                        sorted_centroids.append(best_centroid)
                        remaining_centroids.remove(best_centroid)

            # Tampilkan indeks centroid
            for i, centroid in enumerate(sorted_centroids):
                cv2.putText(frame, f"{i+1}", (int(centroid[0] * scale_x), int(centroid[1] * scale_y)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            curvature_gradient_n_to_o = []
            dashed_lines = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8) # PUBLISH
            if(len(sorted_centroids) > 1):
                for i in range(len(sorted_centroids) - 1):
                    dx = sorted_centroids[i + 1][0] - sorted_centroids[i][0]
                    dy = sorted_centroids[i + 1][1] - sorted_centroids[i][1]
                    
                    curvature_gradient = self.compute_angle_degrees(dx, dy)
                    curvature_gradient_n_to_o.append(curvature_gradient)
                    
                    cv2.putText(frame, f"{curvature_gradient:.1f}", 
                                (int(sorted_centroids[i][0] * scale_x), int(sorted_centroids[i][1] * scale_y) - 10),    
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
            # Gambar analisis kurvatur
            if sorted_centroids:
                curvature_gradient_n_to_o, dashed_lines = self.draw_curvature_analysis(frame, sorted_centroids, scale_x, scale_y)
            # Interpolasi titik-titik antara centroid
            interpolated_points = []
            if len(sorted_centroids) >= 2:
                for i in range(len(sorted_centroids) - 1):
                    start_point = (sorted_centroids[i][0] * scale_x, sorted_centroids[i][1] * scale_y)
                    end_point = (sorted_centroids[i + 1][0] * scale_x, sorted_centroids[i + 1][1] * scale_y)
                    points = self.interpolate_line_points_numpy(start_point, end_point, num_points=10)
                    interpolated_points.extend(points)

                # Gambar titik-titik interpolasi
                for point in interpolated_points:
                    cv2.circle(frame, (int(point[0]), int(point[1])), 2, (255, 255, 0), -1)

            # Look Ahead distance target
            cv2.circle(frame, (int(frame.shape[1]/2), int(frame.shape[0])), self.look_ahead_distance, (0, 255, 255), 2)

            # Cari intersection dashed lines dengan look ahead circle
            center_x = int(frame.shape[1] / 2)
            center_y = int(frame.shape[0])
            intersection_found = False
            intersection_point = [0.0, 0.0, 0.0]  # Inisialisasi dengan nilai default
            
            for i in range(180):
                angle_rad = np.deg2rad(i)
                x = int(center_x + self.look_ahead_distance * np.cos(angle_rad))
                y = int(center_y - self.look_ahead_distance * np.sin(angle_rad))

                if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                    if dashed_lines[y, x] > 0:
                        cv2.circle(frame, (x, y), 5, (0, 255, 255), -1)
                        intersection_point = [x, y, 0.0]  # Simpan titik intersection
                        intersection_found = True
                        break
            cv2.circle(frame, (int(intersection_point[0]), int(intersection_point[1])), 5, (0, 255, 255), 2)
            target_steering_angle = np.arctan2(center_y - intersection_point[1], intersection_point[0] - center_x)

            # self.get_logger().info(f"Point of Interest: {intersection_point}, Target Steering Angle: {target_steering_angle:.2f} radians")
    
            # Initialize buffer and smoothing parameters if not exists
            if not hasattr(self, 'target_angle_buffer'):
                self.target_angle_buffer = 0.0
                self.angle_velocity = 0.0  # radians per frame velocity
                self.angle_acceleration = 0.2  # radians per frame squared (adjust as needed)
                self.max_angle_velocity = 0.5  # maximum radians per frame
                self.missing_frames_count = 0
                self.max_missing_frames = 10  # buffer for 10 frames when missing
                self.use_constant_acceleration = False  # Toggle: True for constant acceleration, False for constant velocity
            
            # Calculate target angle
            raw_target_angle = (float(target_steering_angle) - np.pi / 2) / 2
            raw_target_angle = np.clip(raw_target_angle, -0.8, 0.8)
            
            # Handle missing intersection
            if not intersection_found:
                self.missing_frames_count += 1
                if self.missing_frames_count <= self.max_missing_frames:
                    # Use buffered value, no change in velocity
                    target_angle_smooth = self.target_angle_buffer
                else:
                    if self.use_constant_acceleration:
                        # Gradually return to center (0.0) when missing too long - constant acceleration
                        target_direction = -np.sign(self.target_angle_buffer) if abs(self.target_angle_buffer) > 0.01 else 0
                        
                        # Apply constant acceleration toward center
                        self.angle_velocity += target_direction * self.angle_acceleration
                        self.angle_velocity = np.clip(self.angle_velocity, -self.max_angle_velocity, self.max_angle_velocity)
                        
                        target_angle_smooth = self.target_angle_buffer + self.angle_velocity
                        
                        # Stop at center
                        if abs(target_angle_smooth) < 0.01:
                            target_angle_smooth = 0.0
                            self.angle_velocity = 0.0
                    else:
                        # Constant velocity approach toward center
                        target_direction = -np.sign(self.target_angle_buffer) if abs(self.target_angle_buffer) > 0.01 else 0
                        velocity_step = target_direction * self.max_angle_velocity
                        
                        target_angle_smooth = self.target_angle_buffer + velocity_step
                        
                        # Stop at center
                        if abs(target_angle_smooth) < 0.01:
                            target_angle_smooth = 0.0
            else:
                # Reset missing frames counter
                self.missing_frames_count = 0
                
                if self.use_constant_acceleration:
                    # Calculate desired direction and apply constant acceleration
                    angle_diff = raw_target_angle - self.target_angle_buffer
                    target_direction = np.sign(angle_diff) if abs(angle_diff) > 0.01 else 0
                    
                    # Apply constant acceleration toward target
                    self.angle_velocity += target_direction * self.angle_acceleration
                    self.angle_velocity = np.clip(self.angle_velocity, -self.max_angle_velocity, self.max_angle_velocity)
                    
                    target_angle_smooth = self.target_angle_buffer + self.angle_velocity
                    
                    # Stop when close to target
                    if abs(raw_target_angle - target_angle_smooth) < 0.01:
                        target_angle_smooth = raw_target_angle
                        self.angle_velocity = 0.0
                else:
                    # Constant velocity approach toward target
                    angle_diff = raw_target_angle - self.target_angle_buffer
                    target_direction = np.sign(angle_diff) if abs(angle_diff) > 0.01 else 0
                    velocity_step = target_direction * self.max_angle_velocity
                    
                    target_angle_smooth = self.target_angle_buffer + velocity_step
                    
                    # Clamp to target if we overshoot
                    if (target_direction > 0 and target_angle_smooth > raw_target_angle) or \
                       (target_direction < 0 and target_angle_smooth < raw_target_angle):
                        target_angle_smooth = raw_target_angle
            
            # Update buffer
            self.target_angle_buffer = target_angle_smooth
            
            # Create message
            target_angle_msg = Float32()
            target_angle_msg.data = target_angle_smooth

            fps = 1.0 / (time.time() - start_time + 1e-5)
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            self.debug_blurred_pub.publish(self.bridge.cv2_to_imgmsg(blurred, "bgr8"))
            self.debug_canny_pub.publish(self.bridge.cv2_to_imgmsg(canny, "mono8"))
            self.dashed_lines_pub.publish(self.bridge.cv2_to_imgmsg(lane_mask, "mono8"))
            self.processed_frame_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            self.intersection_point_pub.publish(target_angle_msg)
            self.intersection_found_pub.publish(Bool(data=intersection_found))

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ONNXLaneDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
