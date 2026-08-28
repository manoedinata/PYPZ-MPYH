#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import onnxruntime as ort
import numpy as np
import cv2
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Bool
from std_msgs.msg import Int32
from cv_bridge import CvBridge

class ONNXLaneFollowingNode(Node):
    def __init__(self):
        super().__init__('onnx_lane_following_node')
        
        # Publishers
        self.dashed_lines_pub = self.create_publisher(Image, '/vision/dashed_lines', 1)
        self.processed_frame_pub = self.create_publisher(Image, '/vision/processed_frame', 1)
        self.intersection_point_pub = self.create_publisher(Point, '/vision/intersection_point', 1)
        self.intersection_found_pub = self.create_publisher(Bool, '/vision/intersection_found', 1)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/rs2_cam_main/color/image_raw',  # Change this to your camera topic
            self.image_callback,
            1
        )

        self.marker_sub = self.create_subscription(
            Int32,
            '/sign/picture/id',  # Change this to your marker image topic
            self.marker_callback,
            1
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Load ONNX model
        model_path = os.path.expanduser("~/best224.onnx")
        self.session = ort.InferenceSession(model_path)

        self.get_logger().info(f'Loaded ONNX model from {model_path}')
        
        # Parameters
        self.look_ahead_distance = self.declare_parameter('look_ahead_distance', 200).value
        self.conf_thres = self.declare_parameter('conf_threshold', 0.5).value
        self.min_box_area = self.declare_parameter('min_box_area', 1000).value
        
        self.get_logger().info('ONNX Lane Following Node initialized')

    def lane_finder_flood(self, binary_image):
        """
        Menggunakan algoritma flood fill untuk mendeteksi area jalur pada citra biner.
        """
        mask = np.zeros((binary_image.shape[0] + 2, binary_image.shape[1] + 2), dtype=np.uint8)
        lane_map = np.zeros(binary_image.shape, dtype=np.uint8)

        start_points = [
            (binary_image.shape[1] // 2 - 50, binary_image.shape[0] - 1),
            (binary_image.shape[1] // 2 + 50, binary_image.shape[0] - 1)
        ]

        for pt in start_points:
            if binary_image[pt[1], pt[0]] == 0:
                cv2.floodFill(binary_image, mask, pt, 255,
                              flags=cv2.FLOODFILL_MASK_ONLY | (255 << 8))

        lane_map = mask[1:-1, 1:-1]
        return lane_map

    def interpolate_line_points_numpy(self, start_point, end_point, num_points=10):
        """
        Menginterpolasi titik-titik di antara dua titik menggunakan NumPy dengan sangat padat.
        """
        start = np.array(start_point, dtype=np.float32).flatten()
        end = np.array(end_point, dtype=np.float32).flatten()

        if len(start) != 2 or len(end) != 2:
            raise ValueError("Start and end points must be 2D coordinates")

        # Hitung jarak antara titik start dan end
        distance = np.linalg.norm(end - start)
        
        # Buat titik sangat padat - 1 titik per 2-3 pixel
        adaptive_num_points = max(5, int(distance / 2))
        
        # Interpolasi dari ujung ke ujung tanpa skip
        t_values = np.linspace(0, 1, adaptive_num_points)
        points = start[np.newaxis, :] + t_values[:, np.newaxis] * (end - start)[np.newaxis, :]

        return points

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
                extrap_start = first + direction_start * 400  # panjang maksimum 400 piksel
                cv2.line(frame, tuple(first.astype(int)), tuple(extrap_start.astype(int)), (0, 128, 255), 2)  # Warna orange untuk ujung awal
                cv2.line(dashed_lines, tuple(first.astype(int)), tuple(extrap_start.astype(int)), (255), 2, cv2.LINE_AA)

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
    
    def marker_callback(self, msg):
        """
        Callback untuk menerima pesan marker.
        Saat ini tidak digunakan, tetapi bisa diimplementasikan jika diperlukan.
        """
        self.get_logger().info(f"Received marker ID: {msg.data}")

    def image_callback(self, msg):
        """
        Callback untuk menerima pesan gambar dari kamera.
        Memproses gambar untuk mendeteksi jalur dan mengirimkan hasilnya.
        """
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur_frame = cv2.GaussianBlur(gray_frame, (31, 31), 0)
            canny_frame = cv2.Canny(blur_frame, 50, 70, 3)
            canny_frame = cv2.dilate(canny_frame, np.ones((3, 3), np.uint8), iterations=3)

             # Hitung dan tampilkan FPS
            end_time = time.time()
            fps = 1.0 / (end_time - start_time)
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Publish data
            # Publish dashed lines as image
            # dashed_lines_msg = self.bridge.cv2_to_imgmsg(dashed_lines, "mono8")
            # dashed_lines_msg.header = msg.header
            # self.dashed_lines_pub.publish(dashed_lines_msg)
            
            # Publish intersection point
            # intersection_point.header = msg.header
            # self.intersection_point_pub.publish(intersection_point)
            
            # Publish intersection found status
            # intersection_found_msg = Bool()
            # intersection_found_msg.data = intersection_found
            # self.intersection_found_pub.publish(intersection_found_msg)
            
            # Publish processed frame
            processed_frame_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            processed_frame_msg.header = msg.header
            self.processed_frame_pub.publish(processed_frame_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def image_callback_backup(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return
        try:
            start_time = time.time()
            
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            h, w = frame.shape[:2]
            scale_x = w / 224
            scale_y = h / 224

            # Pra-pemrosesan gambar
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            canny = cv2.Canny(blurred, 50, 95)
            canny = cv2.dilate(canny, np.ones((3, 3), np.uint8), iterations=3)
            lane_mask = self.lane_finder_flood(canny)
            lane_mask = cv2.bitwise_not(lane_mask)

            # Menghapus kontur kecil
            contours, _ = cv2.findContours(lane_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                if cv2.contourArea(cnt) < 25000:
                    cv2.drawContours(lane_mask, [cnt], -1, 0, thickness=cv2.FILLED)

            lane_mask = cv2.bitwise_not(lane_mask)
            masked_lane_frame = cv2.bitwise_and(frame, frame, mask=lane_mask)

            # Pra-pemrosesan untuk model
            img_resized = cv2.resize(masked_lane_frame, (224, 224))
            img_input = img_resized.transpose(2, 0, 1)[None].astype(np.float32) / 255.0

            # Inferensi model
            outputs = self.session.run(None, {self.session.get_inputs()[0].name: img_input})
            output = outputs[0].transpose()  # (1029, 5)

            # Inisialisasi variabel
            obstacle_mask = np.zeros_like(masked_lane_frame, dtype=np.uint8)
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
                    cv2.rectangle(obstacle_mask, (x1, y1), (x2, y2), 255, thickness=cv2.FILLED)

            # Gabungkan mask dengan frame asli
            frame = cv2.addWeighted(frame, 0.5, obstacle_mask, 0.5, 0)

            # Konversi ke grayscale untuk findContours
            obstacle_mask_gray = cv2.cvtColor(obstacle_mask, cv2.COLOR_BGR2GRAY)
            contours, _ = cv2.findContours(obstacle_mask_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
            dashed_lines = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
            
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

            # Cari intersection dashed lines dengan look ahead circle
            center_x = int(frame.shape[1] / 2)
            center_y = int(frame.shape[0])
            intersection_found = False
            intersection_point = Point()
            
            for i in range(180):
                angle_rad = np.deg2rad(i)
                x = int(center_x + self.look_ahead_distance * np.cos(angle_rad))
                y = int(center_y - self.look_ahead_distance * np.sin(angle_rad))

                if 0 <= x < frame.shape[1] and 0 <= y < frame.shape[0]:
                    if dashed_lines[y, x] > 0:
                        cv2.circle(frame, (x, y), 5, (0, 255, 255), -1)
                        intersection_found = True
                        intersection_point.x = float(x)
                        intersection_point.y = float(y)
                        intersection_point.z = 0.0
                        break
            
            # Hitung dan tampilkan FPS
            end_time = time.time()
            fps = 1.0 / (end_time - start_time)
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Publish data
            # Publish dashed lines as image
            dashed_lines_msg = self.bridge.cv2_to_imgmsg(dashed_lines, "mono8")
            dashed_lines_msg.header = msg.header
            self.dashed_lines_pub.publish(dashed_lines_msg)
            
            # Publish intersection point
            intersection_point.header = msg.header
            self.intersection_point_pub.publish(intersection_point)
            
            # Publish intersection found status
            intersection_found_msg = Bool()
            intersection_found_msg.data = intersection_found
            self.intersection_found_pub.publish(intersection_found_msg)
            
            # Publish processed frame
            processed_frame_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            processed_frame_msg.header = msg.header
            self.processed_frame_pub.publish(processed_frame_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ONNXLaneFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
