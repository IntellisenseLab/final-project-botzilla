import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
import cv_bridge
import cv2
import cv2.aruco as aruco
import numpy as np

# --- CONFIGURATION ---
# AprilTag dictionary: DICT_APRILTAG_36h11 is the standard default
APRILTAG_DICT = aruco.DICT_APRILTAG_36h11

# Only track these specific tag IDs as the drop-off zone markers
TARGET_TAG_IDS = [203, 113]

# Normalized horizontal offset threshold: below this = tag is "centered enough"
CENTER_THRESHOLD = 0.07


class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')

        self.bridge = cv_bridge.CvBridge()

        # ArUco detector setup - Search multiple common AprilTag families
        self.families = [
            aruco.DICT_APRILTAG_36h11,
            aruco.DICT_APRILTAG_25h9,
            aruco.DICT_APRILTAG_16h5,
            aruco.DICT_6X6_250  # Just in case they used a standard ArUco app
        ]
        self.dicts = [aruco.getPredefinedDictionary(f) for f in self.families]
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.aruco_params.minDistanceToBorder = 3

        # Subscribe to Kinect RGB stream
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publish whether the target drop-off tag is visible (simple bool for state machine)
        self.drop_off_visible_pub = self.create_publisher(Bool, 'drop_off_visible', 10)

        # Publish angular correction for brain_node to steer toward the tag
        # x = normalized horizontal offset of tag (-1.0 left, +1.0 right, 0.0 = centered)
        # z = 0.0 (distance not needed here, brain drives at a fixed speed)
        self.drop_off_pos_pub = self.create_publisher(Point, 'drop_off_pose', 10)

        # Debug annotated image for rqt
        self.debug_pub = self.create_publisher(Image, '/perception/apriltag_image', 10)

        self.get_logger().info(
            f'AprilTag Node started. Tracking Tag ID={TARGET_TAG_IDS} '
            f'(family: DICT_APRILTAG_36h11).'
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            img_h, img_w = cv_image.shape[:2]
            img_center_x = img_w / 2.0

            # Try detection on normal AND mirrored image
            variations = [("Normal", gray), ("Mirrored", cv2.flip(gray, 1))]
            found = False

            for label, img_to_check in variations:
                for i, aruco_dict in enumerate(self.dicts):
                    corners, ids, _ = aruco.detectMarkers(img_to_check, aruco_dict, parameters=self.aruco_params)
                
                    if ids is not None:
                        detected_ids = ids.flatten().tolist()
                        self.get_logger().info(f"[{label}] Found Tags: {detected_ids} in family {self.families[i]}", throttle_duration_sec=1.0)
                        
                        aruco.drawDetectedMarkers(cv_image, corners, ids)
                        for idx, tag_id in enumerate(detected_ids):
                            if tag_id in TARGET_TAG_IDS:
                                found = True
                                # Get the center of the detected tag
                                tag_corners = corners[idx][0]
                                cx = int(np.mean(tag_corners[:, 0]))
                                cy = int(np.mean(tag_corners[:, 1]))
                                
                                # If mirrored, we must un-mirror the cx coordinate
                                if label == "Mirrored":
                                    cx = img_w - cx
                                    self.get_logger().warn("MIRROR EFFECT DETECTED! Un-flipping coordinates.")

                                # Normalized horizontal offset: -1.0 (left) to +1.0 (right)
                                norm_x = (cx - img_center_x) / img_center_x
                                pos_msg = Point()
                                pos_msg.x = norm_x
                                pos_msg.y = float(cy)
                                pos_msg.z = 0.0
                                self.drop_off_pos_pub.publish(pos_msg)

                                # Draw debug overlay
                                cv2.circle(cv_image, (cx, cy), 8, (0, 0, 255), -1)
                                label_text = f"ID:{tag_id}  x={norm_x:.2f}"
                                cv2.putText(cv_image, label_text, (cx - 40, cy - 14),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                break
                    if found: break
                if found: break

            visible_msg = Bool()
            visible_msg.data = found
            self.drop_off_visible_pub.publish(visible_msg)

            # Publish the debug view
            ros_debug = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.debug_pub.publish(ros_debug)

            cv2.imshow('AprilTag Debug View', cv_image)
            cv2.waitKey(1)

        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'AprilTag detection error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
