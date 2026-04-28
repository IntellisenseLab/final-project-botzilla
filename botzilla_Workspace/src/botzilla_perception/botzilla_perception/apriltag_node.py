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

# Only track this specific tag ID as the drop-off zone marker
TARGET_TAG_ID = 0

# Normalized horizontal offset threshold: below this = tag is "centered enough"
CENTER_THRESHOLD = 0.07


class AprilTagNode(Node):
    def __init__(self):
        super().__init__('apriltag_node')

        self.bridge = cv_bridge.CvBridge()

        # ArUco detector setup
        self.aruco_dict = aruco.getPredefinedDictionary(APRILTAG_DICT)
        self.aruco_params = aruco.DetectorParameters()
        # Older OpenCV versions (pre-4.7) do not have ArucoDetector class.
        # We will use the legacy aruco.detectMarkers() function below.

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
            f'AprilTag Node started. Tracking Tag ID={TARGET_TAG_ID} '
            f'(family: DICT_APRILTAG_36h11).'
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            img_h, img_w = cv_image.shape[:2]
            img_center_x = img_w / 2.0

            # Detect all AprilTag markers using legacy API (OpenCV < 4.7)
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            visible_msg = Bool()
            found = False

            if ids is not None:
                aruco.drawDetectedMarkers(cv_image, corners, ids)

                for i, tag_id in enumerate(ids.flatten()):
                    if tag_id == TARGET_TAG_ID:
                        found = True

                        # Get the center of the detected tag
                        tag_corners = corners[i][0]  # shape (4, 2)
                        cx = int(np.mean(tag_corners[:, 0]))
                        cy = int(np.mean(tag_corners[:, 1]))

                        # Normalized horizontal offset: -1.0 (left) to +1.0 (right)
                        norm_x = (cx - img_center_x) / img_center_x

                        # Publish position for brain steering
                        pos_msg = Point()
                        pos_msg.x = norm_x
                        pos_msg.y = float(cy)  # raw pixel height (can use for alignment)
                        pos_msg.z = 0.0
                        self.drop_off_pos_pub.publish(pos_msg)

                        # Draw debug overlay
                        cv2.circle(cv_image, (cx, cy), 8, (0, 0, 255), -1)
                        label = f"ID:{tag_id}  x={norm_x:.2f}"
                        cv2.putText(cv_image, label, (cx - 40, cy - 14),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                        self.get_logger().debug(
                            f'Drop-off tag found: id={tag_id}, norm_x={norm_x:.3f}'
                        )
                        break  # Only track the first matching tag

            visible_msg.data = found
            self.drop_off_visible_pub.publish(visible_msg)

            # Publish the debug view
            ros_debug = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.debug_pub.publish(ros_debug)

            # cv2.imshow('AprilTag Debug View', cv_image)
            # cv2.waitKey(1)

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
