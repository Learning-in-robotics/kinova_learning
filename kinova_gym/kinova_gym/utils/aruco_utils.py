import cv2
from cv2 import aruco
import numpy as np

import threading
import pynput.keyboard
import time
import yaml

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

from tf.transformations import quaternion_from_euler

from scipy.spatial.transform import Rotation as R


class ArucoDetector:
    def __init__(
        self,
        aruco_dict=aruco.DICT_4X4_100,
        marker_length=0.1,
        marker_id=0,
        debug=False,
    ):
        self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict)
        self.marker_length = marker_length
        self.marker_id = marker_id
        self.parameters = aruco.DetectorParameters()

        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_size = None

        self.debug = debug

    def subscribe(self):
        self.cam_info_sub = rospy.Subscriber(
            "/camera/color/camera_info", CameraInfo, self.cam_info_callback
        )
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, self.image_callback
        )

    def cam_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.image_size = (msg.width, msg.height)

        self.detect(image)

    def update_intrinsic_cam_info(self) -> bool:
        camera_info_msg = rospy.wait_for_message(
            "/camera/color/camera_info", CameraInfo
        )
        if camera_info_msg is None:
            rospy.logerr("No camera info message received")
            return False
        self.camera_matrix = np.array(camera_info_msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(camera_info_msg.D)
        return True

    def get_box_pose_in_cam(self):
        # get the latest image
        image_msg = rospy.wait_for_message("/camera/color/image_raw", Image)

        if image_msg is None:
            rospy.logerr("No image message received")
            return None, None

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        self.image_size = (image_msg.width, image_msg.height)

        # detect the marker
        translation_vector, euler_angles_degrees, img = self.detect(image)

        if translation_vector is None or euler_angles_degrees is None:
            return None, None

        # create posestamped message
        # pose_msg = PoseStamped()
        # pose_msg.header.stamp = image_msg.header.stamp
        # pose_msg.header.frame_id = "camera_color_optical_frame"
        # pose_msg.pose.position.x = translation_vector[0][0]
        # pose_msg.pose.position.y = translation_vector[1][0]
        # pose_msg.pose.position.z = translation_vector[2][0]

        euler_angles_rad = np.radians(euler_angles_degrees).tolist()

        # convert euler angles to quaternion
        # quaternion = quaternion_from_euler(
        #     euler_angles_rad[0], euler_angles_rad[1], euler_angles_rad[2]
        # )

        # pose_msg.pose.orientation.x = quaternion[0]
        # pose_msg.pose.orientation.y = quaternion[1]
        # pose_msg.pose.orientation.z = quaternion[2]
        # pose_msg.pose.orientation.w = quaternion[3]

        return [ translation_vector[0][0],  translation_vector[1][0], translation_vector[2][0],
                 euler_angles_rad[0][0],euler_angles_rad[1][0],euler_angles_rad[2][0]], img

    def detect(self, image):
        # detect single marker and return its pose
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            corners, ids, rejectedImgPoints = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.parameters
            )
            if ids is None or len(ids) == 0 or self.marker_id not in ids:
                return None, None, None

            # estimate pose of each marker and return the values
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                corners[self.marker_id],
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs,
            )

            translation_vector = tvec.reshape((3, 1))

            # convert to real-world coordinate system
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            P = np.hstack((rotation_matrix, translation_vector))

            euler_angles_degrees = cv2.decomposeProjectionMatrix(P)[6]

            # if self.debug:
            annottated_image = self.draw_marker(
                image,
                corners,
                rvec,
                tvec,
                translation_vector,
                euler_angles_degrees,
            )

            return translation_vector, euler_angles_degrees, annottated_image

        except Exception as e:
            rospy.logerr(e)
            return None, None, None

    def get_camera_mount_pose_in_marker_frame(
        self, rotation_matrix, translation_vector
    ):
        # get the camera position in the marker frame

        # transformation matrix
        T = np.zeros((4, 4))
        T[:3, :3] = rotation_matrix
        T[:3, 3] = translation_vector.reshape((3,))
        T[3, 3] = 1

        # inverse transformation matrix
        T_inv = np.linalg.inv(T)

        # camera position in the marker frame
        camera_position_in_marker_frame = T_inv[:3, 3]

        camera_otientation_in_marker_frame = T_inv[:3, :3]

        camera_orient_deg = cv2.decomposeProjectionMatrix(
            np.hstack((camera_otientation_in_marker_frame, np.zeros((3, 1))))
        )[6]

        print(
            "Camera ori in marker frame: {}".format(
                np.deg2rad(camera_orient_deg)
            )
        )

        # camera mount position in the marker frame
        camera_mount_position_in_marker_frame = camera_position_in_marker_frame

        # Provided relative orientation of camera wrt camera mount
        relative_orientation = [0.491, -0.497, 0.502, 0.510]

        # convert relative orientation to rotation vector
        relative_orientation = R.from_quat(relative_orientation).as_rotvec()

        # camera mount orientation in the marker frame
        camera_rel_orent_matrix = cv2.Rodrigues(relative_orientation)[0]

        # camera mount rotation matrix
        camera_mount_rotation_matrix = (
            camera_otientation_in_marker_frame @ camera_rel_orent_matrix
        )

        # camera mount euler angles
        camera_mount_euler_angles = cv2.decomposeProjectionMatrix(
            np.hstack(
                (
                    camera_mount_rotation_matrix,
                    camera_mount_position_in_marker_frame.reshape((3, 1)),
                )
            )
        )[6]

        print(
            "Camera mount position in marker frame: {}".format(
                camera_mount_position_in_marker_frame
            )
        )
        print(
            "Camera mount orientation in marker frame: {}".format(
                np.deg2rad(camera_mount_euler_angles)
            )
        )

    def draw_marker(
        self,
        image,
        corners,
        rvec,
        tvec,
        translation_vector,
        euler_angles_degrees,
    ):
        aruco.drawDetectedMarkers(image, corners)
        cv2.drawFrameAxes(
            image,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            self.marker_length,
        )

        # annotate the image with the marker's translation and rotation
        translation_text = "Translation: x={:.2f}, y={:.2f}, z={:.2f}".format(
            translation_vector[0][0],
            translation_vector[1][0],
            translation_vector[2][0],
        )
        rotation_text = "Rotation: x={:.2f}, y={:.2f}, z={:.2f}".format(
            euler_angles_degrees[0][0],
            euler_angles_degrees[1][0],
            euler_angles_degrees[2][0],
        )

        def draw_text(text, x, y):
            cv2.putText(
                image,
                text,
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        draw_text(translation_text, 10, 30)
        draw_text(rotation_text, 10, 60)

        return image

stop_thread = False

def on_press(key):
    global stop_thread
    if key == pynput.keyboard.Key.esc:
        stop_thread = True

if __name__ == "__main__":
    rospy.init_node("aruco_detector", anonymous=True)
    aruco_detector = ArucoDetector(debug=True)
    aruco_detector.update_intrinsic_cam_info()

    data= []

    listener = pynput.keyboard.Listener(on_press=on_press)
    listener.start()

    def recorded_poses():
        global stop_thread
        while not rospy.is_shutdown() and True:
            pose, img = aruco_detector.get_box_pose_in_cam()
            if pose is not None:
                data.append(pose)
            if stop_thread:
                    break
            rospy.sleep(0.1)

    t = threading.Thread(target=recorded_poses)
    t.start()
    t.join()

    # print(data)
    yaml_data = {"trajectory": np.asarray(data).tolist()}
    with open("recorded_poses.yaml", "w") as yaml_file:
        yaml.dump(yaml_data, yaml_file)

