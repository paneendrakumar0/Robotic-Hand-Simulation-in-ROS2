import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import math
import time

# Lazy imports for camera mode dependencies
cv2 = None
mp = None


class OneEuroFilter:
    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = 1.0
        self.x_prev = float(x0)
        self.dx_prev = float(dx0)
        self.t_prev = float(t0)

    def smoothing_factor(self, t_e, cutoff):
        r = 2 * math.pi * cutoff * t_e
        return r / (r + 1)

    def exponential_smoothing(self, a, x, x_prev):
        return a * x + (1 - a) * x_prev

    def __call__(self, t, x):

        t_e = t - self.t_prev

        # Prevent division by zero or negative time
        if t_e <= 0.0:
            return self.x_prev

        # Calculate the jitter (derivative)
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)

        # Calculate the cutoff frequency based on speed
        # This is the magic: High speed = High cutoff (Low latency)
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self.smoothing_factor(t_e, cutoff)

        # Filter the signal
        x_hat = self.exponential_smoothing(a, x, self.x_prev)

        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat


class AdvancedHandTracker(Node):
    def __init__(self):
        super().__init__("advanced_hand_tracker")

        # Declare parameters
        self.declare_parameter("mode", "camera")
        self.declare_parameter("headless", False)
        self.declare_parameter("gazebo", False)

        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.headless = self.get_parameter("headless").get_parameter_value().bool_value
        self.gazebo = self.get_parameter("gazebo").get_parameter_value().bool_value

        # Publishers
        self.publisher_ = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.gazebo:
            self.trajectory_pub = self.create_publisher(
                JointTrajectory, "/set_joint_trajectory", 10
            )

        self.joint_names = [
            "R_Index_Pitch",
            "R_Middle_Pitch",
            "R_Ring_Pitch",
            "R_Pinky_Pitch",
            "R_Index_Flexor",
            "R_Middle_Flexor",
            "R_Ring_Flexor",
            "R_Pinky_Flexor",
            "R_Index_DIP",
            "R_Middle_DIP",
            "R_Ring_DIP",
            "R_Pinky_DIP",
            "R_Thumb_Yaw",
            "R_Thumb_Roll",
            "R_Thumb_Flexor",
            "R_Thumb_DIP",
            "R_Index_Yaw",
            "R_Middle_Yaw",
            "R_Ring_Yaw",
            "R_Pinky_Yaw",
            "R_Thumb_Pitch",
        ]

        min_cutoff = 0.5
        beta = 0.5

        self.filters = []
        t0 = time.time()
        for _ in range(21):
            self.filters.append(
                OneEuroFilter(t0, 0.0, min_cutoff=min_cutoff, beta=beta)
            )

        self.wrist_filters = [
            OneEuroFilter(t0, 0.0, min_cutoff=0.1, beta=0.1),  # X (Very smooth)
            OneEuroFilter(t0, 0.0, min_cutoff=0.1, beta=0.1),  # Y
            OneEuroFilter(t0, 0.0, min_cutoff=0.1, beta=0.1),  # Z
            OneEuroFilter(t0, 0.0, min_cutoff=1.0, beta=0.5),  # Roll (Responsive)
        ]

        self.get_logger().info(f"Target execution mode: {self.mode.upper()}")

        # Initialize camera and MediaPipe if in camera mode
        self.cap = None
        if self.mode == "camera":
            self.get_logger().info("Initializing camera and MediaPipe...")
            try:
                global cv2, mp
                import cv2
                import mediapipe as mp

                self.mp_hands = mp.solutions.hands
                self.hands = self.mp_hands.Hands(
                    max_num_hands=1, min_detection_confidence=0.7
                )
                self.mp_draw = mp.solutions.drawing_utils

                self.cap = cv2.VideoCapture(0)
                if not self.cap or not self.cap.isOpened():
                    self.get_logger().error(
                        "Could not open camera device 0. Falling back to DEMO mode."
                    )
                    self.mode = "demo"
            except Exception as e:
                self.get_logger().error(
                    f"Error during camera/MediaPipe init: {e}. "
                    "Falling back to DEMO mode."
                )
                self.mode = "demo"

        if self.mode == "demo":
            self.get_logger().info(
                "Running in high-fidelity DEMO mode. Predefined motion loop active."
            )

        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("Advanced Hand Tracker Node Started")

    def timer_callback(self):
        curr_time = time.time()
        target_pos = [0.0] * 21

        # Default neutral wrist position
        y_val = 0.0
        z_val = 0.2
        roll_val = 0.0

        if self.mode == "camera" and self.cap is not None:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warning("Failed to read frame from camera")
                return

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.hands.process(rgb)

            if results.multi_hand_landmarks:
                for hand_lm in results.multi_hand_landmarks:
                    if not self.headless:
                        self.mp_draw.draw_landmarks(
                            frame, hand_lm, self.mp_hands.HAND_CONNECTIONS
                        )

                    lm = hand_lm.landmark

                    # Compute raw wrist values
                    y_val = (0.5 - lm[0].x) * 1.0  # Left/Right
                    z_val = (0.5 - lm[0].y) * 1.0 + 0.2  # Up/Down

                    dx_side = lm[5].x - lm[17].x
                    dy_side = lm[5].y - lm[17].y
                    roll_val = -math.atan2(dy_side, dx_side)

                    # Compute curls
                    def get_curl(tip, wrist):
                        dist = math.sqrt(
                            (lm[tip].x - lm[wrist].x) ** 2
                            + (lm[tip].y - lm[wrist].y) ** 2
                        )
                        return np.interp(dist, [0.15, 0.4], [1.5, 0.0])

                    idx = get_curl(8, 0)
                    mid = get_curl(12, 0)
                    rng = get_curl(16, 0)
                    pnk = get_curl(20, 0)
                    thm = get_curl(4, 0)

                    target_pos[0], target_pos[4], target_pos[8] = idx, idx, idx
                    target_pos[1], target_pos[5], target_pos[9] = mid, mid, mid
                    target_pos[2], target_pos[6], target_pos[10] = rng, rng, rng
                    target_pos[3], target_pos[7], target_pos[11] = pnk, pnk, pnk
                    target_pos[14], target_pos[15], target_pos[20] = thm, thm, thm
                    target_pos[13] = thm * 0.5

            if not self.headless:
                try:
                    cv2.imshow("1-Euro Smoothed View", frame)
                    cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().warning(
                        f"Could not show cv2 window: {e}. Switching to headless."
                    )
                    self.headless = True

        elif self.mode == "demo":
            # 20 second demo cycle
            t_cycle = curr_time % 20.0

            # Initial states
            idx = mid = rng = pnk = thm = 0.0
            roll_val = 0.4 * math.sin(curr_time * 1.0)
            y_val = 0.1 * math.sin(curr_time * 0.5)
            z_val = 0.2 + 0.05 * math.cos(curr_time * 0.5)

            if t_cycle < 4.0:
                # Phase 1: Fist Clench (0.0s to 4.0s)
                # Smooth sinusoidal squeeze and release
                s = 0.5 - 0.5 * math.cos(2.0 * math.pi * t_cycle / 4.0)
                curl = s * 1.5
                idx = mid = rng = pnk = thm = curl
                roll_val = 0.0
                y_val = 0.0
                z_val = 0.2
            elif t_cycle < 9.0:
                # Phase 2: Waving / Side Roll Greeting (4.0s to 9.0s)
                # Fingers relaxed, hand waves side-to-side dynamically
                idx = mid = rng = pnk = 0.3
                thm = 0.2
                roll_val = 0.6 * math.sin(2.0 * math.pi * (t_cycle - 4.0) / 2.5)
                y_val = 0.15 * math.sin(2.0 * math.pi * (t_cycle - 4.0) / 2.5)
                z_val = 0.2
            elif t_cycle < 15.0:
                # Phase 3: Counting 1 to 5 (9.0s to 15.0s)
                t_count = t_cycle - 9.0
                # Start all closed
                idx = mid = rng = pnk = thm = 1.5
                if t_count > 0.0:
                    idx = max(0.0, 1.5 - t_count * 1.5)
                if t_count > 1.0:
                    mid = max(0.0, 1.5 - (t_count - 1.0) * 1.5)
                if t_count > 2.0:
                    rng = max(0.0, 1.5 - (t_count - 2.0) * 1.5)
                if t_count > 3.0:
                    pnk = max(0.0, 1.5 - (t_count - 3.0) * 1.5)
                if t_count > 4.0:
                    thm = max(0.0, 1.5 - (t_count - 4.0) * 1.5)

                roll_val = -0.2
                y_val = -0.05
                z_val = 0.2
            else:
                # Phase 4: Thumbs Up (15.0s to 20.0s)
                idx = mid = rng = pnk = 1.5
                thm = 0.0
                roll_val = 0.8  # rotated thumbs-up pose
                y_val = 0.05
                z_val = 0.2 + 0.05 * math.sin(2.0 * math.pi * (t_cycle - 15.0) / 2.5)

            # Apply mapping to target positions
            target_pos[0], target_pos[4], target_pos[8] = idx, idx, idx
            target_pos[1], target_pos[5], target_pos[9] = mid, mid, mid
            target_pos[2], target_pos[6], target_pos[10] = rng, rng, rng
            target_pos[3], target_pos[7], target_pos[11] = pnk, pnk, pnk
            target_pos[14], target_pos[15], target_pos[20] = thm, thm, thm
            target_pos[13] = thm * 0.5

        # Apply 1-Euro filters to joint positions
        filtered_pos = []
        for i in range(21):
            val = self.filters[i](curr_time, target_pos[i])
            filtered_pos.append(val)

        # Publish joint states / command Gazebo
        if self.gazebo:
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = filtered_pos
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 33000000  # 33ms target interpolation

            traj_msg.points = [point]
            self.trajectory_pub.publish(traj_msg)
        else:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = filtered_pos
            self.publisher_.publish(msg)

        # Broadcast wrist transformation TF
        self.process_wrist(curr_time, y_val, z_val, roll_val)

    def process_wrist(self, t, raw_y, raw_z, raw_roll):
        # Filter wrist translation and rotation
        f_x = self.wrist_filters[0](t, 0.0)  # Keep X locked
        f_y = self.wrist_filters[1](t, raw_y)
        f_z = self.wrist_filters[2](t, raw_z)
        f_roll = self.wrist_filters[3](t, raw_roll)

        t_tf = TransformStamped()
        t_tf.header.stamp = self.get_clock().now().to_msg()
        t_tf.header.frame_id = "world"
        t_tf.child_frame_id = "base_link"

        t_tf.transform.translation.x = f_x
        t_tf.transform.translation.y = f_y
        t_tf.transform.translation.z = f_z

        t_tf.transform.rotation.x = 0.0
        t_tf.transform.rotation.y = 0.0
        t_tf.transform.rotation.z = math.sin(f_roll / 2)
        t_tf.transform.rotation.w = math.cos(f_roll / 2)

        self.tf_broadcaster.sendTransform(t_tf)


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedHandTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()
        if cv2 is not None:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
