import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
            "waist_yaw",
            "neck_yaw",
            "neck_pitch",
            "neck_roll",
            "jaw_pitch",
            "r_shoulder_yaw",
            "r_shoulder_pitch",
            "r_elbow_pitch",
            "r_wrist_roll",
            "l_shoulder_yaw",
            "l_shoulder_pitch",
            "l_elbow_pitch",
            "l_wrist_roll"
        ]

        min_cutoff = 0.01  # Drastically lowered to fix jitter
        beta = 0.05        # Drastically lowered to prevent jitter on fast movements

        self.filters = []
        t0 = time.time()
        for _ in range(34):
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

                self.mp_holistic = mp.solutions.holistic
                self.holistic = self.mp_holistic.Holistic(
                    min_detection_confidence=0.5, min_tracking_confidence=0.5
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
        target_pos = [0.0] * 34

        # Default arm positions
        target_pos[21:34] = [0.0] * 13

        if self.mode == "camera" and self.cap is not None:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warning("Failed to read frame from camera")
                return

            frame = cv2.flip(frame, 1)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.holistic.process(rgb)

            if results.right_hand_landmarks:
                if not self.headless:
                    self.mp_draw.draw_landmarks(
                        frame, results.right_hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS
                    )

                lm = results.right_hand_landmarks.landmark

                # Compute hand curls
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
                
            if results.pose_landmarks:
                if not self.headless:
                    self.mp_draw.draw_landmarks(
                        frame, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS
                    )
                
                plm = results.pose_landmarks.landmark
                # Pose landmarks: 0=Nose, 7=L_Ear, 8=R_Ear, 11=L_Shoulder, 12=R_Shoulder
                # 13=L_Elbow, 14=R_Elbow, 15=L_Wrist, 16=R_Wrist, 23=L_Hip, 24=R_Hip
                r_shoulder, l_shoulder = plm[12], plm[11]
                r_elbow, l_elbow = plm[14], plm[13]
                r_wrist, l_wrist = plm[16], plm[15]
                nose = plm[0]
                r_ear = plm[8]
                l_ear = plm[7]
                
                # Waist/Torso
                waist_yaw = (r_shoulder.z - l_shoulder.z) * -2.0
                target_pos[21] = waist_yaw

                # Neck (3-DOF)
                # Yaw: Ear depth difference
                neck_yaw = (r_ear.z - l_ear.z) * -2.5
                target_pos[22] = neck_yaw

                # Pitch: Nose height vs shoulders
                neck_pitch = ((r_shoulder.y + l_shoulder.y)/2.0 - nose.y - 0.2) * -3.0
                target_pos[23] = neck_pitch
                
                # Roll: Ear height difference
                neck_roll = (r_ear.y - l_ear.y) * 2.0
                target_pos[24] = neck_roll
                
                # Right Arm
                r_dx_shoulder = r_elbow.x - r_shoulder.x
                r_dy_shoulder = r_elbow.y - r_shoulder.y
                target_pos[26] = -r_dx_shoulder * 3.0 # r_shoulder_yaw
                target_pos[27] = (r_dy_shoulder - 0.5) * 2.0 # r_shoulder_pitch
                
                r_dx_elbow = r_wrist.x - r_elbow.x
                r_dy_elbow = r_wrist.y - r_elbow.y
                target_pos[28] = math.atan2(r_dx_elbow, r_dy_elbow) + 1.57 # r_elbow_pitch
                target_pos[29] = 0.0 # r_wrist_roll
                
                # Left Arm
                l_dx_shoulder = l_elbow.x - l_shoulder.x
                l_dy_shoulder = l_elbow.y - l_shoulder.y
                target_pos[30] = l_dx_shoulder * 3.0 # l_shoulder_yaw
                target_pos[31] = (l_dy_shoulder - 0.5) * 2.0 # l_shoulder_pitch
                
                l_dx_elbow = l_wrist.x - l_elbow.x
                l_dy_elbow = l_wrist.y - l_elbow.y
                target_pos[32] = math.atan2(-l_dx_elbow, l_dy_elbow) + 1.57 # l_elbow_pitch
                target_pos[33] = 0.0 # l_wrist_roll

            if results.face_landmarks:
                flm = results.face_landmarks.landmark
                # Mouth vertical opening: 13 (upper lip inner) vs 14 (lower lip inner)
                mouth_open = flm[14].y - flm[13].y
                
                # Map to jaw joint: 0 to -0.5
                jaw_val = np.interp(mouth_open, [0.005, 0.05], [0.0, -0.5])
                target_pos[25] = jaw_val

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
            
            # Demo full body motion
            target_pos[21] = 0.3 * math.sin(curr_time * 0.3) # Waist
            target_pos[22] = 0.4 * math.sin(curr_time * 0.5) # Neck Yaw
            target_pos[23] = 0.2 * math.cos(curr_time * 0.4) # Neck Pitch
            target_pos[24] = 0.2 * math.sin(curr_time * 0.4) # Neck Roll
            target_pos[25] = -0.25 + 0.25 * math.sin(curr_time * 2.0) # Jaw
            
            # Right arm
            target_pos[26] = 0.5 * math.sin(curr_time * 0.5) 
            target_pos[27] = -0.5 * math.cos(curr_time * 0.5) 
            target_pos[28] = 0.5 + 0.5 * math.sin(curr_time)
            target_pos[29] = roll_val
            
            # Left arm
            target_pos[30] = -0.5 * math.sin(curr_time * 0.5) 
            target_pos[31] = -0.5 * math.cos(curr_time * 0.5 + 1.0) 
            target_pos[32] = 0.5 + 0.5 * math.cos(curr_time)
            target_pos[33] = -roll_val

        # Apply 1-Euro filters to joint positions
        filtered_pos = []
        for i in range(34):
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

        pass


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
