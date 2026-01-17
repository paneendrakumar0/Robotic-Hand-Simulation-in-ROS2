import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import cv2
import mediapipe as mp
import numpy as np
import math
import time


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
        if t_e <= 0.0: return self.x_prev

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
        super().__init__('advanced_hand_tracker')
        
        # Publishers
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

   
        self.joint_names = [
            'R_Index_Pitch', 'R_Middle_Pitch', 'R_Ring_Pitch', 'R_Pinky_Pitch',
            'R_Index_Flexor', 'R_Middle_Flexor', 'R_Ring_Flexor', 'R_Pinky_Flexor',
            'R_Index_DIP', 'R_Middle_DIP', 'R_Ring_DIP', 'R_Pinky_DIP',
            'R_Thumb_Yaw', 'R_Thumb_Roll', 'R_Thumb_Flexor', 'R_Thumb_DIP',
            'R_Index_Yaw', 'R_Middle_Yaw', 'R_Ring_Yaw', 'R_Pinky_Yaw', 'R_Thumb_Pitch'
        ]

    
        min_cutoff = 0.5 
        beta = 0.5 
        
     
        self.filters = []
        t0 = time.time()
        for _ in range(21):
            self.filters.append(OneEuroFilter(t0, 0.0, min_cutoff=min_cutoff, beta=beta))
            
      
        self.wrist_filters = [
            OneEuroFilter(t0, 0.0, min_cutoff=0.1, beta=0.1), # X (Very smooth)
            OneEuroFilter(t0, 0.0, min_cutoff=0.1, beta=0.1), # Y
            OneEuroFilter(t0, 0.0, min_cutoff=0.1, beta=0.1), # Z
            OneEuroFilter(t0, 0.0, min_cutoff=1.0, beta=0.5)  # Roll (Responsive)
        ]

    
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = mp.solutions.drawing_utils
        
   
        self.cap = cv2.VideoCapture(0) 
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info("1-Euro Filter Controller Started")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret: return
        
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        
        target_pos = [0.0] * 21
        curr_time = time.time()
        
        if results.multi_hand_landmarks:
            for hand_lm in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_lm, self.mp_hands.HAND_CONNECTIONS)
                
              
                self.process_wrist(hand_lm, curr_time)

            
                lm = hand_lm.landmark
                def get_curl(tip, wrist):
                    dist = math.sqrt((lm[tip].x - lm[wrist].x)**2 + (lm[tip].y - lm[wrist].y)**2)
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

       
        filtered_pos = []
        for i in range(21):
            
            val = self.filters[i](curr_time, target_pos[i])
            filtered_pos.append(val)

     
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = filtered_pos
        self.publisher_.publish(msg)
        
        cv2.imshow("1-Euro Smoothed View", frame)
        cv2.waitKey(1)

    def process_wrist(self, landmarks, t):
        lm = landmarks.landmark
        
      
        raw_y = (0.5 - lm[0].x) * 1.0  # Left/Right
        raw_z = (0.5 - lm[0].y) * 1.0 + 0.2 # Up/Down
        
       
        dx_side = lm[5].x - lm[17].x
        dy_side = lm[5].y - lm[17].y
        raw_roll = -math.atan2(dy_side, dx_side)

       
        f_x = self.wrist_filters[0](t, 0.0) # Keep X locked
        f_y = self.wrist_filters[1](t, raw_y)
        f_z = self.wrist_filters[2](t, raw_z)
        f_roll = self.wrist_filters[3](t, raw_roll)

    
        t_tf = TransformStamped()
        t_tf.header.stamp = self.get_clock().now().to_msg()
        t_tf.header.frame_id = 'world'
        t_tf.child_frame_id = 'base_link'
        
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
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
