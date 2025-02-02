import leap
import numpy as np
import cv2
import socket
import json

# Define Canvas class for visualizing Leap Motion hand skeleton
class Canvas:
    def __init__(self):
        # Initialize basic canvas properties
        self.name = "Leap Motion Skeleton Visualiser"  # Window name
        self.screen_size = [500, 700]  # Canvas dimensions (height, width)
        self.hands_colour = (255, 255, 255)  # Hand skeleton drawing color (white)
        self.font_colour = (0, 255, 44)  # Text color (green)
        # Initialize canvas image as black background
        self.output_image = np.zeros((self.screen_size[0], self.screen_size[1], 3), np.uint8)
        
        # Initialize UDP sender
        self.udp_ip = "127.0.0.1"  # Receiver IP address
        self.udp_port = 12345  # Receiver port number
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create UDP socket
        
    # Quaternion conjugate calculation
    def quaternion_conjugate(self, q):
        # Calculate quaternion conjugate, representing inverse rotation
        return [q[0], -q[1], -q[2], -q[3]]

    # Quaternion multiplication operation
    def quaternion_multiply(self, q1, q2):
        # Calculate product of two quaternions, used for composite rotations
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return [
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ]

    # Rotate vector using quaternion
    def quaternion_rotate_vector(self, q, v):
        # Convert vector to quaternion form for rotation
        v_quat = [0.0] + list(v)
        qv = self.quaternion_multiply(q, v_quat)
        rotated_v = self.quaternion_multiply(qv, self.quaternion_conjugate(q))
        return rotated_v[1:]  # Extract rotated vector component

    # Quaternion normalization
    def quaternion_normalize(self, q):
        # Normalize quaternion to ensure stable rotation calculations
        norm = np.linalg.norm(q)
        return [q_i / norm for q_i in q]

    # Get joint position and project to screen coordinates
    def get_joint_position(self, joint, hand_position, hand_orientation):
        if joint:
            # Get joint's 3D coordinates
            joint_position = (joint.x, joint.y, joint.z)
            # Convert joint coordinates to local coordinate system relative to palm
            v = np.array(joint_position) - np.array(hand_position)
            # Use quaternion to rotate local coordinates back to palm orientation
            q = self.quaternion_normalize(hand_orientation)
            v_rotated = self.quaternion_rotate_vector(q, v)
            # Project rotated 3D coordinates to 2D screen plane
            x_screen = int(v_rotated[0] + (self.screen_size[1] / 2))
            y_screen = int(v_rotated[2] + (self.screen_size[0] / 2))
            return x_screen, y_screen, v_rotated
        else:
            return None
    
    def calculate_angle_3D(self, vector1, vector2, reference_axis=[0, 0, 1]):
        # Convert to numpy arrays
        v1 = np.array(vector1)
        v2 = np.array(vector2)
        ref_axis = np.array(reference_axis)
        
        # Calculate dot product and vector magnitudes
        dot_product = np.dot(v1, v2)
        magnitude_v1 = np.linalg.norm(v1)
        magnitude_v2 = np.linalg.norm(v2)
        
        # Calculate angle (in radians)
        cos_theta = dot_product / (magnitude_v1 * magnitude_v2)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)
        angle_rad = np.arccos(cos_theta)
        
        # Calculate cross product
        cross_product = np.cross(v1, v2)
        # Determine sign based on dot product with reference axis
        sign = np.sign(np.dot(cross_product, ref_axis))
        
        # Convert to degrees and apply sign
        signed_angle = np.degrees(angle_rad) * sign
        
        return signed_angle
    
    # Calculate angle between two vectors
    def calculate_angle(self, vec1, vec2, reference_axis=[-1, 1, 0]): #[-1, 0, 0]
        vec1_norm = vec1 / np.linalg.norm(vec1)  # Normalize
        vec2_norm = vec2 / np.linalg.norm(vec2)
        dot_product = np.dot(vec1_norm, vec2_norm)  # Dot product
        dot_product = np.clip(dot_product, -1.0, 1.0)  # Prevent numerical instability
        angle_rad = np.arccos(dot_product)  # Calculate radians
        
        # Use cross product to determine sign
        cross_product = np.cross(vec1_norm, vec2_norm)  # Calculate cross product
        sign = np.dot(cross_product, reference_axis)  # Determine direction
        if sign < 0:
            angle_rad = -angle_rad  # Set negative angle if direction is opposite to reference axis

        angle_deg = np.degrees(angle_rad)  # Convert to degrees
        return angle_deg
    
    def send_angles_udp(self, angles):
        try:
            # Convert data to JSON format
            data = json.dumps(angles)
            # Send data
            self.sock.sendto(data.encode(), (self.udp_ip, self.udp_port))
        except Exception as e:
            print(f"Error sending angles: {e}")
    
    # Draw hand skeleton
    def render_hands(self, event):
        # Clear canvas
        self.output_image[:, :] = 0
        text_offset = 20  # Text display offset

        # Iterate through each hand's data
        for i in range(len(event.hands)):
            hand = event.hands[i]
            # Get palm position and quaternion orientation
            hand_position = (
                hand.palm.position.x,
                hand.palm.position.y,
                hand.palm.position.z
            )
            quaternion = [
                hand.palm.orientation[3],
                -hand.palm.orientation[0],
                -hand.palm.orientation[1],
                -hand.palm.orientation[2]
            ]

            fingers_angles = []  # Store angles for each finger

            # Iterate through fingers
            for index_digit in range(5):
                digit = hand.digits[index_digit]
                bone_vectors = []  # Store finger bone vectors
                finger_angles = []  # Store finger joint angles
                
                if index_digit == 0:
                    # Iterate through each bone segment of the finger
                    i = 1
                    for index_bone in range(4):
                        bone = digit.bones[index_bone]
                        # Get joint coordinates for bone start and end points
                        start_joint = bone.prev_joint
                        end_joint = bone.next_joint

                        # Get joint positions rotated to palm coordinate system
                        start_data = self.get_joint_position(start_joint, hand_position, quaternion)
                        end_data = self.get_joint_position(end_joint, hand_position, quaternion)
                        
                        if start_data and end_data:
                            # Get rotated 3D coordinates
                            start_xyz = np.array(start_data[2])
                            end_xyz = np.array(end_data[2])
                            # Calculate bone vector
                            if i == 1:
                                bone_vector = [0,0,-1]
                            else:
                                bone_vector = end_xyz - start_xyz
                            bone_vectors.append(bone_vector)
                            
                            # Draw bones and joints
                            start_x, start_y = start_data[0], start_data[1]
                            end_x, end_y = end_data[0], end_data[1]
                            cv2.line(self.output_image, (start_x, start_y), (end_x, end_y), self.hands_colour, 2)
                            cv2.circle(self.output_image, (end_x, end_y), 3, self.hands_colour, -1)
                        i = i + 1
                        
                    # Calculate angles between bones
                    for j in range(len(bone_vectors) - 1):
                        vec1 = bone_vectors[j]
                        vec2 = bone_vectors[j + 1]
                        # Calculate YZ plane angle (flexion/extension)
                        vec1_yz = vec1.copy()
                        vec1_yz[0] = 0  # Zero X component
                        vec2_yz = vec2.copy()
                        vec2_yz[0] = 0
                        
                        if j == 0:
                            angle_yz = self.calculate_angle(vec1_yz, vec2_yz)
                            finger_angles.append(angle_yz)
                        else:
                            angle_3D = self.calculate_angle_3D(vec1, vec2)
                            finger_angles.append(angle_3D)
                            
                        # Calculate XZ plane angle (abduction/adduction)
                        if j == 0:
                            vec1_xz = vec1.copy()
                            vec1_xz[1] = 0
                            vec2_xz = vec2.copy()
                            vec2_xz[1] = 0
                            angle_xz = self.calculate_angle(vec1_xz, vec2_xz)
                            finger_angles.append(angle_xz)
                            
                else:
                    # Iterate through each bone segment of the finger
                    for index_bone in range(4):
                        bone = digit.bones[index_bone]
                        # Get joint coordinates for bone start and end points
                        start_joint = bone.prev_joint
                        end_joint = bone.next_joint

                        # Get joint positions rotated to palm coordinate system
                        start_data = self.get_joint_position(start_joint, hand_position, quaternion)
                        end_data = self.get_joint_position(end_joint, hand_position, quaternion)

                        if start_data and end_data:
                            # Get rotated 3D coordinates
                            start_xyz = np.array(start_data[2])
                            end_xyz = np.array(end_data[2])
                            # Calculate bone vector
                            bone_vector = end_xyz - start_xyz
                            bone_vectors.append(bone_vector)

                            # Draw bones and joints
                            start_x, start_y = start_data[0], start_data[1]
                            end_x, end_y = end_data[0], end_data[1]
                            cv2.line(self.output_image, (start_x, start_y), (end_x, end_y), self.hands_colour, 2)
                            cv2.circle(self.output_image, (end_x, end_y), 3, self.hands_colour, -1)

                    # Calculate angles between bones
                    for j in range(len(bone_vectors) - 1):
                        vec1 = bone_vectors[j]
                        vec2 = bone_vectors[j + 1]
                        # Calculate YZ plane angle (flexion/extension)
                        vec1_yz = vec1.copy()
                        vec1_yz[0] = 0  # Zero X component
                        vec2_yz = vec2.copy()
                        vec2_yz[0] = 0
                        angle_yz = self.calculate_angle(vec1_yz, vec2_yz)
                        finger_angles.append(angle_yz)

                        # Calculate XZ plane angle (abduction/adduction)
                        if j == 0:
                            vec1_xz = vec1.copy()
                            vec1_xz[1] = 0
                            vec2_xz = vec2.copy()
                            vec2_xz[1] = 0
                            angle_xz = self.calculate_angle(vec1_xz, vec2_xz)
                            finger_angles.append(angle_xz)
                            
                fingers_angles.append(finger_angles)
                
            # Display finger angles on canvas right side
            reordered_fingers_angles = [
                [np.radians(row[0]), np.radians(row[1]), np.radians(row[2]), np.radians(row[3])]
                for row in fingers_angles
            ]
            
            # Send data
            self.send_angles_udp(reordered_fingers_angles)
            finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
            for idx, angles in enumerate(reordered_fingers_angles):
                angle_text = f"{finger_names[idx]} Angles: "
                angle_values = [f"{angle:6.2f}" for angle in angles]
                angle_text += ', '.join(angle_values)
                cv2.putText(
                    self.output_image,
                    angle_text,
                    (self.screen_size[1] - 600, text_offset),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    self.font_colour,
                    1
                )
                text_offset += 20

# Define tracking listener class
class TrackingListener(leap.Listener):
    def __init__(self, canvas):
        self.canvas = canvas

    def on_tracking_event(self, event):
        self.canvas.render_hands(event)  # Call canvas method to render hand skeleton

# Main program entry
def main():
    canvas = Canvas()  # Initialize canvas
    tracking_listener = TrackingListener(canvas)  # Create listener
    connection = leap.Connection()  # Create Leap Motion connection
    connection.add_listener(tracking_listener)  # Register listener

    # Open connection and set tracking mode
    with connection.open():
        connection.set_tracking_mode(leap.TrackingMode.Desktop)

        # Real-time skeleton display
        while True:
            cv2.imshow(canvas.name, canvas.output_image)  # Display canvas image
            if cv2.waitKey(1) == ord("x"):  # Exit program on 'x' key press
                break

if __name__ == "__main__":
    main()