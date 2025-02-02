from CPS import CPSClient
import socket
import time
import keyboard  # For key detection

# Define IP and port
IP = '192.168.0.114'
nPort = 10003

# Create CPSClient object
cps = CPSClient()

# Connect to controller
nRet = cps.HRIF_Connect(0, IP, nPort)
print(nRet)

# Define speed ratio
dOverride = 0.1
# Set current speed ratio
nRet = cps.HRIF_SetOverride(0, 0, dOverride)

# Define maximum joint speed
#allJointMaxVel = 180 * 0.3
#Joint = [allJointMaxVel] * 6
Joint = [5, 5, 5, 5, 5, 5]

# Set maximum joint speed
nRet = cps.HRIF_SetJointMaxVel(0, 0, Joint)

# Servo time and lookahead time
dServoTime = 0.02
dLookaheadTime = 0.2
# Start online control
nRet = cps.HRIF_StartServo(0, 0, dServoTime, dLookaheadTime)

def limit_position_and_orientation(x, y, z, rx, ry, rz):
    # Limit x, y, z position
    x = max(-300, min(300, x))
    y = max(-650, min(-380, y))
    z = max(300, min(500, z))
    
    # Limit rotation angles
    rx = max(-315, min(-225, rx))
    ry = max(-180, min(180, ry))
    rz = max(-45, min(45, rz))
    
    return x, y, z, rx, ry, rz

def main():
    # Set local IP address and port
    udp_ip = "127.0.0.1"
    udp_port = 5005

    # Create UDP socket, bind, and set to non-blocking mode
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((udp_ip, udp_port))
    udp_socket.setblocking(0)  # Set to non-blocking

    print("Listening for data on {}:{}...".format(udp_ip, udp_port))
    print("Press 'q' to exit.")

    while True:
        # Check if 'q' is pressed for exit
        if keyboard.is_pressed('q'):
            print("Exiting program...")
            break

        try:
            # Try receiving data
            data, addr = udp_socket.recvfrom(1024)
            message = data.decode()

            # Parse string as position and orientation information (x, y, z, rx, ry, rz)
            try:
                x, y, z, rx, ry, rz, rx2, ry2, rz2 = map(float, message.split(','))

                arm_x = -x
                arm_y = z - 380
                arm_z = y
                
                arm_rx = -rx2 - 270
                arm_ry = rx
                arm_rz = rz
                
                print((arm_x, arm_y, arm_z, arm_rx, arm_ry, arm_rz))

                # Limit both position and rotation
                arm_x, arm_y, arm_z, arm_rx, arm_ry, arm_rz = limit_position_and_orientation(arm_x, arm_y, arm_z, arm_rx, arm_ry, arm_rz)
                print((arm_x, arm_y, arm_z, arm_rx, arm_ry, arm_rz))
                
                
                # Define target position and orientation for the robot arm
                dPcs = [arm_x, arm_y, arm_z, arm_rx, arm_ry, arm_rz]  # Include orientation angles
                dTcp = [0, 0, 0, 0, 0, 0]
                dUcs = [0, 0, 0, 0, 0, 0]

                # Send online position command
                nRet = cps.HRIF_PushServoP(0, 0, dPcs, dTcp, dUcs)
                time.sleep(0.015)

            except ValueError:
                print("Received data is not in the correct format.")

        except socket.error:
            # If no data is received, continue looping
            time.sleep(0.01)

if __name__ == "__main__":
    main()
