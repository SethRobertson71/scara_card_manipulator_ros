import socket
import time

IP_ADDRESS = "10.12.1.224"
DASH_PORT = 29999

def get_current_coords():
    print("--- Starting Coordinate Tracker ---")
    print("Move the robot by hand. Press Ctrl+C to stop.\n")
    
    try:
        # First, ensure the robot is 'Disabled' so you can move it freely
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((IP_ADDRESS, DASH_PORT))
            s.sendall(b"DisableRobot()\n")
            time.sleep(0.5)

        while True:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2)
                s.connect((IP_ADDRESS, DASH_PORT))
                s.sendall(b"GetPose()\n")
                
                data = s.recv(1024).decode().strip()
                
                # The robot returns: 0,{x,y,z,r,joint1,joint2...},GetPose();
                # This logic cleans it up to just show the numbers
                if "{" in data:
                    coords = data.split("{")[1].split("}")[0]
                    print(f"Current Pose [X, Y, Z, R]: {coords}")
                
                time.sleep(2) # Refresh rate
                
    except KeyboardInterrupt:
        print("\nTracker stopped.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    get_current_coords()