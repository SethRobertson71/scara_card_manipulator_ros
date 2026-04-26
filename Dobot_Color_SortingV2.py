import socket
import time

# --- CONFIGURATION ---
IP_ADDRESS = "10.12.1.224"
DASH_PORT = 29999
MOVE_PORT = 30003

# Hardware Pin Mapping (DO17 = Pin 2, DO18 = Pin 3)
PUMP_PIN = 4      
SOLENOID_PIN = 3

# Placeholder for current detected cards (This would be populated by your camera)
# Example format: {'id': 1, 'color': 'green', 'pos': '400, 0, 240, 0, 1'}
detected_cards = [
    {'id': 10, 'color': 'purple', 'pos': '400, 0, 140, 0, 1'},
    {'id': 2, 'color': 'green', 'pos': '400, 0, 140, 0, 1'},
    {'id': 6, 'color': 'red', 'pos': '400, 0, 140, 0, 1'},
    {'id': 1, 'color': 'green', 'pos': '400, 0, 140, 0, 1'}
]

# Bin Coordinates
BINS = {
    "green": "234, -319, 140, -91, 0",
    "red": "-18, -303, 140, -94, 0",
    "purple": "108, -311, 140, -92, 0",
}

def send_cmd(port, cmd):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(2)
            s.connect((IP_ADDRESS, port))
            s.sendall(f"{cmd}\n".encode())
            return s.recv(1024).decode().strip()
    except Exception as e:
        print(f"TCP Error: {e}")
        return None

# --- HARDWARE CONTROL FUNCTIONS ---

def pickup_sequence():
    """Triggers DO17 to start the pump"""
    print(">>> PUMP ON")
    send_cmd(DASH_PORT, f"ToolDO({PUMP_PIN}, 1)")
    time.sleep(0.5) # Allow vacuum to build

def release_sequence():
    """Turns off pump and pulses solenoid for 0.5s"""
    print(">>> PUMP OFF | RELEASING PRESSURE")
    send_cmd(DASH_PORT, f"ToolDO({PUMP_PIN}, 0)")
    send_cmd(DASH_PORT, f"ToolDO({SOLENOID_PIN}, 1)")
    time.sleep(0.5)
    send_cmd(DASH_PORT, f"ToolDO({SOLENOID_PIN}, 0)")

# --- SORTING LOGIC ---

def get_priority_list(cards):
    """Sorts cards by Green(1-4), Red(5-8), then Purple(9-12)"""
    # Define color order
    color_order = {"green": 1, "red": 2, "purple": 3}
    
    # Sort primarily by color priority, secondarily by number ID
    return sorted(cards, key=lambda x: (color_order.get(x['color'], 99), x['id']))

def main():
    # Initialization
    send_cmd(DASH_PORT, "ClearError()")
    send_cmd(DASH_PORT, "EnableRobot()")
    time.sleep(3)
    send_cmd(DASH_PORT, "SpeedFactor(20)")

    while True:
        # 1. Get the latest detection from camera (placeholder for now)
        # If list is empty, loop and wait
        if not detected_cards:
            print("Waiting for cards...")
            time.sleep(1)
            continue

        # 2. Prioritize the cards
        queue = get_priority_list(detected_cards)
        
        # 3. Process the highest priority card available
        target = queue.pop(0)
        print(f"\nProcessing Card {target['id']} ({target['color']})")

        # Move to Pick
        send_cmd(MOVE_PORT, f"MovJ({target['pos']})")
        time.sleep(2)
        
        # Hardware Pick
        pickup_sequence()
        
        # Move to Bin
        bin_coords = BINS.get(target['color'])
        send_cmd(MOVE_PORT, f"MovJ({bin_coords})")
        time.sleep(2)
        
        # Hardware Place
        release_sequence()

        # Remove processed card from the master list
        detected_cards.remove(target)

        # Safety break if list is empty
        if len(detected_cards) == 0:
            print("All detected cards sorted.")
            break

if __name__ == "__main__":
    main()