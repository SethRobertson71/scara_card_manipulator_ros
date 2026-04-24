import socket
import time

# --- CONFIGURATION (from V2) ---
IP_ADDRESS = "192.168.1.6"
DASH_PORT = 29999
MOVE_PORT = 30003

# Hardware Pin Mapping (DO17 = Pin 2, DO18 = Pin 3)
PUMP_PIN = 19      
SOLENOID_PIN = 20 

# Bin Coordinates (Dobot tracker readings)
bins = {
    "red":   "-18, -303, 130, -94, 0",
    "blue":  "108, -311, 130, -92, 0",
    "green": "234, -319, 130, -91, 0",
    "purple": "400, 0, 140, 0, 0",
}


def send_cmd(ip_address, port, cmd):
    """Send command to Dobot and receive response"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(5)
            s.connect((ip_address, port))
            s.sendall(f"{cmd}\n".encode())
            resp = s.recv(1024).decode().strip()
            print(f"[{port}] Sent: {cmd} | Received: {resp}")
            return resp
    except Exception as e:
        print(f"TCP Error: {e}")
        return None


def pickup_sequence(ip_address, dash_port):
    """Triggers pump (vacuum on)"""
    print(">>> PUMP ON")
    send_cmd(ip_address, dash_port, f"ToolDO({PUMP_PIN}, 1)")
    time.sleep(0.5)


def release_sequence(ip_address, dash_port):
    """Turns off pump and pulses solenoid for release"""
    print(">>> PUMP OFF | RELEASING PRESSURE")
    send_cmd(ip_address, dash_port, f"ToolDO({PUMP_PIN}, 0)")
    send_cmd(ip_address, dash_port, f"ToolDO({SOLENOID_PIN}, 1)")
    time.sleep(0.5)
    send_cmd(ip_address, dash_port, f"ToolDO({SOLENOID_PIN}, 0)")


def get_priority_list(cards):
    """Sorts cards by Green(1-4), Red(5-8), then Purple(9-12)"""
    color_order = {"green": 1, "red": 2, "blue": 3, "purple": 4}
    return sorted(cards, key=lambda x: (color_order.get(x['color'], 99), x.get('id', 0)))


class DobotColorSorter:
    def __init__(self, ip_address=IP_ADDRESS, dash_port=DASH_PORT, move_port=MOVE_PORT, speed_factor=40, bins_override=None):
        self.ip_address = ip_address
        self.dash_port = dash_port
        self.move_port = move_port
        self.speed_factor = speed_factor
        self.bins = dict(bins_override) if bins_override else dict(bins)

    def send(self, port, cmd):
        """Wrapper for send_cmd using instance variables"""
        return send_cmd(self.ip_address, port, cmd)

    def initialize_robot(self):
        """Initialize Dobot robot"""
        print("--- Initializing ---")
        self.send(self.dash_port, "ClearError()")
        time.sleep(2)
        self.send(self.dash_port, "EnableRobot()")
        time.sleep(3)
        self.send(self.dash_port, f"SpeedFactor({self.speed_factor})")
        time.sleep(0.5)

    def sort_card(self, card):
        """Sort a single card using V2 hardware sequences"""
        color = card["color"]
        print(f"\n--- Sorting {color.upper()} (Card {card.get('id', '?')}) ---")

        # Move to pick position
        self.send(self.move_port, f"MovJ({card['pos']})")
        time.sleep(2)

        # Pickup using V2 sequence
        pickup_sequence(self.ip_address, self.dash_port)

        # Move to bin
        bin_pos = self.bins.get(color)
        if not bin_pos:
            print(f"No bin configured for color '{color}', skipping place move")
            print(">>> PUMP OFF (SAFETY)")
            send_cmd(self.ip_address, self.dash_port, f"ToolDO({PUMP_PIN}, 0)")
            time.sleep(1)
            return

        self.send(self.move_port, f"MovJ({bin_pos})")
        time.sleep(2)

        # Release using V2 sequence
        release_sequence(self.ip_address, self.dash_port)

    def sort_cards(self, cards):
        """Sort multiple cards in priority order"""
        queue = get_priority_list(cards)
        for card in queue:
            self.sort_card(card)

    def shutdown_robot(self):
        """Shutdown robot safely"""
        print("\nCleaning up...")
        self.send(self.dash_port, "Stop()")
        time.sleep(0.5)
        self.send(self.dash_port, "DisableRobot()")


# Example static data for standalone execution
example_cards = [
    {"id": 1, "color": "green", "pos": "400, 0, 240, 0, 1"},
    {"id": 5, "color": "red", "pos": "400, 0, 240, 0, 1"},
    {"id": 10, "color": "purple", "pos": "400, 0, 240, 0, 1"}
]


def run_static_sorting():
    """Example standalone execution"""
    sorter = DobotColorSorter()
    try:
        sorter.initialize_robot()
        sorter.sort_cards(example_cards)
    finally:
        sorter.shutdown_robot()


if __name__ == "__main__":
    run_static_sorting()