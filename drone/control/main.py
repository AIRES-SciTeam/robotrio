#!/opt/python-venv/bin/python3
from MAVLinkPX4Conn import MAVLinkPX4Conn
from GripperCTRL import GripperCTRL
from GamepadCTRL import GamepadCTRL

if __name__ == "__main__":
    print("\033[92mStarting...\033[0m")
    print("\033[92m* Connecting to PX4...\033[0m")
    mavlink_conn = MAVLinkPX4Conn(conn_type="udp", conn_ip="127.0.0.1", conn_port=18571)
    print("\033[92m* Initializing GripperCTRL...\033[0m")
    gripper_ctrl = GripperCTRL(world="scene", model="x500", cargo="goods", max_cargo_id=4, max_distance=0.6)
    print("\033[92m* Starting GamepadCTRL...\033[0m")
    gamepad_ctrl = GamepadCTRL(deadzone=0.1, mavlink_conn=mavlink_conn, gripper_ctrl=gripper_ctrl)
    gamepad_ctrl.loop()
