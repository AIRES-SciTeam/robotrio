from pymavlink import mavutil


class MAVLinkPX4Conn:
    def __init__(
        self, 
        conn_type="udp",
        conn_ip="127.0.0.1",
        conn_port=18571,
    ):
        self.conn = mavutil.mavlink_connection(
            f"{conn_type}:{conn_ip}:{conn_port}",
            input=False, source_system=255
        )
        print("\033[92mConnected to PX4!\033[0m")

    def get_conn(self):
        return self.conn

    def heartbeat(self):
        self.conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

    def send_arm(self):
        self.conn.mav.command_long_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

    def send_manual_control(self, roll, pitch, yaw, throttle):
        self.conn.mav.manual_control_send(
            self.conn.target_system,
            roll,
            pitch,
            throttle,
            yaw,
            0
        )