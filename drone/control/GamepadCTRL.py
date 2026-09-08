import pygame #type: ignore
import numpy as np


class GamepadCTRL:
    def __init__(
        self,
        mavlink_conn,
        gripper_ctrl,
        deadzone=0.1,
    ):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("\033[91mFailed: no gamepad found.\033[0m")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"\033[92m  Started. Gamepad: {self.joystick.get_name()}\033[0m")

        self.conn = mavlink_conn
        self.gripper_ctrl = gripper_ctrl
        self.deadzone = deadzone

    def _normalize_axis(self, value, invert=False):
        if abs(value) < self.deadzone:
            return 0
        return int((value if not invert else -value) * 1000)

    def _normalize_throttle(self, value, p=2.0):
        value = max(min(value, 1), -1)
        if value + 1 <= self.deadzone:
            return 0
        return int(500 * (1 + np.sign(value) * (np.abs(value) ** p)))

    def _send_ctrl(self, roll, pitch, yaw, throttle):
        self.conn.send_manual_control(roll, pitch, yaw, throttle)

    def _send_arm(self):
        self.conn.send_arm()

    def loop(self):
        print("\033[92mReady.\033[0m\n",
            "   Left Stick: Control Roll and Pitch\n",
            "   Right Stick: Control Yaw\n",
            "   Right Trigger: Control Throttle\n",
            "   Button A: Arm\n",
            "   Button B: Toggle Slow Mode\n",
            "   Button X: Grip/Release\n",
            "   Button Start: Exit"
        )
    
        clock = pygame.time.Clock()
        slow_multiplier = 1.0
        try:
            while True:
                pygame.event.pump()
    
                roll_raw = slow_multiplier * self.joystick.get_axis(1)
                roll = self._normalize_axis(roll_raw, invert=True)
    
                pitch_raw = slow_multiplier * self.joystick.get_axis(0)
                pitch = self._normalize_axis(pitch_raw)
    
                yaw_raw = slow_multiplier * self.joystick.get_axis(2)
                yaw = self._normalize_axis(yaw_raw)
    
                throttle_up = slow_multiplier * self.joystick.get_axis(5)
                throttle_down = slow_multiplier * self.joystick.get_axis(4)
                throttle_up_norm = self._normalize_throttle(throttle_up)
                throttle_down_norm = self._normalize_throttle(throttle_down)
                throttle = 500 + (throttle_up_norm - throttle_down_norm) // 2
    
                self._send_ctrl(roll, pitch, yaw, throttle)
    
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        button = event.button
                        if button == 0:
                            self._send_arm()
                            print("\033[93mSent arm command.\033[0m")
                        elif button == 1:
                            slow_multiplier = 0.5 if slow_multiplier == 1.0 else 1.0
                            print(f"\033[93mSlow mode {'enabled' if slow_multiplier == 0.5 else 'disabled'}\033[0m")
                        elif button == 2:
                            self.gripper_ctrl.toggle()
                        elif button == 7:
                            return
    
                self.conn.heartbeat()
                clock.tick(50)
        except KeyboardInterrupt:
            print("\n\033[92mBridge stopped.\033[0m")
        finally:
            pygame.quit()
