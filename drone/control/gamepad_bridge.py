#!/usr/bin/python3
import pygame
import time
from pymavlink import mavutil
import math

"""
'STABILIZE': 0,
'ACRO': 1,
'ALTCTL': 2,
'POSCTL': 3,
'AUTO': 4,
'LOITER': 5,
'OFFBOARD': 6,
'RATTITUDE': 7,
'TAKEOFF': 8,
'LAND': 9,
'LOITER': 10,
'FOLLOW_ME': 11,
'PRECISION_LAND': 12,
'MISSION': 13,
'RTL': 14
"""

class GamepadToMavlinkBridge:
    def __init__(self, deadzone=0.1):
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No gamepad found")
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Gamepad: {self.joystick.get_name()}")
        
        # Подключение к PX4 - ТОЛЬКО ОТПРАВКА, не прослушивание
        self.master = mavutil.mavlink_connection(
            'udp:127.0.0.1:18570',
            input=False,  # Не слушаем входящие
            source_system=255  # Система GCS
        )
        
        # Ждем heartbeat от PX4
        print("Waiting for heartbeat from PX4...")
        self.master.wait_heartbeat(timeout=5)
        print("Connected to PX4!")
        
        # Настройки
        self.deadzone = deadzone
        self.armed = False
        self.holding = False

    def normalize_axis(self, value, invert=False):
        """Нормализация значения оси от -1 до 1 с deadzone"""
        if abs(value) < self.deadzone:
            return 0.0
        return value if not invert else -value
    
    def send_manual_control(self, roll, pitch, yaw, throttle):
        """
        Отправка MAVLink сообщения MANUAL_CONTROL
        throttle: 0..1000 (где 0 = min, 1000 = max)
        """
        # Конвертация throttle из -1..1 в 0..1000
        # throttle: -1 (min) -> 0, 1 (max) -> 1000
        throttle_mav = int(throttle * 1000)
        throttle_mav = max(0, min(1000, throttle_mav))
        
        # Конвертация roll/pitch/yaw из -1..1 в -1000..1000
        roll_mav = int(roll * 1000)
        pitch_mav = int(pitch * 1000)
        yaw_mav = int(yaw * 1000)
        
        self.master.mav.manual_control_send(
            self.master.target_system,
            roll_mav,
            pitch_mav,
            throttle_mav,
            yaw_mav,
            0  # buttons bitmask
        )

    def send_holding_command(self):
        """Удержание текущей высоты"""
        # Отправляем команду держать текущую высоту
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            
            # Используем позицию по Z, скорость по X,Y
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE,
            
            0, 0, 5,  # позиция (x,y,целевая_высота)
            0, 0, 0,  # скорость (игнор)
            0, 0, 0,  # ускорение
            0,  # yaw
            0   # yaw_rate
        )
    
    def toggle_holding(self):
        self.holding = not self.holding

    def toggle_arming(self):
        """Арминг через COMMAND_LONG"""
        self.armed = not self.armed
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1 if self.armed else 0,  # arm (1 = arm, 0 = disarm)
            0, 0, 0, 0, 0, 0
        )
        print(f"{'Arm' if self.armed else 'Disarm'} command sent")
    
    
    def run(self):
        """Основной цикл"""
        print("Bridge started. Press:") 
        print("  A button - Arm/disarm")
        print("  B button - Change flight mode")
        print("  Start button - Exit")
        
        clock = pygame.time.Clock()
        
        try:
            while True:
                pygame.event.pump()
                
                # Чтение осей
                roll = self.normalize_axis(self.joystick.get_axis(1), invert=True)
                pitch = self.normalize_axis(self.joystick.get_axis(0))
                yaw = self.normalize_axis(self.joystick.get_axis(3))
                
                # Throttle
                throttle = self.normalize_axis(self.joystick.get_axis(5))

                # Отправка управления
                if self.holding:
                    self.send_holding_command()
                else:
                    self.send_manual_control(roll, pitch, yaw, throttle)

                # Обработка кнопок
                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        button = event.button
                        
                        if button == 0:  # A button
                            self.toggle_arming()
                        elif button == 1:  # B button
                            self.toggle_holding()
                            print(f'Holding {self.holding}')
                        elif button == 7:  # Start button
                            return
                
                # Отправка heartbeat для поддержания соединения
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                
                # Вывод для отладки
                # print(f"R:{roll:.2f} P:{pitch:.2f} Y:{yaw:.2f} T:{throttle:.2f}", end='\r')
                
                clock.tick(50)  # 50Hz
                
        except KeyboardInterrupt:
            print("\nBridge stopped")
        finally:
            pygame.quit()

if __name__ == "__main__":
    bridge = GamepadToMavlinkBridge(deadzone=0.1)
    bridge.run()