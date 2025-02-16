from machine import I2C, Pin, Timer
from devices.SSD1306 import SSD1306
from devices.INA219BIDR import INA219BIDR
from devices.motor_control import MotorControl
import time


class Robot:
    def __init__(self):
        self.i2c = I2C(0, scl=Pin(33), sda=Pin(32), freq=400000)
        self.screen = SSD1306(self.i2c)
        self.power_monitor = INA219BIDR(self.i2c)
        self.battery_charge_level()
        self.initialize_screen_updates_timer()
        # self.motors = MotorControl()


    def initialize_screen_updates_timer(self):
        screen_timer = Timer(1)
        screen_timer.init(period=10000, mode=Timer.PERIODIC, callback=self.display_refresh)

    def display_refresh(self, timer):
        self.battery_charge_level()

    def battery_charge_level(self):
        current_batt_v = self.power_monitor.get_battery_charge()
        # self.screen.write_text(f'Batt V: {current_batt_v[0]}', 0)
        self.screen.write_text(f'Batt %: {current_batt_v[1]}', 0)

    def test_run(self):
        self.motors.set_motors_direction('left', 'forward')
        self.motors.set_motors_direction('right', 'forward')
        self.motors.set_motors_pwm('left', 150)
        self.motors.set_motors_pwm('right', 150)
        time.sleep(15)
        self.motors.stop_all_motors()


al_jazari = Robot()
# al_jazari.test_run()
# al_jazari.motors.test_run()














