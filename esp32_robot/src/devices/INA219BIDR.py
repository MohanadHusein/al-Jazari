from machine import I2C, Pin
import time

class INA219BIDR:
    POWER_MONITOR_I2C_ADDR = 0x42 # I2C address of the power monitor IC
    BUS_VOLTAGE_REG = 0x02 # bus voltage measurement register address
    BATT_EMPTY_VOLT = 9.6
    BATT_FULL_VOLT = 12.6

    def __init__(self, i2c):
        self.i2c = i2c

    def measure_voltage(self):
        raw_voltage_measurement = self.read_bytes(self.POWER_MONITOR_I2C_ADDR, self.BUS_VOLTAGE_REG, 2)
        voltage_measurement = (int.from_bytes(raw_voltage_measurement) >> 3) * 0.004 # val shifted 3 times and multiplied by 0.004
        percentage = round((voltage_measurement - self.BATT_EMPTY_VOLT) / (self.BATT_FULL_VOLT - self.BATT_EMPTY_VOLT) * (100 - 0) + 0,2)
        return [voltage_measurement, percentage]

    def read_bytes(self, address, reg_addr, number_of_bytes):
        return self.i2c.readfrom_mem(address, reg_addr, number_of_bytes)

    def write_byte(self,address ,reg_addr, val):
        self.i2c.writeto_mem(address, reg_addr, bytes([val]))


if __name__ == "__main__":
    i2c_chan = I2C(0, scl=Pin(33), sda=Pin(32), freq=400000)
    battery_monitor = INA219BIDR(i2c_chan)

    for x in range(100):
        volt = battery_monitor.measure_voltage()
        print(volt)
        time.sleep(1)