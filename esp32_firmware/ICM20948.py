from machine import I2C, Pin
import time



class ICM20948:
    # I2C address of Accel and Gyro
    ACCEL_GYRO_I2C_ADDR = 0x68
    # I2C address of Magnetometer
    MAGNETO_I2C_ADDR = 0x0C
    # Register addresses
    WHO_AM_I = 0x00
    PWR_MGMT_1 = 0x06
    REG_BANK_SEL = 0x7F
    ACCEL_CONFIG = 0x14 #select bank 2 before writing
    GYRO_CONFIG_1 = 0x01 # select bank 2 before writing
    ACCEL_XOUT_H = 0x2D # start address of accelerometer addresses (6 bytes range)
    GYRO_XOUT_H = 0x33 # start address of gyro addresses (6 bytes range)
    MAGNETO_XOUT_L = 0x11 # start address of magnetometer addresses (6 bytes range)
    INT_PIN_CFG = 0x0f # register for enabling bypass mode (enabling magnetometer)
    CTRL2 = 0x31 # Magnetometer measurement trigger register
    ST2 = 0x18

    def __init__(self, i2c):
        self.i2c = i2c
        self.initialize_sensor()

        #gyro offset
        self.gx_offset = -0.5661449
        self.gy_offset = 0.2418015
        self.gz_offset = 0.3001757

        # accel offset
        self.ax_offset = 0.01760099
        self.ay_offset = -0.01598682
        self.az_offset = 0.01923418

        #mag bias
        self.mag_x_offset = 713.3333
        self.mag_y_offset = -1110.0
        self.mag_z_offset = 4063.333


    def initialize_sensor(self):
        self.sel_reg_bank(0)
        who_am_i = self.read_byte(self.ACCEL_GYRO_I2C_ADDR, reg_addr=self.WHO_AM_I)
        if who_am_i != 0xEA:
            print(f'ERROR: Expected 0xEA, received {who_am_i}')

        self.write_byte(self.ACCEL_GYRO_I2C_ADDR, self.PWR_MGMT_1, 0x01) # Wake up the sensor

        self.sel_reg_bank(2)
        self.write_byte(self.ACCEL_GYRO_I2C_ADDR, self.ACCEL_CONFIG,0x01) # config Accel to 2g
        self.write_byte(self.ACCEL_GYRO_I2C_ADDR, self.GYRO_CONFIG_1, 0x01) # config Gyro to 250 dps

        self.sel_reg_bank(0)
        self.write_byte(self.ACCEL_GYRO_I2C_ADDR, self.INT_PIN_CFG, 0x02) # enable bypass (enabling magnetometer)
        time.sleep(0.1)
        self.write_byte(self.MAGNETO_I2C_ADDR, self.CTRL2, 0x08) # enable mag continuous reading mode
        time.sleep(0.1)

    def read_byte(self, address, reg_addr):
        return self.i2c.readfrom_mem(address, reg_addr, 1)[0]

    def read_bytes(self, address, reg_addr, number_of_bytes):
        return self.i2c.readfrom_mem(address, reg_addr, number_of_bytes)

    def sel_reg_bank(self, bank_addr):
        self.write_byte(self.ACCEL_GYRO_I2C_ADDR, self.REG_BANK_SEL, bank_addr << 4)

    def write_byte(self,address ,reg_addr, val):
        self.i2c.writeto_mem(address, reg_addr, bytes([val]))

    def read_accel_gyro(self):
        accel_scale_factor = 16385 # ACCEL_FS = 2g
        gyro_scale_factor = 131

        raw_accel_gyro_data = self.read_bytes(self.ACCEL_GYRO_I2C_ADDR, self.ACCEL_XOUT_H, 12)

        raw_accel_x = (raw_accel_gyro_data[0] << 8) | raw_accel_gyro_data[1] # combine high and low byte
        if raw_accel_x > 32767: #convert from two's complement to signed value
            raw_accel_x -= 65536
        accel_x = (raw_accel_x / accel_scale_factor) - self.ax_offset

        raw_accel_y = (raw_accel_gyro_data[2] << 8) | raw_accel_gyro_data[3] # combine high and low byte
        if raw_accel_y > 32767: #convert from two's complement to signed value
            raw_accel_y -= 65536
        accel_y = (raw_accel_y / accel_scale_factor) - self.ay_offset

        raw_accel_z = (raw_accel_gyro_data[4] << 8) | raw_accel_gyro_data[5] # combine high and low byte
        if raw_accel_z > 32767: #convert from two's complement to signed value
            raw_accel_z -= 65536
        accel_z = (raw_accel_z / accel_scale_factor) - self.az_offset

        raw_gyro_x = (raw_accel_gyro_data[6] << 8) | raw_accel_gyro_data[7] # combine high and low byte
        if raw_gyro_x > 32767: #convert from two's complement to signed value
            raw_gyro_x -= 65536
        gyro_x = (raw_gyro_x / gyro_scale_factor) - self.gx_offset

        raw_gyro_y = (raw_accel_gyro_data[8] << 8) | raw_accel_gyro_data[9] # combine high and low byte
        if raw_gyro_y > 32767: #convert from two's complement to signed value
            raw_gyro_y -= 65536
        gyro_y = (raw_gyro_y / gyro_scale_factor) - self.gy_offset

        raw_gyro_z = (raw_accel_gyro_data[10] << 8) | raw_accel_gyro_data[11] # combine high and low byte
        if raw_gyro_z > 32767: #convert from two's complement to signed value
            raw_gyro_z -= 65536
        gyro_z = (raw_gyro_z / gyro_scale_factor) - self.gz_offset

        return [accel_x, accel_y, accel_z, gyro_x,gyro_y ,gyro_z]

    def read_magnetometer(self):
        mag_scale_factor = 0.15
        # self.write_byte(self.MAGNETO_I2C_ADDR, self.CTRL2, 1)
        raw_magneto_data = self.read_bytes(self.MAGNETO_I2C_ADDR, self.MAGNETO_XOUT_L, 6)
        self.read_byte(self.MAGNETO_I2C_ADDR, self.ST2) # ST2 register should be read after reading mag data

        raw_mag_x = (raw_magneto_data[1] << 8) | raw_magneto_data[0] # combine high and low byte
        if raw_mag_x > 32767: #convert from two's complement to signed value
            raw_mag_x -= 65536
        magneto_x = (raw_mag_x / mag_scale_factor) - self.mag_x_offset

        raw_mag_y = (raw_magneto_data[3] << 8) | raw_magneto_data[2] # combine high and low byte
        if raw_mag_y > 32767: #convert from two's complement to signed value
            raw_mag_y -= 65536
        magneto_y = (raw_mag_y / mag_scale_factor) - self.mag_y_offset

        raw_mag_z = (raw_magneto_data[5] << 8) | raw_magneto_data[4] # combine high and low byte
        if raw_mag_z > 32767: #convert from two's complement to signed value
            raw_mag_z -= 65536
        magneto_z = (raw_mag_z / mag_scale_factor) - self.mag_z_offset

        return magneto_x, magneto_y, magneto_z

    def calibrate_gyro(self, samples=1000): # used to calibrate gyro bias
        gx_offset, gy_offset, gz_offset = 0, 0, 0

        for _ in range(samples):
            _, _, _, gx, gy, gz = self.read_accel_gyro()
            gx_offset += gx
            gy_offset += gy
            gz_offset += gz
            time.sleep(0.002)  # 2ms delay

        gx_offset /= samples
        gy_offset /= samples
        gz_offset /= samples

        print(f"Gyro Bias: X={gx_offset}, Y={gy_offset}, Z={gz_offset}")

    def calibrate_accel(self, samples=1000):
        ax_offset, ay_offset, az_offset = 0, 0, 0

        for _ in range(samples):
            ax, ay, az, _, _, _ = self.read_accel_gyro()
            ax_offset += ax
            ay_offset += ay
            az_offset += az
            time.sleep(0.002)

        ax_offset /= samples
        ay_offset /= samples
        az_offset /= samples

        az_offset -= 1 # assume robot is upright, adjust for 1g

        print(f"Accel Bias: X={ax_offset}, Y={ay_offset}, Z={az_offset}")
        return ax_offset, ay_offset, az_offset

    def calibrate_magnetometer(self, samples=1000):
        mag_x_min, mag_x_max = float('inf'), float('-inf')
        mag_y_min, mag_y_max = float('inf'), float('-inf')
        mag_z_min, mag_z_max = float('inf'), float('-inf')

        print("Rotate the sensor in all directions...")

        for _ in range(samples):
            mag_x, mag_y, mag_z = self.read_magnetometer()

            mag_x_min = min(mag_x_min, mag_x)
            mag_x_max = max(mag_x_max, mag_x)
            mag_y_min = min(mag_y_min, mag_y)
            mag_y_max = max(mag_y_max, mag_y)
            mag_z_min = min(mag_z_min, mag_z)
            mag_z_max = max(mag_z_max, mag_z)

            time.sleep(0.05)

        # hard iron offset
        mag_x_offset = (mag_x_max + mag_x_min) / 2
        mag_y_offset = (mag_y_max + mag_y_min) / 2
        mag_z_offset = (mag_z_max + mag_z_min) / 2

        print(f"Magnetometer Bias: X={mag_x_offset}, Y={mag_y_offset}, Z={mag_z_offset}")
        return mag_x_offset, mag_y_offset, mag_z_offset

# Example usage
if __name__ == "__main__":
    i2c = I2C(0, scl=Pin(33), sda=Pin(32), freq=400000)
    icm = ICM20948(i2c)

    while True:
        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z  = icm.read_accel_gyro()
        mag_x, mag_y, mag_z = icm.read_magnetometer()

        print(f"Accel_x: {accel_x} | Accel_y: {accel_y} | Accel_z: {accel_z} | "
              f"Gyro_x: {gyro_x} | Gyro_y: {gyro_y} | Gyro_z: {gyro_z} | "
              f"Magneto_x: {mag_x} | Magneto_y: {mag_y} | Magneto_z: {mag_z}")

        time.sleep(0.1)

