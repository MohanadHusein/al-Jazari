from machine import I2C, Pin
i2c = I2C(0, scl=Pin(33), sda=Pin(32), freq=400000)
