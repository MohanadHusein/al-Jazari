from machine import Pin, PWM, Timer
import time
class MotorControl:
    def __init__(self):
        self.pwm_freq = 20000 #20KHz
        self.max_pwm_val = 512
        self.min_pwm_val = 0

        # configuring PWM and digital pin for left motor wheels
        self.pwm_left = PWM(Pin(25), freq=self.pwm_freq, duty=125)
        self.AIN1_left = Pin(21, Pin.OUT)
        self.AIN2_left = Pin(17, Pin.OUT)
        self.AIN1_left.off()
        self.AIN2_left.off()

        # configuring PWM and digital pin for right motor wheels control
        self.pwm_right = PWM(Pin(26), freq=self.pwm_freq, duty=125)
        self.BIN1_right = Pin(22, Pin.OUT)
        self.BIN2_right = Pin(23, Pin.OUT)
        self.BIN1_right.off()
        self.BIN2_right.off()

        # configuring right wheel encoders pins
        self.encoder_right_1 = Pin(27, Pin.IN)
        self.encoder_right_2 = Pin(16, Pin.IN)

        print('configuring right wheel encoders pins interrupt')
        # configuring right wheel encoders pins interrupt
        self.encoder_right_1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.right_motors_encoders_interrupt)
        self.encoder_right_2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.right_motors_encoders_interrupt)

        print('configuring left wheel encoders pins')
        # configuring left wheel encoders pins
        self.encoder_left_1 = Pin(34, Pin.IN)
        self.encoder_left_2 = Pin(35, Pin.IN)

        print('configuring left wheel encoders pins interrupt')
        # configuring left wheel encoders pins interrupt
        self.encoder_left_1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.left_motors_encoders_interrupt)
        self.encoder_left_2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.left_motors_encoders_interrupt)

        print('init encoder tick counters')
        # init encoder tick counters
        self.right_motors_encoders_counter = 0
        self.left_motors_encoders_counter = 0

        #configuring hardware timer 0 to trigger every 100ms
        print('configuring hardware timer 0 to trigger every 100ms')
        encoder_timer = Timer(0)
        encoder_timer.init(period=100, mode=Timer.PERIODIC, callback=self.reset_encoders_counter)

        print('finished initializing')

        # self.set_motors_direction('left', 'forward')
        # self.set_motors_direction('right', 'forward')
        # self.set_motors_pwm('left', 150)
        # self.set_motors_pwm('right', 150)
        # time.sleep(5)
        # self.set_motors_pwm('left', 200)
        # self.set_motors_pwm('right', 200)
        # time.sleep(5)
        # self.set_motors_pwm('left', 250)
        # self.set_motors_pwm('right', 250)
        # time.sleep(5)
        # self.set_motors_pwm('left', 300)
        # self.set_motors_pwm('right', 300)
        #
        # time.sleep(5)
        # self.set_motors_pwm('left', 350)
        # self.set_motors_pwm('right', 350)
        #
        # time.sleep(5)
        # self.set_motors_pwm('left', 400)
        # self.set_motors_pwm('right', 400)
        #
        # time.sleep(5)
        # self.stop_all_motors()

    def reset_encoders_counter(self, timer):
        print(f'Right encoder counter {self.right_motors_encoders_counter}, '
              f'Left encoder counter {self.left_motors_encoders_counter}')
        self.right_motors_encoders_counter = 0
        self.left_motors_encoders_counter = 0

    def right_motors_encoders_interrupt(self, pin):
        self.right_motors_encoders_counter += 1

    def left_motors_encoders_interrupt(self, pin):
        self.left_motors_encoders_counter += 1

    def set_motors_pwm(self, side, pwm_val):
        motor_map = {'right': self.pwm_right,
                     'left' : self.pwm_left}
        motor_map[side].duty(pwm_val)

    def set_motors_direction(self, side, direction):
        motor_map = {'right': [self.BIN1_right, self.BIN2_right],
                     'left' : [self.AIN1_left, self.AIN2_left]}

        if direction == 'forward':
            motor_map[side][0].on()
            motor_map[side][1].off()
        elif direction == 'backward':
            motor_map[side][0].off()
            motor_map[side][1].on()
        elif direction == 'stop':
            motor_map[side][0].off()
            motor_map[side][1].off()

    def stop_all_motors(self):
        self.BIN1_right.off()
        self.BIN2_right.off()
        self.AIN1_left.off()
        self.AIN2_left.off()

    def set_angular_velocity(self):
        pass

    def set_linear_velocity(self):
        pass

    def read_encoders(self):
        pass


        # pwm pins
        # motor enable pins
        # encoder pins

        # configure all gpios for encoder reading and motor driver
        # write functions for calculating linear and angular velocity