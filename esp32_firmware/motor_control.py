from machine import Pin, PWM, Timer
import pid
import time
import math

class MotorControl:
    def __init__(self):
        self.pwm_freq = 1000 #1KHz
        self.max_pwm_val = 850
        self.min_pwm_val = 0
        self.encoder_PPR = 1320 # output shaft PPR (11PPR * 30 gear ratio * 4 quadrature encoders rise & fall)
        self.wheels_radius = 0.04 # m
        self.wheels_circumference = 2 * math.pi * self.wheels_radius
        self.encoder_timer_interval = 50 # millisecond
        self.encoder_timer_interval_s = self.encoder_timer_interval / 1000 # convert to seconds

        self.pid_timer_interval = 50 # millisecond
        self.pid_timer_interval_s = self.encoder_timer_interval / 1000 # convert to seconds

        self.right_linear_velocity_readings = []
        self.left_linear_velocity_readings = []
        self.linear_velocity_window_size = 5

        self.right_linear_velocity = 0
        self.left_linear_velocity = 0
        self.robot_linear_velocity = 0

        self.set_point_right_linear_velocity = 0 #m/s
        # self.right_linear_velocity_pid = pid.PIDController(kp=6, ki=34, kd=0.5) # good one

        self.right_linear_velocity_pid = pid.PIDController(kp=4.3, ki=25.9, kd=0.05) # final gains.

        # self.set_point_left_linear_velocity = 0
        # self.left_linear_velocity_pid = pid.PIDController(kp=5.8, ki=3.7, kd=0.75)

        # configuring PWM and digital pin for left motor wheels
        self.pwm_left = PWM(Pin(25), freq=self.pwm_freq, duty=0)
        self.AIN1_left = Pin(21, Pin.OUT)
        self.AIN2_left = Pin(17, Pin.OUT)
        self.AIN1_left.off()
        self.AIN2_left.off()

        # configuring PWM and digital pin for right motor wheels control
        self.pwm_right = PWM(Pin(26), freq=self.pwm_freq, duty=0)
        self.BIN1_right = Pin(22, Pin.OUT)
        self.BIN2_right = Pin(23, Pin.OUT)
        self.BIN1_right.off()
        self.BIN2_right.off()

        print('configuring right wheel encoders pins')
        # configuring right wheel encoders pins
        self.encoder_right_1 = Pin(27, Pin.IN, Pin.PULL_UP)
        self.encoder_right_2 = Pin(16, Pin.IN, Pin.PULL_UP)

        print('configuring right wheel encoders pins interrupt')
        # configuring right wheel encoders pins interrupt
        self.encoder_right_1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.right_motors_encoders_interrupt)
        self.encoder_right_2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.right_motors_encoders_interrupt)

        print('configuring left wheel encoders pins')
        # configuring left wheel encoders pins
        self.encoder_left_1 = Pin(34, Pin.IN, Pin.PULL_UP)
        self.encoder_left_2 = Pin(35, Pin.IN, Pin.PULL_UP)

        print('configuring left wheel encoders pins interrupt')
        # configuring left wheel encoders pins interrupt
        self.encoder_left_1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.left_motors_encoders_interrupt)
        self.encoder_left_2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.left_motors_encoders_interrupt)

        print('init encoder tick counters')
        # init encoder tick counters
        self.right_motors_encoders_counter = 0
        self.left_motors_encoders_counter = 0

        self.right_pwm_from_pid = 0
        self.left_pwm_from_pid = 0


        #configuring hardware timer 0 to trigger every 100ms
        print('configuring hardware timer 0 to trigger every 100ms')
        encoder_timer = Timer(0)
        encoder_timer.init(period=self.encoder_timer_interval, mode=Timer.PERIODIC, callback=self.reset_encoders_counter)

        #configuring hardware timer 2 to trigger every 50ms - PID loop
        print('configuring hardware timer 0 to trigger every 100ms for PID')
        # encoder_timer = Timer(2)
        # encoder_timer.init(period=self.pid_timer_interval, mode=Timer.PERIODIC, callback=self.run_pid)

        print('finished initializing')
        self.test_run()

        #from motor_control import MotorControl

    def test_run(self):
        self.stop_all_motors()
        self.set_motors_direction('left', 'forward')
        self.set_motors_direction('right', 'forward')
        # for pwm in range(100, 300, 50):
        # for speed in [0, 0.4, 0.2, 0.4, 0.2, 0.4, 0]:
        for speed in [0.6,0.2,0.6, 0]:

        # for speed in [0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15, 0.1, 0]:
        # for speed in [0, 0.1, 0.65, 0.15, 0.2, 0.75, 0.25, 0.3, 0.4, 0.8, 0.45, 0.5, 0.55, 0.6, 0.7, 0.85, 0.35, 0]:
        #     print(speed)

            # print(pwm)
            self.set_point_right_linear_velocity = speed
            # self.set_point_left_linear_velocity = speed

            # self.set_motors_pwm('left', pwm)
            # self.set_motors_pwm('right', pwm)
            time.sleep(10)

        self.stop_all_motors()

    def reset_encoders_counter(self, timer):

        right_pid_output = self.right_linear_velocity_pid.compute(self.set_point_right_linear_velocity, self.right_linear_velocity,self.pid_timer_interval_s)
        right_shifted_pid_pwm = right_pid_output + 5
        right_output_pwm = round(right_shifted_pid_pwm / 10 * 900) # mapping pid output to 0-850 pwm
        right_output_pwm = max(min(right_output_pwm, 900), 0) #capping the values at 850 pwm
        self.right_pwm_from_pid = right_output_pwm

        temp_right_linear_velocity = self.calculate_linear_velocity(self.right_motors_encoders_counter)
        temp_left_linear_velocity = self.calculate_linear_velocity(self.left_motors_encoders_counter)

        self.right_linear_velocity = self.calculate_moving_average(self.right_linear_velocity_readings, temp_right_linear_velocity)
        self.left_linear_velocity = self.calculate_moving_average(self.left_linear_velocity_readings, temp_left_linear_velocity)





        # print(f'Right encoder counter {self.right_linear_velocity} , '
        #       f'Left encoder counter  {self.left_linear_velocity}  '
        #       f' right pwm from PID: {self.right_pwm_from_pid}  '
        #       f'     setpoint: {self.set_point_right_linear_velocity}')

        print(f'{self.right_linear_velocity},{self.set_point_right_linear_velocity}')

        self.set_motors_pwm('right', right_output_pwm)


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
        self.set_motors_pwm('right', 0)
        self.set_motors_pwm('left', 0)

    def calculate_linear_velocity(self, encoder_counts):
        linear_displacement = self.wheels_circumference * (encoder_counts / self.encoder_PPR)
        linear_velocity = linear_displacement / self.encoder_timer_interval_s
        return linear_velocity

    def calculate_moving_average(self, linear_velocity_list, current_linear_velocity_measurement):
        linear_velocity_list.append(current_linear_velocity_measurement)
        if len(linear_velocity_list) > self.linear_velocity_window_size:
            linear_velocity_list.pop(0)

        return round(sum(linear_velocity_list) / len(linear_velocity_list), 2)

    # set linear velocity of right motors from 0-1m/s
    def set_right_linear_velocity(self, velocity):
        self.set_point_right_linear_velocity = velocity

    def set_left_linear_velocity(self, velocity):
        self.set_point_left_linear_velocity = velocity

    def run_pid(self, timer):
        right_pid_output = self.right_linear_velocity_pid.compute(self.set_point_right_linear_velocity, self.right_linear_velocity,self.pid_timer_interval_s)
        right_shifted_pid_pwm = right_pid_output + 5
        right_output_pwm = round(right_shifted_pid_pwm / 10 * 900) # mapping pid output to 0-850 pwm
        right_output_pwm = max(min(right_output_pwm, 900), 0) #capping the values at 850 pwm
        self.right_pwm_from_pid = right_output_pwm

        # left_pid_output = self.left_linear_velocity_pid.compute(self.set_point_left_linear_velocity, self.left_linear_velocity,self.pid_timer_interval_s)
        # left_shifted_pid_pwm = left_pid_output + 5
        # left_output_pwm = round(left_shifted_pid_pwm / 10 * 850) # mapping pid output to 0-850 pwm
        # left_output_pwm = max(min(left_output_pwm, 850), 0) #capping the values at 850 pwm
        # self.left_pwm_from_pid = left_output_pwm




        self.set_motors_pwm('right', right_output_pwm)
        # self.set_motors_pwm('left', left_output_pwm)


    def set_angular_velocity(self):
        pass







        # pwm pins
        # motor enable pins
        # encoder pins

        # configure all gpios for encoder reading and motor driver
        # write functions for calculating linear and angular velocity


