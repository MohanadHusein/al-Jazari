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
        self.velocity_control_timer_interval = 50 # millisecond
        self.velocity_control_timer_interval_s = self.velocity_control_timer_interval / 1000 # convert to seconds

        self.right_linear_velocity_readings = []
        self.left_linear_velocity_readings = []
        self.linear_velocity_window_size = 10

        self.right_linear_velocity = 0 # m/s
        self.left_linear_velocity = 0 # m/s
        self.robot_linear_velocity = 0 # m/s

        self.set_point_right_linear_velocity = 0 # m/s
        self.set_point_left_linear_velocity = 0 # m/s

        # initializing PID controller for right and left motors
        self.right_linear_velocity_pid = pid.PIDController(kp=4.3, ki=20, kd=0.50)
        self.left_linear_velocity_pid = pid.PIDController(kp=4.3, ki=20, kd=0.50)

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

        # configuring right wheel encoders pins
        self.encoder_right_1 = Pin(27, Pin.IN, Pin.PULL_UP)
        self.encoder_right_2 = Pin(16, Pin.IN, Pin.PULL_UP)

        # configuring right wheel encoders pins interrupt
        self.encoder_right_1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.right_motors_encoders_interrupt)
        self.encoder_right_2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.right_motors_encoders_interrupt)

        # configuring left wheel encoders pins
        self.encoder_left_1 = Pin(34, Pin.IN, Pin.PULL_UP)
        self.encoder_left_2 = Pin(35, Pin.IN, Pin.PULL_UP)

        # configuring left wheel encoders pins interrupt
        self.encoder_left_1.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.left_motors_encoders_interrupt)
        self.encoder_left_2.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self.left_motors_encoders_interrupt)

        # init encoder tick counters
        self.right_motors_encoders_counter = 0
        self.left_motors_encoders_counter = 0

        self.right_pwm_from_pid = 0
        self.left_pwm_from_pid = 0

        # reset PIDs initial values
        self.reset_pid_values()

        #configuring hardware timer 0 to trigger every 50ms for velocity control
        velocity_control_timer = Timer(0)
        velocity_control_timer.init(period=self.velocity_control_timer_interval, mode=Timer.PERIODIC, callback=self.velocity_control)

        print('finished initializing')


        self.test_run()

    def velocity_control(self, timer):
        self.adjust_right_motor_velocity()
        self.adjust_left_motor_velocity()

        temp_right_linear_velocity = self.calculate_linear_velocity(self.right_motors_encoders_counter)
        temp_left_linear_velocity = self.calculate_linear_velocity(self.left_motors_encoders_counter)

        self.right_linear_velocity = self.calculate_moving_average(self.right_linear_velocity_readings, temp_right_linear_velocity)
        self.left_linear_velocity = self.calculate_moving_average(self.left_linear_velocity_readings, temp_left_linear_velocity)

        print(f'{self.left_linear_velocity},   {self.right_linear_velocity}   ,{self.set_point_right_linear_velocity}, '
              f'{self.right_pwm_from_pid},{self.set_point_right_linear_velocity},{self.right_linear_velocity}')

        self.set_motors_pwm('right', self.right_pwm_from_pid)
        self.set_motors_pwm('left', self.left_pwm_from_pid)

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

    def calculate_linear_velocity(self, encoder_counts):
        linear_displacement = self.wheels_circumference * (encoder_counts / self.encoder_PPR)
        linear_velocity = linear_displacement / self.velocity_control_timer_interval_s
        return linear_velocity

    def calculate_moving_average(self, linear_velocity_list, current_linear_velocity_measurement):
        linear_velocity_list.append(current_linear_velocity_measurement)
        if len(linear_velocity_list) > self.linear_velocity_window_size:
            linear_velocity_list.pop(0)

        return round(sum(linear_velocity_list) / len(linear_velocity_list), 3)

    def stop_all_motors(self):
        self.BIN1_right.off()
        self.BIN2_right.off()
        self.AIN1_left.off()
        self.AIN2_left.off()
        self.set_motors_pwm('right', 0)
        self.set_motors_pwm('left', 0)

    def test_run(self):
        self.stop_all_motors()
        self.set_motors_direction('left', 'forward')
        self.set_motors_direction('right', 'forward')

        for speed in [0, 0.4, 0]:
            self.set_point_right_linear_velocity = speed
            self.set_point_left_linear_velocity = speed
            time.sleep(5)

        self.stop_all_motors()

    def adjust_right_motor_velocity(self):
        pid_output = self.right_linear_velocity_pid.compute(self.set_point_right_linear_velocity, self.right_linear_velocity, self.velocity_control_timer_interval_s)
        shifted_pid_pwm = pid_output + 5
        output_pwm = round(shifted_pid_pwm / 10 * 900)  # mapping pid output to 0-900 pwm
        self.right_pwm_from_pid = max(min(output_pwm, 900), 0)  # capping the values at 900 pwm

    def adjust_left_motor_velocity(self):
        pid_output = self.left_linear_velocity_pid.compute(self.set_point_left_linear_velocity, self.left_linear_velocity, self.velocity_control_timer_interval_s)
        shifted_pid_pwm = pid_output + 5
        output_pwm = round(shifted_pid_pwm / 10 * 900)  # mapping pid output to 0-900 pwm
        self.left_pwm_from_pid = max(min(output_pwm, 900), 0)  # capping the values at 900 pwm

    def reset_pid_values(self):
        # PIDs start at 450 pwm which causes the robot to jump, this function resets them to 0 after init.
        for x in range(15):
            self.right_linear_velocity_pid.compute(0, 0.35, self.velocity_control_timer_interval_s)
            self.left_linear_velocity_pid.compute(0, 0.35, self.velocity_control_timer_interval_s)

    def set_right_linear_velocity(self, velocity):
        pass

    def set_left_linear_velocity(self, velocity):
        pass

    def set_angular_velocity(self):
        pass