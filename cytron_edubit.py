### cytron_edubit.py v1.0
### A CircuitPython library for the Cytron edu:bit

### Tested with an Adafruit CLUE and CircuitPython 5.3.1

### i2c bus must be set to 100kHz - CircuitPython 5.3.1 default is 400kHz

### MIT License

### Copyright (c) 2020 Kevin J. Walters

### Permission is hereby granted, free of charge, to any person obtaining a copy
### of this software and associated documentation files (the "Software"), to deal
### in the Software without restriction, including without constraination the rights
### to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
### copies of the Software, and to permit persons to whom the Software is
### furnished to do so, subject to the following conditions:

### The above copyright notice and this permission notice shall be included in all
### copies or substantial portions of the Software.

### THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
### IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
### FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
### AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
### LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
### OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
### SOFTWARE.

### This is based on https://github.com/Bhavithiran97/micropython-edubit

### 5.3.1 board.I2C() is still at 400kHz https://github.com/adafruit/Adafruit_CircuitPython_CLUE/issues/36  ### pylint: disable=line-too-long


import time

import board
import analogio
import digitalio

import neopixel
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
import adafruit_bus_device.i2c_device as i2c_device


def constrain(value, v_min, v_max):
    return max(min(value, v_max), v_min)


class Edubit:
    I2C_ADDRESS = 0x08

    NEOPIXEL_PIN = board.P13

    RED_LED_PIN = board.P14
    YELLOW_LED_PIN = board.P15
    GREEN_LED_PIN = board.P16

    SOUND_BIT_PIN = board.P1
    POTENTIO_BIT_PIN = board.P2
    IR_BIT_PIN = board.P8

    REG_ADD_REVISION = 0
    REG_ADD_SERVO_1 = 1
    REG_ADD_SERVO_2 = 2
    REG_ADD_SERVO_3 = 3
    REG_ADD_M1A = 4
    REG_ADD_M1B = 5
    REG_ADD_M2A = 6
    REG_ADD_M2B = 7
    REG_ADD_LB_UTH = 8
    REG_ADD_LB_LTH = 9
    REG_ADD_OV_TH = 10
    REG_ADD_VIN = 11
    REG_ADD_PWR_STATE = 12
    REG_ADD_LB_STATE = 13
    REG_ADD_OV_STATE = 14

    ### These all depend on self.i2c_device being set
    revision_reg = ROUnaryStruct(REG_ADD_REVISION, "<B")

    servo_1_reg = UnaryStruct(REG_ADD_SERVO_1, "<B")
    servo_2_reg = UnaryStruct(REG_ADD_SERVO_2, "<B")
    servo_3_reg = UnaryStruct(REG_ADD_SERVO_3, "<B")
    m1a_reg = UnaryStruct(REG_ADD_M1A, "<B")
    m1b_reg = UnaryStruct(REG_ADD_M1B, "<B")
    m2a_reg = UnaryStruct(REG_ADD_M2A, "<B")
    m2b_reg = UnaryStruct(REG_ADD_M2B, "<B")
    lb_uth_reg = UnaryStruct(REG_ADD_LB_UTH, "<B")
    lb_lth_reg = UnaryStruct(REG_ADD_LB_LTH, "<B")
    ov_th_reg = UnaryStruct(REG_ADD_OV_TH, "<B")

    vin_reg = ROUnaryStruct(REG_ADD_VIN, "<B")
    pwr_state_reg = ROUnaryStruct(REG_ADD_PWR_STATE, "<B")
    lb_state_reg = ROUnaryStruct(REG_ADD_LB_STATE, "<B")
    ov_state_reg = ROUnaryStruct(REG_ADD_OV_STATE, "<B")

    S1 = REG_ADD_SERVO_1
    S2 = REG_ADD_SERVO_2
    S3 = REG_ADD_SERVO_3

    M1 = 0
    M2 = 1
    ALL = 1000

    FORWARD = 0
    BACKWARD = 1

    RED = 0
    YELLOW = 1
    GREEN = 2


    def __init__(self, i2c=None, device_address=I2C_ADDRESS):
        self._i2c = board.I2C() if i2c is None else i2c
        self.i2c_device = i2c_device.I2CDevice(i2c, device_address)

        if self.is_power_on():
            self.init_motor_servos()
        else:
            raise RuntimeError("Edu:bit is not responding to i2c"
                               " - powered on?")

        try:
            self.pixels = neopixel.NeoPixel(self.NEOPIXEL_PIN, 4)
        except ValueError:
            raise RuntimeError("NeoPixel pin already in use"
                               " - Edubit object already instantiated?")

        self.pixels.fill((0, 0, 0))

        self._red_led = digitalio.DigitalInOut(self.RED_LED_PIN)
        self._red_led.switch_to_output(False)
        self._yellow_led = digitalio.DigitalInOut(self.YELLOW_LED_PIN)
        self._yellow_led.switch_to_output(False)
        self._green_led = digitalio.DigitalInOut(self.GREEN_LED_PIN)
        self._green_led.switch_to_output(False)

        self._sound = analogio.AnalogIn(self.SOUND_BIT_PIN)
        self._potentio = analogio.AnalogIn(self.POTENTIO_BIT_PIN)
        self._ir = digitalio.DigitalInOut(self.IR_BIT_PIN)
        self._ir.pull = digitalio.Pull.DOWN

        time.sleep(0.2)  ### 200ms pause - the MicroPython library does this


    def init_motor_servos(self):
        self.brake_motor(self.ALL)
        self.disable_servo(self.ALL)


    def _set_motor_speed(self, motorChannel, fwd_speed, rev_speed):
        if fwd_speed > 0 and rev_speed > 0:
            raise ValueError("At least one speed arg must be 0")

        if motorChannel in (self.M1, self.ALL):
            self.m1a_reg = fwd_speed
            self.m1b_reg = rev_speed

        if motorChannel in (self.M2, self.ALL):
            self.m2a_reg = fwd_speed
            self.m2b_reg = rev_speed


    def brake_motor(self, motorChannel):
        self._set_motor_speed(motorChannel, 0, 0)


    def run_motor(self, motorChannel, direction, speed):
        speed = constrain(speed, 0, 255)
        forward = speed if direction == self.FORWARD else 0
        backward = speed if direction == self.BACKWARD else 0
        self._set_motor_speed(motorChannel, forward, backward)


    def _set_servo_value(self, servo, value):
        if servo in (self.S1, self.ALL):
            self.servo_1_reg = value
        if servo in (self.S2, self.ALL):
            self.servo_2_reg = value
        if servo in (self.S3, self.ALL):
            self.servo_3_reg = value


    def disable_servo(self, servo):
        self._set_servo_value(servo, 0)


    def set_servo_position(self, servo, position):
        position = constrain(position, 0, 180)

        ### This formula comes from MicroPython library
        pulseWidth = int(position * 20 / 18 + 50)
        self._set_servo_value(servo, pulseWidth)


    def is_power_on(self):
        return self.pwr_state_reg != 0


    def is_low_batt(self):
        return self.lb_state_reg != 0


    def is_overvoltage(self):
        return self.ov_state_reg != 0


    def read_Vin(self):
        return self.vin_reg / 10.0


    def set_led(self, color, state):
        if color in (self.RED, self.ALL):
            self._red_led.value = bool(state)
        if color in (self.YELLOW, self.ALL):
            self._yellow_led.value = bool(state)
        if color in (self.GREEN, self.ALL):
            self._green_led.value = bool(state)


    def read_sound_sensor(self):
        return self._sound.value


    def read_pot_value(self):
        return self._potentio.value


    def read_IR_sensor(self):
        return self._ir.value


    def is_IR_sensor_triggered(self):
        return not self._ir.value
