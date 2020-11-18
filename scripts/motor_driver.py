#!/usr/bin/env python
from smbus2 import SMBus, i2c_msg
import rospy
from std_msgs.msg import Int64

# command bytes
EXIT_SAFE_START = 0x83
FORWARD = 0x85
REVERSE = 0x86
BRAKE = 0x92
GET_VAR = 0xA1
SET_LIMIT = 0xA2
STOP_MOTOR = 0xE0

# variables ID
TARGET_SPEED = 20
CURRENT_SPEED = 21
ERROR_STATUS = 0

# limits ID
MAX_SPEED = 0
MAX_ACC = 1
MAX_DEC = 2
BRAKE_DUR = 3


class I2CMotorDriver(object):
    def __init__(self):
        bus = rospy.get_param("polulu_motor/i2c_bus")
        address = rospy.get_param("polulu_motor/motor_address")
        self.rate = rospy.get_param("polulu_motor/rate")
        self.bus = SMBus(bus)
        self.address = address
        self.exit_safe_start()
        # get the motor status
        error_status = self.get_error_status()
        if error_status == 0:
            rospy.loginfo("Motor driver initialized successfully")
        else:
            rospy.loginfo ("cannot initialize motor driver")
        self.config_motor_limits()
        self.config_motor_callbacks()
        self.provide_feedback()

    # Sends the Exit Safe Start command, which is required to drive the motor.
    def exit_safe_start(self):
        write = i2c_msg.write(self.address, [EXIT_SAFE_START])
        self.bus.i2c_rdwr(write)

    def config_motor_limits(self):
        max_speed = rospy.get_param("polulu_motor/max_speed")
        error = self.set_max_speed(max_speed)

        if error == 0:
            rospy.loginfo ("no problem setting speed limits")
        else:
            rospy.loginfo ("cannot set speed limits ")

        max_acc = rospy.get_param("polulu_motor/max_acc")
        error = self.set_max_acc(max_acc)
        if error == 0:
            rospy.loginfo ("no problem setting acceleration limits")
        else:
            rospy.loginfo ("cannot set acceleration limits ")

        max_dec = rospy.get_param("polulu_motor/max_dec")
        error = self.set_max_dec(max_dec)
        if error == 0:
            rospy.loginfo ("no problem setting deceleration limits")
        else:
            rospy.loginfo ("cannot set deceleration limits ")

        brake_dur = rospy.get_param("polulu_motor/brake_dur")
        error = self.set_brake_dur(brake_dur)
        if error == 0:
            rospy.loginfo ("no problem setting braking duration")
        else:
            rospy.loginfo ("cannot set braking duration")

    def config_motor_callbacks(self):
        speed_command_topic = rospy.get_param("polulu_motor/set_target_speed_topic")
        brake_amount_topic = rospy.get_param("polulu_motor/set_brake_amount_topic")
        rospy.Subscriber(speed_command_topic, Int64, self._speed_callback)
        rospy.Subscriber(brake_amount_topic, Int64, self._braking_callback)

    # set the motor speed which is a percentage from 0 to 100. 100% full speed.
    def set_target_speed(self, speed):
        if speed <= 100 and speed >= -100:
            direction = FORWARD
            if speed < 0:
                direction = REVERSE
		speed = -1*speed
            buffer = [direction, 0, speed]
            write = i2c_msg.write(self.address, buffer)
            self.bus.i2c_rdwr(write)
        else:
            rospy.loginfo ("speed should be a percentage from 0 to 100")

    # Gets the specified variable as an unsigned value.
    def _get_variable(self, id):
        write = i2c_msg.write(self.address, [GET_VAR, id])
        read = i2c_msg.read(self.address, 2)
        self.bus.i2c_rdwr(write, read)
        b = list(read)
        return b[0] + 256 * b[1]

    # Gets the specified variable as a signed value.
    def _get_variable_signed(self, id):
        value = self._get_variable(id)
        if value >= 0x8000:
            value -= 0x10000
        return value

    # Gets the target speed (-3200 to 3200).
    def get_target_speed(self):
        return self._get_variable_signed(TARGET_SPEED)

    def get_current_speed(self):
        return self._get_variable_signed(CURRENT_SPEED)

    # Gets a number where each bit represents a different error, and the
    # bit is 1 if the error is currently active.
    # See the user's guide for definitions of the different error bits.
    def get_error_status(self):
        return self._get_variable(ERROR_STATUS)

    def brake(self, brake_amount):
        if brake_amount <= 100 and brake_amount >= 0:
            brake_value = (32/100) * brake_amount
            buffer = [BRAKE, brake_value]
            write = i2c_msg.write(self.address, buffer)
            self.bus.i2c_rdwr(write)
        else:
            rospy.loginfo ("brake amount should be a percentage")

    def _set_limit(self, id, limit):
        limit_byte_1 = limit % 128
        limit_byte_2 = limit / 128
        write = i2c_msg.write(self.address, [SET_LIMIT, id, limit_byte_1, limit_byte_2])
        read = i2c_msg.read(self.address, 1)
        self.bus.i2c_rdwr(write, read)
				b = list(read)
        return b[0]

    # max speed should be given as a percentage from 0 to 100
    def set_max_speed(self, max_speed):
        max_speed_val = int((3200/100) * max_speed)
        error = self._set_limit(MAX_SPEED, max_speed_val)
        return error

    # max acceleration should be given as a percentage from 0 to 100 but 0 means no limits
    def set_max_acc(self, max_acc):
        max_acc_val = int((3200/100) * max_acc)
        error = self._set_limit(MAX_ACC, max_acc_val)
        return error

    # max deceleration should be given as a percentage from 0 to 100 but 0 means no limits
    def set_max_dec(self, max_dec):
        max_dec_val = int((3200/100) * max_dec)
        error = self._set_limit(MAX_DEC, max_dec_val)
        return error

    # brake duration is considered multiples of 4 so if value like 9 is given it will be considered 12
    # the unit is ms
    def set_brake_dur(self, brake_dur):
        brake_dur_val = int(brake_dur % 4 + 4)
        if brake_dur_val > 16384:
            # max allowable value
            brake_dur_val = 16384
        error = self._set_limit(BRAKE_DUR, brake_dur_val)
        return error

    def _speed_callback(self, msg):
        self.brake(0)
        self.set_target_speed(msg.data)

    def _braking_callback(self, msg):
        self.set_target_speed(0)
        self.brake(msg.data)

    def provide_feedback(self):
        target_speed_topic = rospy.get_param("polulu_motor/get_target_speed_topic")
        current_speed_topic = rospy.get_param("polulu_motor/get_current_speed_topic")
        target_speed_pub = rospy.Publisher(target_speed_topic, Int64, queue_size=10)
        current_speed_pub = rospy.Publisher(current_speed_topic, Int64, queue_size=10)
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            target_speed_pub.publish(self.get_target_speed())
            current_speed_pub.publish(self.get_current_speed())
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node("motor_driver_node")
        motor = I2CMotorDriver()
    except rospy.ROSInterruptException:
        pass
