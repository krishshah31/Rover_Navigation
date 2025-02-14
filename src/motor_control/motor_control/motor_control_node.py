import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import serial
from time import time

class Roboclaw:
    class Cmd:
        """Command Enums"""
        M1FORWARD = 0
        M1BACKWARD = 1
        M2FORWARD = 4
        M2BACKWARD = 5
        GETVERSION = 21
        RESETENC = 20
        GETM1ENC = 16
        GETM2ENC = 17
        GETM1SPEED = 18
        GETM2SPEED = 19
        SETM1PID = 28
        SETM2PID = 29
        READM1PID = 55
        READM2PID = 56
        M1DUTY = 32
        M2DUTY = 33
        MIXEDDUTY = 34
        M1SPEED = 35
        M2SPEED = 36
        MIXEDSPEED = 37
        M1SPEEDACCEL = 38
        M2SPEEDACCEL = 39
        MIXEDSPEEDACCEL = 40
        M1SPEEDDIST = 41
        M2SPEEDDIST = 42
        MIXEDSPEEDDIST = 43
        GETMBATT = 24
        GETLBATT = 25
        GETCURRENTS = 49

    def __init__(self, comport, rate, timeout=0.01, retries=3):
        self.comport = comport
        self.rate = rate
        self.timeout = timeout
        self._trystimeout = retries
        self._crc = 0
        self._port = None
        self.open()

    def open(self):
        """Open the serial port."""
        try:
            self._port = serial.Serial(
                port=self.comport,
                baudrate=self.rate,
                timeout=1,
                write_timeout=1,
                inter_byte_timeout=self.timeout
            )
            print(f"Opened port: {self.comport}")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self._port = None
            return False
        return True

    def close(self):
        """Close the serial port."""
        if self._port and self._port.is_open:
            self._port.close()

    def crc_clear(self):
        self._crc = 0

    def crc_update(self, data):
        self._crc ^= (data << 8)
        for _ in range(8):
            if self._crc & 0x8000:
                self._crc = ((self._crc << 1) ^ 0x1021)
            else:
                self._crc <<= 1

    def _send_command(self, address, command):
        self.crc_clear()
        self.crc_update(address)
        self._port.write(address.to_bytes(1, 'big'))
        self.crc_update(command)
        self._port.write(command.to_bytes(1, 'big'))

    def _write_byte(self, val):
        self.crc_update(val & 0xFF)
        self._port.write(val.to_bytes(1, 'big'))

    def _write_checksum(self):
        self._write_word(self._crc & 0xFFFF)
        response = self._port.read(1)
        return response == b'\xAA'

    def _write_word(self, val):
        self._write_byte((val >> 8) & 0xFF)
        self._write_byte(val & 0xFF)

    def _write1(self, address, cmd, val):
        tries = self._trystimeout
        while tries:
            self._send_command(address, cmd)
            self._write_byte(val)
            if self._write_checksum():
                return True
            tries -= 1
        return False

    def forward_m1(self, address, speed):
        return self._write1(address, 0, speed)

    def backward_m1(self, address, speed):
        return self._write1(address, 1, speed)

    def forward_m2(self, address, speed):
        return self._write1(address, 4, speed)

    def backward_m2(self, address, speed):
        return self._write1(address, 5, speed)

    def stop_motors(self, address):
        self.forward_m1(address, 0)
        self.forward_m2(address, 0)

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.roboclaw = Roboclaw('/dev/ttyACM0', 115200)
        self.address = 0x80
        self.emergency_stop = False
        self.recovery_mode = False
        self.command_subscription = self.create_subscription(
            String,
            'motor_command',
            self.command_callback,
            10
        )
        self.get_logger().info("Motor control node has started.")

    def command_callback(self, msg):
        try:
            command_parts = msg.data.strip().split()
            if len(command_parts) < 3:
                self.get_logger().warning("Invalid command format. Expected format: 'DIRECTION M1_SPEED M2_SPEED'")
                return

            direction = command_parts[0].upper()
            m1_speed = int(command_parts[1])
            m2_speed = int(command_parts[2])

            if direction == "RECOVER":
                self.emergency_stop = False
                self.recovery_mode = True
                self.get_logger().info("Recovery mode activated. Commands will now be accepted.")
                return

            if self.emergency_stop and not self.recovery_mode:
                self.get_logger().warn("Emergency stop active. Ignoring commands.")
                return

            self.get_logger().info(f"Received command: {direction}, M1 Speed: {m1_speed}, M2 Speed: {m2_speed}")

            if direction == "BACKWARD":
                self.roboclaw.forward_m1(self.address, m1_speed)
                self.roboclaw.forward_m2(self.address, m2_speed)
            elif direction == "FORWARD":
                self.roboclaw.backward_m1(self.address, m1_speed)
                self.roboclaw.backward_m2(self.address, m2_speed)
            elif direction == "RIGHT":
                self.roboclaw.backward_m1(self.address, m1_speed)
                self.roboclaw.forward_m2(self.address, m2_speed)
            elif direction == "LEFT":
                self.roboclaw.forward_m1(self.address, m1_speed)
                self.roboclaw.backward_m2(self.address, m2_speed)
            elif direction == "STOP":
                self.roboclaw.stop_motors(self.address)
            else:
                self.get_logger().warning(f"Unknown command: {direction}")
        
        except ValueError:
            self.get_logger().error("Invalid speed values. Please provide numerical speed values for M1 and M2.")


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
