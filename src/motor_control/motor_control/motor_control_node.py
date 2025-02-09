import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import struct
import time


# Define the Roboclaw class
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

    # def read_encoder_m1(self, address):
    #     """Read encoder value for Motor 1."""
    #     self._send_command(address, self.Cmd.GETM1ENC)  # Ensure self.Cmd is used
    #     data = self._port.read(6)  # 4 bytes for encoder, 2 bytes for CRC
    #     if len(data) != 6:
    #         raise Exception("Invalid response length")
    #     encoder = int.from_bytes(data[0:4], byteorder='big', signed=True)
    #     crc_received = int.from_bytes(data[4:6], byteorder='big')
    #     if crc_received != self._crc & 0xFFFF:
    #         raise Exception("CRC mismatch")
    #     return encoder

    # def read_encoder_m2(self, address):
    #     """Read encoder value for Motor 2."""
    #     self._send_command(address, self.Cmd.GETM2ENC)  # Ensure self.Cmd is used
    #     data = self._port.read(6)  # 4 bytes for encoder, 2 bytes for CRC
    #     if len(data) != 6:
    #         raise Exception("Invalid response length")
    #     encoder = int.from_bytes(data[0:4], byteorder='big', signed=True)
    #     crc_received = int.from_bytes(data[4:6], byteorder='big')
    #     if crc_received != self._crc & 0xFFFF:
    #         raise Exception("CRC mismatch")
    #     return encoder



# Define the ROS2 node
class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Initialize the Roboclaw object
        self.roboclaw = Roboclaw('/dev/ttyACM0', 115200)
        self.address = 0x80

        # Create a subscriber to receive movement commands
        self.subscription = self.create_subscription(
            String,
            'motor_command',
            self.command_callback,
            10
        )
        self.get_logger().info("Motor control node has been started")

    def command_callback(self, msg):
        command = msg.data.upper()
        self.get_logger().info(f"Received command: {command}")

        if command == "FORWARD":
            self.move_backward()
        elif command == "BACKWARD":
            self.move_forward()
        elif command == "LEFT":
            self.turn_right()
        elif command == "RIGHT":
            self.turn_left()
        elif command == "STOP":
            self.stop()
        else:
            self.get_logger().warning(f"Unknown command: {command}")

    def move_forward(self):
        self.roboclaw.forward_m1(self.address,30)
        self.roboclaw.forward_m2(self.address, 30)
        self.get_logger().info("Moving forward")

    def move_backward(self):
        self.roboclaw.backward_m1(self.address, 30)
        self.roboclaw.backward_m2(self.address, 30)
        self.get_logger().info("Moving backward")

    def turn_left(self):
        self.roboclaw.forward_m2(self.address, 35)
        self.roboclaw.backward_m1(self.address, 35)
        self.get_logger().info("Turning left")

    def turn_right(self):
        self.roboclaw.forward_m1(self.address, 35)
        self.roboclaw.backward_m2(self.address, 35)
        self.get_logger().info("Turning right")

    def stop(self):
        self.roboclaw.stop_motors(self.address)
        self.get_logger().info("Stopping motors")


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
