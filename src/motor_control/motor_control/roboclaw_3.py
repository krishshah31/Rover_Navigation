import random
import serial
import struct
import time

class Roboclaw:
    'Roboclaw Interface Class'

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
        try:
            self._port = serial.Serial(
                port=self.comport,
                baudrate=self.rate,
                timeout=1,
                write_timeout=1,
                inter_byte_timeout=self.timeout
            )
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self._port = None
            return False
        return True

    def close(self):
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
        return self._write1(address, self.Cmd.M1BACKWARD, speed)

    def backward_m1(self, address, speed):
        return self._write1(address, self.Cmd.M1FORWARD, speed)

    def forward_m2(self, address, speed):
        return self._write1(address, self.Cmd.M2BACKWARD, speed)

    def backward_m2(self, address, speed):
        return self._write1(address, self.Cmd.M2FORWARD, speed)

    def stop_motors(self, address):
        self.forward_m1(address, 0)
        self.forward_m2(address, 0)

    def read_encoder_m1(self, address):
        self._send_command(address, self.Cmd.GETM1ENC)
        return self._read_word()

    def read_encoder_m2(self, address):
        self._send_command(address, self.Cmd.GETM2ENC)
        return self._read_word()

    def read_currents(self, address):
        self._send_command(address, self.Cmd.GETCURRENTS)
        return self._read_word()

    def set_m1_pid(self, address, p, i, d, qpps):
        self._send_command(address, self.Cmd.SETM1PID)
        self._write_word(int(d * 65536))
        self._write_word(int(p * 65536))
        self._write_word(int(i * 65536))
        self._write_word(qpps)
        return self._write_checksum()

    def read_m1_pid(self, address):
        self._send_command(address, self.Cmd.READM1PID)
        p = self._read_word() / 65536
        i = self._read_word() / 65536
        d = self._read_word() / 65536
        qpps = self._read_word()
        return (p, i, d, qpps)

    def speed_m1_accel(self, address, accel, speed):
        self._send_command(address, self.Cmd.M1SPEEDACCEL)
        self._write_word(accel)
        self._write_word(speed)
        return self._write_checksum()

    def speed_distance_m1(self, address, speed, distance, buffer):
        self._send_command(address, self.Cmd.M1SPEEDDIST)
        self._write_word(speed)
        self._write_word(distance)
        self._write_byte(buffer)
        return self._write_checksum()
