#!/usr/bin/python3

import serial
import sys
import time

class UartLogsReader:
    timeoutSec = 120

    def __init__(self, device, baudrate):
        self.uart = serial.Serial(device, baudrate, timeout=self.timeoutSec)
        self.uart.reset_input_buffer()
        self.uart.reset_output_buffer()

    def run(self):
        print("Running UART logs reader on {} with speed {}".format(self.uart.port, self.uart.baudrate))

        deadline = time.time() + self.timeoutSec
        while True:
            line = self.uart.readline().rstrip()
            if not line and time.time() > deadline:
                print("ERROR: Timeout")
                return 1

            line = str(line, "utf-8")
            print(line)
            if "PASSED" in line:
                return 0
            if "FAILED" in line:
                return 1

reader = UartLogsReader("/dev/serial0", 115200)
status = reader.run()
sys.exit(status)
