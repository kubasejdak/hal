#!/usr/bin/python3

import json

import serial

class UartTestException(Exception): pass

class TimeoutException(UartTestException): pass

class BadResponseException(UartTestException): pass

class DataMismatchException(UartTestException): pass

class UartEndpoint:
    def __init__(self, device, baudrate):
        self.device = device
        self.uart = None
        self.configurePort(baudrate)

    def configurePort(self, baudrate):
        if self.uart:
            del self.uart

        self.uart = serial.Serial(self.device, baudrate, timeout=10)
        self.uart.reset_input_buffer()
        self.uart.reset_output_buffer()

    def sendRequest(self, request):
        msgOut = json.dumps(request)
        self.uart.write(msgOut.encode() + b"\n")

        return self.receiveRequest()

    def receiveRequest(self):
        msgIn = self.uart.readline().rstrip()
        if not msgIn:
            raise TimeoutException()

        return json.loads(msgIn)

    def sendResponse(self, response):
        msgOut = json.dumps(response)
        self.uart.write(msgOut.encode() + b"\n")
