#!/usr/bin/python3

import hashlib
import random
import secrets
import threading
import time
from abc import ABC, abstractmethod

import common

def generateRandomData(size):
    return secrets.token_bytes(size)

class UartClient(common.UartEndpoint, ABC):
    def run(self):
        print("Running UART test server on {} with speed {}".format(self.uart.port, self.uart.baudrate))

        try:
            self.sendPing()

            baudrates = (115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200)
            for baudrate in baudrates:
                self.sendSettings(baudrate)
                self.sendPing()
                self.testCase()

            self.sendClose()
            print("Test success")
        except common.UartTestException as exc:
            print("Error:", exc.__class__.__name__)

    def sendPing(self):
        response = self.sendRequest({"type": "ping"})
        if response["type"] != "pong":
            raise common.BadResponseException()

    def sendSettings(self, baudrate):
        response = self.sendRequest({"type": "settings", "baudrate": baudrate})
        if response["type"] != "ack":
            raise common.BadResponseException()

        self.configurePort(baudrate)
        time.sleep(2)

    @abstractmethod
    def testCase(self):
        pass

    def sendClose(self):
        response = self.sendRequest({"type": "close"})
        if response["type"] != "ack":
            raise common.BadResponseException()

class SynchronousClient(UartClient):
    def testCase(self):
        size = random.randrange(1, 16 * 1024)
        response = self.sendRequest({"type": "data", "size": size})
        if response["type"] != "ack":
            raise common.BadResponseException()

        while size:
            packetSize = min(random.randrange(1, 32), size)
            sendData = generateRandomData(packetSize)
            self.uart.write(sendData)

            receiveData = self.uart.read(packetSize)
            if sendData != receiveData:
                raise common.DataMismatchException()

            size -= packetSize

class AsynchronousClient(UartClient):
    def threadSender(self, size, results):
        checksum = hashlib.sha256()
        toSend = size
        while toSend:
            packetSize = min(random.randrange(1, 32), toSend)
            data = generateRandomData(packetSize)
            self.uart.write(data)
            checksum.update(data)

            toSend -= packetSize

        results["sender"] = checksum.hexdigest()

    def threadReceiver(self, size, results):
        checksum = hashlib.sha256()
        toReceive = size
        while toReceive:
            data = self.uart.read()
            if not data:
                raise common.TimeoutException()

            checksum.update(data)
            toReceive -= len(data)

        results["receiver"] = checksum.hexdigest()

    def testCase(self):
        size = random.randrange(1, 16 * 1024)
        response = self.sendRequest({"type": "data", "size": size})
        if response["type"] != "ack":
            raise common.BadResponseException()

        results = {}
        sender = threading.Thread(target=self.threadSender, args=(size, results))
        receiver = threading.Thread(target=self.threadReceiver, args=(size, results))

        sender.start()
        receiver.start()
        sender.join()
        receiver.join()

        if results["sender"] != results["receiver"]:
            raise common.DataMismatchException()

client = AsynchronousClient("/dev/ttyUSB0", 115200)
client.run()
