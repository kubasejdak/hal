#!/usr/bin/python3

import common
import daemon

class UartServer(common.UartEndpoint):
    def run(self):
        print("Running UART test server on {} with speed {}".format(self.uart.port, self.uart.baudrate))

        try:
            while True:
                request = self.receiveRequest()
                print(request)
                if request["type"] == "ping":
                    self.onPing()
                elif request["type"] == "settings":
                    self.onSettings(request)
                elif request["type"] == "data":
                    self.onData(request)
                elif request["type"] == "close":
                    self.onClose()
                    break
        except common.UartTestException as exc:
            print("Error:", exc.__class__.__name__)

    def onPing(self):
        self.sendResponse({"type": "pong"})

    def onSettings(self, request):
        self.sendResponse({"type": "ack"})
        self.configurePort(request["baudrate"])

    def onData(self, request):
        self.sendResponse({"type": "ack"})

        size = request["size"]
        for i in range(size):
            data = self.uart.read()
            if not data:
                raise common.TimeoutException()

            self.uart.write(data)

    def onClose(self):
        self.sendResponse({"type": "ack"})

with daemon.DaemonContext():
    server = UartServer("/dev/ttyUSB1", 115200)
    server.run()
