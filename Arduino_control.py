import time
from pySerialTransfer import pySerialTransfer as txfer

class ArduinoLink(object):

    def __init__(self):
        self.link = txfer.SerialTransfer('COM3')
        self.link.open()

    def sendSignal(self, saltTime: float, waterTime: float):
        """send saltTime, waterTime to Arduino to
        indicate the time to turn salt dispenser and water pump on"""
        self.link.tx_obj(saltTime, 0)
        self.link.tx_obj(waterTime, 4)
        self.link.send(8)

    def receiveSignal(self):
        if self.link.available():
            distance = self.link.rx_obj(float, 0, 4)
            saltOn = self.link.rx_obj(bool, 4, 1)
            waterOn = self.link.rx_obj(bool, 5, 1)
            return distance, saltOn, waterOn
        else:
            return -1, False, False

    def clearSerialBuffer(self):
        """clears up the serial buffer by reading every byte"""
        self.link.connection.reset_input_buffer()

if __name__ == '__main__':
    ArduinoTest = ArduinoLink()
    time.sleep(3)
    ArduinoTest.sendSignal(0.0,1.5)
    time.sleep(5)
    # ArduinoTest.sendSignal(-100.0,-100.0)
    while True:
        print(ArduinoTest.receiveSignal())
        ArduinoTest.clearSerialBuffer()
        time.sleep(1)