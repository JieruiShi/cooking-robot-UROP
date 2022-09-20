from nidaqmx.constants import TerminalConfiguration
from nidaqmx.task import Task
import time

class DaqLink(object):
    """Takes care of measuring conductance, pH and temperature using readings from niDAQ device"""

    def __init__(self):
        #specify output pin voltages for pH sensor
        ao_task = Task()
        ao_task.ao_channels.add_ao_voltage_chan("Dev1/ao0")
        ao_task.ao_channels.add_ao_voltage_chan("Dev1/ao1")
        ao_task.write([1.65, 3.3], auto_start=True)

        #configure input channels to pH, T and C accordingly
        self.ai_task = Task()
        self.ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai0", terminal_config = TerminalConfiguration.RSE)
        self.ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai1", terminal_config = TerminalConfiguration.RSE)
        self.ai_task.ai_channels.add_ai_voltage_chan("Dev1/ai2", terminal_config = TerminalConfiguration.RSE)

        self.CVoltage = 4.90 #Voltage when probe is not measuring (open circuit)
        self.TVoltage = 0

    def receiveSignal(self):
        """returns voltage of pH probe, T1 and C1 respectively"""

        return self.ai_task.read()

    def calibrateConductanceParams(self):
        """updates the Cvoltage when it is not measuring anything"""

        self.receiveSignal()[2]

        pass

    def calibrateTemperatureParams(self):
        pass


    def voltageToConductance(self, inputVoltage):
        """Given an input voltage, calculate conductance in mS"""
        resistance = inputVoltage/(self.CVoltage - inputVoltage) * 220
        conductance = 1000/resistance
        return conductance

    def voltageToTemperature(self, inputVoltage):
        pass

if __name__ == '__main__':
    DaqTest = DaqLink()

    while True:
        received = DaqTest.receiveSignal()
        print(received)
        # print(7.0 - (received[0] - 1.65)/0.069)
        # print(DaqTest.voltageToConductance(received[2]))
        time.sleep(0.5)
