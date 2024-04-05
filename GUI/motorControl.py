
import struct

class MOTOR_CONTROL:

    def __init__(self):
        self.motorStateFlag = 0
        self.dataToSend = [0, 0, 0]

    def startStopMotor(self, uart, state):
        self.motorStateFlag = state
        if(self.motorStateFlag):
            self.dataToSend[0] = 0
            self.dataToSend[1] = 1
            uart.send_data(self.dataToSend)
        else:
            self.dataToSend[0] = 0
            self.dataToSend[1] = 0
            uart.send_data(self.dataToSend)

    def setMotorRpm(self, uart, value):
        self.dataToSend[0] = 1
        self.dataToSend[1] = (value >> 8) & 0xFF
        self.dataToSend[2] = value & 0xFF
        uart.send_data(self.dataToSend)
