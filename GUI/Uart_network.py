import serial
import csv


class UART:
    def __init__(self):
        self.messageToSend = 0
        self.uartStateFlag = 0
        self.data = [0, 0]
        self.xAxes = [0.000]
        self.yAxes = [0.000]
        self.received_data = []
        self.timeCount = 0.000

    def uartInit(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.02)
            if self.ser.isOpen():
                print(f"Serial connection opened on {self.port} at {self.baudrate} baud.")
            else:
                print("Serial port is not open.")
        except serial.SerialException as e:
            print("Serial Exception:", e)
            return None

    def uartShutdown(self):
        self.ser.close()

    def uartState(self, uartStateFlag, port, baudrate):
        self.uartStateFlag = uartStateFlag
        self.port = port
        self.baudrate = baudrate
        if(self.uartStateFlag):
            self.uartInit()
        else:
            self.uartShutdown()
            
    
    def send_data(self, data):
        try:
            if self.ser.isOpen():
                self.ser.write(data)  # Sending data as bytes
                print(f"Sent: {data}")
            else:
                print("Serial port is not open.")
        except serial.SerialException as e:
            print("Serial Exception:", e)
    
    def receive_data(self, plotFlag):
        self.plotFlag = plotFlag
        try:
            if self.ser.isOpen():
                if self.ser.in_waiting >0: 
                    self.received_data = self.ser.readline().decode("ascii").strip().split("#")
                    # print(f"{self.received_data}")
                    # print(f"{self.received_data[0]}, {self.received_data[1]}")
                    if(self.received_data[0] == '0'):
                        self.data[0] = round(float(self.received_data[1]), 3)
                        # print(self.data[0])
                    elif(self.received_data[0] == '1'):
                        self.data[1] = int(self.received_data[1])
                        print(self.received_data[1])
                        if(self.plotFlag):
                            self.timeCount = round(self.timeCount + 1, 3)  #period 
                            self.xAxes.append(self.timeCount)
                            self.yAxes.append(self.data[1]) 
                            # self.savedData ={
                            #     '0': self.timeCount,
                            #     '1': self.data[1]
                            # }
                            # # print(self.savedData)
                            # with open('data.csv', 'a', newline='') as file:
                            #     self.fieldnames = ['0', '1']
                            #     writer = csv.DictWriter(file, self.fieldnames)
                            #     writer.writerow(self.savedData)
                        
            else:
                print("Serial port is not open.")
        except serial.SerialException as e:
            print("Serial Exception:", e)
    
    def close_serial(self):
        try:
            if self.ser.isOpen():
                self.ser.close()
                print("Serial connection closed.")
        except serial.SerialException as e:
            print("Serial Exception:", e)
    
       
     