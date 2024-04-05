import customtkinter as ctk
from Uart_network import UART
from motorControl import MOTOR_CONTROL
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import numpy as np
from tkinter import messagebox
import time

ctk.set_default_color_theme("green")
ctk.set_appearance_mode("dark")

class IPS_PROJECT_GUI:
    def __init__(self, root):
        self.root = root
        self.root.title("IPS Project")
        self.root.geometry("1000x600")
        
        self.root.columnconfigure(0, weight= 1)
        self.root.columnconfigure(1, weight= 10)
        
        self.root.rowconfigure(0, weight=10)
        self.root.rowconfigure(1, weight=1)

        self.root.protocol("WM_DELETE_WINDOW", self.close_window)
        
        self.count = 0
        self.setValue = [0]
        self.constantValue = []

        self.uartConnection = UART()
        self.motorIns = MOTOR_CONTROL()
    
        #motorControlFrame
        self.frame11 = ctk.CTkFrame(master=self.root)
        self.frame11.grid(row=0, column=0, rowspan=2,padx=(0,0), pady=(0,0),sticky="nwes")
        self.frame11.columnconfigure(0, weight=1)
        self.frame11.rowconfigure(0, weight=1)
        self.frame11.rowconfigure(1, weight=1)
        self.frame11.rowconfigure(2, weight=1)
        self.frame11.rowconfigure(3, weight=1)
        self.frame11.rowconfigure(4, weight=1)
        self.frame11.rowconfigure(5, weight=1)
        self.frame11.rowconfigure(6, weight=1)
        self.frame11.rowconfigure(7, weight=1)
        self.frame11.rowconfigure(8, weight=1)
        self.frame11.rowconfigure(9, weight=1)
        self.frame11.rowconfigure(10, weight=1)
        self.frame11.rowconfigure(11, weight=1)
        self.frame11.rowconfigure(12, weight=1)
        
        #frame11Content
        self.frame11_title = ctk.CTkLabel(master=self.frame11, text="Control Panel", fg_color="#333333", corner_radius=10)
        self.frame11_title.grid(row=0, column=0, padx=5, sticky="ew")
        self.uartPort = ctk.CTkEntry(master=self.frame11, placeholder_text="Port")
        self.uartPort.grid(row=1, column=0, padx=10, sticky="ews")
        self.uartPort.insert(0, "COM3")
        self.uartBaudrate = ctk.CTkEntry(master=self.frame11, placeholder_text="Baudrate")
        self.uartBaudrate.grid(row=2, column=0, padx=10, sticky="ew")
        self.uartBaudrate.insert(0, "115200")
        self.uartStateSwitch = ctk.CTkSwitch(master= self.frame11 , text="Uart", command=self.uartSwitchAction, onvalue="on", offvalue="off")
        self.uartStateSwitch.grid(row=3, column=0, padx=(10,0), sticky="new")
        self.startPlotButton = ctk.CTkButton(master=self.frame11, text="Start Plot", command=self.startStopPlotAction)
        self.startPlotButton.grid(row=4, column=0, padx=10, sticky="nwe")
        self.resetButton = ctk.CTkButton(master= self.frame11, text="Reset Plot", command=self.resetPlotAction)
        self.resetButton.grid(row=5, column=0, padx=10, sticky="nwe")
        self.motorStateSwitch = ctk.CTkSwitch(master= self.frame11, text="Motor State", command=self.motorSwitchAction, onvalue="on", offvalue="off")
        self.motorStateSwitch.grid(row=6, column=0, padx=(10,0), sticky="sew")
        self.rpm_entry = ctk.CTkEntry(master=self.frame11, placeholder_text="Set RPM []")
        self.rpm_entry.grid(row=7, column=0, padx=10, sticky="ew")
        self.set_rpm_button = ctk.CTkButton(master=self.frame11, text="Set", command=self.rpm_buttonAction)
        self.set_rpm_button.grid(row=8, column=0, padx=10, sticky="wen")
        self.desiredRpmTitle = ctk.CTkLabel(master=self.frame11, text="Desired RPM")
        self.desiredRpmTitle.grid(row=9, column=0, padx=15, sticky="ws")
        self.desiredRpmOutput = ctk.CTkLabel(master=self.frame11, fg_color="#343638", corner_radius=10, text="0")
        self.desiredRpmOutput.grid(row=10, column=0, padx=10, sticky="new")
        self.realRpmTitle = ctk.CTkLabel(master=self.frame11, text="Current")
        self.realRpmTitle.grid(row=11, column=0, padx=15, sticky="ws")
        self.realRpmOutput = ctk.CTkLabel(master=self.frame11, fg_color="#343638", corner_radius=10, text="0")
        self.realRpmOutput.grid(row=12, column=0, padx=10, sticky="new")
        
        self.connectionStateFlag = False
        self.plotFlag = False
        
        plt.xlabel('Time-S')
        plt.ylabel('Speed')
        plt.title('Speed')
        plt.grid(True)
        
        self.canvas = FigureCanvasTkAgg(plt.gcf(), master=self.root)
        self.canvas.draw()
        toolbar = NavigationToolbar2Tk(self.canvas, self.root, pack_toolbar=False)
        toolbar.update()
        self.canvas.get_tk_widget().grid(row=0, column=1, rowspan=1,sticky="nsew")
        toolbar.grid(row=1, column=1, sticky="nswe")

        
        self.whileLoop()

    def startStopPlot(self):
        plt.plot(self.uartConnection.xAxes, self.uartConnection.yAxes,  label='Dataset 1', marker='', linestyle='-', color="blue") 
        self.constantValue = np.full_like(self.uartConnection.xAxes, self.setValue)
        plt.plot(self.uartConnection.xAxes, self.constantValue,  label='Dataset 1', marker='', linestyle='-', color="green")
        plt.xlim(self.uartConnection.timeCount - 0.1, self.uartConnection.timeCount)
        plt.ylim(self.uartConnection.yAxes[-1] - 1000, self.uartConnection.yAxes[-1] + 1000)
        self.canvas.draw()

    def close_window(self):
        self.root.quit()  # Close the Tkinter window

    def resetPlotAction(self):
        # self.uartConnection.xAxes.clear()
        # self.uartConnection.yAxes.clear()
        # # self.constantValue.clear()
        # self.startStopPlotAction()
        pass

    def uartSwitchAction(self):
        try:
            if(self.uartStateSwitch.get() == "on"):
                self.connectionStateFlag = True
            elif(self.uartStateSwitch.get() == "off"):
                self.connectionStateFlag = False
            self.uartConnection.uartState(self.connectionStateFlag, self.uartPort.get(), self.uartBaudrate.get())
            self.updateConfiguration()
        except:
            messagebox.askokcancel("Entry Error", "Please Check your entry or Connection")
            self.uartStateSwitch.deselect()
            self.connectionStateFlag = False

    def rpm_buttonAction(self):
        try:
           self.setValue[0] = int(self.rpm_entry.get())
           self.motorIns.setMotorRpm(self.uartConnection, self.setValue[0])
           self.connectionStateFlag = False
           self.uartConnection.uartState(self.connectionStateFlag, self.uartPort.get(), self.uartBaudrate.get())
           self.updateConfiguration()
           time.sleep(0.7)
           self.connectionStateFlag = True
           self.uartConnection.uartState(self.connectionStateFlag, self.uartPort.get(), self.uartBaudrate.get())
           self.updateConfiguration()
           
           
        #    self.uartConnection.ser.reset_input_buffer()
           self.desiredRpmOutput.configure(text=f"{self.setValue[0]}")
        except:
             messagebox.askokcancel("Entry Error", "Please Set A Integer [0,2000]")

    def motorSwitchAction(self):
        if(self.motorStateSwitch.get() == "on"):
            self.state = True
        elif(self.motorStateSwitch.get() == "off"):
            self.state = False
        self.motorIns.startStopMotor(self.uartConnection, state=self.state)

        
    def startStopPlotAction(self):
        self.plotFlag = not self.plotFlag
        if(self.plotFlag):
            self.startPlotButton.configure(text="Stop Plot")
        else:
            self.startPlotButton.configure(text="Start Plot")
            
    
    def updateConfiguration(self):
        self.realRpmOutput.configure(text=f"{self.uartConnection.data[0]}")
        if(self.plotFlag):
            self.startStopPlot()
    
    
    def whileLoop(self):
        if(self.connectionStateFlag ):
            self.uartConnection.receive_data(self.plotFlag)
            self.updateConfiguration()
        self.root.after(20, self.whileLoop)
 