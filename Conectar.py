#from PySide6.QtCore import QTimer
#from PySide6.QtWidgets import QMainWindow, QApplication

from Adafruit_IO import Client
from interfazzz import Ui_MainWindow
import serial 
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *

ADAFRUIT_IO_USERNAME = "Angie2002"
ADAFRUIT_IO_KEY = "aio_ToIi48ubNUb8NL1bC1I67jXVt7xe"
aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.serial_port = None

        self.pushButton.clicked.connect(self.connect_port)
        self.pushButton_3.clicked.connect(self.modos)
        self.pushButton_2.clicked.connect(self.close_port)
        
        
        self.horizontalSlider.valueChanged.connect(self.slider_pwm)
        self.horizontalSlider_2.valueChanged.connect(self.slider_pwm2)
        self.horizontalSlider_3.valueChanged.connect(self.slider_pwm3)
        self.horizontalSlider_4.valueChanged.connect(self.slider_pwm4)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.receive_data)
        self.timer.start(100)
    
        self.valora = None
        self.valor1a = None
        self.valor2a = None
        self.valor3a = None

    def connect_port(self):
        print("conectando")
        port_name = self.comboBox.currentText()
        try:
            self.serial_port = serial.Serial(port_name, 9600)
        except serial.SerialException as e:
            self.statusbar.showMessage(str(e))

    def send_data(self,data):
        bytes_enviar = int(bin(data), 2).to_bytes((len(bin(data)) + 7) // 8, 'big')
        print(bytes_enviar)
        if self.serial_port is not None:
           self.serial_port.write(bytes_enviar)
           

    def receive_data(self):
        if self.serial_port is not None:
            data = self.serial_port.read_all().decode()
            if data:
                self.textBrowser.append(data)
                
    def modos(self):
       modes = self.comboBox_2.currentText()
       if modes == 'MANUAL':
            self.send_data(5)
            self.send_data(5)
       if modes == 'EEPROM':
            self.send_data(6)
            self.send_data(6)
       if modes == 'SERIAL':
            self.send_data(7)
            self.send_data(7)
       if modes == 'ADAFRUIT':
            valora = aio.receive('v1').value
            valor2a = aio.receive('v2').value
            valor3a = aio.receive('v1').value
            valor4a = aio.receive('v2').value
            self.send_data(1)
            self.send_data(int(valora))
        
                
    def slider_pwm(self,event):
        self.horizontalSlider.setValue(event)
        self.send_data(1)
        self.send_data(event)
        
    def slider_pwm2(self,event):
        self.horizontalSlider_2.setValue(event)
        self.send_data(2)
        self.send_data(event)
        
    def slider_pwm3(self,event):
        self.horizontalSlider_3.setValue(event)
        self.send_data(3)
        self.send_data(event)
        
    def slider_pwm4(self,event):
        self.horizontalSlider_4.setValue(event)
        self.send_data(4)
        self.send_data(event)
            
    def close_port(self):
        if self.serial_port is not None:
            self.serial_port.close()
            self.serial_port = None

if __name__ == "__main__":

    app = QApplication()
    window = MainWindow()
    window.show()
    app.exec_()
