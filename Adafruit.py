import serial
import time
from Adafruit_IO import Client
ADAFRUIT_IO_USERNAME = "Angie2002"
ADAFRUIT_IO_KEY = "aio_ToIi48ubNUb8NL1bC1I67jXVt7xe"

aio = Client(ADAFRUIT_IO_USERNAME, ADAFRUIT_IO_KEY)

# Configuración del puerto serial
pic = serial.Serial('COM7', 9600)
time.sleep(1)  # Espera de 1 segundo para establecer la comunicación

print("Programa Corriendo")

valora = None
valor1a= None
valor2a= None
valor3a= None


while True:
    data = aio.receive('v1')
    data1 = aio.receive('v2')
    data2 = aio.receive('v3')
    data3 = aio.receive('v4')
    
    valor = data.value
    valor1 = data1.value
    valor2 = data2.value
    valor3 = data3.value
    
        
    if (valor != valora) or (valor1 != valor1a) or (valor2 != valor2a) or (valor3 != valor3a):
        
        # Envía los datos al PIC
        print('Enviados:', (valor))
        
         # Envía los datos al PIC
        print('Enviados:', (valor1))
        
         # Envía los datos al PIC
        print('Enviados:', (valor2))
        
         # Envía los datos al PIC
        print('Enviados:', (valor3))
        
        valora = valor
        valor1a = valor1
        valor2a = valor2
        valor3a = valor3
    time.sleep(1)    
   
