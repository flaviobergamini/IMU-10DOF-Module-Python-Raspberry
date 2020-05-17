import serial
import time

class IMU10DOF(object):
        def __init__(self, port, baudrate):                                   #Recebe os dados para realizar um conexão serial
            self.COMserial = serial.Serial(port, baudrate=baudrate)
            self.COMserial.flushInput
            self.COMserial.flushOutput

        def read_magnetometer(self):
            time.sleep(2)
            self.COMserial.write('m'.encode())
            try:
                mag = self.COMserial.readline()
                return mag
            except:
                self.COMserial.flushInput
                self.COMserial.flushOutput
                return bytes(0)

        def read_accelerometer(self):
            time.sleep(2)
            self.COMserial.write('a'.encode())
            try:
                acc = self.COMserial.readline()
                return acc
            except:
                self.COMserial.flushInput
                self.COMserial.flushOutput
                return bytes(0)

        def read_gyroscope(self):
            time.sleep(2)
            self.COMserial.write('g'.encode())
            try:
                gyr = self.COMserial.readline()
                return gyr
            except:
                self.COMserial.flushInput
                self.COMserial.flushOutput
                return bytes(0)

        def get_heading(self):
            time.sleep(2)
            self.COMserial.write('x'.encode())              # Angulo entre o norte magnetico e a projeção da posição X-Axis
            try:
                hea = self.COMserial.readline()
                return hea
            except:
                self.COMserial.flushInput
                self.COMserial.flushOutput
                return bytes(0)

        def get_tilt_teading(self):
            time.sleep(2)
            self.COMserial.write('y'.encode())              # Angulo entre o norte magnetico e a projeção da posição X-Axis no plano horizontal
            try:
                til = self.COMserial.readline()
                return til
            except:
                self.COMserial.flushInput
                self.COMserial.flushOutput
                return bytes(0)

        def altitude(self):
            time.sleep(2)
            self.COMserial.write('h'.encode())              # Altura em metros
            try:
                alt = self.COMserial.readline()
                return alt
            except:
                self.COMserial.flushInput
                self.COMserial.flushOutput
                return bytes(0)