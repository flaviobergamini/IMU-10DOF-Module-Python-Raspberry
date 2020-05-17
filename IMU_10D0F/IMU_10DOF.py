import serial
import time

class IMU10DOF(object):
        def __init__(self, port, baudrate):                                   #Recebe os dados para realizar um conexão serial
            self.COMserial = serial.Serial(port, baudrate=baudrate)

        def read_magnetometer(self):
            time.sleep(2)
            self.COMserial.write('m'.encode())
            mag = self.COMserial.readline()
            return mag

        def read_accelerometer(self):
            time.sleep(2)
            self.COMserial.write('a'.encode())
            acc = self.COMserial.readline()
            return acc

        def read_gyroscope(self):
            time.sleep(2)
            self.COMserial.write('g'.encode())
            gyr = self.COMserial.readline()
            return gyr

        def get_heading(self):
            time.sleep(2)
            self.COMserial.write('x'.encode())
            hea = self.COMserial.readline()
            return hea

        def get_tilt_teading(self):
            time.sleep(2)
            self.COMserial.write('y'.encode())
            til = self.COMserial.readline()
            return til