from IMU_10D0F import IMU10DOF

from binascii import hexlify
from binascii import unhexlify
import serial
import time
import sys
import math

import glob
import serial.tools.list_ports
from geopy.geocoders import Nominatim
from geopy import distance
from geopy.distance import lonlat, distance

geolocator = Nominatim(user_agent="flaviobergamini")
location = geolocator.reverse("-22.256653, -45.697336")
#global serial

def parse_dados(txt):
        txt = txt.decode("utf-8")
        inicio = 0
        fim = 0
        dado = []
        for i in range(int(len(txt))):
            if(txt[i] != ','):
                fim += 1
            else:
                dado.append(txt[inicio:fim])
                inicio = fim + 1
                fim = fim + 1
        dado.append(txt[inicio:(int(len(txt)))])

        print("x = ", dado[0])
        print("y = ", dado[1])
        print("z = ", dado[2])

        return dado

def find_angle(x1, y1, x2, y2):
    tea = serial.get_tilt_teading()                   # leitura de angulos de relevo na função de angulos em relação ao norte
    print(tea.decode("utf-8"))
    if(x1 >= 0 and y1 >= 0 and x2 >= 0 and y2 >= 0):

        if(x1 > x2) and (y1 > y2):
            aux = x1
            x1 = x2
            x2 = aux

            aux = y1
            y1 = y2
            y2 = aux

        if(x2-x1) == 0:
            print("angulo em 0 para o norte")
            return 0
        elif (x2 != x1 and (y2-y1) == 0):
            print("angulo em 90 para o norte")
            return 90
        else:
            if(x1 < x2) and (y1 > y2):
                m = (y2-y1)/(x2-x1)
                m1 = (math.atan(m)*180)/math.pi
                print("reta ao leste, com norte em graus: ", (m1*(-1)+ 90))
                return (m1*(-1)+ 90)
            else:
                m = (y2-y1)/(x2-x1)
                m1 = (math.atan(m)*180)/math.pi
                print("reta ao leste, com norte em graus: ", m1)
                return m1
        

    elif(x1 <= 0 and y1 >= 0 and x2 <= 0 and y2 >= 0):
        if(x1*(-1) > x2*(-1) and y1 > y2):
            aux = x1
            x1 = x2
            x2 = aux

            aux = y1
            y1 = y2
            y2 = aux
            
        if(x2-x1) == 0:
            print("angulo em 0 para o norte")
            return 0
        elif (x2 != x1 and (y2-y1) == 0):
            print("angulo em 90 para o norte")
            return 90

        else:
            if(x1*(-1) < x2*(-1)) and (y1 > y2):
                m = (y2-y1)/(x2-x1)
                m1 = (math.atan(m)*180)/math.pi
                print("reta ao oeste, com norte em graus: ", m1) 
                return m1
            else:
                m = (y2-y1)/(x2-x1)
                m1 = (math.atan(m)*180)/math.pi
                print("reta ao oeste, com norte em graus: ", (m1*(-1)+ 90)) 
                return (m1*(-1) + 90)

    elif(x1 <= 0 and y1 <= 0 and x2 <= 0 and y2 <= 0):
        if(x1*(-1) > x2*(-1) and y1*(-1) > y2*(-1)):
            aux = x1
            x1 = x2
            x2 = aux

            aux = y1
            y1 = y2
            y2 = aux

        if(x2-x1) == 0:
            print("angulo em 0 para o norte")
            return 0
        elif (x2 != x1 and (y2-y1) == 0):
            print("angulo em 90 para o norte")
            return 90

        else:
            if(x1*(-1) < x2*(-1) and y1*(-1) > y2*(-1)):
                m = (y2-y1)/(x2-x1)
                m1 = (math.atan(m)*180)/math.pi
                print("reta ao oeste, com norte em graus: ", (m1*(-1) + 270))
                return (m1*(-1) + 270) 
            else:
                m = (y2-y1)/(x2-x1)
                m1 = (math.atan(m)*180)/math.pi
                print("reta ao oeste, com norte em graus: ", (180 + m1)) 
                return (180 + m1)
    
    elif(x1 >= 0 and y1 <= 0 and x2 >= 0 and y2 <= 0):
        if(x1 > x2 and y1*(-1) > y2*(-1)):
            aux = x1
            x1 = x2
            x2 = aux

            aux = y1
            y1 = y2
            y2 = aux

        if(x2-x1) == 0:
            print("angulo em 0 para o norte")
            return 0
        elif (x2 != x1 and (y2-y1) == 0):
            print("angulo em 90 para o norte")
            return 90
            
        else:
            if(x1 < x2 and y1*(-1) > y2*(-1)):
                m = (y2-y1)/(x2-x1)
                m1 = (math.atan(m)*180)/math.pi
                print("reta ao leste, com norte em graus: ", (180 + m1)) 
                return (180 + m1)
            else:
                m = (y2-y1)/(x2-x1)
                m1 = (math.atan(m)*180)/math.pi
                print("reta ao leste, com norte em graus: ", (m1*(-1) + 270))
                return (m1*(-1) + 270) 

def gps(i):      # coordenadas que futuramente serão lidas por um GPS
    if(i == 1):
        return (-22.257672, -45.695628)
    else:
        return (-22.257487, -45.695878)

def find_serial():
    ports = serial.tools.list_ports.comports()
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/ttyACM[0-9]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    result = str(result)
    result = result[2:(int(len(result))-2)]
    print(result)
    return result

if __name__ == "__main__":
    ser = True
    while(ser == True):
        try:
            serial = IMU10DOF(find_serial() ,115200)
            ser = False
        except:
            print('Erro, porta serial não encontrada')
            time.sleep(0.5)

    p1 = gps(1)
    p2 = gps(2)
    print("Distancia: ",distance(lonlat(*p1), lonlat(*p2)))
    find_angle(p1[0], p1[1], p2[0], p2[1])

    while(ser == False):
        try:
            mag = serial.read_magnetometer()
            parse_dados(mag)
            print('----------------------------------------')
            acc = serial.read_accelerometer()
            parse_dados(acc)
            print('----------------------------------------')
            gyr = serial.read_gyroscope()
            parse_dados(gyr)
            print('----------------------------------------')
            hea = serial.get_heading()
            print(hea.decode("utf-8"))
            print('----------------------------------------')
            tea = serial.get_tilt_teading()
            print(tea.decode("utf-8"))
            print('----------------------------------------')
            print('************************************************')
        except:
            print('Erro, porta serial não encontrada')
            ser = True
