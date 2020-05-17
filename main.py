from IMU_10D0F import IMU10DOF
from binascii import hexlify
from binascii import unhexlify
import serial
import time

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


if __name__ == "__main__":

    serial = IMU10DOF('/dev/ttyACM0',115200)
    #serial = serial.Serial('/dev/ttyACM0',115200)
    print('oiiiii')
    while(True):
        #serial.write("m")
        #x = serial.readline()
        #y = serial.readline()
        #z = str(hexlify(serial.read()))
        #print(y)

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

        #print(parse_dados(serial.read_magnetometer()))
        #print('------------------------------------------')
        #print(parse_dados(serial.read_accelerometer))
        #print('------------------------------------------')
        #print(parse_dados(serial.read_gyroscope))
        #print('------------------------------------------')
        time.sleep(0.5)