import smbus
import time
import math

bus = smbus.SMBus(1)

# Acesso aos endereços do MPU9250

MPU9250_ADDRESS_AD0_LOW = 0x68
MPU9250_DEFAULT_ADDRESS = MPU9250_ADDRESS_AD0_LOW
MPU9250_RA_INT_PIN_CFG = 0x37

# Acesso aos endereços do magnetômetro do MPU9250
MPU9150_RA_MAG_ADDRESS = 0x0C
MPU9150_RA_MAG_XOUT_L = 0x03
MPU9150_RA_MAG_YOUT_L = 0x05
MPU9150_RA_MAG_ZOUT_L = 0x07

# Acesso aos endereços do acelerômetro do MPU9250
MPU9250_RA_ACCEL_XOUT_H = 0x3B
MPU9250_RA_ACCEL_XOUT_L = 0x3C
MPU9250_RA_ACCEL_YOUT_H = 0x3D
MPU9250_RA_ACCEL_YOUT_L = 0x3E
MPU9250_RA_ACCEL_ZOUT_H = 0x3F
MPU9250_RA_ACCEL_ZOUT_L = 0x40

# Acesso aos endereços do giroscópio do MPU9250
MPU9250_RA_GYRO_CONFIG = 0x1B
MPU9250_RA_GYRO_XOUT_H = 0x43
MPU9250_RA_GYRO_XOUT_L = 0x44
MPU9250_RA_GYRO_YOUT_H = 0x45
MPU9250_RA_GYRO_YOUT_L = 0x46
MPU9250_RA_GYRO_ZOUT_H = 0x47
MPU9250_RA_GYRO_ZOUT_L = 0x48



class IMU10DOF(object):
        def __init__(self):
                pass

        def read_magnetometer(self):
                global mx
                global my
                global mz
                bus.write_byte_data(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_INT_PIN_CFG, 0x02)     # habilitar leituras
                time.sleep(0.1)
                bus.write_byte_data(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01)     # ativa o magnetômetro
                time.sleep(0.1)
                mx = bus.read_byte_data(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L) # busca o valor no eixo x
                my = bus.read_byte_data(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_YOUT_L) # busca o valor no eixo y
                mz = bus.read_byte_data(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_ZOUT_L) # busca o valor no eixo z
                mx = mx * 1200 / 4096
                my = my * 1200 / 4096
                mz = mz * 1200 / 4096
                self.getHeading()
                return mx, my, mz

        def read_accelerometer(self):
                global ax
                global ay
                global az
                bus.write_byte_data(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_INT_PIN_CFG, 0x02) # habilitar leituras
                time.sleep(0.1)
                ax = bus.read_byte_data(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_XOUT_H) # busca o valor no eixo x
                ay = bus.read_byte_data(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_YOUT_H) # busca o valor no eixo y
                az = bus.read_byte_data(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_ACCEL_ZOUT_H) # busca o valor no eixo z
                ax = ax << 8 | ax
                ay = ay << 8 | ay
                az = az << 8 | az
                ax = ax / 16384
                ay = ay / 16384
                az = az / 16384
                return ax, ay, az

        def read_gyroscope(self):
                bus.write_byte_data(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_INT_PIN_CFG, 0x02) # habilitar leituras
                time.sleep(0.1)
                gx = bus.read_byte_data(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_XOUT_H) # busca o valor no eixo x
                gy = bus.read_byte_data(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_YOUT_H) # busca o valor no eixo y
                gz = bus.read_byte_data(MPU9250_DEFAULT_ADDRESS, MPU9250_RA_GYRO_ZOUT_H) # busca o valor no eixo z
                gx = gx << 8 | gx
                gy = gy << 8 | gy
                gz = gz << 8 | gz
                gx = gx * 250 / 32768
                gy = gy * 250 / 32768
                gz = gz * 250 / 32768
                return gx, gy, gz

        def getHeading(self):
                heading = 180 * math.atan2(my, mx) / math.pi
                if heading < 0 :
                        heading += 360
                return heading

        def getTiltHeading(self):
                global tiltheading
                pitch = math.asin(-ax)
                roll = math.asin(ay / math.cos(pitch))

                xh = mx * math.cos(pitch) + mz * math.sin(pitch)
                yh = mx * math.sin(roll) * math.sin(pitch) + my * math.cos(roll) - mz * math.sin(roll) * math.cos(pitch)
                zh = -mx * math.cos(roll) * math.sin(pitch) + my * math.sin(roll) + mz * math.cos(roll) * math.cos(pitch)
                tiltheading = 180 * math.atan2(yh, xh) / math.pi
                if yh < 0:    
                        tiltheading += 360
                return tiltheading



while True:
        print("******Magnetômetro******")
        print(read_magnetometer())
        print("************************")
        print()
        print('------Acelerometro------')
        print(read_accelerometer())
        print('------------------------')
        print()
        print('=======Giroscopio=======')
        print(read_gyroscope())
        print('========================')
        time.sleep(0.5)
        print("The clockwise angle between the magnetic north and X-Axis:")
        print(getHeading())
        print("The clockwise angle between the magnetic north and the projection of the positive X-Axis in the horizontal plane:")
        print(tiltheading)
