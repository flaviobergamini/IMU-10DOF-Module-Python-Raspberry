from IMU_10D0F import IMU10DOF
import time

if __name__ == "__main__":

    reads_Robot = IMU10DOF()

    print("******Magnetômetro******")
    print(reads_Robot.read_magnetometer())
    print("************************")
    print()
    print('------Acelerometro------')
    print(reads_Robot.read_accelerometer())
    print('------------------------')
    print()
    print('=======Giroscopio=======')
    print(reads_Robot.read_gyroscope())
    print('========================')
    print("O ângulo no sentido horário entre o norte magnético e o eixo X:")
    print(reads_Robot.getHeading())
    print("O ângulo no sentido horário entre o norte magnético e a projeção do eixo X positivo no plano horizontal:")
    print(reads_Robot.getTiltHeading())
    time.sleep(0.5)