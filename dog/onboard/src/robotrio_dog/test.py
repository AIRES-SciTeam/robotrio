from xgolib import XGO_DOG 


def main():
    dog = XGO_DOG(port="/dev/ttyAMA0", baud=115200)
    print(dog.read_battery())
    print(dog.read_roll(), dog.read_pitch(), dog.read_yaw())
    print(dog.read_motor())
    print(dog.motor([53], [1]))
    print(dog.read_motor())

    dog.stop()
    dog.ser.close()

if __name__ == "__main__":
    main()
