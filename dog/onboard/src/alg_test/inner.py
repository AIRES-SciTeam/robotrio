from xgolib import XGO_DOG
import time
from itertools import product


def exec(dog : XGO_DOG, img_recv):
    dog.arm(75, 0)
    # print(f"Версия оборудования: {dog.read_firmware()}")
    # # Проверенный рабочий диапазон X с шагом 10 мм.
    # # Прямоугольник диапазонов не гарантирует достижимость каждой точки.
    # xs = range(60, 111, 10)
    # zs = range(-95, 156, 10)
    # for index, (z, x) in enumerate(product(zs, xs), start=1):
    #     print(f"[{index}/{len(xs) * len(zs)}] Задано положение руки (arm): ({x}, {z})", flush=True)
    #     dog.arm(x, z)
    #     time.sleep(2)
    #     print(f"Положение моторов: {dog.read_motor()}", flush=True)
    # angles = range(70, 271, 10)
    # radii = range(80, 141, 10)
    # for index, (r, θ) in enumerate(product(radii, angles), start=1):
    #     print(f"[{index}/{len(angles) * len(radii)}] Задано положение руки (arm_polar): ({θ}, {r})", flush=True)
    #     dog.arm_polar(θ, r)
    #     time.sleep(2)
    #     print(f"Положение моторов: {dog.read_motor()}", flush=True)
            
        
def run(img_recv):
    dog = None
    try:
        dog = XGO_DOG(port="/dev/ttyAMA0", baud=115200)
        exec(dog, img_recv)
    except KeyboardInterrupt:
        pass
    finally:
        if dog is not None:
            dog.stop()
            dog.reset()
