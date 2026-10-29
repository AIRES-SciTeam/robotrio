import cv2
import numpy as np
import glob

# --- Настройки ---
pattern_size = (9, 6)      # внутренние углы (ширина, высота)
square_size = 0.025        # размер клетки в метрах
images_path = "imgs/*.jpg"   # или *.jpg

# --- Подготовка 3D-точек ---
objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

objpoints = []
imgpoints = []
used = 0

for fname in sorted(glob.glob(images_path)):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

    if ret:
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        objpoints.append(objp)
        imgpoints.append(corners2)
        used += 1
        print(f"[OK]   {fname}")
    else:
        print(f"[SKIP] {fname} — доска не найдена")

print(f"\nИспользовано кадров: {used} из {len(glob.glob(images_path))}")

if used < 10:
    print("⚠️  Мало кадров с найденной доской. Доснимите ещё.")
else:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print(f"\nСреднеквадратичная ошибка (RMS): {ret:.4f} пикселей")
    print("\nМатрица камеры (fx, fy, cx, cy):")
    print(mtx)
    print("\nКоэффициенты дисторсии (k1, k2, p1, p2, k3):")
    print(dist.ravel())

    # Сохранить результат в файл для дальнейшего использования
    np.savez("camera_calib.npz", mtx=mtx, dist=dist)
    print("\nПараметры сохранены в camera_calib.npz")