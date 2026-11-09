from typing import List, Dict, Any, Tuple
from pathlib import Path
from datetime import datetime
import json, math
import threading, time
from picamera2 import Picamera2
from picamera2.outputs import FileOutput
from picamera2.encoders import H264Encoder
from pupil_apriltags import Detection, Detector
import numpy as np
import cv2
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

from .logger_fabric import LoggerFabric, Handler


# region Буфер трансляции
class Frames:
    """
    Хранит последний JPEG-кадр и уведомляет HTTP-клиентов об обновлении.
    """
    def __init__(self):
        """
        Создаёт пустой буфер и условие для ожидания новых кадров.
        """
        self.cond = threading.Condition()
        self.jpeg = None
        self.sequence = 0
        self.closed = False

    def publish(self, jpeg):
        """
        Заменяет текущий кадр JPEG-байтами и будит ожидающих клиентов.
        """
        with self.cond:
            self.jpeg = jpeg
            self.sequence += 1
            self.cond.notify_all()

    def close(self):
        """
        Сигнализирует о завершении трансляции и будит ожидающих клиентов.
        """
        with self.cond:
            self.closed = True
            self.cond.notify_all()
# endregion

# region HTTP-обработчик
def get_streamhandler_for(frames : Frames, logger):
    """
    Возвращает класс HTTP-обработчика, связанный с буфером frames и логгером.
    """
    class StreamHandler(BaseHTTPRequestHandler):
        """
        Обслуживает страницу просмотра и MJPEG-трансляцию.
        """
        def log_message(self, format, *args):
            """
            Записывает сообщение HTTP-запроса в логгер камеры.
            """
            logger.info("%s | %s", self.client_address[0], format % args)

        def log_error(self, format, *args):
            """
            Записывает HTTP-ошибку в логгер камеры.
            """
            logger.error("%s | %s", self.client_address[0], format % args)

        def do_GET(self):
            """
            Отдаёт страницу / или поток /stream.mjpg, иначе возвращает 404.
            """
            if self.path == "/":
                body = (b'<!doctype html><title>Dog camera</title>'
                    b'<body style="background:#111;color:white;font-family:sans-serif">'
                    b'<h1>Dog camera / AprilTag</h1>'
                    b'<img src="/stream.mjpg" style="max-width:100%"></body>')
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return
            self.connection.settimeout(5)
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            sequence = -1
            try:
                while True:
                    with frames.cond:
                        frames.cond.wait_for(
                            lambda: frames.closed or frames.sequence != sequence
                        )
                        if frames.closed:
                            return
                        sequence, jpeg = frames.sequence, frames.jpeg
                    if jpeg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg + b"\r\n")
                    self.wfile.flush()
            except (OSError, TimeoutError):
                pass
    return StreamHandler
# endregion

# region HTTP-сервер
class StreamServer(ThreadingHTTPServer):
    """
    HTTP-сервер с записью исключений обработчиков в логгер камеры.
    """
    def __init__(self, server_address, handler, logger):
        """
        Сохраняет логгер и создаёт сервер для заданного обработчика.
        """
        self._logger = logger
        super().__init__(server_address, handler)

    def handle_error(self, request, client_address):
        """
        Записывает необработанное исключение запроса вместе с traceback.
        """
        self._logger.exception("Stream request failed: %s", client_address)
# endregion

# region CameraHandler
class CameraHandler:
    """
    Модуль получения кадров, детекции AprilTag и HTTP-трансляции с камеры.

    Параметры:
    - apriltags_families: Семейства тегов для создания детекторов.
    - logger_fabric: Фабрика логгеров модуля, Picamera2 и pupil_apriltags.
    - save_to: Каталог для сохранения снимков, видео и итоговой статистики.
        При None статистика сохраняется в текущую рабочую папку.
    - record: Флаг записи видео в H.264.
    - log_level: Минимальный уровень сообщений логгера.
    - camera_params: Параметры калибровки (fx, fy, cx, cy) в пикселях.
    - stream_port: Порт HTTP-трансляции.

    Пояснения:
    Камера и трансляция запускаются при создании объекта.
    Разметка тегов добавляется в трансляцию, запись энкодера не меняется.
    """
    # region Инициализация и остановка
    def __init__(
        self,
        logger_fabric : LoggerFabric,
        apriltags_families : List[str],
        *,
        save_to : str | None = None,
        record : bool = False,
        log_level : str = "info",
        camera_params : Tuple[float, float, float, float] = (794.46859366, 793.68863406, 631.52134703, 337.94348282),
        stream_port : int = 8080
    ):
        """
        Настраивает камеру, детекторы и статистику, затем запускает обработку.
        """
        self._logger = logger_fabric.get_logger(f"{type(self).__name__}", log_level)
        self._logger.info("Initializing")

        self.stream_port = stream_port

        self._dir = None
        if save_to is not None:
            self._dir = Path(save_to)
            self._dir.mkdir(parents=True, exist_ok=True)
        self._meta_path = (self._dir or Path.cwd()) / f"meta_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S_%f')}.json"

        self._timestamp = None
        self._received_at = None
        self._frame = None
        self._frame_lock = threading.Lock()
        self._detector_lock = threading.Lock()

        logger_fabric.redirect_logging("picamera2", self._logger)
        logger_fabric.redirect_logging("pupil_apriltags", self._logger)
        self._camera = Picamera2()
        self._camera.configure(
            self._camera.create_video_configuration(
                main={"format": "RGB888"}
            )
        )
        self._camera.post_callback = self._camera_callback
        self._encoder = None
        if record:
            if self._dir is None:
                self._logger.error("The save_to directory is required for video recording")
                self._encoder = H264Encoder(bitrate=5_000_000)

        self._detectors = {family: Detector(families=family) for family in apriltags_families}
        self._camera_params = camera_params

        self._meta = {
            "min_frame_interval" : float("inf"),
            "max_frame_interval" : -float("inf"),
            "avg_frame_interval" : 0,
            "min_receive_interval" : float("inf"),
            "max_receive_interval" : -float("inf"),
            "avg_receive_interval" : 0,
            "min_delay" : float("inf"),
            "max_delay" : -float("inf"),
            "avg_delay" : 0,
            "min_fps" : float("inf"),
            "max_fps": -float("inf"),
            "avg_fps" : 0,
        }
        self._total_frames_count = 0
        self._second_frames_count = 0 
        self._start_time = None
        self._second_start_time = None

        self._logger.info("Initialized")
        self._start()

    def _start(self):
        """
        Запускает запись при наличии энкодера, камеру и HTTP-трансляцию.
        """
        self._start_time = time.clock_gettime_ns(time.CLOCK_BOOTTIME) / 1e9
        self._second_start_time = self._start_time 
        if self._encoder is not None:
            self._camera.start_encoder(
                self._encoder,
                FileOutput(str(self._dir / f"video_{time.strftime('%Y-%m-%d_%H-%M-%S')}.h264"))
            )
        self._camera.start()
        self._start_stream()
        self._logger.info("Started")

    def stop(self):
        """
        Завершает трансляцию, закрывает камеру и сохраняет итоговую статистику.
        """
        if hasattr(self, "_stream_stop"):
            self._stream_stop.set()
            self._stream_frames.close()
            if hasattr(self, "_video_thread"):
                self._video_thread.join()
                self._server.shutdown()
                self._http_thread.join()
            if hasattr(self, "_server"):
                self._server.server_close()
        self._camera.close()
        self._save_meta()
    # endregion

    # region Получение кадров и статистика
    def _camera_callback(self, request):
        """
        Сохраняет полученный кадр и временные метки, обновляет статистику.
        """
        received_at = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
        timestamp = request.get_metadata()["SensorTimestamp"]
        frame = request.make_array("main")
        with self._frame_lock:
            prev_timestamp = self._timestamp
            prev_received_at = self._received_at
            self._frame = frame
            self._timestamp = timestamp
            self._received_at = received_at
            self._calc_meta(prev_timestamp, prev_received_at, received_at)

    def _calc_meta(self, prev_timestamp, prev_received_at, received_at):
        """
        Обновляет min, max, avg интервалов съёмки, получения и задержки, FPS.
        Временные метки передаются в наносекундах CLOCK_BOOTTIME;
        интервалы и задержка сохраняются в секундах. Первый кадр не образует интервал.
        """
        self._total_frames_count += 1
        self._second_frames_count += 1
        delay = (received_at - self._timestamp) / 1e9
        self._meta["min_delay"] = min(self._meta["min_delay"], delay)
        self._meta["max_delay"] = max(self._meta["max_delay"], delay)
        self._meta["avg_delay"] += (delay - self._meta["avg_delay"]) / self._total_frames_count

        if prev_timestamp is not None and prev_received_at is not None:
            intervals_count = self._total_frames_count - 1
            frame_interval = (self._timestamp - prev_timestamp) / 1e9
            receive_interval = (received_at - prev_received_at) / 1e9
            for name, value in (
                ("frame_interval", frame_interval),
                ("receive_interval", receive_interval)
            ):
                self._meta[f"min_{name}"] = min(self._meta[f"min_{name}"], value)
                self._meta[f"max_{name}"] = max(self._meta[f"max_{name}"], value)
                self._meta[f"avg_{name}"] += (value - self._meta[f"avg_{name}"]) / intervals_count

        received_at /= 1e9
        if received_at - self._second_start_time > 1:
            fps = self._second_frames_count / (received_at - self._second_start_time)
            self._second_start_time = received_at
            self._second_frames_count = 0

            self._meta["min_fps"] = min(self._meta["min_fps"], fps)
            self._meta["max_fps"] = max(self._meta["max_fps"], fps)
        elapsed = received_at - self._start_time
        if elapsed > 0:
            self._meta["avg_fps"] = self._total_frames_count / elapsed

    def get_frame(self):
        """
        Возвращает последний кадр без копирования или None до первого кадра.
        """
        with self._frame_lock:
            return self._frame
    # endregion

    # region Детекция тегов
    def get_poses(
        self,
        tags : Dict[str, float], # "["family1:tag1" : size1, ..., "familyN:tagN" : sizeN]", size : mm
        *,
        frame = None,
    ):
        """
        Определяет позы запрошенных тегов относительно камеры.

        Параметры:
        - tags: Словарь {"family:id": размер стороны тега в мм}.
        - frame: BGR-кадр. None выбирает последний полученный кадр.

        Возвращает:
        Словарь {"family:id": Detection}, где pose_t выражен в мм.
        Без кадра возвращает пустой словарь; неизвестные семейства пропускает с записью в лог.
        """
        if frame is None:
            with self._frame_lock:
                frame = self._frame
        if frame is None:
            return {}
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tag_detections = dict()
        families = {tag.split(":")[0] for tag in tags}
        for family in families:
            if family not in self._detectors:
                self._logger.error("No tag detector for family: %s", family)
                continue
            with self._detector_lock:
                detections = self._detectors[family].detect(
                    frame_gray, estimate_tag_pose=True,
                    camera_params=self._camera_params,
                    tag_size=1.0
                )
            for detection in detections:
                tag = f"{family}:{detection.tag_id}"
                if tag not in tags:
                    continue
                size = tags[tag]
                detection.pose_t *= size
                if detection.pose_err is not None:
                    detection.pose_err *= size ** 2
                tag_detections[tag] = detection
        return tag_detections
    # endregion

    # region Сохранение снимков и статистики
    def _save_meta(self):
        """
        Сохраняет статистику и число кадров в JSON, интервалы указаны в секундах.
        Неизмеренные минимумы и максимумы записывает как null.
        """
        with self._frame_lock:
            meta = {
                name: value if math.isfinite(value) else None
                for name, value in self._meta.items()
            }
            meta["total_frames_count"] = self._total_frames_count
            meta["time_unit"] = "seconds"
        with self._meta_path.open("w", encoding="utf-8") as file:
            json.dump(meta, file, ensure_ascii=False, indent=4, allow_nan=False)
            file.write("\n")
        self._logger.info("Camera statistics saved: %s", self._meta_path)

    def shot(self):
        """
        Сохраняет последний кадр в JPEG с временной меткой в имени файла.
        """
        if self._dir is None:
            self._logger.error("The save_to directory is required for snapshots")
            return
        with self._frame_lock:
            frame = self._frame
            timestamp = self._timestamp
        if frame is None:
            self._logger.warning("No camera frame available")
            return
        file_path = self._dir / f"photo_{timestamp}.jpg"
        try:
            if not cv2.imwrite(str(file_path), frame):
                self._logger.error("Failed to save snapshot: %s", file_path)
        except Exception:
            self._logger.exception("Failed to save snapshot: %s", file_path)
    # endregion

    # region HTTP-трансляция
    def _start_stream(self):
        """
        Создаёт буфер кадров, HTTP-сервер и потоки обслуживания трансляции.
        """
        self._stream_frames = Frames()
        self._stream_stop = threading.Event()
        self._server = StreamServer(
            ("0.0.0.0", self.stream_port),
            get_streamhandler_for(self._stream_frames, self._logger),
            self._logger
        )
        self._server.daemon_threads = True
        self._http_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._video_thread = threading.Thread(target=self._publish_frames, daemon=True)
        self._http_thread.start()
        self._video_thread.start()
        self._logger.info(f"Camera stream at: http://<robot-IP>:{self.stream_port}/")

    def _publish_frames(self):
        """
        Публикует JPEG-кадры для HTTP-трансляции до установки _stream_stop.
        Обводит все найденные теги настроенных семейств без оценки позы.
        Разметка наносится на копию кадра и не меняет запись энкодера.
        """
        try:
            while not self._stream_stop.is_set():
                frame = self.get_frame()
                if frame is not None:
                    frame = frame.copy()
                    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    detections = []
                    for family, detector in self._detectors.items():
                        with self._detector_lock:
                            family_detections = detector.detect(frame_gray)
                        detections.extend(
                            (f"{family}:{detection.tag_id}", detection)
                            for detection in family_detections
                        )
                    for tag, detection in detections:
                        corners = np.rint(detection.corners).astype(np.int32)
                        center = tuple(np.rint(detection.center).astype(int))
                        cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
                        cv2.circle(frame, center, 4, (0, 0, 255), -1)
                        cv2.putText(
                            frame, tag, center,
                            cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2
                        )
                    ok, jpeg = cv2.imencode(".jpg", frame)
                    if ok:
                        self._stream_frames.publish(jpeg.tobytes())
                    else:
                        self._logger.warning("Failed to encode stream frame")
                self._stream_stop.wait(0.1)
        except Exception:
            self._logger.exception("Camera stream failed")
        finally:
            self._stream_frames.close()
    # endregion
# endregion
