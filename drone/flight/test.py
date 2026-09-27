#!/opt/python-venv/bin/python3
"""Read-only comparison of PX4 local NED and the SDF spawn position."""
import argparse
from pathlib import Path
import statistics
import time
import xml.etree.ElementTree as ET


def sdf_position(path):
    world = ET.parse(path).getroot().find('world')
    for entry in world.findall('include'):
        if entry.findtext('name') == 'x500':
            pose = entry.find('pose')
            if pose.get('relative_to'):
                raise ValueError('Expected a world-relative pose')
            return tuple(map(float, pose.text.split()[:3]))
    raise ValueError('x500 not found in SDF')


def enu_to_ned(p):
    return (p[1], p[0], -p[2])


def show(label, values):
    print(f"{label}: ({', '.join(f'{v:+.4f}' for v in values)}) m")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--address', default='udpin:127.0.0.1:14541')
    parser.add_argument('--seconds', type=float, default=5.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    from pymavlink import mavutil
    sdf = sdf_position(Path(__file__).parents[2] / 'scene/scene.world')
    positions, speeds = [], []
    conn = mavutil.mavlink_connection(args.address)
    try:
        print('Оставьте дрон неподвижным. Ожидаю телеметрию; команды не отправляются.')
        hb = conn.wait_heartbeat(timeout=10)
        if hb is None:
            raise TimeoutError('No heartbeat')
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=.2)
            if msg is None or msg.get_srcSystem() != hb.get_srcSystem():
                continue
            positions.append((msg.x, msg.y, msg.z))
            speeds.append((msg.vx**2 + msg.vy**2 + msg.vz**2)**.5)
    finally:
        conn.close()
    if not positions:
        raise RuntimeError('No LOCAL_POSITION_NED samples')
    mean = tuple(statistics.mean(axis) for axis in zip(*positions))
    spread = tuple(max(axis)-min(axis) for axis in zip(*positions))
    show('SDF: мировые ENU (восток, север, вверх)', sdf)
    show('SDF после ENU -> NED', enu_to_ned(sdf))
    show(f'PX4: среднее {len(positions)} измерений, локальные NED', mean)
    show('Размах координат PX4 (max-min)', spread)
    show('Смещение = PX4 - NED(SDF)', tuple(a-b for a,b in zip(mean,enu_to_ned(sdf))))
    print(f'Максимальная скорость за измерение: {max(speeds):.4f} м/с')
    print('Преобразование: цель local NED = (world_y, world_x, -world_z) + смещение.')
    print('Разница включает разные начала координат; это не готовая поправка центровки.')
    print('SDF задаёт стартовую позу модели: сравнение корректно, если дрон остался на месте старта.')
    print('По Z дополнительно возможна разница между точкой модели и точкой отсчёта PX4.')


if __name__ == '__main__':
    main()
