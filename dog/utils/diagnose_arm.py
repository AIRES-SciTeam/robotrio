"""Trace installed xgolib arm packets; no hardware access unless --port is set."""

import argparse
import importlib.metadata
import inspect
import math
import time

from xgolib import XGO_DOG


class TraceSerial:
    def __init__(self, serial=None, gap=0):
        self.serial = serial
        self.gap = gap
        self.packets = []

    def write(self, data):
        packet = bytes(data)
        assert len(packet) == packet[2], "Invalid packet length"
        assert sum(packet[2:-2]) % 256 == 255, "Invalid checksum"
        self.packets.append(packet)
        print(f"TX {packet.hex(' ')}  register=0x{packet[4]:02x} data={list(packet[5:-3])}")
        if self.serial is not None:
            result = self.serial.write(packet)
            self.serial.flush()
            time.sleep(self.gap)
            return result
        return len(packet)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Read firmware on this port; --send also permits arm movement")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--send", action="store_true", help="Actually send the specified arm target")
    parser.add_argument("--gap-ms", type=float, default=10, help="Delay after each transmitted packet")
    parser.add_argument("--method", choices=("arm", "arm_polar"), default="arm")
    parser.add_argument("--target", nargs=2, type=float, metavar=("X_OR_THETA", "Z_OR_R"))
    args = parser.parse_args()
    if args.send and (not args.port or args.target is None):
        parser.error("--send requires --port and --target")
    if not math.isfinite(args.gap_ms) or args.gap_ms < 0:
        parser.error("--gap-ms must be finite and nonnegative")
    if args.target and not all(math.isfinite(value) for value in args.target):
        parser.error("Target coordinates must be finite")

    sdk = inspect.getmodule(XGO_DOG)
    print("SDK file:", sdk.__file__)
    try:
        print("Distribution version:", importlib.metadata.version("xgolib"))
    except importlib.metadata.PackageNotFoundError:
        print("Distribution version: unknown")
    print("Module version:", getattr(sdk, "__version__", "unknown"))
    real_serial = None
    try:
        if args.port:
            dog = XGO_DOG(port=args.port, baud=args.baud)
            real_serial = dog.ser
            print("Firmware:", dog.version)
        else:
            sdk.changePara("xgomini")
            dog = XGO_DOG.__new__(XGO_DOG)
            dog.verbose = False
        print("ARM_LIMIT:", sdk.XGOparam["ARM_LIMIT"])
        dog.ser = TraceSerial(real_serial if args.send else None, args.gap_ms / 1000)
        print("Mode:", "SEND TO ROBOT" if args.send else "CAPTURE ONLY; arm packets are not sent")
        cases = [(args.method, args.target)] if args.target else [
            ("arm", (0, 0)), ("arm", (100, 100)),
            ("arm_polar", (90, 90)), ("arm_polar", (180, 120)),
        ]
        for method, target in cases:
            print(f"{method}{tuple(target)}")
            start = 0 if method == "arm" else 2
            for value, bounds in zip(target, sdk.XGOparam["ARM_LIMIT"][start:start + 2]):
                if not bounds[0] <= value <= bounds[1]:
                    print(f"WARNING: {value} is outside {bounds}; SDK clips it")
            before = len(dog.ser.packets)
            getattr(dog, method)(*target)
            if len(dog.ser.packets) == before:
                raise RuntimeError("SDK produced no packets")
    finally:
        if real_serial is not None:
            real_serial.close()


if __name__ == "__main__":
    main()
