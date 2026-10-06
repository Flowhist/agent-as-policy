"""One bounded DH AG gripper step; no homing, retries or automatic return.

Requires pymodbus 3.14. Serial disconnect is not a hardware stop.
"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from pymodbus.client import ModbusSerialClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--delta", type=int, required=True,
                        help="Raw position delta, -10 to 10 (1000 = full open)")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not 1 <= abs(args.delta) <= 10:
        parser.error("delta magnitude must be 1 to 10")
    folder = Path(__file__).resolve().parent.parent / "logs/runs"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / ("gripper-step-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".jsonl")
    client = ModbusSerialClient(
        "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ0296VS-if00-port0",
        baudrate=9600, bytesize=8, parity="N", stopbits=1, timeout=1, retries=0)
    with path.open("w", encoding="utf-8") as log:
        def emit(step, **fields):
            line = json.dumps({"time": datetime.now(timezone.utc).isoformat(),
                               "step": step, **fields})
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        def read(address):
            response = client.read_holding_registers(address, count=1, device_id=2)
            if response.isError() or len(response.registers) != 1:
                raise RuntimeError(f"Read {address:#x}: {response}")
            return response.registers[0]

        def write(address, value):
            emit("write_requested", address=hex(address), value=value)
            response = client.write_register(address, value, device_id=2)
            if response.isError() or response.address != address or response.registers != [value]:
                raise RuntimeError(f"Unconfirmed write {address:#x}: {response}; no retry")
            emit("write_acknowledged", address=hex(address), value=value)

        def snapshot():
            initialized, status, position = read(0x0200), read(0x0201), read(0x0202)
            emit("feedback", initialized=initialized, status=status, position_permille=position)
            if initialized != 1 or status not in (0, 1) or not 0 <= position <= 1000:
                raise RuntimeError("Not initialized, unexpected feedback, contact or dropped object")
            return status, position

        try:
            if not client.connect():
                raise RuntimeError("Serial connection failed")
            client.socket.exclusive = True
            time.sleep(0.5)
            status, start = snapshot()
            if status != 1:
                raise RuntimeError("Gripper must be idle without an object")
            target = start + args.delta
            if not 0 <= target <= 1000:
                raise RuntimeError("Target outside 0..1000")
            emit("plan", start=start, target=target, force_percent=20, execute=args.execute)
            if not args.execute:
                return 0
            time.sleep(0.2)
            if snapshot() != (1, start):
                raise RuntimeError("State changed before execution")
            write(0x0101, 20)
            if read(0x0101) != 20:
                raise RuntimeError("Force setting readback failed")
            write(0x0103, target)
            deadline = time.monotonic() + 5
            stable = []
            while time.monotonic() < deadline:
                time.sleep(0.15)
                status, actual = snapshot()
                if not min(start, target) - 2 <= actual <= max(start, target) + 2:
                    raise RuntimeError("Unexpected displacement; no further commands")
                if status == 1 and abs(actual - target) <= 2:
                    stable.append(actual)
                    if len(stable) >= 3 and max(stable[-3:]) - min(stable[-3:]) <= 1:
                        emit("verified", start=start, target=target, actual=actual,
                             delta=actual-start, force_percent=20)
                        return 0
                else:
                    stable.clear()
            raise RuntimeError("Arrival not verified within 5 seconds; no retry or return")
        except (Exception, KeyboardInterrupt) as exc:
            emit("error", type=type(exc).__name__, message=str(exc),
                 note="Serial close does not stop or power off the gripper")
            return 1
        finally:
            client.close()
            emit("serial_closed")
            print(f"Log: {path}")


if __name__ == "__main__":
    raise SystemExit(main())
