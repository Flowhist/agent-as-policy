"""Read DH AG registers without initialization or motion. Requires pymodbus 3.14."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from pymodbus.client import ModbusSerialClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ0296VS-if00-port0")
    parser.add_argument("--baudrate", type=int, choices=(9600, 115200), default=9600)
    parser.add_argument("--device-id", type=int, choices=range(1, 248), default=2)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    folder = root / "logs/runs"
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / ("gripper-check-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".jsonl")
    client = ModbusSerialClient(args.port, baudrate=args.baudrate, bytesize=8,
                               parity="N", stopbits=1, timeout=1, retries=0)
    with path.open("w", encoding="utf-8") as log:
        def emit(step, **fields):
            line = json.dumps({"time": datetime.now(timezone.utc).isoformat(),
                               "step": step, **fields})
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        try:
            emit("configuration", **vars(args), read_only=True)
            if not client.connect():
                raise RuntimeError("Serial port could not be opened")
            client.socket.exclusive = True
            time.sleep(0.5)
            registers = ((0x0200, "initialization", 0, 2),
                         (0x0201, "grip_status", 0, 3),
                         (0x0202, "position_permille", 0, 1000))
            for address, name, low, high in registers:
                response = client.read_holding_registers(address, count=1,
                                                         device_id=args.device_id)
                if response.isError():
                    raise RuntimeError(f"{name}: {response}")
                if len(response.registers) != 1 or not low <= response.registers[0] <= high:
                    raise RuntimeError(f"Invalid {name}: {response.registers}")
                emit(name, address=hex(address), value=response.registers[0])
            return 0
        except Exception as exc:
            emit("error", type=type(exc).__name__, message=str(exc))
            return 1
        finally:
            client.close()
            emit("serial_closed")
            print(f"Log: {path}")


if __name__ == "__main__":
    raise SystemExit(main())
