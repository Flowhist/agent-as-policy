"""Hardware adapters for the single-owner web control worker."""

import math
import time

from scripts.hardware.sdk_runtime import sdk_runtime
from scripts.hardware.control_settings import load_settings, validate_force, validate_rate


def sdk_call(method, *args):
    ec = {}
    result = method(*args, ec)
    if ec.get("ec") != 0:
        raise RuntimeError(f"{method.__name__}: {ec}")
    return result


def jog_timeout(delta, rate):
    return 10 + 4 * abs(delta) * (0.01 / rate)


def joint_commands(sdk, targets_deg, rate, zones):
    """Build native MoveAbsJ commands for both web targets and checked routes."""
    validate_rate(rate)
    descriptor = getattr(sdk.MoveAbsJCommand, 'jointSpeed', None)
    if not hasattr(descriptor, '__set__'):
        raise RuntimeError('Installed SDK lacks native MoveAbsJCommand.jointSpeed')
    if len(targets_deg) != len(zones):
        raise ValueError('Each joint target needs a zone')
    commands = []
    for target, zone in zip(targets_deg, zones):
        command = sdk.MoveAbsJCommand(list(map(math.radians, target)), -1, zone)
        if not math.isfinite(command.zone) or not math.isclose(command.zone, zone, abs_tol=1e-9):
            raise RuntimeError('SDK blending radius readback differs from requested radius')
        command.jointSpeed = rate
        if not math.isfinite(command.jointSpeed) or not math.isclose(
                command.jointSpeed, rate, rel_tol=0, abs_tol=1e-9):
            raise RuntimeError('SDK joint speed readback differs from requested rate')
        commands.append(command)
    return commands


def joint_progress(robot, sdk, command_id):
    """Ignore stale queue events; reject controller errors on the current queue."""
    info = sdk_call(robot.queryEventInfo, sdk.Event.moveExecution)
    keys = sdk.EventInfoKey.MoveExecution
    if info.get(keys.ID) != command_id.content():
        return None
    error = info[keys.Error]
    if isinstance(error, sdk.PyErrorCode):
        code = error.value()
    elif type(error) is int:
        code = error
    elif isinstance(error, dict) and type(error.get('ec')) is int:
        code = error['ec']
    else:
        raise RuntimeError('Unsupported controller event error representation')
    if code != 0:
        raise RuntimeError(f'Controller motion error: {code}')
    index = info[keys.WaypointIndex]
    if type(index) is not int:
        raise RuntimeError('Invalid controller waypoint index')
    return index, info.get(keys.ReachTarget) is True


class Hardware:
    def __init__(self):
        self.robot = None
        self.gripper = None
        self.sdk = None
        self.runtime = None
        self.robot_connected = False
        self.limits_deg = None
        self.drag_requested = False
        self.on_robot = lambda state: None
        self.on_gripper = lambda state: None

    def connect_robot(self):
        if self.robot:
            return
        self.runtime = sdk_runtime()
        self.runtime.__enter__()
        try:
            import xCoreSDK_python as sdk
            self.sdk = sdk
            self.robot = sdk.xMateRobot()
            self.robot.connectToRobot("192.168.0.160", "")
            self.robot_connected = True
            info = sdk_call(self.robot.robotInfo)
            if info.type != "XMS4-R800-B4S1A2C" or info.joint_num != 6:
                raise RuntimeError("Unexpected robot model")
            self.ready()
            bounds = sdk.PyTypeVectorArrayDouble2()
            sdk_call(self.robot.getSoftLimit, bounds)
            limits = bounds.content()
            if len(limits) < 6:
                raise RuntimeError("Missing joint bounds")
            self.limits_deg = [[math.degrees(lo), math.degrees(hi)] for lo, hi in limits[:6]]
        except Exception:
            self.disconnect_robot()
            raise

    def robot_state(self):
        if not self.robot:
            return {"connected": False}
        r = self.robot
        joints = sdk_call(r.jointPos)[:6]
        tcp = sdk_call(r.posture, self.sdk.CoordinateType.endInRef)
        flange = sdk_call(r.posture, self.sdk.CoordinateType.flangeInBase)
        if len(joints) != 6 or not all(math.isfinite(v) for v in joints + list(tcp) + list(flange)):
            raise RuntimeError("Invalid robot feedback")
        state = {"connected": True, "joints_deg": list(map(math.degrees, joints)),
                "limits_deg": self.limits_deg,
                "tcp_m_rad": list(tcp), "flange_m_rad": list(flange),
                "power": str(sdk_call(r.powerState)).split(".")[-1],
                "mode": str(sdk_call(r.operateMode)).split(".")[-1],
                "operation": str(sdk_call(r.operationState)).split(".")[-1]}
        self.on_robot(state)
        return state

    def ready(self):
        state = self.robot_state()
        if (not state["connected"] or state["mode"] != "manual"
                or state["operation"] != "idle" or state["power"] not in ("on", "off")):
            raise RuntimeError("Robot must be connected, manual and idle, without a power/safety fault")
        return state

    def jog(self, joint, delta, rate, check, target_deg=None):
        validate_rate(rate)
        if type(joint) is not int or joint not in range(1, 7):
            raise ValueError('Joint must be 1..6')
        start = self.ready()["joints_deg"]
        if target_deg is not None:
            delta = target_deg - start[joint - 1]
        if not math.isfinite(delta) or abs(delta) > 50:
            raise RuntimeError("Target is more than 50 degrees from current position")
        if abs(delta) < 0.01:
            return {"actual_deg": start, "already_at_target": True}
        target = start.copy()
        target[joint - 1] += delta
        bounds = self.sdk.PyTypeVectorArrayDouble2()
        sdk_call(self.robot.getSoftLimit, bounds)
        limits = bounds.content()
        if len(limits) < 6 or any(not lo <= math.radians(v) <= hi
                                  for v, (lo, hi) in zip(target, limits)):
            raise RuntimeError("Target outside reported joint bounds")
        try:
            check()
            commands = joint_commands(self.sdk, [target], rate, [0.])
            sdk_call(self.robot.setMotionControlMode, self.sdk.MotionControlMode.NrtCommandMode)
            sdk_call(self.robot.moveReset)
            sdk_call(self.robot.adjustSpeedOnline, 1.0)
            sdk_call(self.robot.setPowerState, True)
            if sdk_call(self.robot.powerState) != self.sdk.PowerState.on:
                raise RuntimeError("Power on not confirmed")
            check()
            current = self.ready()["joints_deg"]
            if max(abs(a - b) for a, b in zip(current, start)) > 0.03:
                raise RuntimeError('Joint start changed before submission')
            command_id = self.sdk.PyString()
            sdk_call(self.robot.moveAppend, commands, command_id)
            check()
            sdk_call(self.robot.moveStart)
            deadline = time.monotonic() + jog_timeout(delta, rate)
            while True:
                check()
                progress = joint_progress(self.robot, self.sdk, command_id)
                if progress is not None and progress[0] != 0:
                    raise RuntimeError('Invalid controller waypoint index')
                state = self.robot_state()
                actual = state["joints_deg"]
                if state["power"] != "on" or state["mode"] != "manual":
                    raise RuntimeError("Power or mode changed during jog")
                if any(not min(s, t) - 0.05 <= a <= max(s, t) + 0.05
                       for a, s, t in zip(actual, start, target)):
                    raise RuntimeError("Unexpected displacement")
                if (progress == (0, True) and state["operation"] == "idle"
                        and max(abs(a - t) for a, t in zip(actual, target)) <= 0.005):
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError("Jog timed out; no retry")
                time.sleep(0.1)
            sdk_call(self.robot.stop)
            samples = []
            for _ in range(5):
                time.sleep(0.2)
                check()
                actual = self.robot_state()["joints_deg"]
                if max(abs(a - t) for a, t in zip(actual, target)) > 0.01:
                    raise RuntimeError("Stopped position verification failed")
                samples.append(actual)
            if any(max(axis) - min(axis) > 0.002 for axis in zip(*samples)):
                raise RuntimeError("Position not stable after stop")
            return {"start_deg": start, "actual_deg": actual,
                    "motion_interface": "MoveAbsJ", "joint_rate": rate, "online_scale": 1.0}
        finally:
            self.safe_stop()
            sdk_call(self.robot.moveReset)

    def drag(self, enabled):
        if not self.robot:
            raise RuntimeError("Robot disconnected")
        if enabled:
            self.ready()
            self.drag_requested = True
            ec = {}
            # False retains the physical end-button requirement.
            self.robot.enableDrag(0, 2, ec, False)
            if ec.get("ec") != 0:
                raise RuntimeError(f"enableDrag: {ec}")
            deadline = time.monotonic() + 2
            while self.robot_state()["operation"] != "drag":
                if time.monotonic() >= deadline:
                    raise RuntimeError("Drag activation not confirmed")
                time.sleep(0.1)
        else:
            self.safe_stop()

    def safe_stop(self):
        if not self.robot:
            return
        errors = []
        actions = [(self.robot.stop, ())]
        try:
            dragging = self.drag_requested or sdk_call(self.robot.operationState) == self.sdk.OperationState.drag
        except Exception:
            dragging = True
        if dragging:
            actions.append((self.robot.disableDrag, ()))
        for method, args in actions:
            try:
                sdk_call(method, *args)
            except Exception as exc:
                errors.append(str(exc))
        self.drag_requested = False
        try:
            deadline = time.monotonic() + 5
            while sdk_call(self.robot.operationState) != self.sdk.OperationState.idle:
                if time.monotonic() >= deadline:
                    raise RuntimeError('Stop did not reach idle; inspect controller')
                time.sleep(0.05)
        except Exception as exc:
            errors.append(str(exc))
        # Still attempt power-off if stopping or feedback failed; never report it as confirmed.
        try:
            sdk_call(self.robot.setPowerState, False)
        except Exception as exc:
            errors.append(str(exc))
        try:
            state = self.robot_state()
            if state["power"] != "off" or state["operation"] != "idle":
                errors.append("Power off / idle not confirmed")
        except Exception as exc:
            errors.append(str(exc))
        if errors:
            raise RuntimeError("; ".join(errors))

    def disconnect_robot(self):
        try:
            if self.robot and self.robot_connected:
                sdk_call(self.robot.disconnectFromRobot)
        finally:
            self.robot = None
            self.robot_connected = False
            if self.runtime:
                self.runtime.__exit__(None, None, None)
                self.runtime = None

    def connect_gripper(self):
        if self.gripper:
            return
        from pymodbus.client import ModbusSerialClient
        self.gripper = ModbusSerialClient(
            "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ0296VS-if00-port0",
            baudrate=9600, bytesize=8, parity="N", stopbits=1, timeout=0.5, retries=0)
        try:
            if not self.gripper.connect():
                raise RuntimeError("Cannot open gripper serial port")
            self.gripper.socket.exclusive = True
            time.sleep(0.5)
            self.gripper_state()
        except Exception:
            self.disconnect_gripper()
            raise

    def read(self, address):
        response = self.gripper.read_holding_registers(address, count=1, device_id=2)
        if response.isError() or len(response.registers) != 1:
            raise RuntimeError(f"Gripper read {address:#x}: {response}")
        return response.registers[0]

    def write(self, address, value):
        response = self.gripper.write_register(address, value, device_id=2)
        if response.isError() or response.address != address or response.registers != [value]:
            raise RuntimeError("Gripper command unconfirmed; no retry")

    def gripper_state(self):
        if not self.gripper:
            return {"connected": False}
        init, status, pos = self.read(0x200), self.read(0x201), self.read(0x202)
        if init not in (0, 1, 2) or status not in (0, 1, 2, 3) or not 0 <= pos <= 1000:
            raise RuntimeError("Invalid gripper feedback")
        state = {"connected": True, "initialized": init, "status": status, "position": pos}
        self.on_gripper(state)
        return state

    def gripper_step(self, delta, check, target=None, force_percent=None):
        force_percent = validate_force(load_settings()['gripper']['force_percent']
                                       if force_percent is None else force_percent)
        state = self.gripper_state()
        if not state["connected"] or state["initialized"] != 1 or state["status"] not in (1, 2):
            raise RuntimeError("Gripper must be initialized and stopped; no automatic homing")
        start = state["position"]
        target = start + delta if target is None else target
        if abs(target - start) > 500:
            raise RuntimeError("Target is more than 50% from current gripper position")
        if not 0 <= target <= 1000:
            raise RuntimeError("Gripper target outside 0..1000")
        if target == start:
            return {"actual": start, "already_at_target": True}
        check()
        self.write(0x101, force_percent)
        if self.read(0x101) != force_percent:
            raise RuntimeError("Force setting readback failed")
        check()
        self.write(0x103, target)
        deadline = time.monotonic() + 15
        samples = []
        previous_status = None
        while time.monotonic() < deadline:
            time.sleep(0.15)
            check()
            state = self.gripper_state()
            actual = state["position"]
            if state["initialized"] != 1:
                raise RuntimeError("Gripper initialization state changed")
            if state["status"] == 3:
                raise RuntimeError("Gripper reports object dropped")
            if not min(start, target) - 2 <= actual <= max(start, target) + 2:
                raise RuntimeError("Unexpected gripper displacement")
            # Contact can stop closing before the requested empty-space target.
            holding = state["status"] == 2 and target < start
            if previous_status != state["status"]:
                samples.clear()
            previous_status = state["status"]
            if holding or (state["status"] == 1 and abs(actual - target) <= 2):
                samples.append(actual)
                if len(samples) >= 3 and max(samples[-3:]) - min(samples[-3:]) <= 1:
                    return {"start": start, "actual": actual, "force_percent": force_percent,
                            "holding": holding}
            else:
                samples.clear()
        raise RuntimeError("Gripper arrival timeout; serial close is not a stop")

    def disconnect_gripper(self):
        if self.gripper:
            self.gripper.close()
            self.gripper = None


class SimHardware:
    """Explicit simulation, never imports SDK or opens a device."""
    def __init__(self):
        self.robot = False
        self.gripper = False
        self.joints = [-26.48, -2.73, -109.25, 0.74, -70.78, -24.59]
        self.position = 1000
        self.operation = "idle"

    def connect_robot(self): self.robot = True
    def disconnect_robot(self): self.robot = False
    def connect_gripper(self): self.gripper = True
    def disconnect_gripper(self): self.gripper = False

    def robot_state(self):
        return {"connected": self.robot, "joints_deg": self.joints.copy(),
                "limits_deg": [[-165, 165], [-115, 115], [-165, 140], [-115, 115], [-165, 165], [-115, 115]],
                "tcp_m_rad": [0.46, -0.073, 0.315, -3.13, 0.048, 3.106],
                "flange_m_rad": [0.45, -0.07, 0.562, -3.13, 0.048, 3.106],
                "power": "off", "mode": "manual", "operation": self.operation}

    def gripper_state(self):
        return {"connected": self.gripper, "initialized": 1, "status": 1, "position": self.position}

    def jog(self, joint, delta, rate, check, target_deg=None):
        if not self.robot or self.operation != "idle": raise RuntimeError("Robot not ready")
        if target_deg is not None:
            delta = target_deg - self.joints[joint - 1]
        target = self.joints[joint - 1] + delta
        low, high = self.robot_state()["limits_deg"][joint - 1]
        if abs(delta) > 50 or not low <= target <= high:
            raise RuntimeError("Target outside joint step or position bounds")
        for _ in range(10):
            check()
            time.sleep(0.05)
        self.joints[joint - 1] += delta
        return {"actual_deg": self.joints.copy()}

    def gripper_step(self, delta, check, target=None, force_percent=None):
        if target is not None:
            delta = target - self.position
        if abs(delta) > 500:
            raise RuntimeError("Target is more than 50% from current gripper position")
        if not self.gripper or not 0 <= self.position + delta <= 1000:
            raise RuntimeError("Gripper not ready or outside bounds")
        check()
        self.position += delta
        return {"actual": self.position, "force_percent": force_percent}

    def drag(self, enabled):
        if not self.robot: raise RuntimeError("Robot disconnected")
        self.operation = "drag" if enabled else "idle"

    def safe_stop(self): self.operation = "idle"
