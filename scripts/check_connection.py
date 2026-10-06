"""Query an idle SR4-C without enabling power or changing modes.

Run only while the robot is idle: SDK disconnect can stop robot motion.
"""

import json
import math
import os
from pathlib import Path
import sys
from datetime import datetime
from sdk_runtime import sdk_runtime


ROOT = Path(__file__).resolve().parent.parent


def main():
    log_dir = ROOT / "logs" / "runs"
    sdk_dir = ROOT / "logs" / "sdk"
    log_dir.mkdir(parents=True, exist_ok=True)
    sdk_dir.mkdir(parents=True, exist_ok=True)
    # Native SDK logs are relative to the process working directory.
    os.chdir(sdk_dir)
    report = log_dir / ("configuration-check-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".jsonl")
    with report.open("w", encoding="utf-8") as log:
        def emit(step, **fields):
            record = {"time": datetime.now().astimezone().isoformat(),
                      "step": step, **fields}
            line = json.dumps(record, ensure_ascii=True, default=str)
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        robot = None
        connected = False
        failed = False
        with sdk_runtime():
            try:
                import xCoreSDK_python as sdk

                emit("import", python=sys.version, sdk=sdk.BaseRobot.sdkVersion())
                robot = sdk.xMateRobot()
                emit("connect_start", ip="192.168.0.160")
                robot.connectToRobot("192.168.0.160", "")
                connected = True
                emit("connected")

                def query(name, call, serialize=lambda value: value):
                    nonlocal failed
                    ec = {}
                    value = call(ec)
                    emit(name, value=serialize(value), error=ec)
                    if ec.get("ec") != 0:
                        failed = True
                    return value

                query("robotInfo", robot.robotInfo,
                      lambda info: {key: getattr(info, key) for key in
                                    ("id", "version", "type", "joint_num", "mac")})
                query("powerState", robot.powerState, str)
                query("operateMode", robot.operateMode, str)
                query("operationState", robot.operationState, str)
                query("jointPos_rad", robot.jointPos)
                query("posture_endInRef_m_rad", lambda ec:
                      robot.posture(sdk.CoordinateType.endInRef, ec))
                query("posture_flangeInBase_m_rad", lambda ec:
                      robot.posture(sdk.CoordinateType.flangeInBase, ec))
                query("jointPos_deg", robot.jointPos,
                      lambda values: [math.degrees(v) for v in values[:6]])
                query("baseFrame_m_rad", robot.baseFrame)

                def frame(value):
                    return {"trans_m": value.trans, "rpy_rad": value.rpy}

                def load(value):
                    return {"mass_kg": value.mass, "cog_m": value.cog,
                            "inertia_kg_m2": value.inertia}

                query("sdk_toolset", robot.toolset, lambda value:
                      {"end": frame(value.end), "ref": frame(value.ref),
                       "load": load(value.load)})
                query("saved_tools", robot.toolsInfo, lambda values:
                      [{"name": v.name, "robotHeld": v.robotHeld,
                        "pose": frame(v.pos), "load": load(v.load)} for v in values])
                limits = sdk.PyTypeVectorArrayDouble2()
                query("soft_limits", lambda ec: robot.getSoftLimit(limits, ec),
                      lambda enabled: {"enabled": enabled, "limits_rad": limits.content()})
                query("recent_warning_error_logs", lambda ec:
                      robot.queryControllerLog(10, {sdk.LogInfoLevel.warning,
                                                   sdk.LogInfoLevel.error}, ec),
                      lambda values: [{key: getattr(v, key) for key in
                                       ("id", "timestamp", "content", "repair")}
                                      for v in values])
            except Exception as exc:
                failed = True
                emit("exception", type=type(exc).__name__, message=str(exc))
            finally:
                if connected:
                    try:
                        ec = {}
                        robot.disconnectFromRobot(ec)
                        emit("disconnect", error=ec)
                        failed = failed or ec.get("ec") != 0
                    except Exception as exc:
                        failed = True
                        emit("disconnect_exception", message=str(exc))
        return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
