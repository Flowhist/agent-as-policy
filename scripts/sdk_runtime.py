"""Load the matching official SDK binary on Windows or Linux."""

from contextlib import contextmanager
import importlib.abc
import importlib.machinery
import os
from pathlib import Path
import sys


VISION_MODULES = {"cv2", "pyrealsense2"}


class _NativeIsolationGuard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        name = fullname.split(".", 1)[0]
        if name == "xCoreSDK_python":
            if VISION_MODULES.intersection(sys.modules):
                raise RuntimeError("SDK and vision libraries require separate processes")
        elif name in VISION_MODULES and "xCoreSDK_python" in sys.modules:
            raise RuntimeError("SDK and vision libraries require separate processes")
        return None


# Keep the guard after disconnect: native SDK libraries remain loaded.
sys.meta_path.insert(0, _NativeIsolationGuard())


@contextmanager
def sdk_runtime():
    if VISION_MODULES.intersection(sys.modules):
        raise RuntimeError("SDK and vision libraries require separate processes")
    platform_dir = {"win32": "windows", "linux": "linux"}.get(sys.platform)
    if platform_dir is None:
        raise RuntimeError(f"Unsupported SDK platform: {sys.platform}")
    lib = Path(__file__).resolve().parent.parent / "vendor/xCoreSDK-Python/Release" / platform_dir
    if not any((lib / ("xCoreSDK_python" + suffix)).is_file()
               for suffix in importlib.machinery.EXTENSION_SUFFIXES):
        raise RuntimeError(f"SDK binary for Python {sys.version_info.major}.{sys.version_info.minor} missing in {lib}")
    handle = os.add_dll_directory(str(lib)) if sys.platform == "win32" else None
    lock_file = None
    try:
        if sys.platform == "linux":
            import fcntl
            lock_path = Path(__file__).resolve().parent.parent / "logs/robot-sdk.lock"
            lock_path.parent.mkdir(parents=True, exist_ok=True)
            lock_file = lock_path.open("a")
            try:
                fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("Robot SDK is already held by another project process") from exc
        sys.path.insert(0, str(lib))
        try:
            yield
        finally:
            sys.path.remove(str(lib))
    finally:
        if lock_file is not None:
            lock_file.close()
        if handle is not None:
            handle.close()
