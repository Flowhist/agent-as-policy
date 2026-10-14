"""Wait for a device worker's cleanup when its caller is interrupted."""

import os
import signal
import subprocess


def run_worker(command, *, cwd):
    process = None
    interrupted = None

    def forward(signum, _frame):
        nonlocal interrupted
        interrupted = signum
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGTERM)

    signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    previous = {sig: signal.signal(sig, forward) for sig in signals}
    try:
        env = dict(os.environ, EMBODIED_WORKER_PARENT=str(os.getpid()))
        process = subprocess.Popen(command, cwd=cwd, env=env, start_new_session=True,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if interrupted is not None:
            forward(interrupted, None)
        stdout, stderr = process.communicate()
        if interrupted is not None:
            raise InterruptedError(f'Worker interrupted by signal {interrupted}; '
                                   f'exit={process.returncode}; {stderr[-2000:]}')
        return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)
    finally:
        # Never SIGKILL an SDK owner before it has attempted stop/off/disconnect.
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGTERM)
            process.communicate()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
