#!/usr/bin/env python3
"""Run a Carat portfolio on Linux (Python 3.8+)."""

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
import json
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
from typing import Optional


SCRIPT_DIR = Path(__file__).resolve().parent
PORTFOLIOS = {
    "safety": ("portfolio_safety.json", (".aig", ".aag", ".btor2")),
    "array": ("portfolio_array.json", (".btor2",)),
    "liveness": ("portfolio_liveness.json", (".aig", ".aag")),
}
RESULTS = {"Safe", "Unsafe", "Unknown"}
DEFINITE = {"Safe", "Unsafe"}
STOP_GRACE_SECONDS = 0.5
MIB = 1024 * 1024

# Set limits in a fresh interpreter before exec, not in preexec_fn: the
# coordinator already has threads, so running Python after fork is unsafe.
# exec preserves the PID/process group and the limits for the Carat process.
MEMORY_LIMIT_LAUNCHER = """
import os
import resource
import sys

limit = int(sys.argv[1])
for inherited in resource.getrlimit(resource.RLIMIT_AS):
    if inherited != resource.RLIM_INFINITY:
        limit = min(limit, inherited)
try:
    resource.setrlimit(resource.RLIMIT_AS, (limit, limit))
    print('Memory limit: {} bytes (RLIMIT_AS)'.format(limit), file=sys.stderr, flush=True)
    os.execv(sys.argv[2], sys.argv[2:])
except (OSError, ValueError) as error:
    print('Cannot apply memory limit or start Carat: {}'.format(error), file=sys.stderr)
    sys.exit(1)
"""


def _memory_limit(value, where):
    if type(value) is not int or not 0 <= value <= sys.maxsize // MIB:
        raise ValueError("{}: memory_limit_mb must be a non-negative integer <= {} (MiB; 0 adds no limit)".format(
            where, sys.maxsize // MIB))
    return value


def _check_args(args, where):
    if not isinstance(args, list) or any(
        not isinstance(arg, str) or not arg or "\0" in arg for arg in args
    ):
        raise ValueError("{} must be an array of nonempty strings".format(where))
    options = set()
    for arg in args:
        if arg == "--":
            raise ValueError("{} must not contain '--'".format(where))
        if not re.match(r"^--?[A-Za-z]", arg):
            continue
        option = arg.split("=", 1)[0] if arg.startswith("--") else arg[:2]
        if option in {"-w", "-h", "--help", "--wl-bitblast-only"} or option.startswith("--bmc_cnf"):
            raise ValueError("{}: {} is not a portfolio solver option".format(where, option))
        if option in options:
            raise ValueError("{}: duplicate option {}".format(where, option))
        options.add(option)


def _load_config(path):
    with path.open(encoding="utf-8") as stream:
        config = json.load(stream)
    if not isinstance(config, dict) or set(config) - {"common_args", "workers", "memory_limit_mb"}:
        raise ValueError("configuration must contain only common_args, workers and memory_limit_mb")
    memory_limit_mb = _memory_limit(config.get("memory_limit_mb", 0), "configuration")
    common = config.get("common_args", [])
    _check_args(common, "common_args")
    workers = config.get("workers")
    if not isinstance(workers, list):
        raise ValueError("workers must be an array")
    names, tasks = set(), []
    for worker in workers:
        if not isinstance(worker, dict) or set(worker) - {"name", "args", "memory_limit_mb"}:
            raise ValueError("each worker must contain only name, args and memory_limit_mb")
        name = worker.get("name")
        if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name):
            raise ValueError("worker names must use letters, digits, '.', '_' or '-', starting with a letter or digit")
        if name in names:
            raise ValueError("duplicate worker name: {}".format(name))
        names.add(name)
        args = worker.get("args", [])
        _check_args(args, name)
        _check_args(common + args, name)
        limit = _memory_limit(worker.get("memory_limit_mb", memory_limit_mb), name)
        tasks.append((name, common + args, limit))
    if not tasks:
        raise ValueError("configuration has no workers")
    return tasks


@dataclass
class Task:
    name: str
    directory: Path
    process: Optional[subprocess.Popen] = None
    waiter: Optional[threading.Thread] = None
    result: Optional[str] = None
    error: Optional[str] = None
    stopped: Optional[str] = None


def _wait_for_task(task, completed):
    try:
        code = task.process.wait()
        if not task.stopped:
            if code != 0:
                task.error = "abnormal exit"
            else:
                last = ""
                with (task.directory / "stdout.log").open(encoding="utf-8", errors="replace") as stream:
                    for line in stream:
                        if line.strip():
                            last = line.strip()
                if last in RESULTS:
                    task.result = last
                else:
                    task.error = "missing final Safe/Unsafe/Unknown result"
    except Exception as error:
        task.error = str(error)
    finally:
        completed.put(task)


def _report_task(task):
    code = task.process.returncode if task.process is not None else "not started"
    if task.stopped:
        status = "terminated by portfolio: " + task.stopped
    elif task.error:
        status = "failed: " + task.error
    else:
        status = "completed: " + task.result
    print("\n[{}] {} (exit={})".format(task.name, status, code), file=sys.stderr)
    has_output = False
    for name in ("stdout", "stderr"):
        path = task.directory / (name + ".log")
        if path.is_file() and path.stat().st_size:
            has_output = True
            print("--- {} ---".format(name), file=sys.stderr)
            with path.open(encoding="utf-8", errors="replace") as stream:
                shutil.copyfileobj(stream, sys.stderr)
            print(file=sys.stderr)
    if not has_output:
        print("(no output)", file=sys.stderr)
    sys.stderr.flush()


def _report_tasks(reports):
    # A slow terminal must not delay the coordinator stopping other solvers.
    try:
        while True:
            task = reports.get()
            if task is None:
                return
            _report_task(task)
    except OSError:
        # Log-output failure must not prevent child process cleanup.
        return


def _signal_group(task, signum):
    try:
        os.killpg(task.process.pid, signum)
        return True
    except ProcessLookupError:
        return False


def _stop_tasks(tasks, reason):
    started = [task for task in tasks if task.process is not None]
    for task in started:
        if task.process.poll() is None:
            task.stopped = reason
        # Also clean descendants whose group leader has already exited.
        _signal_group(task, signal.SIGTERM)
    deadline = time.monotonic() + STOP_GRACE_SECONDS
    alive = started
    while alive:
        alive = [task for task in alive if _signal_group(task, 0)]
        if not alive or time.monotonic() >= deadline:
            break
        time.sleep(0.01)
    for task in alive:
        _signal_group(task, signal.SIGKILL)
    for task in started:
        task.process.wait()
        if task.waiter is not None and task.waiter.ident is not None:
            task.waiter.join()


def _run(carat, model, workers, directory, witness, interrupted):
    completed, reports = queue.Queue(), queue.Queue()
    tasks, finished = [], []
    winner = None
    reporter = threading.Thread(target=_report_tasks, args=(reports,))
    reporter.start()

    def record(task):
        finished.append(task)
        reports.put(task)

    try:
        for name, args, memory_limit_mb in workers:
            if interrupted:
                break
            task = Task(name, directory / name)
            tasks.append(task)
            try:
                task.directory.mkdir()
                command = [str(carat), str(model)] + args
                if witness:
                    output = task.directory / "witness"
                    output.mkdir()
                    # AIGER witness paths currently use string concatenation.
                    command += ["-w", str(output) + os.sep]
                if memory_limit_mb:
                    command = [sys.executable, "-S", "-c", MEMORY_LIMIT_LAUNCHER,
                               str(memory_limit_mb * MIB)] + command
                with (task.directory / "stdout.log").open("wb") as stdout, (task.directory / "stderr.log").open("wb") as stderr:
                    task.process = subprocess.Popen(
                        command, cwd=str(task.directory), stdin=subprocess.DEVNULL,
                        stdout=stdout, stderr=stderr, start_new_session=True,
                    )
            except OSError as error:
                task.error = "cannot start: {}".format(error)
                completed.put(task)
                continue
            task.waiter = threading.Thread(target=_wait_for_task, args=(task, completed))
            task.waiter.start()

        while len(finished) < len(tasks) and not interrupted:
            try:
                task = completed.get(timeout=0.1)
            except queue.Empty:
                continue
            record(task)
            if task.result in DEFINITE:
                winner = task
                break
    finally:
        reason = "winner={}".format(winner.name) if winner else "portfolio stopped"
        if interrupted:
            reason = "received {}".format(signal.Signals(interrupted[0]).name)
        try:
            _stop_tasks(tasks, reason)
            while not completed.empty():
                record(completed.get_nowait())
        finally:
            reports.put(None)
            reporter.join()

    if interrupted:
        return "Unknown", 128 + interrupted[0], None
    conclusions = {task.result for task in finished if not task.stopped and task.result in DEFINITE}
    if len(conclusions) > 1:
        raise ValueError("conflicting Safe and Unsafe results; inspect the worker logs")
    if winner is not None:
        return winner.result, 0, winner
    return "Unknown", 2 if any(task.result == "Unknown" for task in finished) else 1, None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  %(prog)s --type safety model.aig\n"
            "  %(prog)s --type array model.btor2\n"
            "  %(prog)s --type liveness model.aig\n"
            "\nBoth --type and the input model are required.\n"
            "\nJSON memory_limit_mb: per-worker virtual address space limit in MiB\n"
            "(1 MiB = 1048576 bytes, Linux RLIMIT_AS; not an RSS limit). Set it at\n"
            "the top level for all workers, or on a worker to override the default.\n"
            "Omitted defaults to 0; 0 adds no limit. Existing stricter limits remain\n"
            "in effect. This is a per-process cap, not a total portfolio budget."
        ),
    )
    parser.add_argument("model", type=Path, help="required input model file")
    parser.add_argument("--type", required=True, choices=tuple(PORTFOLIOS),
                        help="required problem type: safety for AIGER/array-free BTOR2, array for BTOR2 safety with arrays, liveness for AIGER liveness")
    parser.add_argument("--carat", type=Path, default=SCRIPT_DIR.parent / "build" / "carat",
                        help="Carat executable (default: project build/carat)")
    parser.add_argument("--log-dir", type=Path, help="keep a unique run directory here instead of deleting temporary logs")
    parser.add_argument("-w", type=Path, help="copy the winning task's witness files into this directory")
    args = parser.parse_args()
    config_name, extensions = PORTFOLIOS[args.type]
    interrupted = []

    def handle_signal(signum, frame):
        # Do not raise between Popen() and registering the new process.
        if not interrupted:
            interrupted.append(signum)

    previous = {sig: signal.signal(sig, handle_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    result, code = "Unknown", 1
    try:
        if not sys.platform.startswith("linux"):
            raise ValueError("the portfolio scripts support Linux only")
        carat = args.carat.expanduser().resolve()
        # Preserve the input symlink's extension: Carat selects its frontend
        # using the path it receives, not the symlink target's filename.
        model = args.model.expanduser().absolute()
        if not carat.is_file() or not os.access(str(carat), os.X_OK):
            raise ValueError("Carat executable not found or not executable: {}".format(carat))
        if not model.is_file() or model.suffix not in extensions:
            raise ValueError("expected an existing {} model: {}".format("/".join(extensions), model))
        workers = _load_config(SCRIPT_DIR / config_name)
        with ExitStack() as stack:
            if args.log_dir is not None:
                log_dir = args.log_dir.expanduser().resolve()
                log_dir.mkdir(parents=True, exist_ok=True)
                directory = Path(tempfile.mkdtemp(prefix="carat-portfolio-", dir=str(log_dir)))
            else:
                directory = Path(stack.enter_context(tempfile.TemporaryDirectory(prefix="carat-portfolio-")))
            print("Run directory: {}{}".format(directory, " (kept)" if args.log_dir else " (temporary)"), file=sys.stderr)
            result, code, winner = _run(carat, model, workers, directory, args.w is not None, interrupted)
            if winner is not None and args.w is not None:
                output = args.w.expanduser().resolve()
                output.mkdir(parents=True, exist_ok=True)
                files = list((winner.directory / "witness").iterdir())
                for path in files:
                    shutil.copy2(str(path), str(output / path.name))
                print("Witness: {} file(s) copied to {}".format(len(files), output), file=sys.stderr)
    except (OSError, ValueError, RuntimeError) as error:
        code = 1
        print("Portfolio error: {}".format(error), file=sys.stderr)
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    if interrupted:
        result, code = "Unknown", 128 + interrupted[0]
    print(result, flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
