#! /usr/bin/env python3
"""
Simple YAML-driven V2X sweep runner.

For each run in scripts/x2v_automate.yaml:
1. Update the YAML attack params.
2. Start the unified dummy vehicle variants script.
3. Wait 2 seconds.
4. Run sumo2V_v5_nvN_v1.py.
5. Stop the dummy.

USE_FALLBACK / FALLBACK_MODE come from x2v_constants.py (dummy + SUMO both import them).
"""

import os
import re
import shlex
import signal
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from x2v_constants import USE_FALLBACK


REPO_ROOT = Path(__file__).resolve().parents[1]
AUTOMATE_YAML = REPO_ROOT / "scripts" / "x2v_automate.yaml"
SUMO_SCRIPT = REPO_ROOT / "scripts" / "sumo2V_v5_nvN_v1.py"
DUMMY_SCRIPT = REPO_ROOT / "test_dummies" / "dummy_samemachine_mpc_vehicle_variants.py"


@dataclass(frozen=True)
class Run:
    run_id: str
    delay_seconds: float
    attack_start_time: float
    bool_attack: bool = True


def load_runs():
    with AUTOMATE_YAML.open("r") as f:
        config = yaml.safe_load(f) or {}

    runs = []
    for entry in config.get("runs", []):
        if not isinstance(entry, dict):
            continue

        for run_id, value in entry.items():
            if value is None:
                print(f"[sweep] skipping {run_id}: no value")
                continue

            value_text = str(value).strip()
            if value_text.lower() in ("nominal", "none", "no_attack", "no attack"):
                runs.append(Run(run_id=str(run_id), delay_seconds=0.0, attack_start_time=10.0, bool_attack=False))
                continue

            numbers = re.findall(r"[-+]?\d+(?:\.\d+)?", value_text)
            if len(numbers) != 2:
                raise ValueError(f"Could not parse {run_id}: expected delay/start, got {value!r}")

            runs.append(
                Run(
                    run_id=str(run_id),
                    delay_seconds=float(numbers[0]),
                    attack_start_time=float(numbers[1]),
                )
            )

    if not runs:
        raise ValueError(f"No runs found in {AUTOMATE_YAML}")
    return runs


def set_yaml_value(text, key, value):
    pattern = re.compile(rf"(?m)^(\s*{re.escape(key)}\s*:\s*)([^#\n]*)(.*)$")

    def replace(match):
        comment = match.group(3)
        if comment.startswith("#"):
            comment = " " + comment
        return f"{match.group(1)}{value}{comment}"

    text, count = pattern.subn(replace, text, count=1)
    if count != 1:
        raise ValueError(f"Could not find `{key}` in {AUTOMATE_YAML}")
    return text


def update_yaml(run):
    text = AUTOMATE_YAML.read_text()
    text = set_yaml_value(text, "BOOL_ATTACK", "True" if run.bool_attack else "False")
    text = set_yaml_value(text, "ATTACK_START_TIME", str(run.attack_start_time))
    text = set_yaml_value(text, "ATTACK_ACTIVE", "False")
    text = set_yaml_value(text, "DELAY_SECONDS", str(run.delay_seconds))
    AUTOMATE_YAML.write_text(text)


def terminal_command(title, command):
    gnome_terminal = shutil.which("gnome-terminal")
    if gnome_terminal:
        return [gnome_terminal, "--title", title, "--", "bash", "-lc", command]

    xfce_terminal = shutil.which("xfce4-terminal")
    if xfce_terminal:
        return [xfce_terminal, "--title", title, "--command", f"bash -lc {shlex.quote(command)}"]

    xterm = shutil.which("xterm") or shutil.which("x-terminal-emulator")
    if xterm:
        return [xterm, "-T", title, "-e", "bash", "-lc", command]

    raise RuntimeError("Could not find gnome-terminal, xfce4-terminal, xterm, or x-terminal-emulator")


def open_terminal(title, command, env):
    return subprocess.Popen(
        terminal_command(title, command),
        cwd=str(REPO_ROOT),
        env=env,
        start_new_session=True,
    )


def process_is_running(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def wait_for_status_file(statusfile):
    while not statusfile.exists():
        time.sleep(0.5)
    return int(statusfile.read_text().strip())


def stop_dummy(pidfile):
    try:
        if pidfile.exists():
            pid = int(pidfile.read_text().strip())
            os.kill(pid, signal.SIGINT)
            return
        return
    except (ProcessLookupError, ValueError):
        return
    finally:
        if "pid" in locals():
            deadline = time.time() + 5
            while process_is_running(pid) and time.time() < deadline:
                time.sleep(0.2)

            if process_is_running(pid):
                print("[sweep] dummy vehicle did not stop after SIGINT; killing it")
                os.kill(pid, signal.SIGKILL)
        pidfile.unlink(missing_ok=True)


def main():
    runs = load_runs()
    print(f"[sweep] loaded {len(runs)} runs from {AUTOMATE_YAML}")
    print(f"[sweep] USE_FALLBACK={USE_FALLBACK} (from x2v_constants.py)")

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    python_paths = [str(REPO_ROOT / "scripts")]
    if env.get("PYTHONPATH"):
        python_paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(python_paths)

    for run in runs:
        print(
            f"\n[sweep] {run.run_id}: "
            f"BOOL_ATTACK={run.bool_attack}, "
            f"DELAY_SECONDS={run.delay_seconds}, "
            f"ATTACK_START_TIME={run.attack_start_time}"
        )
        update_yaml(run)

        dummy_pidfile = Path(f"/tmp/x2v_dummy_{os.getpid()}_{run.run_id}.pid")
        sumo_statusfile = Path(f"/tmp/x2v_sumo_{os.getpid()}_{run.run_id}.status")
        dummy_pidfile.unlink(missing_ok=True)
        sumo_statusfile.unlink(missing_ok=True)

        python = shlex.quote(sys.executable)
        repo_root = shlex.quote(str(REPO_ROOT))
        dummy_script = shlex.quote(str(DUMMY_SCRIPT))
        sumo_script = shlex.quote(str(SUMO_SCRIPT))
        dummy_pidfile_arg = shlex.quote(str(dummy_pidfile))
        sumo_statusfile_arg = shlex.quote(str(sumo_statusfile))

        dummy_command = f"cd {repo_root}; echo $$ > {dummy_pidfile_arg}; exec {python} {dummy_script}"
        sumo_command = (
            f"cd {repo_root}; "
            f"{python} {sumo_script}; "
            f"status=$?; echo $status > {sumo_statusfile_arg}; exit $status"
        )

        open_terminal(
            title=f"V2X {run.run_id} dummy",
            command=dummy_command,
            env=env,
        )

        try:
            time.sleep(2)
            sumo_terminal = open_terminal(
                title=f"V2X {run.run_id} SUMO",
                command=sumo_command,
                env=env,
            )
            sumo_terminal.poll()
            sumo_status = wait_for_status_file(sumo_statusfile)
            if sumo_status != 0:
                raise RuntimeError(f"SUMO exited with code {sumo_status}")
        finally:
            stop_dummy(dummy_pidfile)
            sumo_statusfile.unlink(missing_ok=True)

    print("\n[sweep] done")


if __name__ == "__main__":
    main()
