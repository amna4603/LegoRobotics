#!/usr/bin/env python3
import socket
import json
import subprocess
from typing import Any, Dict, Optional

# UDP settings (must match MATLAB exoPort)
LISTEN_IP = "0.0.0.0"   # listen on all interfaces
LISTEN_PORT = 5014

# Path to your movement script
EXO_SCRIPT = "/home/pi/lego_arm/tests/test_exo_movements.py"

# MATLAB sends conf as 0..100 (percent). Set your threshold in percent here.
CONF_THRESH_PERCENT = 10.0


def run_exo_action(action: str) -> None:
    """
    Map 'grab' / 'lift' actions to specific exo movements.
    Adjust the parameters as you like.
    """
    if action == "grab":
        cmd = [
            "python3", EXO_SCRIPT,
            "--power", "0",
            "--tflex", "2.0",
            "--textend", "2.0",
            "--reps", "1",
            "--ramp", "0.8",
            "--vmax", "50",
            "--vhard", "120",
        ]
    elif action == "lift":
        cmd = [
            "python3", EXO_SCRIPT,
            "--power", "80",
            "--tflex", "3.4",
            "--textend", "2.0",
            "--reps", "1",
            "--ramp", "0.6",
            "--vmax", "160",
            "--vhard", "280",
        ]
    else:
        print(f"[WARN] Unknown action '{action}', ignoring.")
        return

    print(f"[EXO] Running action '{action}' with command: {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True)
        print("[EXO] Movement done.")
    except subprocess.CalledProcessError as e:
        print(f"[ERROR] Movement script failed: {e}")


def _to_float(x: Any) -> Optional[float]:
    """Best-effort conversion to float (handles numbers or numeric strings)."""
    if x is None:
        return None
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def handle_message(msg: Dict[str, Any], addr) -> None:
    msg_type = msg.get("type", "")

    if msg_type == "ping":
        print("[INFO] Ping received from MATLAB/Unity.")
        return

    if msg_type != "pred":
        print(f"[WARN] Unknown message type '{msg_type}', skipping.")
        return

    # MATLAB sends:
    # type="pred", trial_id=i, cmd="grab"/"lift", conf=<0..100>, t=<posix time>
    action = msg.get("cmd", None)          # <-- changed from "class" to "cmd"
    conf   = _to_float(msg.get("conf", None))
    trial  = msg.get("trial_id", None)
    ts     = msg.get("t", None)

    print(f"[PRED] from={addr}, trial={trial}, cmd={action}, conf={conf}, t={ts}")

    # Confidence gate (percent 0..100 from MATLAB)
    if conf is not None and conf < CONF_THRESH_PERCENT:
        print(f"[INFO] Low confidence ({conf} < {CONF_THRESH_PERCENT}) → ignoring.")
        return

    if action in ("grab", "lift"):
        run_exo_action(action)
    else:
        print(f"[WARN] Unsupported cmd '{action}', ignoring.")


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"[SERVER] Listening on {LISTEN_IP}:{LISTEN_PORT}")

    while True:
        data, addr = sock.recvfrom(4096)
        msg_str = data.decode("utf-8", errors="ignore").strip()
        print(f"[RX] From {addr}: {msg_str}")

        try:
            msg = json.loads(msg_str)
        except json.JSONDecodeError:
            print("[WARN] Not valid JSON, skipping.")
            continue

        if not isinstance(msg, dict):
            print("[WARN] JSON is not an object/dict, skipping.")
            continue

        handle_message(msg, addr)


if __name__ == "__main__":
    main()
