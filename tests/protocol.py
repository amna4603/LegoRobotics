#!/usr/bin/env python3
"""
Rehab protocol controller for Lego elbow exoskeleton (BrickPi3)

- 3 phases with different power/assistance
- Flex/extend cycles on motors B and C
- Press 'Q' + Enter at any time to stop the session
- Continuous velocity monitoring using motor encoders
"""

import time
import sys
import select
import math

import brickpi3 as BP

# ------------------------- USER PARAMETERS -------------------------

# Phase definitions
PHASES = [
    {
        "name": "Phase 1: Warm-Up & High Support",
        "label": "PHASE 1",
        "total_minutes": 3.0,
        "cycles": 3,
        "power": 95,
        "tflex": 2.5,
        "textend": 1.4,
        "rest_between_cycles": 3.0,
        "ramp": 0.7,
        "confidence_threshold": 0.3,
        "timeout_neutral": 10.0
    },
    {
        "name": "Phase 2: Moderate Support",
        "label": "PHASE 2",
        "total_minutes": 4.0,
        "cycles": 3,
        "power": 70,
        "tflex": 2.0,
        "textend": 1.5,
        "rest_between_cycles": 3.0,
        "ramp": 0.5,
        "confidence_threshold": 0.7,
        "timeout_neutral": 10.0
    },
    {
        "name": "Phase 3: Low Support & Higher Difficulty",
        "label": "PHASE 3",
        "total_minutes": 3.0,
        "cycles": 3,
        "power": 45,
        "tflex": 3.0,
        "textend": 2.0,
        "rest_between_cycles": 3.0,
        "ramp": 0.4,
        "confidence_threshold": 0.75,
        "timeout_neutral": 5.0
    },
]

# Time resolution for the control loops
DT = 0.05  # 50 ms

# Velocity monitoring (encoder units are degrees)
VEL_LIMIT_DEG_PER_S = 400.0  # quite generous to avoid constant stopping
VEL_ALPHA = 0.2              # smoothing factor for EMA of |velocity|


# ------------------------- UTILITY FUNCTIONS -------------------------

def check_for_quit() -> bool:
    """
    Non-blocking check if the user typed 'q' or 'Q' + Enter in the terminal.
    Returns True if we should stop immediately.
    """
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        line = sys.stdin.readline()
        if not line:
            return False
        if line.strip().lower() == 'q':
            print("\n[INFO] 'Q' detected – stopping protocol and returning to neutral.")
            return True
    return False


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


def set_elbow_power(bp, power):
    """
    Apply same power command to both elbow motors (B and C).
    NOTE: we must use bp.PORT_B / bp.PORT_C, not BP.PORT_B.
    """
    p = int(clamp(power, -100, 100))
    bp.set_motor_power(bp.PORT_B, p)
    bp.set_motor_power(bp.PORT_C, p)


def stop_elbow(bp):
    set_elbow_power(bp, 0)


def velocity_monitor_init(bp):
    """
    Initialise encoder-based velocity monitoring.
    Returns state: (enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev)
    """
    enc_prev_B = bp.get_motor_encoder(bp.PORT_B)
    enc_prev_C = bp.get_motor_encoder(bp.PORT_C)
    vel_ma_B = 0.0
    vel_ma_C = 0.0
    t_prev = time.time()
    return enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev


def velocity_monitor_step(bp, enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev):
    """
    One monitoring step:
    - compute encoder velocities for both motors
    - update exponential moving average of |velocity|
    - warn if above limit, but do NOT stop the system (to avoid "stops always").

    Returns updated (enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev).
    """
    t_now = time.time()
    dt = t_now - t_prev
    if dt <= 0:
        return enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev

    enc_now_B = bp.get_motor_encoder(bp.PORT_B)
    enc_now_C = bp.get_motor_encoder(bp.PORT_C)

    vel_B = (enc_now_B - enc_prev_B) / dt  # deg/s
    vel_C = (enc_now_C - enc_prev_C) / dt  # deg/s

    # EMA of absolute velocity
    vel_ma_B = (1.0 - VEL_ALPHA) * vel_ma_B + VEL_ALPHA * abs(vel_B)
    vel_ma_C = (1.0 - VEL_ALPHA) * vel_ma_C + VEL_ALPHA * abs(vel_C)

    max_vel = max(vel_ma_B, vel_ma_C)

    if max_vel > 0.5 * VEL_LIMIT_DEG_PER_S:
        # Only warn when we get close to the limit
        print(f"[VEL] |ω| ≈ {max_vel:.1f} deg/s (limit {VEL_LIMIT_DEG_PER_S:.0f})")

    # If you really want an emergency stop when velocity is too high,
    # you can uncomment this block (and maybe keep the limit high):
    #
    # if max_vel > VEL_LIMIT_DEG_PER_S:
    #     print(f"[SAFETY] Velocity above limit ({max_vel:.1f} deg/s). Stopping.")
    #     raise KeyboardInterrupt

    return enc_now_B, enc_now_C, vel_ma_B, vel_ma_C, t_now


def apply_power_profile(bp, target_power, duration, ramp, direction=1):
    """
    Apply a trapezoidal/triangular power profile:
        ramp up -> hold -> ramp down
    with continuous velocity monitoring using encoders.

    - target_power: absolute power level (0–100)
    - duration: total desired duration in seconds
    - ramp: ramp time in seconds for up and down
    - direction: +1 for flexion, -1 for extension
    """
    target_power = abs(target_power)
    ramp = max(0.0, ramp)
    duration = max(0.0, duration)

    if duration == 0:
        return

    # Initialise velocity monitoring
    enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev = velocity_monitor_init(bp)

    # Number of steps in ramp
    n_ramp_steps = max(1, int(ramp / DT)) if ramp > 0 else 1
    ramp_step = target_power / n_ramp_steps

    # 1) Ramp up
    for i in range(1, n_ramp_steps + 1):
        if check_for_quit():
            raise KeyboardInterrupt
        p = direction * int(i * ramp_step)
        set_elbow_power(bp, p)
        time.sleep(DT)
        enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev = velocity_monitor_step(
            bp, enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev
        )

    # 2) Hold
    core_time = duration - 2 * ramp
    if core_time > 0:
        t0 = time.time()
        while time.time() - t0 < core_time:
            if check_for_quit():
                raise KeyboardInterrupt
            set_elbow_power(bp, direction * int(target_power))
            time.sleep(DT)
            enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev = velocity_monitor_step(
                bp, enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev
            )

    # 3) Ramp down
    for i in range(n_ramp_steps, -1, -1):
        if check_for_quit():
            raise KeyboardInterrupt
        p = direction * int(i * ramp_step)
        set_elbow_power(bp, p)
        time.sleep(DT)
        enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev = velocity_monitor_step(
            bp, enc_prev_B, enc_prev_C, vel_ma_B, vel_ma_C, t_prev
        )

    stop_elbow(bp)


def flex_extend_cycle(bp, cfg, cycle_idx, total_cycles):
    """
    Perform one full flex-extend cycle with the given phase config.
    """
    print(f"\n  -> Cycle {cycle_idx}/{total_cycles}: flexion")
    apply_power_profile(
        bp,
        target_power=cfg["power"],
        duration=cfg["tflex"],
        ramp=cfg["ramp"],
        direction=-1  # flexion is now negative
    )

    print(f"  -> Cycle {cycle_idx}/{total_cycles}: extension")
    apply_power_profile(
        bp,
        target_power=cfg["power"],
        duration=cfg["textend"],
        ramp=cfg["ramp"],
        direction=+1  # extension is now positive
    )



def wait_with_quit(duration):
    """
    Wait for 'duration' seconds, checking for 'Q' regularly.
    """
    t0 = time.time()
    while time.time() - t0 < duration:
        if check_for_quit():
            raise KeyboardInterrupt
        time.sleep(DT)


def run_phase(bp, phase_idx, cfg, session_start_time):
    """
    Execute one phase of the protocol.
    """
    print("\n" + "=" * 70)
    print(f"{cfg['label']} – {cfg['name']}")
    print("=" * 70)
    minutes_since_start = (time.time() - session_start_time) / 60.0
    print(f"[INFO] Time since session start: {minutes_since_start:.1f} min")
    print(f"[INFO] Target: ~{cfg['total_minutes']} min, "
          f"{cfg['cycles']} cycles, power={cfg['power']}%")
    print("[INFO] Press 'Q' + Enter at ANY time to stop the session.\n")

    print(f"[BCI NOTE] Intended classifier confidence threshold: "
          f"{cfg['confidence_threshold']:.2f}")
    print(f"[BCI NOTE] Intended timeout to neutral: "
          f"{cfg['timeout_neutral']:.1f} s\n")

    total_cycles = cfg["cycles"]
    auto_start = True  # later we'll replace this with MI triggers

    for c in range(1, total_cycles + 1):
        if check_for_quit():
            raise KeyboardInterrupt

        print(f"\n[{cfg['label']}] Preparing cycle {c}/{total_cycles} ...")
        if auto_start:
            print(f"[{cfg['label']}] (AUTO) starting movement.")
        else:
            # FUTURE: wait here for MI trigger
            time.sleep(0.5)

        flex_extend_cycle(bp, cfg, c, total_cycles)

        if c < total_cycles and cfg["rest_between_cycles"] > 0:
            print(f"[{cfg['label']}] Resting {cfg['rest_between_cycles']} s "
                  f"before next cycle...")
            wait_with_quit(cfg["rest_between_cycles"])

    print(f"\n[{cfg['label']}] Completed all {total_cycles} cycles.")
    print(f"[{cfg['label']}] Returning to neutral / low torque.")
    stop_elbow(bp)


def main():
    print("==============================================")
    print("      MI-BCI Rehab Protocol – Elbow Exo       ")
    print("==============================================")
    print("3-phase protocol, press 'Q' + Enter to stop.\n")

    bp = BP.BrickPi3()
    session_start_time = time.time()

    try:
        try:
            v_batt = bp.get_voltage_battery()
            print(f"[INFO] Battery voltage: {v_batt:.2f} V\n")
        except Exception as e:
            print(f"[WARN] Could not read battery voltage: {e}\n")

        stop_elbow(bp)

        for idx, phase_cfg in enumerate(PHASES, start=1):
            run_phase(bp, idx, phase_cfg, session_start_time)

        print("\n[SESSION] All phases completed successfully.")
        elapsed_min = (time.time() - session_start_time) / 60.0
        print(f"[SESSION] Total duration: {elapsed_min:.1f} minutes.")

    except KeyboardInterrupt:
        print("\n[ABORT] Session interrupted by user.")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
    finally:
        print("[CLEANUP] Stopping motors and resetting BrickPi3...")
        try:
            stop_elbow(bp)
            bp.reset_all()
        except Exception:
            pass
        print("[CLEANUP] Done.")


if __name__ == "__main__":
    main()
