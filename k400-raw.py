#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
import math
import signal
import threading

from k400_raw.hidpp import LOG, WPID_K400_PLUS
from k400_raw.pointer import PointerBackend
from k400_raw.touchpad import TouchpadBackend
from k400_raw.runner import run


def build_parser():
    p = argparse.ArgumentParser(
        description="Logitech K400 Plus HID++ raw-touch bridge (touchpad or custom pointer mode)."
    )
    p.add_argument("--mode", choices=("touchpad", "pointer"), default="touchpad")
    p.add_argument("--slot", type=int, default=0, help="Receiver slot override; 0 auto-detects")
    p.add_argument("--wpid", default=WPID_K400_PLUS)
    p.add_argument("--receiver-path")
    p.add_argument("--feature-wait-seconds", type=float, default=15.0)
    p.add_argument("--reconnect-delay-seconds", type=float, default=2.0)
    p.add_argument("--fn-mode", choices=("leave","standard","special"), default="standard")
    p.add_argument("--invert-x", action="store_true")
    p.add_argument("--invert-y", action="store_true")
    p.add_argument("--print-raw", action="store_true")
    p.add_argument("--log-level", choices=("warning","info","debug"), default="warning")
    p.add_argument("-v", "--verbose", action="count", default=0)

    # Native touchpad backend: intentionally minimal.
    p.add_argument("--touchpad-name", default="Logitech K400 Raw Touchpad")
    p.add_argument("--touchpad-resolution-units-per-mm", type=int, default=0,
                   help="0 derives resolution from the K400 raw DPI field")

    # Advanced custom pointer backend.
    p.add_argument("--pointer-name", default="Logitech K400 Raw Pointer")
    p.add_argument("--pointer-gain", type=float, default=1.0)
    p.add_argument("--pointer-gain-x", type=float)
    p.add_argument("--pointer-gain-y", type=float)
    p.add_argument("--pointer-precision-gain", type=float, default=0.6)
    p.add_argument("--pointer-precision-speed", type=float, default=4.0)
    p.add_argument("--pointer-precision-transition", type=float, default=5.0)
    p.add_argument("--pointer-precision-filter-ms", type=float, default=35.0)
    p.add_argument("--pointer-tap", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pointer-two-finger-tap", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pointer-tap-max-ms", type=float, default=250.0)
    p.add_argument("--pointer-tap-move-units", type=float, default=100.0)
    p.add_argument("--pointer-tap-drag-window-ms", type=float, default=300.0)
    p.add_argument("--pointer-tap-drag-activation-units", type=float, default=20.0)
    p.add_argument("--pointer-scroll", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pointer-scroll-start-units", type=float, default=30.0)
    p.add_argument("--pointer-scroll-units-per-step", type=float, default=80.0)
    p.add_argument("--pointer-scroll-invert", action="store_true")
    p.add_argument("--pointer-horizontal-scroll", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--pointer-hscroll-units-per-step", type=float, default=80.0)
    p.add_argument("--pointer-hscroll-invert", action="store_true")
    p.add_argument("--pointer-scroll-axis-lock", choices=("on","off"), default="on")
    p.add_argument("--pointer-scroll-axis-lock-ratio", type=float, default=1.5)
    p.add_argument("--pointer-multitouch-entry-grace-ms", type=float, default=50.0)
    p.add_argument("--pointer-multitouch-exit-grace-ms", type=float, default=80.0)
    p.add_argument("--pointer-kinetic-scroll", action=argparse.BooleanOptionalAction, default=True)
    p.add_argument("--pointer-kinetic-history-ms", type=float, default=80.0)
    p.add_argument("--pointer-kinetic-start-velocity", type=float, default=0.70)
    p.add_argument("--pointer-kinetic-stop-velocity", type=float, default=0.20)
    p.add_argument("--pointer-kinetic-decay-ms", type=float, default=400.0)
    p.add_argument("--pointer-kinetic-max-ms", type=float, default=1600.0)
    p.add_argument("--pointer-max-raw-jump", type=int, default=600)
    p.add_argument("--no-uinput", action="store_true", help="Pointer-mode diagnostic only")
    return p


def validate(p, a):
    if not 0 <= a.slot <= 15: p.error("--slot must be 0..15")
    if a.touchpad_resolution_units_per_mm < 0: p.error("touchpad resolution must be >= 0")
    pos = [
        (a.feature_wait_seconds,"--feature-wait-seconds"),
        (a.reconnect_delay_seconds,"--reconnect-delay-seconds"),
        (a.pointer_gain,"--pointer-gain"),
        (a.pointer_precision_gain,"--pointer-precision-gain"),
        (a.pointer_tap_max_ms,"--pointer-tap-max-ms"),
        (a.pointer_tap_move_units,"--pointer-tap-move-units"),
        (a.pointer_scroll_units_per_step,"--pointer-scroll-units-per-step"),
        (a.pointer_hscroll_units_per_step,"--pointer-hscroll-units-per-step"),
        (a.pointer_kinetic_history_ms,"--pointer-kinetic-history-ms"),
        (a.pointer_kinetic_start_velocity,"--pointer-kinetic-start-velocity"),
        (a.pointer_kinetic_stop_velocity,"--pointer-kinetic-stop-velocity"),
        (a.pointer_kinetic_decay_ms,"--pointer-kinetic-decay-ms"),
        (a.pointer_kinetic_max_ms,"--pointer-kinetic-max-ms"),
    ]
    for value,name in pos:
        if not math.isfinite(value) or value <= 0: p.error(f"{name} must be positive")
    for value,name in [(a.pointer_gain_x,"--pointer-gain-x"),(a.pointer_gain_y,"--pointer-gain-y")]:
        if value is not None and (not math.isfinite(value) or value <= 0): p.error(f"{name} must be positive")
    if not (0 < a.pointer_precision_gain <= 1): p.error("--pointer-precision-gain must be >0 and <=1")
    if a.pointer_precision_speed < 0: p.error("--pointer-precision-speed must be >=0")
    if a.pointer_precision_transition <= a.pointer_precision_speed:
        p.error("--pointer-precision-transition must exceed --pointer-precision-speed")
    if a.pointer_precision_filter_ms < 0: p.error("--pointer-precision-filter-ms must be >=0")
    if a.pointer_tap_drag_window_ms < 0: p.error("--pointer-tap-drag-window-ms must be >=0")
    if a.pointer_tap_drag_activation_units < 0: p.error("--pointer-tap-drag-activation-units must be >=0")
    if a.pointer_scroll_start_units < 0: p.error("--pointer-scroll-start-units must be >=0")
    if a.pointer_scroll_axis_lock_ratio <= 1: p.error("--pointer-scroll-axis-lock-ratio must be >1")
    if a.pointer_multitouch_entry_grace_ms < 0 or a.pointer_multitouch_exit_grace_ms < 0:
        p.error("multitouch grace values must be >=0")
    if a.pointer_kinetic_stop_velocity >= a.pointer_kinetic_start_velocity:
        p.error("kinetic stop velocity must be below start velocity")


def main():
    p=build_parser(); a=p.parse_args(); validate(p,a)
    level={"warning":logging.WARNING,"info":logging.INFO,"debug":logging.DEBUG}[a.log_level]
    if a.verbose==1: level=logging.INFO
    elif a.verbose>=2: level=logging.DEBUG
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(levelname)-8s %(name)s: %(message)s")
    LOG.setLevel(level)

    backend = TouchpadBackend(a) if a.mode == "touchpad" else PointerBackend(a)
    stop=threading.Event()
    def sig(signum,_frame):
        LOG.info("Received signal %s; stopping", signum); stop.set()
    signal.signal(signal.SIGINT,sig); signal.signal(signal.SIGTERM,sig)
    try:
        return run(a, backend, stop)
    finally:
        backend.close()

if __name__ == "__main__":
    raise SystemExit(main())
