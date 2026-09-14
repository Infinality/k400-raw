#!/usr/bin/env python3
"""
k400-raw-pointer.py

Experimental Logitech K400 Plus HID++ 0x6100 raw-touch -> relative-pointer
daemon, with optional raw-vs-native comparison/data-collection mode.

Targets:
  - Logitech K400 Plus WPID 404D
  - HID++ 2.0 feature 0x6100 (TOUCHPAD_RAW_XY)
  - Solaar 1.1.20 Python modules
  - python-evdev for /dev/uinput and evdev comparison capture

Normal mode:
  * enable RAW | ENHANCED (0x05)
  * turn raw absolute touch coordinates into a virtual REL_X/REL_Y pointer
  * base linear gain with an optional low-speed precision/deceleration region
  * one-finger tap -> left click; two-finger tap -> right click
  * two-finger raw scrolling
  * leave the physical K400 buttons alone

Comparison mode (--compare-device):
  * request RAW | ENHANCED | RAW_AND_NATIVE (0x15)
  * verify the complete HID++ state readback
  * capture raw HID++ XY and the physical K400 evdev stream simultaneously
  * write an event CSV
  * write a 100 ms (configurable) window summary CSV
  * print raw-velocity buckets and native/raw path-gain medians at exit
  * capture native wheel/button/key events too, useful for testing taps/scroll

The physical evdev device is opened read-only and is NOT grabbed.
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import os
import signal
import statistics
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

try:
    from evdev import InputDevice, UInput, ecodes, list_devices
except ImportError as exc:
    raise SystemExit(
        "python-evdev is required. On Fedora install it with:\n"
        "  sudo dnf install python3-evdev"
    ) from exc

try:
    from logitech_receiver import base
    from logitech_receiver import listener
    from logitech_receiver import receiver as receiver_mod
    from logitech_receiver.hidpp20_constants import SupportedFeature
except ImportError as exc:
    raise SystemExit(
        "Solaar's Python modules were not found.\n"
        "On Fedora install the stock package with:\n"
        "  sudo dnf install solaar\n"
        "Then run this daemon with /usr/bin/python3."
    ) from exc


LOG = logging.getLogger("k400-raw-pointer")

WPID_K400_PLUS = "404D"

RAW_FLAG = 0x01
ENHANCED_FLAG = 0x04
RAW_AND_NATIVE_FLAG = 0x10

RAW_ENHANCED = RAW_FLAG | ENHANCED_FLAG          # 0x05
RAW_ENHANCED_DUAL = RAW_ENHANCED | RAW_AND_NATIVE_FLAG  # 0x15

ORIGIN_LOWER_LEFT = 0x01
ORIGIN_LOWER_RIGHT = 0x02
ORIGIN_UPPER_LEFT = 0x03
ORIGIN_UPPER_RIGHT = 0x04


class DeviceNotReady(RuntimeError):
    """Expected transient state: matching K400 exists but is asleep/offline."""


@dataclass
class Touch:
    finger_id: int
    contact_type: int
    contact_status: int
    x: int
    y: int
    z: int
    area: int


@dataclass
class RawFrame:
    timestamp: int
    finger_count: int
    end_of_frame: bool
    spurious: bool
    button: bool
    touch1: Touch
    touch2: Touch


@dataclass
class TouchpadInfo:
    x_size: int
    y_size: int
    z_range: int
    area_range: int
    timestamp_units: int
    max_fingers: int
    origin: int
    pen_support: bool
    mapping_version: int
    dpi: int


@dataclass
class TimedMotion:
    host_ns: int
    dx: float
    dy: float


def parse_touch(data: bytes, offset: int) -> Touch:
    x_high = data[offset]
    y_high = data[offset + 2]
    return Touch(
        finger_id=data[offset + 6] >> 4,
        contact_type=x_high >> 6,
        contact_status=y_high >> 6,
        x=((x_high & 0x3F) << 8) | data[offset + 1],
        y=((y_high & 0x3F) << 8) | data[offset + 3],
        z=data[offset + 4],
        area=data[offset + 5],
    )


def parse_frame(data: bytes) -> Optional[RawFrame]:
    if len(data) < 16:
        return None
    return RawFrame(
        timestamp=int.from_bytes(data[0:2], "big"),
        finger_count=data[15] & 0x0F,
        end_of_frame=bool(data[8] & 0x01),
        spurious=bool(data[8] & 0x02),
        button=bool(data[8] & 0x04),
        touch1=parse_touch(data, 2),
        touch2=parse_touch(data, 9),
    )


def parse_info(data: bytes) -> TouchpadInfo:
    if not data or len(data) < 15:
        raise RuntimeError(f"Short/empty TOUCHPAD_RAW_XY GetTouchpadInfo response: {data!r}")
    return TouchpadInfo(
        x_size=int.from_bytes(data[0:2], "big"),
        y_size=int.from_bytes(data[2:4], "big"),
        z_range=data[4],
        area_range=data[5],
        timestamp_units=data[6],
        max_fingers=data[7],
        origin=data[8],
        pen_support=bool(data[9]),
        mapping_version=data[12],
        dpi=int.from_bytes(data[13:15], "big"),
    )


def active_touches(frame: RawFrame) -> list[Touch]:
    touches = []
    for touch in (frame.touch1, frame.touch2):
        if touch.finger_id != 0 and touch.contact_status != 0:
            touches.append(touch)
    if touches:
        return touches

    # Defensive fallback in case this firmware represents contact status
    # differently while still supplying finger IDs.
    return [touch for touch in (frame.touch1, frame.touch2) if touch.finger_id != 0]


def active_single_touch(frame: RawFrame) -> Optional[Touch]:
    if frame.finger_count != 1:
        return None

    for touch in (frame.touch1, frame.touch2):
        if touch.finger_id != 0 and touch.contact_status != 0:
            return touch

    # Defensive fallback for unusual firmware status encoding.
    for touch in (frame.touch1, frame.touch2):
        if touch.finger_id != 0:
            return touch
    return None


class StartedEventsListener(listener.EventsListener):
    def __init__(self, receiver, callback):
        self.started_event = threading.Event()
        super().__init__(receiver, callback)

    def has_started(self):
        self.started_event.set()


class RawPointerEngine:
    """Pointer, tap, and scroll processing for HID++ 0x6100 raw frames.

    Pointer motion uses a base linear gain.  At very low raw velocity an
    optional precision multiplier reduces that gain, then smoothly returns to
    exactly 1.0 above precision_transition.  KDE/libinput can therefore remain
    Flat/neutral.

    Short grace windows around one-finger <-> two-finger transitions prevent
    asynchronous finger landing/lifting from being misinterpreted as pointer
    motion.  During those windows the pointer anchor follows the remaining
    finger but no REL_X/REL_Y is emitted.
    """

    def __init__(
        self,
        gain_x: float,
        gain_y: float,
        invert_x: bool,
        invert_y: bool,
        max_raw_jump: int,
        ui: Optional[UInput],
        print_raw: bool,
        timestamp_unit_ms: float,
        precision_gain: float,
        precision_speed: float,
        precision_transition: float,
        precision_filter_ms: float,
        tap_enabled: bool,
        two_finger_tap_enabled: bool,
        tap_max_ms: float,
        tap_move_units: float,
        scroll_enabled: bool,
        scroll_start_units: float,
        scroll_units_per_step: float,
        scroll_invert: bool,
        horizontal_scroll: bool,
        hscroll_units_per_step: float,
        hscroll_invert: bool,
        multitouch_entry_grace_ms: float,
        multitouch_exit_grace_ms: float,
        double_tap_window_ms: float,
        double_tap_move_units: float,
        tap_drag_window_ms: float,
        tap_drag_move_units: float,
        scroll_axis_lock: bool,
        scroll_axis_lock_ratio: float,
        kinetic_scroll: bool,
        kinetic_history_ms: float,
        kinetic_start_velocity: float,
        kinetic_stop_velocity: float,
        kinetic_decay_ms: float,
        kinetic_max_ms: float,
    ):
        self.gain_x = gain_x
        self.gain_y = gain_y
        self.x_sign = -1.0 if invert_x else 1.0
        self.y_sign = -1.0 if invert_y else 1.0
        self.max_raw_jump = max_raw_jump
        self.ui = ui
        self.print_raw = print_raw
        self.timestamp_unit_ms = timestamp_unit_ms

        self.precision_gain = precision_gain
        self.precision_speed = precision_speed
        self.precision_transition = precision_transition
        self.precision_filter_ms = precision_filter_ms

        self.tap_enabled = tap_enabled
        self.two_finger_tap_enabled = two_finger_tap_enabled
        self.tap_max_ms = tap_max_ms
        self.tap_move_units = tap_move_units

        self.scroll_enabled = scroll_enabled
        self.scroll_start_units = scroll_start_units
        self.scroll_units_per_step = scroll_units_per_step
        self.scroll_invert = scroll_invert
        self.horizontal_scroll = horizontal_scroll
        self.hscroll_units_per_step = hscroll_units_per_step
        self.hscroll_invert = hscroll_invert

        self.multitouch_entry_grace_ms = multitouch_entry_grace_ms
        self.multitouch_exit_grace_ms = multitouch_exit_grace_ms

        # Tap-generated single clicks can be deferred briefly so that a
        # qualifying second tap becomes a clean double-click pair rather than
        # first triggering an application's single-click action.
        self.double_tap_window_ms = double_tap_window_ms
        self.double_tap_move_units = double_tap_move_units

        # Tap-and-drag is intentionally independent of the short double-tap
        # deadline.  A completed tap remains eligible as a drag source for a
        # somewhat longer interval, without delaying the ordinary click.
        self.tap_drag_window_ms = tap_drag_window_ms
        self.tap_drag_move_units = tap_drag_move_units

        # Axis locking only matters when horizontal scrolling is enabled.
        # A sufficiently dominant initial axis stays locked for the remainder
        # of that two-finger scroll gesture.
        self.scroll_axis_lock = scroll_axis_lock
        self.scroll_axis_lock_ratio = scroll_axis_lock_ratio

        # Stock K400 firmware performs post-release wheel coasting itself:
        # full wheel detents continue after both fingers leave, with event
        # intervals increasing as velocity decays.  Recreate that behavior
        # from the raw centroid velocity.
        self.kinetic_scroll = kinetic_scroll
        self.kinetic_history_ms = kinetic_history_ms
        self.kinetic_start_velocity = kinetic_start_velocity
        self.kinetic_stop_velocity = kinetic_stop_velocity
        self.kinetic_decay_ms = kinetic_decay_ms
        self.kinetic_max_ms = kinetic_max_ms
        self._scroll_velocity_history = deque()
        self._kinetic_lock = threading.Lock()
        self._kinetic_cancel: Optional[threading.Event] = None
        self._kinetic_thread: Optional[threading.Thread] = None

        self._tap_lock = threading.Lock()
        self._emit_lock = threading.Lock()
        self._pending_single_timer: Optional[threading.Timer] = None
        self._pending_single_deadline_ns = 0
        self._pending_single_x = 0
        self._pending_single_y = 0

        # Set when a second finger-down has consumed/cancelled the pending
        # first-click timer and is being tested as the second half of a
        # double tap.  If that contact turns into a drag/multitouch gesture,
        # the held first click is emitted immediately rather than lost.
        self._double_second_active = False

        # Tap-and-drag state. A valid one-finger tap arms a short independent
        # drag window. A nearby subsequent one-finger contact that begins
        # moving becomes BTN_LEFT-down pointer motion.
        self._drag_source_deadline_ns = 0
        self._drag_source_x = 0
        self._drag_source_y = 0
        self._tap_drag_candidate = False
        self._tap_drag_active = False

        # One-finger pointer state.
        self.prev_x: Optional[int] = None
        self.prev_y: Optional[int] = None
        self.prev_finger_id: Optional[int] = None
        self.prev_hid_ts: Optional[int] = None
        self.frac_x = 0.0
        self.frac_y = 0.0
        self.filtered_speed: Optional[float] = None

        # One-finger tap candidate.
        self.single_active = False
        self.single_start_ns = 0
        self.single_start_x = 0
        self.single_start_y = 0
        self.single_max_move = 0.0
        self.single_tap_candidate = False
        # Fresh one-finger contacts wait briefly before pointer motion is
        # allowed, giving a slightly delayed second finger time to arrive.
        self.single_motion_grace_until_ns = 0

        # Multi-touch state.
        self.multitouch_session = False
        self.two_active = False
        self.two_start_ns = 0
        self.two_start_centroid: Optional[tuple[float, float]] = None
        self.two_last_centroid: Optional[tuple[float, float]] = None
        self.two_start_positions: dict[int, tuple[int, int]] = {}
        self.two_max_finger_move = 0.0
        self.two_tap_candidate = False
        self.two_scrolling = False
        self.scroll_axis: Optional[str] = None
        self.scroll_frac = 0.0
        self.hscroll_frac = 0.0

        # When a scrolling gesture temporarily falls from two fingers to one,
        # wait briefly before handing the remaining finger to pointer motion.
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0

    def _reset_pointer(self, touch: Optional[Touch] = None, hid_ts: Optional[int] = None,
                       reset_fraction: bool = True):
        if touch is None:
            self.prev_x = None
            self.prev_y = None
            self.prev_finger_id = None
            self.prev_hid_ts = None
            self.filtered_speed = None
        else:
            self.prev_x = touch.x
            self.prev_y = touch.y
            self.prev_finger_id = touch.finger_id
            self.prev_hid_ts = hid_ts

        if reset_fraction:
            self.frac_x = 0.0
            self.frac_y = 0.0

    def reset_runtime(self):
        """Cancel all in-progress pointer/gesture state without emitting events.

        Used after a device reconnect/power-cycle so stale absolute coordinates
        can never turn into a large relative jump.
        """
        self._cancel_kinetic_scroll()
        self._cancel_pending_single_click()
        self._end_tap_drag("runtime reset")
        self._double_second_active = False
        self._drag_source_deadline_ns = 0
        self._tap_drag_candidate = False
        self._reset_pointer()
        self.single_active = False
        self.single_tap_candidate = False
        self.single_start_ns = 0
        self.single_max_move = 0.0
        self.single_motion_grace_until_ns = 0

        self.multitouch_session = False
        self.two_active = False
        self.two_tap_candidate = False
        self.two_scrolling = False
        self.scroll_axis = None
        self.two_start_ns = 0
        self.two_start_centroid = None
        self.two_last_centroid = None
        self.two_start_positions = {}
        self.two_max_finger_move = 0.0
        self.scroll_frac = 0.0
        self.hscroll_frac = 0.0
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0
        self._scroll_velocity_history.clear()

    def _precision_multiplier(self, raw_speed: float, dt_ms: float) -> float:
        if self.precision_gain >= 0.999999:
            return 1.0

        if self.precision_filter_ms <= 0.0:
            self.filtered_speed = raw_speed
        elif self.filtered_speed is None:
            self.filtered_speed = raw_speed
        else:
            alpha = 1.0 - math.exp(-max(dt_ms, 0.001) / self.precision_filter_ms)
            self.filtered_speed += alpha * (raw_speed - self.filtered_speed)

        speed = raw_speed if self.filtered_speed is None else self.filtered_speed

        if speed <= self.precision_speed:
            return self.precision_gain
        if speed >= self.precision_transition:
            return 1.0

        # Smoothstep keeps both ends of the transition slope-free.
        t = (speed - self.precision_speed) / (
            self.precision_transition - self.precision_speed
        )
        t = max(0.0, min(1.0, t))
        smooth = t * t * (3.0 - 2.0 * t)
        return self.precision_gain + (1.0 - self.precision_gain) * smooth

    def _emit_click(self, code: int, label: str):
        if self.ui is None:
            LOG.debug("%s recognized (no uinput; not emitted)", label)
            return

        # A timer may emit a deferred tap click while the HID++ listener thread
        # is simultaneously emitting motion/scroll. Serialize complete uinput
        # click sequences so their EV_KEY/SYN pairs cannot interleave.
        with self._emit_lock:
            self.ui.write(ecodes.EV_KEY, code, 1)
            self.ui.syn()
            self.ui.write(ecodes.EV_KEY, code, 0)
            self.ui.syn()
        LOG.debug("%s emitted", label)

    def _emit_double_left_click(self):
        if self.ui is None:
            LOG.debug("one-finger double tap recognized (no uinput; not emitted)")
            return

        # Two complete clicks back-to-back are well inside any desktop/app
        # double-click threshold.  We intentionally do not emit the first click
        # until the second tap has qualified, preventing an application from
        # acting on a temporary single click between taps.
        with self._emit_lock:
            for _ in range(2):
                self.ui.write(ecodes.EV_KEY, ecodes.BTN_LEFT, 1)
                self.ui.syn()
                self.ui.write(ecodes.EV_KEY, ecodes.BTN_LEFT, 0)
                self.ui.syn()
        LOG.debug("one-finger double tap -> BTN_LEFT double-click emitted")

    def _clear_pending_single_locked(self):
        timer = self._pending_single_timer
        self._pending_single_timer = None
        self._pending_single_deadline_ns = 0
        self._pending_single_x = 0
        self._pending_single_y = 0
        return timer

    def _cancel_pending_single_click(self):
        with self._tap_lock:
            timer = self._clear_pending_single_locked()
        if timer is not None:
            timer.cancel()

    def _pending_single_timeout(self):
        # Timer callback: consume the pending state atomically, then emit after
        # releasing the tap lock.
        with self._tap_lock:
            if self._pending_single_timer is None:
                return
            self._clear_pending_single_locked()

        self._emit_click(
            ecodes.BTN_LEFT,
            "deferred one-finger tap -> BTN_LEFT",
        )

    def _begin_possible_second_tap(self, host_ns: int, x: int, y: int):
        """Consume a pending first tap as soon as a qualifying 2nd touch begins.

        The double-tap deadline is measured from the FIRST finger-down, so the
        first click may already have only a small amount of timer time left
        when this second contact appears.  Cancelling here (rather than waiting
        for second finger-up) prevents that timer from firing during the second
        tap.

        Returns True when this contact has taken ownership of the held first
        click as a possible double tap.
        """
        if self.double_tap_window_ms <= 0:
            self._double_second_active = False
            return False

        timer_to_cancel = None
        claimed = False

        with self._tap_lock:
            if self._pending_single_timer is not None:
                within_time = host_ns <= self._pending_single_deadline_ns
                within_space = (
                    math.hypot(
                        x - self._pending_single_x,
                        y - self._pending_single_y,
                    )
                    <= self.double_tap_move_units
                )

                if within_time and within_space:
                    timer_to_cancel = self._clear_pending_single_locked()
                    self._double_second_active = True
                    claimed = True

        if timer_to_cancel is not None:
            timer_to_cancel.cancel()

        if claimed:
            LOG.debug(
                "second finger-down claimed pending tap as possible double tap"
            )
        return claimed

    def _abort_possible_second_tap(self, reason: str):
        """A claimed second tap stopped being a tap; release the first click."""
        if not self._double_second_active:
            return

        self._double_second_active = False
        LOG.debug("double-tap candidate aborted (%s); emitting held first click", reason)
        self._emit_click(
            ecodes.BTN_LEFT,
            "held first tap after failed double-tap candidate -> BTN_LEFT",
        )

    def _handle_first_single_tap(
        self,
        tap_start_ns: int,
        release_ns: int,
        x: int,
        y: int,
    ):
        """Handle a valid first tap using a finger-down-based deadline."""
        if self.double_tap_window_ms <= 0:
            self._emit_click(ecodes.BTN_LEFT, "one-finger tap -> BTN_LEFT")
            return

        deadline_ns = (
            tap_start_ns + int(self.double_tap_window_ms * 1_000_000.0)
        )
        remaining_ns = deadline_ns - release_ns

        # If the tap itself already consumed the whole double-tap window there
        # is nothing left to wait for after release.
        if remaining_ns <= 0:
            self._emit_click(
                ecodes.BTN_LEFT,
                "one-finger tap -> BTN_LEFT (double-tap window already expired)",
            )
            return

        timer = threading.Timer(
            remaining_ns / 1_000_000_000.0,
            self._pending_single_timeout,
        )
        timer.daemon = True

        previous_timer = None
        with self._tap_lock:
            # Normally there cannot be another pending tap here: a nearby
            # second finger-down would have claimed it.  If an unrelated old
            # tap is still pending, release it now and replace it cleanly.
            if self._pending_single_timer is not None:
                previous_timer = self._clear_pending_single_locked()

            self._pending_single_deadline_ns = deadline_ns
            self._pending_single_x = x
            self._pending_single_y = y
            self._pending_single_timer = timer

        if previous_timer is not None:
            previous_timer.cancel()
            self._emit_click(
                ecodes.BTN_LEFT,
                "previous deferred one-finger tap -> BTN_LEFT",
            )

        timer.start()
        LOG.debug(
            "one-finger tap: total double-tap window %.1f ms; %.1f ms remains after release",
            self.double_tap_window_ms,
            remaining_ns / 1_000_000.0,
        )

    def _emit_button_state(self, code: int, pressed: bool, label: str):
        if self.ui is None:
            LOG.debug("%s recognized (no uinput; not emitted)", label)
            return

        with self._emit_lock:
            self.ui.write(ecodes.EV_KEY, code, 1 if pressed else 0)
            self.ui.syn()
        LOG.debug("%s emitted", label)

    def _flush_pending_single_click(self, label: str) -> bool:
        """Emit an armed first tap immediately, if its timer is still pending."""
        with self._tap_lock:
            timer = self._clear_pending_single_locked()

        if timer is None:
            return False

        timer.cancel()
        self._emit_click(ecodes.BTN_LEFT, label)
        return True

    def _remember_tap_for_drag(self, release_ns: int, x: int, y: int):
        if self.tap_drag_window_ms <= 0:
            self._drag_source_deadline_ns = 0
            return

        self._drag_source_deadline_ns = (
            release_ns + int(self.tap_drag_window_ms * 1_000_000.0)
        )
        self._drag_source_x = x
        self._drag_source_y = y

    def _begin_possible_tap_drag(self, host_ns: int, x: int, y: int) -> bool:
        self._tap_drag_candidate = False

        if self.tap_drag_window_ms <= 0:
            return False
        if not self._drag_source_deadline_ns:
            return False
        if host_ns > self._drag_source_deadline_ns:
            self._drag_source_deadline_ns = 0
            return False

        distance = math.hypot(
            x - self._drag_source_x,
            y - self._drag_source_y,
        )
        if distance > self.tap_drag_move_units:
            return False

        self._tap_drag_candidate = True
        LOG.debug(
            "new touch is tap-drag candidate: dt remaining=%.1f ms distance=%.1f units",
            (self._drag_source_deadline_ns - host_ns) / 1_000_000.0,
            distance,
        )
        return True

    def _start_tap_drag(self):
        """Convert a recent tap + moving second contact into a held left drag."""
        if self._tap_drag_active or not self._tap_drag_candidate:
            return False

        # If the second touch arrived inside the double-tap window, v4.7+
        # cancelled the first tap's delayed click while determining whether a
        # double-click was intended.  A drag proves it was not a double-click:
        # emit that first click, then start a new held press for dragging.
        if self._double_second_active:
            self._double_second_active = False
            self._emit_click(
                ecodes.BTN_LEFT,
                "first tap before tap-and-drag -> BTN_LEFT",
            )
        else:
            # The first tap may still have a deferred-click timer if the second
            # contact is a little farther away than the double-tap spatial
            # threshold but still close enough for tap-and-drag.
            self._flush_pending_single_click(
                "first tap before tap-and-drag -> BTN_LEFT"
            )

        self._tap_drag_candidate = False
        self._drag_source_deadline_ns = 0
        self.single_tap_candidate = False
        self.single_motion_grace_until_ns = 0
        self._tap_drag_active = True

        self._emit_button_state(
            ecodes.BTN_LEFT,
            True,
            "tap-and-drag -> BTN_LEFT down",
        )
        return True

    def _end_tap_drag(self, reason: str):
        if not self._tap_drag_active:
            return

        self._tap_drag_active = False
        self._tap_drag_candidate = False
        self._emit_button_state(
            ecodes.BTN_LEFT,
            False,
            f"tap-and-drag end ({reason}) -> BTN_LEFT up",
        )

    def _emit_scroll(self, wheel: int = 0, hwheel: int = 0):
        """Emit stock-K400-style legacy + high-resolution wheel pairs.

        The physical K400 emits, for each full vertical detent:
            REL_WHEEL          +/-1
            REL_WHEEL_HI_RES   +/-120
        in the same SYN_REPORT.  Mirror that exactly.  Horizontal scrolling
        follows the corresponding REL_HWHEEL / REL_HWHEEL_HI_RES convention.
        """
        if self.ui is None or (wheel == 0 and hwheel == 0):
            return

        with self._emit_lock:
            if wheel:
                self.ui.write(ecodes.EV_REL, ecodes.REL_WHEEL, wheel)
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_WHEEL_HI_RES,
                    wheel * 120,
                )
            if hwheel:
                self.ui.write(ecodes.EV_REL, ecodes.REL_HWHEEL, hwheel)
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_HWHEEL_HI_RES,
                    hwheel * 120,
                )
            self.ui.syn()

    def _record_scroll_velocity(self, host_ns: int, wheel_raw_delta: float):
        """Record signed raw scroll displacement for release-velocity estimate."""
        self._scroll_velocity_history.append((host_ns, wheel_raw_delta))

        cutoff_ns = host_ns - int(self.kinetic_history_ms * 1_000_000.0)
        while (
            len(self._scroll_velocity_history) > 1
            and self._scroll_velocity_history[0][0] < cutoff_ns
        ):
            self._scroll_velocity_history.popleft()

    def _estimate_release_scroll_velocity(self, host_ns: int) -> float:
        """Return signed raw-units/ms release velocity over recent history."""
        if len(self._scroll_velocity_history) < 2:
            return 0.0

        cutoff_ns = host_ns - int(self.kinetic_history_ms * 1_000_000.0)
        samples = [
            (ts, delta)
            for ts, delta in self._scroll_velocity_history
            if ts >= cutoff_ns
        ]
        if len(samples) < 2:
            return 0.0

        elapsed_ms = (samples[-1][0] - samples[0][0]) / 1_000_000.0
        if elapsed_ms <= 0.0:
            return 0.0

        # The first delta belongs partly to the interval preceding its sample;
        # omit it so numerator and denominator describe the same span.
        displacement = sum(delta for _, delta in samples[1:])
        return displacement / elapsed_ms

    def _cancel_kinetic_scroll(self):
        with self._kinetic_lock:
            cancel = self._kinetic_cancel
            self._kinetic_cancel = None
            self._kinetic_thread = None
        if cancel is not None:
            cancel.set()

    def _start_kinetic_scroll(self, release_velocity: float):
        """Continue full wheel detents with stock-like decelerating cadence.

        Velocity is expressed in the same signed raw-units/ms used by active
        two-finger scrolling.  The model is:

            v(t) = v0 * exp(-t / tau)

        but, like the stock K400 firmware, output magnitude stays one complete
        wheel detent (+/-1 and +/-120 hi-res).  Deceleration therefore appears
        as progressively larger time intervals between events.
        """
        self._cancel_kinetic_scroll()

        if not self.kinetic_scroll:
            return

        speed = abs(release_velocity)
        if speed < self.kinetic_start_velocity:
            LOG.debug(
                "kinetic scroll not started: release velocity %.3f < %.3f units/ms",
                speed,
                self.kinetic_start_velocity,
            )
            return

        direction = 1 if release_velocity > 0 else -1
        cancel = threading.Event()

        def worker():
            v = speed
            tau = self.kinetic_decay_ms
            step_raw = self.scroll_units_per_step
            start_time = time.monotonic()
            events = 0

            LOG.debug(
                "kinetic scroll started: release=%.3f units/ms direction=%+d "
                "tau=%.0f ms",
                speed,
                direction,
                tau,
            )

            try:
                while not cancel.is_set():
                    elapsed_total_ms = (time.monotonic() - start_time) * 1000.0
                    if elapsed_total_ms >= self.kinetic_max_ms:
                        break
                    if v <= self.kinetic_stop_velocity:
                        break

                    # Under exponential decay, the total future displacement
                    # available at current velocity is v*tau.  If that cannot
                    # reach another full detent, the stock-style coast is done.
                    available = v * tau
                    if available <= step_raw:
                        break

                    # Solve:
                    #   step = v*tau*(1-exp(-dt/tau))
                    # for the time until the next full detent.
                    ratio = 1.0 - (step_raw / available)
                    if ratio <= 0.0:
                        break
                    wait_ms = -tau * math.log(ratio)

                    remaining_ms = self.kinetic_max_ms - elapsed_total_ms
                    wait_ms = min(wait_ms, remaining_ms)
                    if wait_ms <= 0.0:
                        break

                    if cancel.wait(wait_ms / 1000.0):
                        break

                    actual_dt_ms = wait_ms
                    v *= math.exp(-actual_dt_ms / tau)
                    if v <= self.kinetic_stop_velocity:
                        break

                    self._emit_scroll(direction, 0)
                    events += 1

                    LOG.debug(
                        "kinetic scroll detent #%d: interval=%.1f ms velocity=%.3f",
                        events,
                        wait_ms,
                        v,
                    )
            finally:
                LOG.debug(
                    "kinetic scroll ended after %d detent(s), %.0f ms",
                    events,
                    (time.monotonic() - start_time) * 1000.0,
                )
                with self._kinetic_lock:
                    if self._kinetic_cancel is cancel:
                        self._kinetic_cancel = None
                        self._kinetic_thread = None

        thread = threading.Thread(
            target=worker,
            name="k400-kinetic-scroll",
            daemon=True,
        )
        with self._kinetic_lock:
            self._kinetic_cancel = cancel
            self._kinetic_thread = thread
        thread.start()

    def _start_single(self, touch: Touch, host_ns: int, hid_ts: int):
        self._cancel_kinetic_scroll()
        self._begin_possible_second_tap(host_ns, touch.x, touch.y)
        self._begin_possible_tap_drag(host_ns, touch.x, touch.y)

        self.single_active = True
        self.single_start_ns = host_ns
        self.single_start_x = touch.x
        self.single_start_y = touch.y
        self.single_max_move = 0.0
        self.single_tap_candidate = self.tap_enabled
        self.single_motion_grace_until_ns = (
            host_ns + int(self.multitouch_entry_grace_ms * 1_000_000.0)
        )
        self._reset_pointer(touch, hid_ts)

    def _update_single_tap(self, touch: Touch) -> bool:
        """Update tap slop; return True when this frame starts tap-and-drag."""
        if self._tap_drag_active:
            return False

        dist = math.hypot(
            touch.x - self.single_start_x,
            touch.y - self.single_start_y,
        )
        self.single_max_move = max(self.single_max_move, dist)

        if self.single_max_move <= self.tap_move_units:
            return False

        # Movement that would normally invalidate the second tap instead
        # becomes a drag when this touch followed a recent nearby tap.
        if self._tap_drag_candidate and self._start_tap_drag():
            return True

        self.single_tap_candidate = False
        self._tap_drag_candidate = False
        self._abort_possible_second_tap(
            "second contact moved beyond tap threshold"
        )
        return False

    def _finish_single(self, host_ns: int):
        if self._tap_drag_active:
            self._end_tap_drag("finger released")
            self.single_active = False
            self.single_tap_candidate = False
            self.single_motion_grace_until_ns = 0
            self._drag_source_deadline_ns = 0
            return

        valid_tap = False
        if self.single_active and self.single_tap_candidate:
            duration_ms = (host_ns - self.single_start_ns) / 1_000_000.0
            valid_tap = duration_ms <= self.tap_max_ms

        if self._double_second_active:
            if valid_tap:
                self._double_second_active = False
                self._tap_drag_candidate = False
                self._drag_source_deadline_ns = 0
                self._emit_double_left_click()
            else:
                self._tap_drag_candidate = False
                self._abort_possible_second_tap(
                    "second contact released without qualifying as a tap"
                )
        elif valid_tap:
            # Arm tap-and-drag independently of the shorter double-click
            # handling. This does not delay the normal synthetic click.
            self._remember_tap_for_drag(
                host_ns,
                self.single_start_x,
                self.single_start_y,
            )
            self._handle_first_single_tap(
                self.single_start_ns,
                host_ns,
                self.single_start_x,
                self.single_start_y,
            )
        else:
            self._tap_drag_candidate = False

        self.single_active = False
        self.single_tap_candidate = False
        self.single_motion_grace_until_ns = 0

    def _start_two(self, touches: list[Touch], host_ns: int):
        self._cancel_kinetic_scroll()
        self._end_tap_drag("multi-touch began")
        self._tap_drag_candidate = False
        self._scroll_velocity_history.clear()
        touches = touches[:2]
        cx = sum(t.x for t in touches) / 2.0
        cy = sum(t.y for t in touches) / 2.0

        self.multitouch_session = True
        self.two_active = True
        self.two_start_ns = host_ns
        self.two_start_centroid = (cx, cy)
        self.two_last_centroid = (cx, cy)
        self.two_start_positions = {t.finger_id: (t.x, t.y) for t in touches}
        self.two_max_finger_move = 0.0
        self.two_tap_candidate = self.two_finger_tap_enabled
        self.two_scrolling = False
        self.scroll_axis = None
        self.scroll_frac = 0.0
        self.hscroll_frac = 0.0
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0

        # A two-finger gesture cancels the single-finger tap candidate.
        self.single_tap_candidate = False
        self._reset_pointer()

    def _resume_two_after_exit_gap(self, touches: list[Touch]):
        """Resume an existing scroll after a brief 2->1->2 contact gap.

        Re-anchor the centroid so the temporary missing finger cannot turn
        into a large wheel delta when the second contact returns.
        """
        touches = touches[:2]
        cx = sum(t.x for t in touches) / 2.0
        cy = sum(t.y for t in touches) / 2.0
        self.two_last_centroid = (cx, cy)
        self._scroll_velocity_history.clear()
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0
        LOG.debug("two-finger scroll resumed inside exit grace; centroid re-anchored")

    def _process_two(self, touches: list[Touch], host_ns: int):
        if len(touches) < 2:
            return

        touches = touches[:2]
        if not self.two_active:
            self._start_two(touches, host_ns)
            return

        ids = {t.finger_id for t in touches}
        if ids != set(self.two_start_positions):
            self.two_tap_candidate = False

        for t in touches:
            start = self.two_start_positions.get(t.finger_id)
            if start is not None:
                self.two_max_finger_move = max(
                    self.two_max_finger_move,
                    math.hypot(t.x - start[0], t.y - start[1]),
                )

        if self.two_max_finger_move > self.tap_move_units:
            self.two_tap_candidate = False

        cx = sum(t.x for t in touches) / 2.0
        cy = sum(t.y for t in touches) / 2.0

        assert self.two_start_centroid is not None
        assert self.two_last_centroid is not None

        start_dx = (cx - self.two_start_centroid[0]) * self.x_sign
        start_dy = (cy - self.two_start_centroid[1]) * self.y_sign

        dx = (cx - self.two_last_centroid[0]) * self.x_sign
        dy = (cy - self.two_last_centroid[1]) * self.y_sign
        self.two_last_centroid = (cx, cy)

        if not self.two_scrolling:
            if (
                abs(start_dy) >= self.scroll_start_units
                or (
                    self.horizontal_scroll
                    and abs(start_dx) >= self.scroll_start_units
                )
            ):
                self.two_scrolling = True
                self.two_tap_candidate = False

                if not self.horizontal_scroll:
                    self.scroll_axis = "vertical"
                elif not self.scroll_axis_lock:
                    self.scroll_axis = "free"
                else:
                    ax = abs(start_dx)
                    ay = abs(start_dy)
                    if ay >= ax * self.scroll_axis_lock_ratio:
                        self.scroll_axis = "vertical"
                    elif ax >= ay * self.scroll_axis_lock_ratio:
                        self.scroll_axis = "horizontal"
                    else:
                        self.scroll_axis = "free"

                LOG.debug(
                    "two-finger scroll started (axis=%s)",
                    self.scroll_axis,
                )
            else:
                return

        if not self.scroll_enabled:
            return

        if self.scroll_axis == "vertical":
            dx = 0.0
        elif self.scroll_axis == "horizontal":
            dy = 0.0

        # Standard REL_WHEEL: positive means wheel-up.  With the transformed
        # screen-coordinate dy, negate by default so finger-down maps to
        # wheel-down.  --scroll-invert flips this.
        wheel_delta = -dy
        if self.scroll_invert:
            wheel_delta = -wheel_delta

        if wheel_delta:
            self._record_scroll_velocity(host_ns, wheel_delta)

        self.scroll_frac += wheel_delta / self.scroll_units_per_step
        wheel_out = int(self.scroll_frac)
        self.scroll_frac -= wheel_out

        hwheel_out = 0
        if self.horizontal_scroll:
            hdelta = dx
            if self.hscroll_invert:
                hdelta = -hdelta
            self.hscroll_frac += hdelta / self.hscroll_units_per_step
            hwheel_out = int(self.hscroll_frac)
            self.hscroll_frac -= hwheel_out

        if wheel_out or hwheel_out:
            self._emit_scroll(wheel_out, hwheel_out)

    def _finish_two(self, host_ns: int, allow_kinetic: bool = True):
        release_velocity = 0.0

        if self.two_active and self.two_tap_candidate and not self.two_scrolling:
            duration_ms = (host_ns - self.two_start_ns) / 1_000_000.0
            if duration_ms <= self.tap_max_ms:
                self._emit_click(ecodes.BTN_RIGHT, "two-finger tap -> BTN_RIGHT")

        if self.two_scrolling:
            if allow_kinetic:
                release_velocity = self._estimate_release_scroll_velocity(host_ns)
            LOG.debug(
                "two-finger scroll ended%s%s",
                (
                    "; release velocity="
                    if allow_kinetic
                    else "; kinetic suppressed for one-finger handoff"
                ),
                (
                    f"{release_velocity:.3f} raw units/ms"
                    if allow_kinetic
                    else ""
                ),
            )

        self.two_active = False
        self.two_tap_candidate = False
        was_scrolling = self.two_scrolling
        self.two_scrolling = False
        self.scroll_axis = None
        self.two_start_centroid = None
        self.two_last_centroid = None
        self.two_start_positions = {}
        self.scroll_frac = 0.0
        self.hscroll_frac = 0.0
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0
        self._scroll_velocity_history.clear()

        if was_scrolling and allow_kinetic:
            self._start_kinetic_scroll(release_velocity)

    def _process_pointer_motion(self, touch: Touch, frame: RawFrame):
        if (
            self.prev_x is None
            or self.prev_y is None
            or self.prev_finger_id != touch.finger_id
            or self.prev_hid_ts is None
        ):
            self._reset_pointer(touch, frame.timestamp)
            return

        sensor_dx = touch.x - self.prev_x
        sensor_dy = touch.y - self.prev_y
        hid_dt = (frame.timestamp - self.prev_hid_ts) & 0xFFFF
        dt_ms = max(hid_dt * self.timestamp_unit_ms, 0.001)

        self.prev_x = touch.x
        self.prev_y = touch.y
        self.prev_finger_id = touch.finger_id
        self.prev_hid_ts = frame.timestamp

        if self.max_raw_jump > 0 and (
            abs(sensor_dx) > self.max_raw_jump or abs(sensor_dy) > self.max_raw_jump
        ):
            LOG.warning(
                "Ignoring/re-anchoring implausible raw jump dx=%d dy=%d",
                sensor_dx, sensor_dy,
            )
            self.frac_x = 0.0
            self.frac_y = 0.0
            self.filtered_speed = None
            self.single_tap_candidate = False
            return

        raw_speed = math.hypot(sensor_dx, sensor_dy) / dt_ms
        precision_mult = self._precision_multiplier(raw_speed, dt_ms)

        dx = sensor_dx * self.x_sign
        dy = sensor_dy * self.y_sign

        self.frac_x += dx * self.gain_x * precision_mult
        self.frac_y += dy * self.gain_y * precision_mult

        out_x = int(self.frac_x)
        out_y = int(self.frac_y)
        self.frac_x -= out_x
        self.frac_y -= out_y

        if out_x == 0 and out_y == 0:
            return

        if self.ui is not None:
            with self._emit_lock:
                if out_x:
                    self.ui.write(ecodes.EV_REL, ecodes.REL_X, out_x)
                if out_y:
                    self.ui.write(ecodes.EV_REL, ecodes.REL_Y, out_y)
                self.ui.syn()

        LOG.debug(
            "raw dx=%d dy=%d dt=%.3f speed=%.4f filtered=%.4f precision=%.4f "
            "-> rel dx=%d dy=%d rem=(%.4f,%.4f)",
            sensor_dx, sensor_dy, dt_ms, raw_speed,
            self.filtered_speed if self.filtered_speed is not None else raw_speed,
            precision_mult, out_x, out_y, self.frac_x, self.frac_y,
        )

    def process(self, frame: RawFrame, host_ns: int):
        if self.print_raw:
            LOG.debug(
                "raw ts=%d count=%d eof=%s spurious=%s "
                "f1(id=%d st=%d x=%d y=%d) f2(id=%d st=%d x=%d y=%d)",
                frame.timestamp,
                frame.finger_count,
                frame.end_of_frame,
                frame.spurious,
                frame.touch1.finger_id,
                frame.touch1.contact_status,
                frame.touch1.x,
                frame.touch1.y,
                frame.touch2.finger_id,
                frame.touch2.contact_status,
                frame.touch2.x,
                frame.touch2.y,
            )

        if frame.spurious:
            return

        touches = active_touches(frame)
        count = len(touches)

        if count > 0:
            self._cancel_kinetic_scroll()

        if count >= 2:
            self._tap_drag_candidate = False

            if self._double_second_active:
                self._abort_possible_second_tap(
                    "second contact became a multi-touch gesture"
                )

            if self.multitouch_session and self.scroll_exit_pending:
                # The second finger came back before the exit grace expired:
                # this is still the same scroll stroke. Re-anchor the current
                # centroid and continue without generating a wheel jump.
                self._resume_two_after_exit_gap(touches)

            if not self.multitouch_session:
                # A second finger means the original one-finger contact was
                # the beginning of a multi-touch gesture, not a completed tap.
                self.single_tap_candidate = False
                self.single_active = False
                self.single_motion_grace_until_ns = 0
                self._reset_pointer()

            self._process_two(touches, host_ns)
            return

        if count == 1:
            touch = touches[0]

            if self.multitouch_session:
                if self.two_scrolling:
                    # Do not immediately hand a lone remaining finger to the
                    # pointer. Fingers commonly leave a touchpad a few reports
                    # apart, and that last finger may still be moving quickly
                    # as the hand lifts. Follow it as an anchor, but suppress
                    # REL motion until the short exit grace expires.
                    if not self.scroll_exit_pending:
                        self.scroll_exit_pending = True
                        self.scroll_exit_until_ns = (
                            host_ns
                            + int(self.multitouch_exit_grace_ms * 1_000_000.0)
                        )
                        LOG.debug(
                            "two-finger scroll entered one-finger exit grace (%.1f ms)",
                            self.multitouch_exit_grace_ms,
                        )

                    self._reset_pointer(
                        touch,
                        frame.timestamp,
                        reset_fraction=True,
                    )

                    if host_ns < self.scroll_exit_until_ns:
                        return

                    # One finger has genuinely remained down past the grace
                    # period. End the scroll and hand off at the *current*
                    # coordinate. This frame emits no pointer motion; the next
                    # frame supplies the first relative delta.
                    self._finish_two(host_ns, allow_kinetic=False)
                    self.multitouch_session = False

                    self.single_active = True
                    self.single_start_ns = host_ns
                    self.single_start_x = touch.x
                    self.single_start_y = touch.y
                    self.single_max_move = 0.0
                    self.single_tap_candidate = False
                    self.single_motion_grace_until_ns = 0

                    self._reset_pointer(
                        touch,
                        frame.timestamp,
                        reset_fraction=True,
                    )
                    LOG.debug(
                        "two-finger scroll -> one-finger pointer handoff "
                        "after %.1f ms exit grace (finger id=%d x=%d y=%d)",
                        self.multitouch_exit_grace_ms,
                        touch.finger_id,
                        touch.x,
                        touch.y,
                    )
                    return

                # Non-scrolling two-finger contact dropping to one finger:
                # use the same exit grace as scrolling. This preserves a
                # normal two-finger tap when the fingers lift a few reports
                # apart, but if one finger genuinely remains down, hand it
                # cleanly back to pointer motion.
                if not self.scroll_exit_pending:
                    self.scroll_exit_pending = True
                    self.scroll_exit_until_ns = (
                        host_ns
                        + int(self.multitouch_exit_grace_ms * 1_000_000.0)
                    )
                    LOG.debug(
                        "two-finger contact entered one-finger exit grace (%.1f ms)",
                        self.multitouch_exit_grace_ms,
                    )

                self._reset_pointer(
                    touch,
                    frame.timestamp,
                    reset_fraction=True,
                )

                if host_ns < self.scroll_exit_until_ns:
                    return

                # One finger genuinely remained after the grace period, so
                # this is no longer a two-finger tap. Cancel that candidate
                # before finishing the two-finger state.
                self.two_tap_candidate = False
                self._finish_two(host_ns, allow_kinetic=False)
                self.multitouch_session = False

                self.single_active = True
                self.single_start_ns = host_ns
                self.single_start_x = touch.x
                self.single_start_y = touch.y
                self.single_max_move = 0.0
                self.single_tap_candidate = False
                self.single_motion_grace_until_ns = 0

                self._reset_pointer(
                    touch,
                    frame.timestamp,
                    reset_fraction=True,
                )

                LOG.debug(
                    "two-finger contact -> one-finger pointer handoff "
                    "after %.1f ms exit grace (finger id=%d x=%d y=%d)",
                    self.multitouch_exit_grace_ms,
                    touch.finger_id,
                    touch.x,
                    touch.y,
                )
                return

            if not self.single_active:
                self._start_single(touch, host_ns, frame.timestamp)
                return

            drag_started = self._update_single_tap(touch)
            if drag_started:
                # Start the held button at the current coordinate. The next
                # frame supplies the first drag motion, avoiding a slop-sized
                # cursor/selection jump on activation.
                self._reset_pointer(
                    touch,
                    frame.timestamp,
                    reset_fraction=True,
                )
                return

            # Fresh one-finger contacts are ambiguous for a few reports: this
            # may be the first half of an intended two-finger gesture. During
            # the entry grace we continuously re-anchor instead of accumulating
            # motion, so a delayed second finger cannot produce a cursor jump.
            if host_ns < self.single_motion_grace_until_ns:
                self._reset_pointer(
                    touch,
                    frame.timestamp,
                    reset_fraction=True,
                )
                return

            if self.single_motion_grace_until_ns:
                # Grace just expired. Re-anchor once at the current location;
                # actual pointer motion starts with the following raw frame.
                self.single_motion_grace_until_ns = 0
                self._reset_pointer(
                    touch,
                    frame.timestamp,
                    reset_fraction=True,
                )
                LOG.debug(
                    "one-finger entry grace expired; pointer motion armed"
                )
                return

            self._process_pointer_motion(touch, frame)
            return

        # Zero active fingers: finish whichever gesture was in progress.
        if self.multitouch_session:
            self._finish_two(host_ns)
            self.multitouch_session = False
        else:
            self._finish_single(host_ns)

        self.single_active = False
        self.single_motion_grace_until_ns = 0
        self.scroll_exit_pending = False
        self.scroll_exit_until_ns = 0
        self._reset_pointer()


class ComparisonRecorder:
    """
    Record two streams against the same host monotonic clock:
      * HID++ raw frames
      * standard physical evdev frames

    The event CSV is intentionally lossless enough for later re-analysis.
    The summary CSV groups both streams into fixed host-time windows, which
    avoids relying on exact callback ordering between hidraw and evdev.
    """

    EVENT_HEADER = [
        "host_ns", "source", "native_device",
        "hid_ts", "finger_count", "x", "y", "raw_dx", "raw_dy", "raw_dt_ms",
        "native_rel_x", "native_rel_y", "wheel", "hwheel",
        "keys",
    ]

    SUMMARY_HEADER = [
        "window_start_ns", "window_end_ns", "window_ms",
        "raw_dx_sum", "raw_dy_sum", "raw_path",
        "native_dx_sum", "native_dy_sum", "native_path",
        "raw_speed_units_per_ms", "native_per_raw_path_gain",
    ]

    def __init__(
        self,
        csv_path: Path,
        window_ms: float,
        invert_x: bool,
        invert_y: bool,
        timestamp_unit_ms: float,
    ):
        self.csv_path = csv_path
        self.summary_path = csv_path.with_name(csv_path.stem + ".summary.csv")
        self.window_ms = window_ms
        self.invert_x = invert_x
        self.invert_y = invert_y
        self.timestamp_unit_ms = timestamp_unit_ms

        self.lock = threading.Lock()
        self.fp = csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.fp)
        self.writer.writerow(self.EVENT_HEADER)
        self.fp.flush()

        self.raw_samples: list[TimedMotion] = []
        self.native_samples: list[TimedMotion] = []

        self.prev_x: Optional[int] = None
        self.prev_y: Optional[int] = None
        self.prev_finger_id: Optional[int] = None
        self.prev_hid_ts: Optional[int] = None

        self.native_frames = 0
        self.native_motion_frames = 0
        self.native_key_frames = 0
        self.native_scroll_frames = 0
        self.native_device_stats: dict[str, dict[str, int]] = {}

    def _raw_reanchor(self, touch: Optional[Touch], hid_ts: int):
        if touch is None:
            self.prev_x = self.prev_y = self.prev_finger_id = None
            self.prev_hid_ts = None
        else:
            self.prev_x = touch.x
            self.prev_y = touch.y
            self.prev_finger_id = touch.finger_id
            self.prev_hid_ts = hid_ts

    def record_raw(self, frame: RawFrame, host_ns: int):
        touch = active_single_touch(frame)

        raw_dx = raw_dy = 0
        raw_dt_ms = 0.0
        x = y = ""

        if touch is None or frame.spurious:
            self._raw_reanchor(None, frame.timestamp)
        else:
            x, y = touch.x, touch.y
            if (
                self.prev_x is not None
                and self.prev_y is not None
                and self.prev_finger_id == touch.finger_id
                and self.prev_hid_ts is not None
            ):
                raw_dx = touch.x - self.prev_x
                raw_dy = touch.y - self.prev_y

                if self.invert_x:
                    raw_dx = -raw_dx
                if self.invert_y:
                    raw_dy = -raw_dy

                hid_dt = (frame.timestamp - self.prev_hid_ts) & 0xFFFF
                raw_dt_ms = hid_dt * self.timestamp_unit_ms

                self.raw_samples.append(TimedMotion(host_ns, raw_dx, raw_dy))

            self._raw_reanchor(touch, frame.timestamp)

        with self.lock:
            self.writer.writerow([
                host_ns, "raw", "",
                frame.timestamp, frame.finger_count, x, y,
                raw_dx, raw_dy, f"{raw_dt_ms:.6f}",
                "", "", "", "", "",
            ])

    def record_native_frame(
        self,
        host_ns: int,
        native_device: str,
        rel_x: int,
        rel_y: int,
        wheel: int,
        hwheel: int,
        keys: list[str],
    ):
        self.native_frames += 1
        stats = self.native_device_stats.setdefault(
            native_device, {"frames": 0, "motion": 0, "scroll": 0, "keys": 0}
        )
        stats["frames"] += 1
        if rel_x or rel_y:
            stats["motion"] += 1
            self.native_motion_frames += 1
            self.native_samples.append(TimedMotion(host_ns, rel_x, rel_y))
        if wheel or hwheel:
            self.native_scroll_frames += 1
            stats["scroll"] += 1
        if keys:
            self.native_key_frames += 1
            stats["keys"] += 1

        with self.lock:
            self.writer.writerow([
                host_ns, "native", native_device,
                "", "", "", "", "", "", "",
                rel_x, rel_y, wheel, hwheel, ";".join(keys),
            ])

    def flush(self):
        with self.lock:
            self.fp.flush()

    def close_and_summarize(self):
        with self.lock:
            self.fp.flush()
            self.fp.close()

        rows = self._build_summary_rows()

        with self.summary_path.open("w", newline="", encoding="utf-8") as fp:
            w = csv.writer(fp)
            w.writerow(self.SUMMARY_HEADER)
            w.writerows(rows)

        self._print_bucket_summary(rows)

        LOG.warning(
            "Comparison capture: native frames=%d, motion=%d, scroll=%d, key/button=%d",
            self.native_frames,
            self.native_motion_frames,
            self.native_scroll_frames,
            self.native_key_frames,
        )
        if self.native_device_stats:
            LOG.warning("Native activity by event node:")
            for dev, stats in sorted(self.native_device_stats.items()):
                LOG.warning(
                    "  %s: frames=%d motion=%d scroll=%d key/button=%d",
                    dev, stats["frames"], stats["motion"], stats["scroll"], stats["keys"]
                )
        else:
            LOG.warning("No native evdev events were captured from any monitored node.")
        LOG.warning("Comparison event CSV: %s", self.csv_path)
        LOG.warning("Comparison summary CSV: %s", self.summary_path)

    def _build_summary_rows(self):
        all_samples = self.raw_samples + self.native_samples
        if not all_samples:
            return []

        start_ns = min(s.host_ns for s in all_samples)
        end_ns = max(s.host_ns for s in all_samples)
        window_ns = int(self.window_ms * 1_000_000)
        if window_ns <= 0:
            window_ns = 100_000_000

        raw = sorted(self.raw_samples, key=lambda s: s.host_ns)
        native = sorted(self.native_samples, key=lambda s: s.host_ns)
        ri = ni = 0
        rows = []

        ws = start_ns - (start_ns % window_ns)
        while ws <= end_ns:
            we = ws + window_ns

            raw_dx = raw_dy = raw_path = 0.0
            while ri < len(raw) and raw[ri].host_ns < we:
                if raw[ri].host_ns >= ws:
                    raw_dx += raw[ri].dx
                    raw_dy += raw[ri].dy
                    raw_path += math.hypot(raw[ri].dx, raw[ri].dy)
                ri += 1

            native_dx = native_dy = native_path = 0.0
            while ni < len(native) and native[ni].host_ns < we:
                if native[ni].host_ns >= ws:
                    native_dx += native[ni].dx
                    native_dy += native[ni].dy
                    native_path += math.hypot(native[ni].dx, native[ni].dy)
                ni += 1

            if raw_path > 0 or native_path > 0:
                raw_speed = raw_path / self.window_ms
                gain = (native_path / raw_path) if raw_path > 0 else float("nan")
                rows.append([
                    ws, we, f"{self.window_ms:.3f}",
                    f"{raw_dx:.6f}", f"{raw_dy:.6f}", f"{raw_path:.6f}",
                    f"{native_dx:.6f}", f"{native_dy:.6f}", f"{native_path:.6f}",
                    f"{raw_speed:.9f}",
                    "" if math.isnan(gain) else f"{gain:.9f}",
                ])

            ws = we

        return rows

    def _print_bucket_summary(self, rows):
        # Chosen to cover the observed K400 range:
        # fine ~0.05-0.2 raw units/ms, normal ~1-4, fast ~5-12+.
        edges = [0.0, 0.10, 0.25, 0.50, 1.0, 2.0, 4.0, 8.0, 16.0, float("inf")]
        buckets: list[list[float]] = [[] for _ in range(len(edges) - 1)]

        for row in rows:
            speed = float(row[9])
            gain_text = row[10]
            if not gain_text:
                continue
            gain = float(gain_text)
            for i in range(len(edges) - 1):
                if edges[i] <= speed < edges[i + 1]:
                    buckets[i].append(gain)
                    break

        LOG.warning("Native/raw path gain by raw velocity (%g ms windows):", self.window_ms)
        any_data = False
        for i, values in enumerate(buckets):
            if not values:
                continue
            any_data = True
            lo, hi = edges[i], edges[i + 1]
            hi_text = "inf" if math.isinf(hi) else f"{hi:g}"
            LOG.warning(
                "  raw speed [%g, %s) units/ms: n=%d median=%.6f mean=%.6f",
                lo, hi_text, len(values), statistics.median(values), statistics.fmean(values)
            )
        if not any_data:
            LOG.warning("  no windows contained both raw motion and native REL motion")


class NativeEvdevReader:
    def __init__(self, device_path: str, recorder: ComparisonRecorder, stop_event: threading.Event):
        self.device_path = device_path
        self.native_device = device_path
        self.recorder = recorder
        self.stop_event = stop_event
        self.thread = threading.Thread(target=self._run, name="K400NativeEvdev", daemon=True)
        self.device: Optional[InputDevice] = None
        self.error: Optional[BaseException] = None

    def start(self):
        self.device = InputDevice(self.device_path)
        LOG.warning(
            "Comparison physical evdev: %s name=%r phys=%r uniq=%r",
            self.device.path, self.device.name, self.device.phys, self.device.uniq
        )
        self.native_device = f"{self.device.path}|{self.device.name}"
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass

    def _run(self):
        import select

        rel_x = rel_y = wheel = hwheel = 0
        keys: list[str] = []

        try:
            assert self.device is not None
            while not self.stop_event.is_set():
                ready, _, _ = select.select([self.device.fd], [], [], 0.25)
                if not ready:
                    continue

                for event in self.device.read():
                    if event.type == ecodes.EV_REL:
                        if event.code == ecodes.REL_X:
                            rel_x += event.value
                        elif event.code == ecodes.REL_Y:
                            rel_y += event.value
                        elif event.code == ecodes.REL_WHEEL:
                            wheel += event.value
                        elif event.code == ecodes.REL_HWHEEL:
                            hwheel += event.value
                    elif event.type == ecodes.EV_KEY:
                        name = ecodes.KEY.get(event.code, str(event.code))
                        if isinstance(name, list):
                            name = "/".join(name)
                        keys.append(f"{name}:{event.value}")
                    elif event.type == ecodes.EV_SYN and event.code == ecodes.SYN_REPORT:
                        if rel_x or rel_y or wheel or hwheel or keys:
                            self.recorder.record_native_frame(
                                time.monotonic_ns(),
                                self.native_device,
                                rel_x, rel_y, wheel, hwheel, keys
                            )
                        rel_x = rel_y = wheel = hwheel = 0
                        keys = []
        except BaseException as exc:
            if not self.stop_event.is_set():
                self.error = exc
                LOG.exception("Physical evdev comparison reader failed")



def discover_k400_event_nodes() -> list[str]:
    """Return all evdev nodes whose kernel-visible name contains 'K400'.

    We intentionally monitor all of them. A Unifying receiver can have several
    paired K400s, including offline ones, and the by-id receiver symlink does
    not necessarily identify the active child HID/event node.
    """
    found = []
    for path in list_devices():
        dev = None
        try:
            dev = InputDevice(path)
            name = dev.name or ""
            if "k400" not in name.lower():
                continue

            caps = dev.capabilities()
            rel_codes = set(caps.get(ecodes.EV_REL, []))
            key_codes = set(caps.get(ecodes.EV_KEY, []))

            pointerish = (
                ecodes.REL_X in rel_codes
                or ecodes.REL_Y in rel_codes
                or ecodes.REL_WHEEL in rel_codes
                or ecodes.BTN_LEFT in key_codes
                or ecodes.BTN_RIGHT in key_codes
            )
            if pointerish:
                found.append(path)
        except (PermissionError, OSError):
            continue
        finally:
            if dev is not None:
                try:
                    dev.close()
                except Exception:
                    pass
    return sorted(found)


def make_uinput(name: str) -> UInput:
    # Physical buttons still come from the K400 itself. The virtual device also
    # advertises buttons because raw tap gestures can synthesize clicks.
    capabilities = {
        ecodes.EV_REL: [
            ecodes.REL_X,
            ecodes.REL_Y,
            ecodes.REL_WHEEL,
            ecodes.REL_HWHEEL,
            ecodes.REL_WHEEL_HI_RES,
            ecodes.REL_HWHEEL_HI_RES,
        ],
        ecodes.EV_KEY: [ecodes.BTN_LEFT, ecodes.BTN_RIGHT],
    }
    return UInput(
        capabilities,
        name=name,
        vendor=0x046D,
        product=0x6100,
        version=1,
        bustype=ecodes.BUS_USB,
    )


def find_k400(
    slot: Optional[int],
    expected_wpid: str,
    receiver_path: Optional[str],
):
    """Find the intended K400 Plus.

    Auto mode (slot 0) may encounter multiple paired records with the same
    model WPID, including old/offline K400 pairings.  Prefer a currently
    responding K400 and, when needed, probe for TOUCHPAD_RAW_XY (0x6100) to
    identify a usable touchpad endpoint.

    An explicit positive --slot remains authoritative and is allowed to be
    asleep; wait_for_feature() will then wait for that selected device to wake.
    """
    opened_receivers = []
    matches = []
    expected = expected_wpid.upper()
    explicit_slot = slot is not None and slot > 0
    raw_feature = SupportedFeature.TOUCHPAD_RAW_XY

    for dev_info in base.receivers():
        if receiver_path and dev_info.path != receiver_path:
            continue

        try:
            r = receiver_mod.create_receiver(base, dev_info)
            if not r:
                continue
            opened_receivers.append(r)

            if explicit_slot:
                slots = [slot]
            else:
                try:
                    max_devices = int(getattr(r, "max_devices", 0) or 0)
                except (TypeError, ValueError):
                    max_devices = 0
                if max_devices <= 0:
                    max_devices = 6
                slots = range(1, max_devices + 1)

            for candidate_slot in slots:
                try:
                    dev = r[candidate_slot]
                except Exception as exc:
                    LOG.debug(
                        "Receiver %s has no usable slot %d: %s",
                        dev_info.path,
                        candidate_slot,
                        exc,
                    )
                    continue

                if not dev:
                    continue

                wpid = (dev.wpid or "").upper()
                if wpid != expected:
                    LOG.debug(
                        "Receiver %s slot %d: WPID=%s (not target)",
                        dev_info.path,
                        candidate_slot,
                        wpid,
                    )
                    continue

                # Explicit selection is authoritative.  Do not require it to
                # be awake during discovery; this preserves the useful ability
                # to start the service while the specified keyboard is asleep.
                if explicit_slot:
                    matches.append(
                        {
                            "receiver": r,
                            "device": dev,
                            "path": dev_info.path,
                            "slot": candidate_slot,
                            "online": None,
                            "raw_feature": None,
                        }
                    )
                    continue

                try:
                    online = bool(dev.ping())
                except Exception as exc:
                    LOG.debug(
                        "K400 candidate %s slot %d ping failed: %s",
                        dev_info.path,
                        candidate_slot,
                        exc,
                    )
                    online = False

                raw_index = None
                if online:
                    try:
                        raw_index = dev.features[raw_feature]
                    except Exception as exc:
                        LOG.debug(
                            "K400 candidate %s slot %d could not probe 0x6100: %s",
                            dev_info.path,
                            candidate_slot,
                            exc,
                        )

                LOG.debug(
                    "K400 candidate receiver=%s slot=%d online=%s raw6100=%s serial=%s",
                    dev_info.path,
                    candidate_slot,
                    online,
                    bool(raw_index),
                    getattr(dev, "serial", "") or "?",
                )

                matches.append(
                    {
                        "receiver": r,
                        "device": dev,
                        "path": dev_info.path,
                        "slot": candidate_slot,
                        "online": online,
                        "raw_feature": raw_index,
                    }
                )

        except Exception as exc:
            LOG.debug(
                "Unable to inspect receiver %s: %s",
                getattr(dev_info, "path", "?"),
                exc,
            )

    def close_except(selected_receiver=None):
        for receiver in opened_receivers:
            if receiver is selected_receiver:
                continue
            try:
                receiver.close()
            except Exception:
                pass

    def select(match, reason: str):
        close_except(match["receiver"])
        LOG.info(
            "Selected Logitech K400 Plus WPID %s on receiver %s slot %d (%s)",
            expected,
            match["path"],
            match["slot"],
            reason,
        )
        return match["receiver"], match["device"]

    if explicit_slot:
        if len(matches) == 1:
            return select(matches[0], "explicit slot override")
        close_except()
        scope = (
            f"receiver {receiver_path} slot {slot}"
            if receiver_path
            else f"slot {slot} on detected Logitech receivers"
        )
        raise RuntimeError(
            f"Could not find Logitech K400 Plus WPID {expected} in {scope}."
        )

    # Best evidence: the candidate is awake AND answers the exact raw-touch
    # feature this daemon requires.  This also filters same-WPID records that
    # are not the usable touchpad endpoint.
    raw_matches = [m for m in matches if m["online"] and m["raw_feature"]]
    if len(raw_matches) == 1:
        return select(raw_matches[0], "online + TOUCHPAD_RAW_XY 0x6100")
    if len(raw_matches) > 1:
        locations = ", ".join(
            f'{m["path"]} slot {m["slot"]}' for m in raw_matches
        )
        close_except()
        raise RuntimeError(
            f"Found multiple online K400 Plus devices exposing TOUCHPAD_RAW_XY: "
            f"{locations}. Use --receiver-path and/or --slot to select one."
        )

    # If exactly one matching K400 is awake, use it and let wait_for_feature()
    # perform the authoritative 0x6100 check with normal retry semantics.
    online_matches = [m for m in matches if m["online"]]
    if len(online_matches) == 1:
        return select(online_matches[0], "only responding WPID match")
    if len(online_matches) > 1:
        locations = ", ".join(
            f'{m["path"]} slot {m["slot"]}' for m in online_matches
        )
        close_except()
        raise RuntimeError(
            f"Found multiple responding K400 Plus devices with WPID {expected} "
            f"but no unique 0x6100 candidate: {locations}. "
            "Use --receiver-path and/or --slot to select one."
        )

    # One paired record is still unambiguous even while asleep; hold onto it
    # and wait for wake-up.  Multiple same-model offline pairings cannot be
    # distinguished until one becomes active, so retry quietly instead of
    # forcing users to hard-code a slot for a normal single-active-K400 setup.
    if len(matches) == 1:
        return select(matches[0], "sole paired WPID match; currently asleep")
    if len(matches) > 1:
        locations = ", ".join(
            f'{m["path"]} slot {m["slot"]}' for m in matches
        )
        close_except()
        raise DeviceNotReady(
            f"Multiple paired K400 Plus records are present but none is currently "
            f"responding ({locations}); waiting for the active K400 to wake."
        )

    close_except()
    scope = receiver_path or "detected Logitech receivers"
    raise RuntimeError(
        f"Could not find a paired Logitech K400 Plus WPID {expected} on {scope}. "
        "Verify that Solaar can see the device."
    )


def wait_for_feature(dev, feature, timeout_seconds: float = 10.0, interval_seconds: float = 0.25) -> int:
    """Wake the device and wait for a HID++ 2.0 feature to become queryable.

    Solaar's FeaturesArray refuses feature discovery while dev.online is false.
    A Unifying peripheral may be paired/present but asleep when the daemon first
    opens the receiver.  The old startup path ignored a failed ping and then
    treated a one-shot feature lookup miss as "unsupported".

    This helper retries ping/feature discovery and also performs a direct
    ROOT.GetFeature query as a fallback.  If a transient lookup cached False,
    the false entry is removed before installing the known-good feature index.
    """
    deadline = time.monotonic() + timeout_seconds
    last_error = None
    attempts = 0

    while time.monotonic() < deadline:
        attempts += 1

        try:
            dev.ping()
        except Exception as exc:
            last_error = exc

        if dev.online:
            try:
                index = dev.features[feature]
                if index:
                    LOG.debug(
                        "HID++ feature 0x%04X available at index %d after %d attempt(s)",
                        int(feature), index, attempts
                    )
                    return index
            except Exception as exc:
                last_error = exc

            # Fallback: ask ROOT.GetFeature directly. This bypasses the
            # FeaturesArray online/cache path but uses the same HID++ request.
            try:
                response = dev.request(0x0000, struct.pack("!H", int(feature)))
                if response and response[0]:
                    index = response[0]

                    # FeaturesArray may have cached a false result.  Its public
                    # __delitem__ intentionally forbids deletion, so use the
                    # underlying dict operation only for this recovery case.
                    if dict.get(dev.features, feature) is False:
                        dict.__delitem__(dev.features, feature)

                    dev.features[feature] = index
                    if len(response) > 1:
                        dev.features.flags[feature] = response[1]
                    if len(response) > 2:
                        dev.features.version[feature] = response[2]

                    LOG.debug(
                        "Recovered HID++ feature 0x%04X directly at index %d after %d attempt(s)",
                        int(feature), index, attempts
                    )
                    return index
            except Exception as exc:
                last_error = exc

        LOG.debug(
            "Waiting for K400 to wake / HID++ feature 0x%04X (online=%s protocol=%s attempt=%d)",
            int(feature), dev.online, getattr(dev, "_protocol", None), attempts
        )
        time.sleep(interval_seconds)

    detail = f"; last error: {last_error!r}" if last_error is not None else ""
    protocol = getattr(dev, "_protocol", None)
    message = (
        f"Timed out waiting for HID++ feature 0x{int(feature):04X} "
        f"(online={dev.online}, protocol={protocol}){detail}."
    )
    if not dev.online:
        raise DeviceNotReady(message + " Device is asleep/offline; waiting for wake-up.")
    raise RuntimeError(message + " Device is online but the required feature did not become available.")




def set_raw_report_state(dev, feature, requested_state: int) -> int:
    """Set HID++ 0x6100 raw report state and verify the complete readback."""
    dev.feature_request(feature, 0x20, bytes([requested_state]))
    verify_reply = dev.feature_request(feature, 0x10)
    if not verify_reply or (verify_reply[0] & RAW_FLAG) == 0:
        raise RuntimeError(
            f"Failed to enable HID++ raw reporting; readback was {verify_reply!r}"
        )

    actual_state = verify_reply[0]
    LOG.info(
        "Requested HID++ raw state 0x%02X; device readback is 0x%02X",
        requested_state, actual_state
    )
    if actual_state != requested_state:
        LOG.warning(
            "Device did not preserve every requested state bit: requested=0x%02X actual=0x%02X",
            requested_state, actual_state
        )
    return actual_state




def configure_fn_mode(dev, mode: str, feature_wait_seconds: float) -> None:
    """Configure Logitech NEW_FN_INVERSION (0x40A2).

    Solaar semantics:
      false (0x00): F1..F12 are standard F-keys; hold Fn for special actions.
      true  (0x01): special actions are primary; hold Fn for F1..F12.

    'leave' performs no operation.
    """
    if mode == "leave":
        return

    feature = SupportedFeature.NEW_FN_INVERSION
    try:
        wait_for_feature(
            dev,
            feature,
            timeout_seconds=min(feature_wait_seconds, 5.0),
        )

        before = dev.feature_request(feature, 0x00)
        before_value = (before[0] & 0x01) if before else None

        target = 0x00 if mode == "standard" else 0x01

        if before_value != target:
            dev.feature_request(feature, 0x10, bytes([target]))

        after = dev.feature_request(feature, 0x00)
        if not after:
            raise RuntimeError("NEW_FN_INVERSION readback returned no data")

        actual = after[0] & 0x01
        default = (after[1] & 0x01) if len(after) > 1 else None

        if actual != target:
            raise RuntimeError(
                f"NEW_FN_INVERSION readback mismatch: requested={target} actual={actual}"
            )

        LOG.info(
            "K400 F-key mode: %s (fn-swap=%s, firmware default=%s)",
            "standard F1-F12" if mode == "standard" else "special actions",
            bool(actual),
            "special" if default else "standard" if default is not None else "unknown",
        )

    except Exception as exc:
        LOG.warning(
            "Could not configure K400 F-key mode %r: %s",
            mode,
            exc,
            exc_info=LOG.isEnabledFor(logging.DEBUG),
        )


def origin_default_inversions(origin: int) -> tuple[bool, bool]:
    # Screen-relative coordinates: +X right, +Y down.
    invert_x = origin in (ORIGIN_LOWER_RIGHT, ORIGIN_UPPER_RIGHT)
    invert_y = origin in (ORIGIN_LOWER_LEFT, ORIGIN_LOWER_RIGHT)
    return invert_x, invert_y


def default_compare_csv() -> Path:
    stamp = time.strftime("%Y%m%d-%H%M%S")
    return Path(f"/tmp/k400-compare-{stamp}.csv")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="K400 Plus HID++ 0x6100 raw-XY pointer daemon and comparison logger."
    )
    parser.add_argument(
        "--slot",
        type=int,
        default=0,
        help="Receiver slot override; 0 (default) auto-scans all paired receiver slots."
    )
    parser.add_argument("--wpid", default=WPID_K400_PLUS, help="Expected Logitech WPID (default: 404D)")
    parser.add_argument("--receiver-path", help="Optional receiver hidraw path, e.g. /dev/hidraw0")
    parser.add_argument(
        "--feature-wait-seconds", type=float, default=10.0,
        help="How long to wait/retry if the K400 is asleep during HID++ feature discovery (default: 10)"
    )
    parser.add_argument(
        "--reconnect-delay-seconds", type=float, default=2.0,
        help="Delay before rediscovering the receiver after a HID++ session failure (default: 2)"
    )
    parser.add_argument(
        "--fn-mode",
        choices=("leave", "standard", "special"),
        default="leave",
        help="K400 F-key mode: 'standard' makes F1-F12 primary; "
             "'special' makes media/shortcut actions primary; "
             "'leave' does not change the device (default: leave)"
    )

    parser.add_argument("--gain", type=float, default=1.0,
                        help="Constant raw-coordinate -> relative-motion gain (default: 1.0)")
    parser.add_argument("--gain-x", type=float, help="Override X gain")
    parser.add_argument("--gain-y", type=float, help="Override Y gain")
    parser.add_argument(
        "--precision-gain", type=float, default=0.35,
        help="Low-speed multiplier relative to the base gain (default: 0.35; 1.0 disables shaping)"
    )
    parser.add_argument(
        "--precision-speed", type=float, default=0.12,
        help="Filtered raw speed at/below which --precision-gain is fully applied, units/ms (default: 0.12)"
    )
    parser.add_argument(
        "--precision-transition", type=float, default=0.75,
        help="Filtered raw speed at/above which full base gain is restored, units/ms (default: 0.75)"
    )
    parser.add_argument(
        "--precision-filter-ms", type=float, default=35.0,
        help="Time constant for raw-speed smoothing; 0 disables smoothing (default: 35 ms)"
    )
    parser.add_argument(
        "--tap", action=argparse.BooleanOptionalAction, default=True,
        help="Enable one-finger tap-to-left-click (default: enabled)"
    )
    parser.add_argument(
        "--two-finger-tap", action=argparse.BooleanOptionalAction, default=True,
        help="Enable two-finger tap-to-right-click (default: enabled)"
    )
    parser.add_argument(
        "--tap-max-ms", type=float, default=250.0,
        help="Maximum tap duration (default: 250 ms)"
    )
    parser.add_argument(
        "--tap-move-units", type=float, default=45.0,
        help="Maximum raw displacement allowed for a tap (default: 45 units)"
    )
    parser.add_argument(
        "--double-tap-window-ms", type=float, default=140.0,
        help="Total double-tap window measured from first finger-down; a single "
             "tap waits only for the remainder after release. 0 restores immediate "
             "single-click behavior (default: 140 ms)"
    )
    parser.add_argument(
        "--double-tap-move-units", type=float, default=60.0,
        help="Maximum raw distance between two tap locations for double-tap "
             "recognition (default: 60 units)"
    )
    parser.add_argument(
        "--tap-drag-window-ms", type=float, default=300.0,
        help="After a valid tap, allow a nearby retouch to become tap-and-drag "
             "for this long; 0 disables tap-and-drag (default: 300 ms)"
    )
    parser.add_argument(
        "--tap-drag-move-units", type=float, default=120.0,
        help="Maximum raw distance between the initial tap and retouch for "
             "tap-and-drag eligibility (default: 120 units)"
    )
    parser.add_argument(
        "--scroll", action=argparse.BooleanOptionalAction, default=True,
        help="Enable raw two-finger vertical scrolling (default: enabled)"
    )
    parser.add_argument(
        "--scroll-start-units", type=float, default=30.0,
        help="Two-finger centroid displacement before scrolling begins (default: 30 raw units)"
    )
    parser.add_argument(
        "--scroll-units-per-step", type=float, default=80.0,
        help="Two-finger raw Y units per REL_WHEEL step; lower is faster (default: 80)"
    )
    parser.add_argument(
        "--scroll-invert", action="store_true",
        help="Reverse vertical two-finger scroll direction"
    )
    parser.add_argument(
        "--horizontal-scroll", action=argparse.BooleanOptionalAction, default=False,
        help="Enable two-finger horizontal scrolling (default: disabled)"
    )
    parser.add_argument(
        "--hscroll-units-per-step", type=float, default=80.0,
        help="Two-finger raw X units per REL_HWHEEL step (default: 80)"
    )
    parser.add_argument(
        "--hscroll-invert", action="store_true",
        help="Reverse horizontal two-finger scroll direction"
    )
    parser.add_argument(
        "--scroll-axis-lock",
        choices=("on", "off"),
        default="on",
        help="Lock a clearly dominant two-finger scroll to its initial axis "
             "(default: on; only meaningful with horizontal scrolling enabled)"
    )
    parser.add_argument(
        "--scroll-axis-lock-ratio", type=float, default=1.5,
        help="Dominant-axis ratio required for vertical/horizontal lock; "
             "larger values require a straighter gesture (default: 1.5)"
    )
    parser.add_argument(
        "--multitouch-entry-grace-ms", type=float, default=50.0,
        help="Suppress pointer motion this long after a fresh one-finger contact "
             "while waiting for a possible second finger (default: 50 ms)"
    )
    parser.add_argument(
        "--multitouch-exit-grace-ms", type=float, default=80.0,
        help="After a two-finger scroll falls to one finger, suppress pointer "
             "motion this long before seamless handoff (default: 80 ms)"
    )
    parser.add_argument(
        "--kinetic-scroll",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Recreate stock K400 post-release wheel coasting (default: enabled)"
    )
    parser.add_argument(
        "--kinetic-history-ms", type=float, default=80.0,
        help="Recent raw scroll history used to estimate release velocity "
             "(default: 80 ms)"
    )
    parser.add_argument(
        "--kinetic-start-velocity", type=float, default=0.70,
        help="Minimum absolute release velocity in raw units/ms required to "
             "start coasting (default: 0.70)"
    )
    parser.add_argument(
        "--kinetic-stop-velocity", type=float, default=0.20,
        help="Stop kinetic scrolling when modeled velocity falls to this "
             "raw-units/ms value (default: 0.20)"
    )
    parser.add_argument(
        "--kinetic-decay-ms", type=float, default=400.0,
        help="Exponential velocity decay time constant; larger values coast "
             "longer (default: 400 ms)"
    )
    parser.add_argument(
        "--kinetic-max-ms", type=float, default=1600.0,
        help="Hard maximum post-release coasting duration (default: 1600 ms)"
    )
    parser.add_argument("--invert-x", action="store_true",
                        help="Toggle X inversion inferred from touchpad origin")
    parser.add_argument("--invert-y", action="store_true",
                        help="Toggle Y inversion inferred from touchpad origin")
    parser.add_argument("--max-raw-jump", type=int, default=600,
                        help="Re-anchor instead of emitting beyond this raw-frame jump; 0 disables")
    parser.add_argument("--name", default="Logitech K400 Raw Pointer",
                        help="Virtual pointer name")
    parser.add_argument("--print-raw", action="store_true",
                        help="Log every HID++ raw frame")
    parser.add_argument("--no-uinput", action="store_true",
                        help="Do not create/move a virtual pointer")

    parser.add_argument(
        "--dual-mode", action="store_true",
        help="Request HID++ RAW|ENHANCED|RAW_AND_NATIVE (0x15) instead of 0x05"
    )
    parser.add_argument(
        "--compare-device",
        action="append",
        default=[],
        help="Physical K400 evdev node to capture read-only. May be specified more "
             "than once. Implies --dual-mode."
    )
    parser.add_argument(
        "--compare-auto-k400",
        action="store_true",
        help="Auto-discover and monitor every pointer-like /dev/input/event* node "
             "whose device name contains K400. Implies --dual-mode."
    )
    parser.add_argument(
        "--compare-csv",
        help="Event CSV path. Default in comparison mode: /tmp/k400-compare-<timestamp>.csv"
    )
    parser.add_argument(
        "--compare-window-ms", type=float, default=100.0,
        help="Host-time aggregation window for comparison summary (default: 100 ms)"
    )

    parser.add_argument(
        "--log-level",
        choices=("warning", "info", "debug"),
        default="warning",
        help="Daemon log verbosity (default: warning). -v/-vv override this for manual debugging."
    )
    parser.add_argument("-v", "--verbose", action="count", default=0)
    args = parser.parse_args()

    if not (0 <= args.slot <= 15):
        parser.error("--slot must be between 0 and 15 (0 = auto-scan)")

    level = {
        "warning": logging.WARNING,
        "info": logging.INFO,
        "debug": logging.DEBUG,
    }[args.log_level]
    if args.verbose == 1:
        level = logging.INFO
    elif args.verbose >= 2:
        level = logging.DEBUG

    # Apply the requested level to the whole process.  At the normal WARNING
    # default this also suppresses Solaar/hidapi INFO chatter in journald.
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)-8s %(name)s: %(message)s",
    )

    if not math.isfinite(args.gain) or args.gain <= 0:
        parser.error("--gain must be a positive finite number")
    for value, name in ((args.gain_x, "--gain-x"), (args.gain_y, "--gain-y")):
        if value is not None and (not math.isfinite(value) or value <= 0):
            parser.error(f"{name} must be a positive finite number")
    if not math.isfinite(args.feature_wait_seconds) or args.feature_wait_seconds <= 0:
        parser.error("--feature-wait-seconds must be positive")
    if not math.isfinite(args.reconnect_delay_seconds) or args.reconnect_delay_seconds <= 0:
        parser.error("--reconnect-delay-seconds must be positive")
    if not math.isfinite(args.precision_gain) or not (0 < args.precision_gain <= 1.0):
        parser.error("--precision-gain must be > 0 and <= 1")
    if not math.isfinite(args.precision_speed) or args.precision_speed < 0:
        parser.error("--precision-speed must be >= 0")
    if (
        not math.isfinite(args.precision_transition)
        or args.precision_transition <= args.precision_speed
    ):
        parser.error("--precision-transition must be greater than --precision-speed")
    if not math.isfinite(args.precision_filter_ms) or args.precision_filter_ms < 0:
        parser.error("--precision-filter-ms must be >= 0")
    if not math.isfinite(args.tap_max_ms) or args.tap_max_ms <= 0:
        parser.error("--tap-max-ms must be positive")
    if not math.isfinite(args.tap_move_units) or args.tap_move_units <= 0:
        parser.error("--tap-move-units must be positive")
    if (
        not math.isfinite(args.double_tap_window_ms)
        or args.double_tap_window_ms < 0
    ):
        parser.error("--double-tap-window-ms must be >= 0")
    if (
        not math.isfinite(args.double_tap_move_units)
        or args.double_tap_move_units <= 0
    ):
        parser.error("--double-tap-move-units must be positive")
    if (
        not math.isfinite(args.tap_drag_window_ms)
        or args.tap_drag_window_ms < 0
    ):
        parser.error("--tap-drag-window-ms must be >= 0")
    if (
        not math.isfinite(args.tap_drag_move_units)
        or args.tap_drag_move_units <= 0
    ):
        parser.error("--tap-drag-move-units must be positive")
    if not math.isfinite(args.scroll_start_units) or args.scroll_start_units < 0:
        parser.error("--scroll-start-units must be >= 0")
    if not math.isfinite(args.scroll_units_per_step) or args.scroll_units_per_step <= 0:
        parser.error("--scroll-units-per-step must be positive")
    if not math.isfinite(args.hscroll_units_per_step) or args.hscroll_units_per_step <= 0:
        parser.error("--hscroll-units-per-step must be positive")
    if (
        not math.isfinite(args.scroll_axis_lock_ratio)
        or args.scroll_axis_lock_ratio <= 1.0
    ):
        parser.error("--scroll-axis-lock-ratio must be > 1.0")
    if (
        not math.isfinite(args.multitouch_entry_grace_ms)
        or args.multitouch_entry_grace_ms < 0
    ):
        parser.error("--multitouch-entry-grace-ms must be >= 0")
    if (
        not math.isfinite(args.multitouch_exit_grace_ms)
        or args.multitouch_exit_grace_ms < 0
    ):
        parser.error("--multitouch-exit-grace-ms must be >= 0")
    if not math.isfinite(args.kinetic_history_ms) or args.kinetic_history_ms <= 0:
        parser.error("--kinetic-history-ms must be positive")
    if (
        not math.isfinite(args.kinetic_start_velocity)
        or args.kinetic_start_velocity <= 0
    ):
        parser.error("--kinetic-start-velocity must be positive")
    if (
        not math.isfinite(args.kinetic_stop_velocity)
        or args.kinetic_stop_velocity <= 0
    ):
        parser.error("--kinetic-stop-velocity must be positive")
    if args.kinetic_stop_velocity >= args.kinetic_start_velocity:
        parser.error("--kinetic-stop-velocity must be less than --kinetic-start-velocity")
    if not math.isfinite(args.kinetic_decay_ms) or args.kinetic_decay_ms <= 0:
        parser.error("--kinetic-decay-ms must be positive")
    if not math.isfinite(args.kinetic_max_ms) or args.kinetic_max_ms <= 0:
        parser.error("--kinetic-max-ms must be positive")
    if not math.isfinite(args.compare_window_ms) or args.compare_window_ms <= 0:
        parser.error("--compare-window-ms must be positive")

    gain_x = args.gain if args.gain_x is None else args.gain_x
    gain_y = args.gain if args.gain_y is None else args.gain_y

    compare_devices = list(args.compare_device)
    if args.compare_auto_k400:
        auto_devices = discover_k400_event_nodes()
        LOG.warning("Auto-discovered K400 comparison nodes: %s", auto_devices or "<none>")
        for path in auto_devices:
            if path not in compare_devices:
                compare_devices.append(path)

    comparison_mode = bool(compare_devices)
    if comparison_mode:
        args.dual_mode = True

    stop_event = threading.Event()

    def request_stop(signum, _frame):
        LOG.info("Received signal %s; stopping", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    # The uinput device is deliberately kept alive across HID++ reconnects, so
    # KDE/X11 sees one stable virtual pointer rather than a new device every
    # time the K400 or receiver reconnects.
    ui = None
    recorder: Optional[ComparisonRecorder] = None
    native_readers: list[NativeEvdevReader] = []
    return_code = 0
    requested_state = RAW_ENHANCED_DUAL if args.dual_mode else RAW_ENHANCED

    try:
        if not args.no_uinput:
            ui = make_uinput(args.name)
            virtual_path = ui.device.path if ui.device is not None else ui.devnode
            LOG.info("Created persistent virtual pointer: %s (%s)", args.name, virtual_path)
            time.sleep(0.25)

        session_number = 0
        comparison_initialized = False

        while not stop_event.is_set():
            session_number += 1
            r = dev = event_listener = None
            feature = SupportedFeature.TOUCHPAD_RAW_XY
            feature_index = None
            pointer: Optional[RawPointerEngine] = None
            old_raw_state: Optional[int] = None
            raw_enabled = False
            reconfigure_event = threading.Event()

            try:
                LOG.debug("Opening K400 HID++ session #%d", session_number)
                r, dev = find_k400(args.slot, args.wpid, args.receiver_path)
                feature_index = wait_for_feature(
                    dev,
                    feature,
                    timeout_seconds=args.feature_wait_seconds,
                )

                info = parse_info(dev.feature_request(feature, 0x00))
                timestamp_unit_ms = info.timestamp_units / 10.0
                # HID++ 0x6100 spec exception: field value 8 still means 1 ms.
                if info.timestamp_units == 8:
                    timestamp_unit_ms = 1.0

                LOG.info(
                    "K400 raw touchpad: %dx%d units, native DPI=%d, origin=%d, max_fingers=%d, "
                    "timestamp_unit=%.3f ms",
                    info.x_size, info.y_size, info.dpi, info.origin,
                    info.max_fingers, timestamp_unit_ms
                )

                default_invert_x, default_invert_y = origin_default_inversions(info.origin)
                invert_x = default_invert_x ^ args.invert_x
                invert_y = default_invert_y ^ args.invert_y

                if session_number == 1:
                    LOG.info(
                        "Base gain: X=%.6f Y=%.6f; invert_x=%s invert_y=%s",
                        gain_x, gain_y, invert_x, invert_y
                    )
                    LOG.info(
                        "Precision curve: multiplier=%.3f at <= %.3f units/ms, "
                        "smoothly reaching 1.0 at %.3f units/ms; filter=%.1f ms",
                        args.precision_gain, args.precision_speed,
                        args.precision_transition, args.precision_filter_ms
                    )
                    LOG.info(
                        "Gestures: tap=%s two-finger-tap=%s scroll=%s "
                        "(start=%.1f units, %.1f units/wheel-step; "
                        "entry-grace=%.1f ms exit-grace=%.1f ms; "
                        "double-tap=%.1f ms/%.1f units; "
                        "tap-drag=%.1f ms/%.1f units)",
                        args.tap, args.two_finger_tap, args.scroll,
                        args.scroll_start_units, args.scroll_units_per_step,
                        args.multitouch_entry_grace_ms,
                        args.multitouch_exit_grace_ms,
                        args.double_tap_window_ms,
                        args.double_tap_move_units,
                        args.tap_drag_window_ms,
                        args.tap_drag_move_units,
                    )
                    LOG.info(
                        "Scroll axis lock: %s ratio=%.2f; horizontal-scroll=%s",
                        args.scroll_axis_lock,
                        args.scroll_axis_lock_ratio,
                        args.horizontal_scroll,
                    )
                    LOG.info(
                        "Kinetic scroll: enabled=%s history=%.0f ms "
                        "start=%.3f stop=%.3f units/ms decay=%.0f ms max=%.0f ms",
                        args.kinetic_scroll,
                        args.kinetic_history_ms,
                        args.kinetic_start_velocity,
                        args.kinetic_stop_velocity,
                        args.kinetic_decay_ms,
                        args.kinetic_max_ms,
                    )


                if comparison_mode and not comparison_initialized:
                    csv_path = Path(args.compare_csv) if args.compare_csv else default_compare_csv()
                    csv_path.parent.mkdir(parents=True, exist_ok=True)
                    recorder = ComparisonRecorder(
                        csv_path=csv_path,
                        window_ms=args.compare_window_ms,
                        invert_x=invert_x,
                        invert_y=invert_y,
                        timestamp_unit_ms=timestamp_unit_ms,
                    )
                    for device_path in compare_devices:
                        reader = NativeEvdevReader(device_path, recorder, stop_event)
                        reader.start()
                        native_readers.append(reader)
                    comparison_initialized = True

                pointer = RawPointerEngine(
                    gain_x=gain_x,
                    gain_y=gain_y,
                    invert_x=invert_x,
                    invert_y=invert_y,
                    max_raw_jump=args.max_raw_jump,
                    ui=ui,
                    print_raw=args.print_raw,
                    timestamp_unit_ms=timestamp_unit_ms,
                    precision_gain=args.precision_gain,
                    precision_speed=args.precision_speed,
                    precision_transition=args.precision_transition,
                    precision_filter_ms=args.precision_filter_ms,
                    tap_enabled=args.tap,
                    two_finger_tap_enabled=args.two_finger_tap,
                    tap_max_ms=args.tap_max_ms,
                    tap_move_units=args.tap_move_units,
                    scroll_enabled=args.scroll,
                    scroll_start_units=args.scroll_start_units,
                    scroll_units_per_step=args.scroll_units_per_step,
                    scroll_invert=args.scroll_invert,
                    horizontal_scroll=args.horizontal_scroll,
                    hscroll_units_per_step=args.hscroll_units_per_step,
                    hscroll_invert=args.hscroll_invert,
                    multitouch_entry_grace_ms=args.multitouch_entry_grace_ms,
                    multitouch_exit_grace_ms=args.multitouch_exit_grace_ms,
                    double_tap_window_ms=args.double_tap_window_ms,
                    double_tap_move_units=args.double_tap_move_units,
                    tap_drag_window_ms=args.tap_drag_window_ms,
                    tap_drag_move_units=args.tap_drag_move_units,
                    scroll_axis_lock=(args.scroll_axis_lock == "on"),
                    scroll_axis_lock_ratio=args.scroll_axis_lock_ratio,
                    kinetic_scroll=args.kinetic_scroll,
                    kinetic_history_ms=args.kinetic_history_ms,
                    kinetic_start_velocity=args.kinetic_start_velocity,
                    kinetic_stop_velocity=args.kinetic_stop_velocity,
                    kinetic_decay_ms=args.kinetic_decay_ms,
                    kinetic_max_ms=args.kinetic_max_ms,
                )

                def notification_callback(n):
                    nonlocal feature_index

                    if n.devnumber != dev.number:
                        return

                    # HID++ receiver connection notification (0x41).  The
                    # upper nibble of data[0] contains connection flags; bit
                    # 0x40 means the wireless link is NOT established.  Normal
                    # K400 idle/sleep therefore produces a link-down event and
                    # must not be treated as a failed HID++ session.
                    #
                    # Only a genuine link-up schedules 0x6100/F-key
                    # reinitialization. This mirrors Solaar's own connection
                    # notification parsing and prevents idle reconnect churn.
                    if n.sub_id == 0x41:
                        flags = (n.data[0] & 0xF0) if n.data else 0x40
                        link_established = not bool(flags & 0x40)
                        dev.online = link_established

                        if pointer is not None:
                            pointer.reset_runtime()

                        if link_established:
                            reconfigure_event.set()
                            LOG.debug(
                                "K400 link established; scheduling raw-mode reinitialization"
                            )
                        else:
                            LOG.debug(
                                "K400 wireless link inactive/asleep; waiting for wake-up"
                            )
                        return

                    if n.sub_id != feature_index:
                        return
                    if (n.address >> 4) != 0x00:  # 0x6100 event function 0 = DualXY
                        return

                    frame = parse_frame(n.data)
                    if frame is None:
                        LOG.warning("Short raw XY notification: %r", n.data)
                        return

                    host_ns = time.monotonic_ns()
                    if recorder is not None:
                        recorder.record_raw(frame, host_ns)
                    pointer.process(frame, host_ns)

                event_listener = StartedEventsListener(r, notification_callback)
                event_listener.start()
                if not event_listener.started_event.wait(timeout=3.0):
                    raise RuntimeError("Timed out starting HID++ receiver listener")

                state_reply = dev.feature_request(feature, 0x10)
                if state_reply:
                    old_raw_state = state_reply[0]
                else:
                    old_raw_state = 0x00
                    LOG.warning(
                        "Could not read previous raw-report state; assuming 0x00 for restore"
                    )

                LOG.debug(
                    "Previous HID++ 0x6100 raw-report state: 0x%02X",
                    old_raw_state
                )

                actual_state = set_raw_report_state(dev, feature, requested_state)
                raw_enabled = True

                configure_fn_mode(
                    dev,
                    args.fn_mode,
                    feature_wait_seconds=args.feature_wait_seconds,
                )

                if args.dual_mode and not (actual_state & RAW_AND_NATIVE_FLAG):
                    LOG.warning(
                        "RAW_AND_NATIVE bit (0x10) is NOT set in readback; standard native "
                        "tracking is unlikely to be available for comparison."
                    )

                if comparison_mode:
                    LOG.info(
                        "Comparison mode active on %d evdev node(s).",
                        len(native_readers)
                    )
                else:
                    LOG.info(
                        "K400 raw pointer active. Connection notifications will automatically "
                        "re-enable 0x6100 after a device power-cycle."
                    )

                while not stop_event.is_set():
                    if not event_listener.is_alive():
                        raise ConnectionError("HID++ receiver listener stopped")

                    for reader in native_readers:
                        if reader.error is not None:
                            raise ConnectionError(
                                f"Physical evdev reader failed for {reader.device_path}: "
                                f"{reader.error}"
                            )

                    # Wait in short increments so SIGTERM remains responsive.
                    if not reconfigure_event.wait(timeout=1.0):
                        continue
                    reconfigure_event.clear()

                    if stop_event.is_set():
                        break

                    LOG.info(
                        "Reinitializing K400 raw mode after device connection/power-cycle"
                    )
                    pointer.reset_runtime()

                    # A connection notification may precede the device becoming
                    # fully responsive by a few milliseconds. Reuse the wake/
                    # feature retry helper rather than assuming it is ready.
                    feature_index = wait_for_feature(
                        dev,
                        feature,
                        timeout_seconds=args.feature_wait_seconds,
                    )
                    actual_state = set_raw_report_state(
                        dev,
                        feature,
                        requested_state,
                    )
                    raw_enabled = True

                    configure_fn_mode(
                        dev,
                        args.fn_mode,
                        feature_wait_seconds=args.feature_wait_seconds,
                    )

                    LOG.info(
                        "K400 raw mode re-enabled successfully (state 0x%02X)",
                        actual_state
                    )

            except KeyboardInterrupt:
                stop_event.set()
            except DeviceNotReady as exc:
                if stop_event.is_set():
                    break
                LOG.debug(
                    "K400 not ready (%s). Retrying discovery in %.1f s.",
                    exc,
                    args.reconnect_delay_seconds,
                )
            except Exception as exc:
                if stop_event.is_set():
                    break
                LOG.warning(
                    "K400 HID++ session #%d lost (%s). Will rediscover receiver/device in %.1f s.",
                    session_number,
                    exc,
                    args.reconnect_delay_seconds,
                    exc_info=LOG.isEnabledFor(logging.DEBUG),
                )
            finally:
                if pointer is not None:
                    try:
                        pointer.reset_runtime()
                    except Exception:
                        LOG.debug("Error resetting pointer runtime state", exc_info=True)

                # Restore only on an intentional process stop.  During a broken
                # receiver session the transport may no longer exist, and a
                # reconnect will establish a fresh device state anyway.
                if stop_event.is_set() and dev is not None and raw_enabled:
                    restore = 0x00 if old_raw_state is None else old_raw_state
                    try:
                        dev.feature_request(feature, 0x20, bytes([restore]))
                        LOG.info(
                            "Restored HID++ 0x6100 raw-report state to 0x%02X",
                            restore
                        )
                    except Exception:
                        LOG.warning(
                            "Could not restore raw-report state during shutdown; "
                            "device/receiver may already be disconnected.",
                            exc_info=LOG.isEnabledFor(logging.DEBUG),
                        )

                if event_listener is not None:
                    try:
                        event_listener.stop()
                        event_listener.join(timeout=2.0)
                    except Exception:
                        LOG.debug("Error stopping HID++ listener", exc_info=True)

                if r is not None:
                    try:
                        r.close()
                    except Exception:
                        LOG.debug("Error closing receiver session", exc_info=True)

            if not stop_event.is_set():
                # Keep the uinput device alive; only recycle the HID++ side.
                stop_event.wait(args.reconnect_delay_seconds)

    except KeyboardInterrupt:
        stop_event.set()
    except Exception:
        LOG.exception("Fatal error")
        return_code = 1
    finally:
        for reader in native_readers:
            try:
                reader.stop()
            except Exception:
                LOG.debug(
                    "Error stopping physical evdev comparison reader %s",
                    reader.device_path,
                    exc_info=True,
                )

        if recorder is not None:
            try:
                recorder.close_and_summarize()
            except Exception:
                LOG.exception("Error finalizing comparison CSV/summary")

        if ui is not None:
            try:
                ui.close()
            except Exception:
                pass

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
