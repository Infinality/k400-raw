from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import Optional

from evdev import UInput, ecodes

from .hidpp import Touch, RawFrame, active_touches, active_single_touch

LOG = logging.getLogger("k400-raw")


def make_pointer_uinput(name: str) -> UInput:
    capabilities = {
        ecodes.EV_REL: [
            ecodes.REL_X, ecodes.REL_Y,
            ecodes.REL_WHEEL, ecodes.REL_HWHEEL,
            ecodes.REL_WHEEL_HI_RES, ecodes.REL_HWHEEL_HI_RES,
        ],
        ecodes.EV_KEY: [ecodes.BTN_LEFT, ecodes.BTN_RIGHT],
    }
    return UInput(
        capabilities, name=name, vendor=0x046D, product=0x6100,
        version=1, bustype=ecodes.BUS_USB,
    )

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
        scroll_units_per_detent: float,
        scroll_invert: bool,
        horizontal_scroll: bool,
        hscroll_units_per_detent: float,
        hscroll_invert: bool,
        multitouch_entry_grace_ms: float,
        multitouch_exit_grace_ms: float,
        tap_drag_window_ms: float,
        tap_drag_activation_units: float,
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
        self.scroll_units_per_detent = scroll_units_per_detent
        self.scroll_invert = scroll_invert
        self.horizontal_scroll = horizontal_scroll
        self.hscroll_units_per_detent = hscroll_units_per_detent
        self.hscroll_invert = hscroll_invert

        self.multitouch_entry_grace_ms = multitouch_entry_grace_ms
        self.multitouch_exit_grace_ms = multitouch_exit_grace_ms

        # Match libinput's tap-and-drag model: a valid tap presses BTN_LEFT
        # when the tap completes, but the release is held briefly.  If a
        # second finger lands inside this window, the already-held button can
        # become a drag without generating a second BTN_LEFT press (and thus
        # without triggering a desktop double-click action first).  If no
        # second touch arrives, the held press is released by a timer and the
        # ordinary single click completes.
        self.tap_drag_window_ms = tap_drag_window_ms

        # On a second touch after a tap, suppress only this small amount of
        # motion while deciding double-click vs. drag.  The button is already
        # logically held from the first tap.  Release inside the threshold
        # completes a normal second click; crossing it arms drag motion.
        self.tap_drag_activation_units = tap_drag_activation_units

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

        self._emit_lock = threading.Lock()

        # Tap-and-drag state.  A valid tap leaves BTN_LEFT logically held for
        # a short window, mirroring libinput's TAP_STATE_*TAPPED behavior.
        # A second touch either turns that held press into a drag or, if it is
        # released without meaningful motion, completes a second click.
        self._drag_source_deadline_ns = 0
        self._drag_source_x = 0
        self._drag_source_y = 0
        self._tap_drag_candidate = False
        self._tap_drag_active = False
        self._tap_hold_active = False
        self._tap_hold_timer: Optional[threading.Timer] = None
        self._tap_hold_generation = 0
        self._tap_state_lock = threading.RLock()

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
        # High-resolution wheel conversion state.
        #
        # *_v120_frac preserves sub-v120 fractions from the raw->wheel
        # conversion.  *_legacy_v120 accumulates integer v120 output until a
        # full +/-120 detent is reached, at which point the corresponding
        # legacy REL_WHEEL/REL_HWHEEL event is emitted for compatibility.
        self.scroll_v120_frac = 0.0
        self.hscroll_v120_frac = 0.0
        self.scroll_legacy_v120 = 0
        self.hscroll_legacy_v120 = 0

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
        self._cancel_tap_sequence("runtime reset", release_button=True)
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
        self.scroll_v120_frac = 0.0
        self.hscroll_v120_frac = 0.0
        self.scroll_legacy_v120 = 0
        self.hscroll_legacy_v120 = 0
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

        # Serialize complete uinput click sequences so button events cannot
        # interleave with motion/scroll writes from another helper thread.
        with self._emit_lock:
            self.ui.write(ecodes.EV_KEY, code, 1)
            self.ui.syn()
            self.ui.write(ecodes.EV_KEY, code, 0)
            self.ui.syn()
        LOG.debug("%s emitted", label)








    def _emit_button_state(self, code: int, pressed: bool, label: str):
        if self.ui is None:
            LOG.debug("%s recognized (no uinput; not emitted)", label)
            return

        with self._emit_lock:
            self.ui.write(ecodes.EV_KEY, code, 1 if pressed else 0)
            self.ui.syn()
        LOG.debug("%s emitted", label)


    def _cancel_tap_hold_timer_locked(self):
        timer = self._tap_hold_timer
        self._tap_hold_timer = None
        self._tap_hold_generation += 1
        if timer is not None:
            timer.cancel()

    def _tap_hold_timeout(self, generation: int):
        """Complete a normal single tap if no retouch arrived in time."""
        with self._tap_state_lock:
            if generation != self._tap_hold_generation:
                return
            self._tap_hold_timer = None
            if not self._tap_hold_active:
                return
            if self._tap_drag_candidate or self._tap_drag_active:
                return

            self._tap_hold_active = False
            self._drag_source_deadline_ns = 0
            self._emit_button_state(
                ecodes.BTN_LEFT,
                False,
                "tap hold timeout -> BTN_LEFT up",
            )

    def _arm_tap_hold(self, release_ns: int, x: int, y: int):
        """Press BTN_LEFT now and defer only its release for drag detection."""
        if self.tap_drag_window_ms <= 0:
            self._emit_click(ecodes.BTN_LEFT, "one-finger tap -> BTN_LEFT")
            self._drag_source_deadline_ns = 0
            return

        with self._tap_state_lock:
            self._cancel_tap_hold_timer_locked()

            # Defensive cleanup: this should never normally be active here,
            # but never stack a new synthetic press on top of an old one.
            if self._tap_hold_active:
                self._tap_hold_active = False
                self._emit_button_state(
                    ecodes.BTN_LEFT,
                    False,
                    "stale tap hold -> BTN_LEFT up",
                )

            self._tap_hold_active = True
            self._tap_drag_candidate = False
            self._tap_drag_active = False
            self._drag_source_deadline_ns = (
                release_ns + int(self.tap_drag_window_ms * 1_000_000.0)
            )
            self._drag_source_x = x
            self._drag_source_y = y

            self._emit_button_state(
                ecodes.BTN_LEFT,
                True,
                "one-finger tap -> BTN_LEFT down (release deferred for tap-and-drag)",
            )

            self._tap_hold_generation += 1
            generation = self._tap_hold_generation
            timer = threading.Timer(
                self.tap_drag_window_ms / 1000.0,
                self._tap_hold_timeout,
                args=(generation,),
            )
            timer.daemon = True
            self._tap_hold_timer = timer
            timer.start()

    def _begin_possible_tap_drag(self, host_ns: int, x: int, y: int) -> bool:
        self._tap_drag_candidate = False

        with self._tap_state_lock:
            if not self._tap_hold_active:
                return False
            if not self._drag_source_deadline_ns:
                return False
            if host_ns > self._drag_source_deadline_ns:
                # The timer should normally have completed the click already,
                # but handle scheduler delay deterministically here too.
                self._cancel_tap_hold_timer_locked()
                self._tap_hold_active = False
                self._drag_source_deadline_ns = 0
                self._emit_button_state(
                    ecodes.BTN_LEFT,
                    False,
                    "expired tap hold -> BTN_LEFT up",
                )
                return False

            # Keep the original BTN_LEFT press held.  Do not emit a second
            # press here: desktops commonly recognize double-click on that
            # press, which would trigger e.g. titlebar maximize before drag.
            self._cancel_tap_hold_timer_locked()
            self._tap_drag_candidate = True
            LOG.debug(
                "new touch joined held tap: dt remaining=%.1f ms",
                (self._drag_source_deadline_ns - host_ns) / 1_000_000.0,
            )
            return True

    def _start_tap_drag(self):
        """Convert the held first-tap press into active drag motion."""
        with self._tap_state_lock:
            if self._tap_drag_active or not self._tap_drag_candidate:
                return False
            if not self._tap_hold_active:
                return False

            self._tap_drag_candidate = False
            self._drag_source_deadline_ns = 0
            self.single_tap_candidate = False
            self.single_motion_grace_until_ns = 0
            self._tap_drag_active = True
            LOG.debug("tap-and-drag armed using already-held BTN_LEFT")
            return True

    def _release_tap_hold(self, reason: str):
        with self._tap_state_lock:
            self._cancel_tap_hold_timer_locked()
            if not self._tap_hold_active:
                return
            self._tap_hold_active = False
            self._drag_source_deadline_ns = 0
            self._emit_button_state(
                ecodes.BTN_LEFT,
                False,
                f"{reason} -> BTN_LEFT up",
            )

    def _cancel_tap_sequence(self, reason: str, release_button: bool = True):
        with self._tap_state_lock:
            self._cancel_tap_hold_timer_locked()
            self._tap_drag_candidate = False
            self._tap_drag_active = False
            self._drag_source_deadline_ns = 0
            if release_button and self._tap_hold_active:
                self._tap_hold_active = False
                self._emit_button_state(
                    ecodes.BTN_LEFT,
                    False,
                    f"tap sequence cancelled ({reason}) -> BTN_LEFT up",
                )
            elif not release_button:
                self._tap_hold_active = False

    def _end_tap_drag(self, reason: str):
        if not self._tap_drag_active:
            return

        self._tap_drag_active = False
        self._tap_drag_candidate = False
        self._release_tap_hold(f"tap-and-drag end ({reason})")

    def _emit_hires_scroll(
        self,
        wheel_v120: float = 0.0,
        hwheel_v120: float = 0.0,
    ):
        """Emit fine-grained high-resolution wheel motion.

        Linux defines +/-120 REL_*WHEEL_HI_RES units as one logical wheel
        detent.  Pointer mode therefore maps raw touch displacement directly
        into v120 units and emits those small increments at the raw report
        cadence instead of waiting for a whole detent.

        Legacy REL_WHEEL / REL_HWHEEL compatibility events are generated only
        when the running high-resolution total crosses a full +/-120 boundary.
        High-resolution-aware consumers see the fine increments; legacy-only
        consumers retain ordinary detent semantics.
        """
        if self.ui is None or (wheel_v120 == 0.0 and hwheel_v120 == 0.0):
            return (0, 0)

        with self._emit_lock:
            # Preserve fractional v120 conversion error between frames.
            self.scroll_v120_frac += wheel_v120
            wheel_hi = int(self.scroll_v120_frac)
            self.scroll_v120_frac -= wheel_hi

            self.hscroll_v120_frac += hwheel_v120
            hwheel_hi = int(self.hscroll_v120_frac)
            self.hscroll_v120_frac -= hwheel_hi

            wheel_legacy = 0
            hwheel_legacy = 0

            if wheel_hi:
                self.scroll_legacy_v120 += wheel_hi
                wheel_legacy = int(self.scroll_legacy_v120 / 120)
                self.scroll_legacy_v120 -= wheel_legacy * 120

            if hwheel_hi:
                self.hscroll_legacy_v120 += hwheel_hi
                hwheel_legacy = int(self.hscroll_legacy_v120 / 120)
                self.hscroll_legacy_v120 -= hwheel_legacy * 120

            if not (wheel_hi or hwheel_hi or wheel_legacy or hwheel_legacy):
                return (0, 0)

            # Put the high-resolution values and any corresponding legacy
            # boundary crossings in the same SYN_REPORT.
            if wheel_hi:
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_WHEEL_HI_RES,
                    wheel_hi,
                )
            if wheel_legacy:
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_WHEEL,
                    wheel_legacy,
                )

            if hwheel_hi:
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_HWHEEL_HI_RES,
                    hwheel_hi,
                )
            if hwheel_legacy:
                self.ui.write(
                    ecodes.EV_REL,
                    ecodes.REL_HWHEEL,
                    hwheel_legacy,
                )

            self.ui.syn()
            return (wheel_hi, hwheel_hi)

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
        """Continue the existing exponential coast using fine v120 events.

        The decay model is unchanged:

            v(t) = v0 * exp(-t / tau)

        To preserve the old pointer backend's total coast distance, first
        compute how many complete raw scroll detents that model would have
        produced before the configured stop-velocity/max-duration limit.
        The high-resolution worker then emits exactly that many logical
        detents worth of v120 motion, but subdivides the same displacement at
        an 8 ms cadence instead of emitting one +/-120 jump at a time.

        Thus:
          * release threshold is unchanged;
          * tau/stop velocity/max duration are unchanged;
          * total logical coast distance is unchanged;
          * only event granularity changes.
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
        tau = self.kinetic_decay_ms
        step_raw = self.scroll_units_per_detent

        # Determine the same continuous-decay horizon used by the old
        # full-detent implementation.
        if self.kinetic_stop_velocity > 0.0:
            if speed <= self.kinetic_stop_velocity:
                return
            stop_horizon_ms = tau * math.log(
                speed / self.kinetic_stop_velocity
            )
            horizon_ms = min(self.kinetic_max_ms, stop_horizon_ms)
        else:
            # A zero stop velocity means decay only by the max-duration cap.
            horizon_ms = self.kinetic_max_ms
        if horizon_ms <= 0.0:
            return

        total_decay_raw = speed * tau * (
            1.0 - math.exp(-horizon_ms / tau)
        )

        # The legacy full-detent implementation emitted only complete detents.  Keep
        # exactly that total coast distance so smoothing alone cannot make a
        # flick travel farther.
        detent_count = int(total_decay_raw / step_raw)
        if detent_count <= 0:
            LOG.debug(
                "kinetic scroll not started: decay contains less than one "
                "complete scroll detent (%.1f < %.1f raw units)",
                total_decay_raw,
                step_raw,
            )
            return

        target_raw = detent_count * step_raw
        cancel = threading.Event()

        def worker():
            # Match roughly the K400 raw-report cadence.  This is intentionally
            # a time slice, not a fixed v120 quantum: fast coast motion gets
            # larger events while slow tail motion naturally becomes very fine.
            interval_ms = 8.0
            v = speed
            emitted_raw = 0.0
            target_v120 = detent_count * 120
            emitted_v120 = 0
            start_time = time.monotonic()
            last_time = start_time
            events = 0

            LOG.debug(
                "kinetic scroll started (hi-res): release=%.3f units/ms "
                "direction=%+d tau=%.0f ms target=%d detent(s) interval=%.1f ms",
                speed,
                direction,
                tau,
                detent_count,
                interval_ms,
            )

            try:
                while not cancel.is_set() and emitted_raw < target_raw:
                    elapsed_total_ms = (time.monotonic() - start_time) * 1000.0
                    if elapsed_total_ms >= horizon_ms:
                        break

                    remaining_horizon_ms = horizon_ms - elapsed_total_ms
                    wait_ms = min(interval_ms, remaining_horizon_ms)
                    if wait_ms <= 0.0:
                        break

                    if cancel.wait(wait_ms / 1000.0):
                        break

                    now = time.monotonic()
                    actual_dt_ms = (now - last_time) * 1000.0
                    last_time = now

                    # Do not integrate past the configured decay horizon even
                    # if the Python thread was scheduled late.
                    elapsed_before_ms = elapsed_total_ms
                    allowed_dt_ms = min(
                        actual_dt_ms,
                        max(0.0, horizon_ms - elapsed_before_ms),
                    )
                    if allowed_dt_ms <= 0.0:
                        break

                    decay = math.exp(-allowed_dt_ms / tau)
                    raw_displacement = v * tau * (1.0 - decay)
                    v *= decay

                    remaining_raw = target_raw - emitted_raw
                    raw_displacement = min(raw_displacement, remaining_raw)
                    if raw_displacement <= 0.0:
                        break

                    emitted_raw += raw_displacement
                    v120 = (
                        direction
                        * raw_displacement
                        * 120.0
                        / step_raw
                    )
                    wheel_hi, _ = self._emit_hires_scroll(v120, 0.0)
                    emitted_v120 += abs(wheel_hi)
                    if wheel_hi:
                        events += 1

                # Floating-point subdivision can otherwise leave a final
                # sub-unit residual (e.g. 479.999... -> 479).  The old
                # implementation's coast distance was an integer number of
                # detents, so flush exactly the remaining integer v120 units.
                remaining_v120 = target_v120 - emitted_v120
                if remaining_v120 > 0 and not cancel.is_set():
                    wheel_hi, _ = self._emit_hires_scroll(
                        direction * remaining_v120,
                        0.0,
                    )
                    emitted_v120 += abs(wheel_hi)
                    if wheel_hi:
                        events += 1

                LOG.debug(
                    "kinetic scroll ended (hi-res): events=%d "
                    "logical=%.2f detent(s) target=%d duration=%.0f ms",
                    events,
                    emitted_v120 / 120.0,
                    detent_count,
                    (time.monotonic() - start_time) * 1000.0,
                )
            finally:
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

        # A valid tap followed by a retouch is ambiguous until the second
        # contact either moves far enough to become a drag or is released as
        # the second click.  BTN_LEFT is already held from the first tap, so
        # there is deliberately no new button event here.  While unresolved,
        # process() suppresses only the small activation slop.
        if self._tap_drag_candidate:
            self.single_tap_candidate = False
            self.single_motion_grace_until_ns = 0

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

        LOG.debug(
            "one-finger tap candidate cancelled by movement: "
            "max=%.1f raw units threshold=%.1f",
            self.single_max_move,
            self.tap_move_units,
        )
        self.single_tap_candidate = False
        self._tap_drag_candidate = False
        return False

    def _finish_single(self, host_ns: int):
        if self._tap_drag_active:
            self._end_tap_drag("finger released")
            self.single_active = False
            self.single_tap_candidate = False
            self.single_motion_grace_until_ns = 0
            return

        if self._tap_drag_candidate:
            # The second contact was released without crossing the drag slop.
            # The first tap's BTN_LEFT is still held.  Complete that first
            # click now, then emit the second click only if the retouch itself
            # still qualifies as a tap.  Thus the desktop sees the second
            # press only on finger release, never while the user may still
            # turn the gesture into a drag.
            duration_ms = (host_ns - self.single_start_ns) / 1_000_000.0
            self._tap_drag_candidate = False
            self._release_tap_hold("tap retouch release completes first click")

            if duration_ms <= self.tap_max_ms:
                self._emit_click(
                    ecodes.BTN_LEFT,
                    "tap retouch released -> second BTN_LEFT click",
                )
            else:
                LOG.debug(
                    "tap retouch released after %.1f ms (> %.1f ms); "
                    "second click suppressed",
                    duration_ms,
                    self.tap_max_ms,
                )

            self.single_active = False
            self.single_tap_candidate = False
            self.single_motion_grace_until_ns = 0
            return

        valid_tap = False
        if self.single_active and self.single_tap_candidate:
            duration_ms = (host_ns - self.single_start_ns) / 1_000_000.0
            valid_tap = duration_ms <= self.tap_max_ms

        if valid_tap:
            # Mirror libinput's tap-and-drag state machine: emit the tap press
            # now but defer only its release for a short drag-detection window.
            # This is intentionally different from the old implementation that
            # deferred the entire first click.
            self._arm_tap_hold(
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
        self._cancel_tap_sequence("multi-touch began", release_button=True)
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
        self.scroll_v120_frac = 0.0
        self.hscroll_v120_frac = 0.0
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

        # Convert raw centroid displacement directly to Linux v120 wheel
        # units.  A full logical detent remains exactly the same physical
        # distance as before:
        #
        #     scroll_units_per_detent raw units == 120 v120 units
        #
        # but those 120 units are now emitted incrementally at the raw report
        # cadence instead of as one large jump.
        wheel_v120 = (
            wheel_delta * 120.0 / self.scroll_units_per_detent
            if wheel_delta
            else 0.0
        )

        hwheel_v120 = 0.0
        if self.horizontal_scroll:
            hdelta = dx
            if self.hscroll_invert:
                hdelta = -hdelta
            if hdelta:
                hwheel_v120 = (
                    hdelta * 120.0 / self.hscroll_units_per_detent
                )

        if wheel_v120 or hwheel_v120:
            self._emit_hires_scroll(wheel_v120, hwheel_v120)

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
        self.scroll_v120_frac = 0.0
        self.hscroll_v120_frac = 0.0
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

            if self._tap_drag_candidate:
                # The second contact after a tap is unresolved. BTN_LEFT is
                # already held from the first tap. Keep the pointer anchored
                # until it either:
                #   * releases inside the slop -> complete first click + second click
                #   * crosses the slop         -> continue held-button drag
                #
                # No second BTN_LEFT press occurs on finger-down or drag start,
                # so a desktop cannot fire its double-click action before drag.
                drag_dist = math.hypot(
                    touch.x - self.single_start_x,
                    touch.y - self.single_start_y,
                )
                if drag_dist <= self.tap_drag_activation_units:
                    self._reset_pointer(
                        touch,
                        frame.timestamp,
                        reset_fraction=True,
                    )
                    return

                if self._start_tap_drag():
                    self._reset_pointer(
                        touch,
                        frame.timestamp,
                        reset_fraction=True,
                    )
                    LOG.debug(
                        "tap-and-drag motion armed after %.1f raw units "
                        "(threshold=%.1f); BTN_LEFT remained held",
                        drag_dist,
                        self.tap_drag_activation_units,
                    )
                    return

            self._update_single_tap(touch)

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


class PointerBackend:
    mode = "pointer"

    def __init__(self, args):
        self.args = args
        self.ui = None
        self.engine = None
        self._logged_config = False

    def configure(self, info, invert_x: bool, invert_y: bool):
        if self.ui is None and not self.args.no_uinput:
            self.ui = make_pointer_uinput(self.args.pointer_name)
            path = self.ui.device.path if self.ui.device is not None else self.ui.devnode
            LOG.info("Created persistent virtual pointer: %s (%s)", self.args.pointer_name, path)
            time.sleep(0.25)

        timestamp_unit_ms = info.timestamp_units / 10.0
        if info.timestamp_units == 8:
            timestamp_unit_ms = 1.0

        gain_x = self.args.pointer_gain if self.args.pointer_gain_x is None else self.args.pointer_gain_x
        gain_y = self.args.pointer_gain if self.args.pointer_gain_y is None else self.args.pointer_gain_y

        self.engine = RawPointerEngine(
            gain_x=gain_x, gain_y=gain_y,
            invert_x=invert_x, invert_y=invert_y,
            max_raw_jump=self.args.pointer_max_raw_jump,
            ui=self.ui, print_raw=self.args.print_raw,
            timestamp_unit_ms=timestamp_unit_ms,
            precision_gain=self.args.pointer_precision_gain,
            precision_speed=self.args.pointer_precision_speed,
            precision_transition=self.args.pointer_precision_transition,
            precision_filter_ms=self.args.pointer_precision_filter_ms,
            tap_enabled=self.args.pointer_tap,
            two_finger_tap_enabled=self.args.pointer_two_finger_tap,
            tap_max_ms=self.args.pointer_tap_max_ms,
            tap_move_units=self.args.pointer_tap_move_units,
            scroll_enabled=self.args.pointer_scroll,
            scroll_start_units=self.args.pointer_scroll_start_units,
            scroll_units_per_detent=self.args.pointer_scroll_units_per_detent,
            scroll_invert=self.args.pointer_scroll_invert,
            horizontal_scroll=self.args.pointer_horizontal_scroll,
            hscroll_units_per_detent=self.args.pointer_hscroll_units_per_detent,
            hscroll_invert=self.args.pointer_hscroll_invert,
            multitouch_entry_grace_ms=self.args.pointer_multitouch_entry_grace_ms,
            multitouch_exit_grace_ms=self.args.pointer_multitouch_exit_grace_ms,
            tap_drag_window_ms=self.args.pointer_tap_drag_window_ms,
            tap_drag_activation_units=self.args.pointer_tap_drag_activation_units,
            scroll_axis_lock=self.args.pointer_scroll_axis_lock == "on",
            scroll_axis_lock_ratio=self.args.pointer_scroll_axis_lock_ratio,
            kinetic_scroll=self.args.pointer_kinetic_scroll,
            kinetic_history_ms=self.args.pointer_kinetic_history_ms,
            kinetic_start_velocity=self.args.pointer_kinetic_start_velocity,
            kinetic_stop_velocity=self.args.pointer_kinetic_stop_velocity,
            kinetic_decay_ms=self.args.pointer_kinetic_decay_ms,
            kinetic_max_ms=self.args.pointer_kinetic_max_ms,
        )

        if not self._logged_config:
            LOG.info(
                "Pointer mode: gain X=%.3f Y=%.3f; precision %.3f <= %.3f -> 1.0 at %.3f units/ms",
                gain_x, gain_y, self.args.pointer_precision_gain,
                self.args.pointer_precision_speed, self.args.pointer_precision_transition,
            )
            LOG.info(
                "Pointer tap press is immediate; release is held up to %.0f ms for tap-and-drag; retouch release becomes second click, motion becomes drag; drag slop %.1f raw units",
                self.args.pointer_tap_drag_window_ms, self.args.pointer_tap_drag_activation_units,
            )
            self._logged_config = True

    def process(self, frame: RawFrame, host_ns: int):
        if self.engine is not None:
            self.engine.process(frame, host_ns)

    def reset_runtime(self):
        if self.engine is not None:
            self.engine.reset_runtime()

    def close(self):
        self.reset_runtime()
        if self.ui is not None:
            self.ui.close()
            self.ui = None
