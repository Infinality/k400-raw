from __future__ import annotations

import logging
import struct
import time
import threading
from dataclasses import dataclass
from typing import Optional

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

LOG = logging.getLogger("k400-raw")

WPID_K400_PLUS = "404D"
RAW_FLAG = 0x01
ENHANCED_FLAG = 0x04
RAW_AND_NATIVE_FLAG = 0x10
RAW_ENHANCED = RAW_FLAG | ENHANCED_FLAG
RAW_ENHANCED_DUAL = RAW_ENHANCED | RAW_AND_NATIVE_FLAG
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
                candidate_devices = []
                try:
                    dev = r[slot]
                except Exception as exc:
                    LOG.debug(
                        "Receiver %s has no usable slot %d: %s",
                        dev_info.path,
                        slot,
                        exc,
                    )
                    dev = None
                if dev:
                    candidate_devices.append(dev)
            else:
                # Let Solaar enumerate the receiver's actual pairing count
                # rather than probing every possible slot up to max_devices.
                # Receiver.__iter__ stops after it has found count() paired
                # devices, which avoids noisy reads of trailing empty slots
                # such as slot 6 on a five-device Unifying receiver.
                try:
                    candidate_devices = list(r)
                except Exception as exc:
                    LOG.debug(
                        "Could not enumerate paired devices on receiver %s: %s",
                        dev_info.path,
                        exc,
                    )
                    candidate_devices = []

            for dev in candidate_devices:
                candidate_slot = int(dev.number)

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
        return (
            match["receiver"],
            match["device"],
            match["path"],
            match["slot"],
        )

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
