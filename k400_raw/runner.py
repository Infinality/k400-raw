from __future__ import annotations

import logging
import threading
import time

from .hidpp import (
    DeviceNotReady, StartedEventsListener, SupportedFeature, RAW_ENHANCED,
    configure_fn_mode, find_k400, origin_default_inversions, parse_frame,
    parse_info, set_raw_report_state, wait_for_feature,
)

LOG = logging.getLogger("k400-raw")


def run(args, backend, stop_event: threading.Event) -> int:
    explicit_slot = args.slot > 0
    preferred_receiver_path = args.receiver_path
    preferred_slot = args.slot if explicit_slot else None
    preferred_failures = 0
    full_scan_next = not explicit_slot and preferred_slot is None
    preferred_rescan_after = 3
    session_number = 0

    while not stop_event.is_set():
        session_number += 1
        r = dev = event_listener = None
        feature = SupportedFeature.TOUCHPAD_RAW_XY
        feature_index = None
        old_raw_state = None
        raw_enabled = False
        reconfigure_event = threading.Event()
        reconfigure_failures = 0

        try:
            use_cached_endpoint = (
                not explicit_slot and preferred_slot is not None and not full_scan_next
            )
            if explicit_slot:
                discovery_slot = args.slot
                discovery_path = args.receiver_path
                reason = "explicit endpoint"
            elif use_cached_endpoint:
                discovery_slot = preferred_slot
                discovery_path = preferred_receiver_path
                reason = "cached endpoint"
            else:
                discovery_slot = 0
                discovery_path = args.receiver_path
                reason = "full auto-scan"
                full_scan_next = False

            LOG.debug(
                "Opening HID++ session #%d via %s (receiver=%s slot=%s)",
                session_number, reason, discovery_path or "auto", discovery_slot or "auto",
            )
            r, dev, selected_receiver_path, selected_slot = find_k400(
                discovery_slot, args.wpid, discovery_path
            )
            feature_index = wait_for_feature(
                dev, feature, timeout_seconds=args.feature_wait_seconds
            )

            if not explicit_slot:
                preferred_receiver_path = selected_receiver_path
                preferred_slot = selected_slot
                preferred_failures = 0

            info = parse_info(dev.feature_request(feature, 0x00))
            default_ix, default_iy = origin_default_inversions(info.origin)
            invert_x = default_ix ^ args.invert_x
            invert_y = default_iy ^ args.invert_y

            LOG.info(
                "K400 raw surface: %dx%d, dpi=%d, origin=%d, max_fingers=%d",
                info.x_size, info.y_size, info.dpi, info.origin, info.max_fingers,
            )
            backend.configure(info, invert_x, invert_y)

            def callback(n):
                nonlocal feature_index
                if n.devnumber != dev.number:
                    return

                if n.sub_id == 0x41:
                    flags = (n.data[0] & 0xF0) if n.data else 0x40
                    link_established = not bool(flags & 0x40)
                    dev.online = link_established
                    backend.reset_runtime()
                    if link_established:
                        LOG.debug("K400 link established; scheduling raw-mode reinitialization")
                        reconfigure_event.set()
                    else:
                        LOG.debug("K400 asleep/offline")
                    return

                if n.sub_id != feature_index or (n.address >> 4) != 0x00:
                    return
                frame = parse_frame(n.data)
                if frame is None:
                    LOG.warning("Short raw XY notification: %r", n.data)
                    return
                backend.process(frame, time.monotonic_ns())

            event_listener = StartedEventsListener(r, callback)
            event_listener.start()
            if not event_listener.started_event.wait(timeout=3.0):
                raise RuntimeError("Timed out starting HID++ receiver listener")

            state_reply = dev.feature_request(feature, 0x10)
            old_raw_state = state_reply[0] if state_reply else 0x00
            actual_state = set_raw_report_state(dev, feature, RAW_ENHANCED)
            raw_enabled = True
            configure_fn_mode(dev, args.fn_mode, args.feature_wait_seconds)

            LOG.info(
                "K400 %s mode active: receiver=%s slot=%d raw_state=0x%02X",
                backend.mode, selected_receiver_path, selected_slot, actual_state,
            )

            while not stop_event.is_set():
                if not event_listener.is_alive():
                    raise ConnectionError("HID++ receiver listener stopped")
                if not reconfigure_event.wait(timeout=1.0):
                    continue
                reconfigure_event.clear()
                if stop_event.is_set():
                    break

                backend.reset_runtime()
                try:
                    feature_index = wait_for_feature(
                        dev, feature, timeout_seconds=min(args.feature_wait_seconds, 4.0)
                    )
                    actual_state = set_raw_report_state(dev, feature, RAW_ENHANCED)
                    raw_enabled = True
                    configure_fn_mode(dev, args.fn_mode, args.feature_wait_seconds)
                    reconfigure_failures = 0
                    LOG.info("K400 raw mode re-enabled after wake (0x%02X)", actual_state)
                except DeviceNotReady as exc:
                    reconfigure_failures = 0
                    LOG.debug(
                        "K400 went offline during reinitialization (%s); waiting for next link-up", exc
                    )
                except Exception as exc:
                    reconfigure_failures += 1
                    if not event_listener.is_alive() or reconfigure_failures >= 3:
                        raise ConnectionError("Repeated raw-mode reinitialization failure") from exc
                    LOG.debug(
                        "Transient reinitialization failure %d/3: %s",
                        reconfigure_failures, exc,
                    )
                    if stop_event.wait(args.reconnect_delay_seconds):
                        break
                    reconfigure_event.set()

        except DeviceNotReady as exc:
            if not explicit_slot and preferred_slot is not None:
                preferred_failures += 1
                if preferred_failures >= preferred_rescan_after:
                    full_scan_next = True
                    preferred_failures = 0
            LOG.debug("K400 not ready (%s); retrying", exc)
        except Exception as exc:
            if stop_event.is_set():
                break
            if not explicit_slot and preferred_slot is not None:
                if "Could not find Logitech K400 Plus WPID" in str(exc):
                    full_scan_next = True
                else:
                    preferred_failures += 1
                    if preferred_failures >= preferred_rescan_after:
                        full_scan_next = True
                        preferred_failures = 0
            LOG.warning(
                "K400 HID++ session #%d lost (%s). Retrying in %.1f s.",
                session_number, exc, args.reconnect_delay_seconds,
                exc_info=LOG.isEnabledFor(logging.DEBUG),
            )
        finally:
            try:
                backend.reset_runtime()
            except Exception:
                LOG.debug("Error resetting backend runtime", exc_info=True)

            if stop_event.is_set() and dev is not None and raw_enabled:
                try:
                    restore = 0x00 if old_raw_state is None else old_raw_state
                    dev.feature_request(feature, 0x20, bytes([restore]))
                    LOG.info("Restored HID++ raw-report state to 0x%02X", restore)
                except Exception:
                    LOG.warning("Could not restore raw-report state during shutdown")

            if event_listener is not None:
                try:
                    event_listener.stop(); event_listener.join(timeout=2.0)
                except Exception:
                    LOG.debug("Error stopping receiver listener", exc_info=True)
            if r is not None:
                try:
                    r.close()
                except Exception:
                    LOG.debug("Error closing receiver", exc_info=True)

        if not stop_event.is_set():
            stop_event.wait(args.reconnect_delay_seconds)

    return 0
