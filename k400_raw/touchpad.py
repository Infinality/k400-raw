from __future__ import annotations

import logging
import time
from evdev import AbsInfo, UInput, ecodes

from .hidpp import active_touches

LOG = logging.getLogger("k400-raw")

def make_absinfo(minimum: int, maximum: int, resolution: int = 0) -> AbsInfo:
    return AbsInfo(
        value=minimum,
        min=minimum,
        max=maximum,
        fuzz=0,
        flat=0,
        resolution=resolution,
    )

def make_touchpad_uinput(
    name: str,
    x_max: int,
    y_max: int,
    resolution: int,
) -> UInput:
    """Create a two-slot Protocol-B absolute multitouch touchpad."""

    capabilities = {
        ecodes.EV_KEY: [
            ecodes.BTN_TOUCH,
            ecodes.BTN_TOOL_FINGER,
            ecodes.BTN_TOOL_DOUBLETAP,
        ],
        ecodes.EV_ABS: [
            (ecodes.ABS_X, make_absinfo(0, x_max, resolution)),
            (ecodes.ABS_Y, make_absinfo(0, y_max, resolution)),
            (ecodes.ABS_MT_SLOT, make_absinfo(0, 1, 0)),
            (ecodes.ABS_MT_TRACKING_ID, make_absinfo(0, 65535, 0)),
            (ecodes.ABS_MT_POSITION_X, make_absinfo(0, x_max, resolution)),
            (ecodes.ABS_MT_POSITION_Y, make_absinfo(0, y_max, resolution)),
        ],
    }

    return UInput(
        capabilities,
        name=name,
        # Use Logitech's vendor ID but a deliberately synthetic product ID.
        # This avoids pretending the virtual device is a real Logitech USB
        # product while still making its origin obvious in diagnostics.
        vendor=0x046D,
        product=0xF400,
        version=1,
        bustype=ecodes.BUS_USB,
        phys="k400-raw-touchpad/input0",
        # Absolute trackpads are indirect pointing devices, not touchscreens.
        input_props=[ecodes.INPUT_PROP_POINTER],
    )

class RawTouchpadBridge:
    """Translate K400 raw contacts into Linux Type-B MT events."""

    def __init__(
        self,
        ui: UInput,
        x_max: int,
        y_max: int,
        invert_x: bool,
        invert_y: bool,
        print_raw: bool = False,
    ):
        self.ui = ui
        self.x_max = x_max
        self.y_max = y_max
        self.invert_x = invert_x
        self.invert_y = invert_y
        self.print_raw = print_raw

        self.finger_to_slot: dict[int, int] = {}
        self.slot_to_finger: dict[int, int] = {}
        self.slot_xy: dict[int, tuple[int, int]] = {}
        self.next_tracking_id = 1

        self.last_btn_touch = 0
        self.last_tool_finger = 0
        self.last_tool_doubletap = 0

    def _xy(self, touch) -> tuple[int, int]:
        x = int(touch.x)
        y = int(touch.y)
        if self.invert_x:
            x = self.x_max - x
        if self.invert_y:
            y = self.y_max - y
        x = max(0, min(self.x_max, x))
        y = max(0, min(self.y_max, y))
        return x, y

    def _new_tracking_id(self) -> int:
        tid = self.next_tracking_id
        self.next_tracking_id += 1
        if self.next_tracking_id > 65535:
            self.next_tracking_id = 1
        return tid

    def release_all(self):
        changed = False

        for slot in sorted(self.slot_to_finger):
            self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_SLOT, slot)
            self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_TRACKING_ID, -1)
            changed = True

        self.finger_to_slot.clear()
        self.slot_to_finger.clear()
        self.slot_xy.clear()

        if self.last_btn_touch:
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOUCH, 0)
            changed = True
        if self.last_tool_finger:
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOOL_FINGER, 0)
            changed = True
        if self.last_tool_doubletap:
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOOL_DOUBLETAP, 0)
            changed = True

        self.last_btn_touch = 0
        self.last_tool_finger = 0
        self.last_tool_doubletap = 0

        if changed:
            self.ui.syn()

    def process(self, frame):
        if frame.spurious:
            return

        touches = active_touches(frame)
        # The K400 feature reports at most two contacts.  If malformed data
        # ever contains duplicate IDs, keep only the first occurrence.
        current = {}
        for touch in touches:
            if touch.finger_id and touch.finger_id not in current:
                current[touch.finger_id] = touch

        if self.print_raw:
            LOG.debug(
                "raw ts=%d count=%d contacts=%s",
                frame.timestamp,
                len(current),
                [
                    (fid, touch.x, touch.y, touch.contact_status)
                    for fid, touch in current.items()
                ],
            )

        old_ids = set(self.finger_to_slot)
        new_ids = set(current)

        # Release vanished contacts first so a slot can be reused in this frame.
        for finger_id in sorted(old_ids - new_ids):
            slot = self.finger_to_slot.pop(finger_id)
            self.slot_to_finger.pop(slot, None)
            self.slot_xy.pop(slot, None)
            self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_SLOT, slot)
            self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_TRACKING_ID, -1)

        # Allocate slots to newly appearing contacts.
        for finger_id in sorted(new_ids - old_ids):
            free = next((s for s in (0, 1) if s not in self.slot_to_finger), None)
            if free is None:
                LOG.warning(
                    "More active contacts than available MT slots; ignoring finger id %d",
                    finger_id,
                )
                continue

            self.finger_to_slot[finger_id] = free
            self.slot_to_finger[free] = finger_id

            self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_SLOT, free)
            self.ui.write(
                ecodes.EV_ABS,
                ecodes.ABS_MT_TRACKING_ID,
                self._new_tracking_id(),
            )

        # Emit current coordinates for every tracked contact. Sending each
        # coordinate every frame keeps the bridge transparent and lets
        # libinput see the raw sensor report cadence.
        for finger_id, slot in sorted(self.finger_to_slot.items(), key=lambda p: p[1]):
            touch = current.get(finger_id)
            if touch is None:
                continue
            x, y = self._xy(touch)
            self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_SLOT, slot)
            self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_POSITION_X, x)
            self.ui.write(ecodes.EV_ABS, ecodes.ABS_MT_POSITION_Y, y)
            self.slot_xy[slot] = (x, y)

        count = len(self.slot_to_finger)

        # Legacy single-touch axes are required for touchpads even when the
        # MT protocol is available.  Mirror the lowest-numbered active slot.
        if count:
            primary_slot = min(self.slot_to_finger)
            xy = self.slot_xy.get(primary_slot)
            if xy is not None:
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_X, xy[0])
                self.ui.write(ecodes.EV_ABS, ecodes.ABS_Y, xy[1])

        btn_touch = 1 if count else 0
        tool_finger = 1 if count == 1 else 0
        tool_doubletap = 1 if count >= 2 else 0

        if btn_touch != self.last_btn_touch:
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOUCH, btn_touch)
            self.last_btn_touch = btn_touch
        if tool_finger != self.last_tool_finger:
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOOL_FINGER, tool_finger)
            self.last_tool_finger = tool_finger
        if tool_doubletap != self.last_tool_doubletap:
            self.ui.write(ecodes.EV_KEY, ecodes.BTN_TOOL_DOUBLETAP, tool_doubletap)
            self.last_tool_doubletap = tool_doubletap

        self.ui.syn()


class TouchpadBackend:
    mode = "touchpad"

    def __init__(self, args):
        self.args = args
        self.ui = None
        self.bridge = None
        self.geometry = None

    def configure(self, info, invert_x: bool, invert_y: bool):
        resolution = (
            self.args.touchpad_resolution_units_per_mm
            if self.args.touchpad_resolution_units_per_mm > 0
            else max(1, int(round(info.dpi / 25.4)))
        )
        geometry = (info.x_size, info.y_size, resolution)

        if self.ui is None:
            self.ui = make_touchpad_uinput(
                self.args.touchpad_name, info.x_size, info.y_size, resolution
            )
            self.geometry = geometry
            path = self.ui.device.path if self.ui.device is not None else self.ui.devnode
            LOG.info(
                "Created persistent virtual touchpad: %s (%s), %dx%d, %d units/mm (~%.1fx%.1f mm)",
                self.args.touchpad_name, path, info.x_size, info.y_size, resolution,
                info.x_size / resolution, info.y_size / resolution,
            )
            time.sleep(0.5)
        elif geometry != self.geometry:
            LOG.warning(
                "K400 geometry changed after reconnect: was %s now %s; keeping persistent virtual geometry",
                self.geometry, geometry,
            )

        self.bridge = RawTouchpadBridge(
            ui=self.ui,
            x_max=self.geometry[0], y_max=self.geometry[1],
            invert_x=invert_x, invert_y=invert_y, print_raw=self.args.print_raw,
        )

    def process(self, frame, host_ns: int):
        if self.bridge is not None:
            self.bridge.process(frame)

    def reset_runtime(self):
        if self.bridge is not None:
            self.bridge.release_all()

    def close(self):
        self.reset_runtime()
        if self.ui is not None:
            self.ui.close()
            self.ui = None
