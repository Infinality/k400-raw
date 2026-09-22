# Logitech K400 Plus Raw Touch Bridge

Linux userspace bridge for the **Logitech Wireless Touch Keyboard K400 Plus** (WPID `404D`) using Logitech HID++ `TOUCHPAD_RAW_XY` feature `0x6100`.

The K400 Plus exposes substantially finer raw touch coordinates than its normal firmware-generated `REL_X/REL_Y` mouse stream. This project consumes that raw surface directly and offers two backends:

- **`touchpad` (default/recommended):** Exposes a genuine two-contact Linux multitouch touchpad and lets libinput/KDE/Gnome handle motion, tapping, scrolling and gestures.  Enables configuration in the KDE/Gnome/libinput UI touchpad settings.

- **`pointer` (advanced):** Converts the same raw coordinates into a custom relative pointer with tunable low-speed precision, reconstructed taps/scrolling, and stock-style kinetic wheel coasting and custom gains that can exceed KDE/Gnome default maximums.  Emits high resolution scroll events along with regular scroll events, allowing for fine touchpad-like scrolling movement.  Requires custom configuration in the included sysconfig configuration file but defaults to the touchpad-like values.  The KDE/Gnome **touchpad** configuration UI **WILL NOT WORK**.  The **mouse** configuration UI will work, however it's recommended to keep it set to default speed (0.0) and disable acceleration, only editing the settings in the sysconfig file.

The default touchpad backend is intentionally thin. It does not apply a private gain or duplicate libinput gesture policy.

## Why

The stock K400 Plus mouse path is already quantized before Linux receives it. Increasing speed later cannot recover the lost fine movement. HID++ `0x6100` reports the underlying touch sensor at roughly 1390 DPI (~55 units/mm), including genuine one-unit coordinate changes.

In `touchpad` mode that resolution is exposed directly to libinput. On the tested K400 this provides one-pixel fine positioning even at KDE's maximum touchpad speed while also enabling the native Touchpad settings UI.

## Repository layout

```text
k400-raw.py                 single executable / backend selector
k400_raw/                   Python package used by k400-raw.py
    __init__.py             package marker / version
    hidpp.py                shared receiver, discovery, reconnect and raw parser
    runner.py               shared HID++ session / reconnect loop
    touchpad.py             thin Type-B multitouch backend
    pointer.py              custom relative-pointer/gesture backend
k400-raw.service            one systemd service for either mode
k400-raw.sysconfig          Fedora/RHEL configuration
99-k400-raw-touchpad.rules  optional classification fallback
```

## Requirements

- Linux with `hidraw` and `uinput`
- Python 3.9+
- `python-evdev`
- Solaar providing `logitech_receiver` (tested with Solaar 1.1.20)
- systemd for the supplied service

Fedora:

```bash
sudo dnf install solaar python3-evdev
```

The Solaar **package** is required; a continuously running Solaar GUI/tray process is not recommended because both programs access the same receiver and Solaar can reapply device settings on reconnect.  I have accidentally had it running and didn't encounter issues, but it's not recommended to run.

## Install the service

Install the tree. Run these commands from the **extracted repository root**, where both `k400-raw.py` and the `k400_raw/` directory are present:

```bash
sudo install -d /usr/local/libexec/k400-raw
sudo install -m 0755 k400-raw.py /usr/local/libexec/k400-raw/k400-raw.py
sudo cp -a k400_raw /usr/local/libexec/k400-raw/
sudo install -m 0644 k400-raw.service /etc/systemd/system/k400-raw.service
sudo install -m 0644 k400-raw.sysconfig /etc/sysconfig/k400-raw
sudo systemctl daemon-reload
sudo systemctl enable --now k400-raw.service
```

The default sysconfig selects `K400_MODE=touchpad`. To set to the advanced custom pointer backend, edit `/etc/sysconfig/k400-raw`, set `K400_MODE=pointer`, and restart the same service.


`K400_SLOT=0` auto-detects the active K400 Plus and caches the last endpoint that successfully exposes `0x6100`. A positive slot remains available as a manual override.

## Touchpad mode (default)

```ini
K400_MODE=touchpad
```

The virtual device is named `Logitech K400 Raw Touchpad` and exposes:

- `ABS_X`, `ABS_Y`
- `ABS_MT_SLOT`, `ABS_MT_TRACKING_ID`
- `ABS_MT_POSITION_X`, `ABS_MT_POSITION_Y`
- `BTN_TOUCH`, `BTN_TOOL_FINGER`, `BTN_TOOL_DOUBLETAP`
- `INPUT_PROP_POINTER`

The raw DPI field is converted to Linux absolute-axis resolution in units/mm, so the tested 1390-DPI K400 is advertised at ~55 units/mm and ~65x37 mm.

Current systemd/udev should automatically classify it as `ID_INPUT_TOUCHPAD=1`. The included udev rule is only a fallback if automatic classification fails.

### Desktop settings

Use the normal Touchpad settings in KDE/GNOME. On KDE Plasma the tested device exposes native controls for tap-to-click, tap-and-drag, drag lock, two-finger/edge scrolling, natural scrolling, pointer speed, and acceleration profile.

### Touchpad-mode limitations

- K400 raw hardware reports at most **two contacts**, so genuine 3+ finger gestures cannot be exposed.
- The raw interface does not provide useful pressure/contact-area ranges on the tested hardware.
- Kinetic/coasting finger scrolling is application/toolkit policy. Chromium-based applications commonly coast; many other applications stop when the fingers lift. The daemon deliberately does not inject wheel inertia because that would mix finger-scroll and wheel semantics and can double-coast applications that already implement kinetic scrolling.

## Pointer mode (advanced)

```ini
K400_MODE=pointer
```

Pointer mode has more configurable custom behavior:

- constant raw-coordinate gain;
- configurable low-speed precision multiplier;
- fractional accumulation;
- tap-to-left-click and two-finger right-click;
- tap-and-drag;
- two-finger wheel scrolling;
- optional horizontal scrolling/axis lock;
- stock-style post-release kinetic wheel coasting;
- clean asynchronous one-/two-finger transition handling.

For pointer mode, configure the virtual `Logitech K400 Raw Pointer` as Flat/no acceleration and neutral downstream speed if you want the daemon's curve without a second acceleration layer.

### Tap behavior in pointer mode

A following touch inside `K400_POINTER_TAP_DRAG_WINDOW_MS` immediately presses and holds `BTN_LEFT`:

- release without dragging -> ordinary second click / desktop double-click recognition;
- move -> tap-and-drag.

A small `K400_POINTER_TAP_DRAG_ACTIVATION_UNITS` deadzone suppresses tiny second-touch jitter while the button is held so a double-click is not accidentally converted into a microscopic drag. This deadzone applies only to the post-tap held contact, not ordinary pointer movement.

This behavior is intentionally close to native libinput tap-and-drag semantics.

## Shared reconnect behavior

Both modes use the same HID++ core. It:

- auto-discovers the active WPID `404D` pairing;
- prefers a candidate that is online and exposes `TOUCHPAD_RAW_XY`;
- caches the proven receiver/slot so transient failures do not rescan every pairing;
- treats normal K400 sleep as a link-down state rather than a failed session;
- reapplies raw mode and F-key mode on real wake/link-up;
- recovers after K400 power-cycle and receiver unplug/replug;
- keeps the virtual input device alive across HID++ reconnects.

On clean shutdown the prior HID++ raw-report state is restored.

## Logging

```ini
K400_LOG_LEVEL=warning
```

Available: `warning`, `info`, `debug`. Even at daemon DEBUG level, Solaar/hidapi transport packet logging remains at WARNING so journald is not flooded with every HID++ frame.

## F-key mode

```ini
K400_FN_MODE=standard
```

- `standard`: F1-F12 primary, Fn for special actions
- `special`: special/media actions primary
- `leave`: do not change the device

## Physical buttons

The K400's physical left/right buttons remain on the original kernel input device. They are not grabbed. Both KDE/KWin and X11/libinput combine their events with virtual pointer/touchpad motion correctly at the seat level on the tested system.

## Other distributions

The code is not Fedora-specific. Install current Solaar and python-evdev using your distribution packages. If your distro uses `/etc/default` rather than `/etc/sysconfig`, adjust the service's `EnvironmentFile=` path.

Enterprise distributions may ship older Solaar builds; Solaar 1.1.20 is the tested baseline. The project imports Solaar's internal `logitech_receiver` modules, so future Solaar internal API changes may require updates.

## Disclaimer

Not affiliated with or endorsed by Logitech or the Solaar project. This software changes HID++ device state while running; use at your own risk.  This project was developed with substantial use of AI tools.
