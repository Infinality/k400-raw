# Logitech K400 Plus Raw Pointer Daemon

A Linux userspace pointer/gesture daemon for the **Logitech Wireless Touch Keyboard K400 Plus** that bypasses the K400's normal quantized relative-pointer path and instead consumes the touchpad's HID++ raw coordinate stream.

The daemon converts the K400 Plus touch sensor's raw absolute coordinates into a high-resolution relative pointer, then reconstructs the useful stock touchpad behaviors in userspace: tap-to-click, two-finger right-click, tap-and-drag, two-finger scrolling, scroll coasting, and clean one-/two-finger transitions.

The result is especially useful when you want a **fast constant pointer speed without sacrificing fine positioning**.

This project is intentionally hardware-specific. It was developed and tested with a **K400 Plus, Logitech WPID `404D`, using HID++ `TOUCHPAD_RAW_XY` feature `0x6100`**.

> **Status:** Experimental, but intended for normal daily use on the tested K400 Plus. Review the caveats below before installing it as a system service.

---

## Why this exists

The stock K400 Plus Linux pointer path works, but it has an important limitation if you prefer a high, constant pointer speed.

The normal K400 firmware presents already-processed integer relative motion (`REL_X` / `REL_Y`). At low finger speeds, some of the original sensor resolution has already been quantized or discarded before Linux sees it. Multiplying those relative counts later — with X11 transformation matrices, `evsieve`, compositor sensitivity, or similar mechanisms — can make the pointer fast, but it cannot recover the missing fine motion.

That produces a familiar tradeoff:

- low gain: fine positioning is possible, but traversing a large/high-DPI display is slow;
- high constant gain: the pointer travels the desired distance, but the smallest movement can become several screen pixels;
- adaptive acceleration: restores fine motion, but changes gain with velocity, which some users do not want.

The K400 Plus also exposes the underlying touch sensor through Logitech HID++ feature `0x6100` (`TOUCHPAD_RAW_XY`). In raw mode, the sensor reports much finer absolute coordinates before the normal firmware pointer quantization.

This daemon uses that raw stream and preserves fractional motion internally, allowing a high overall pointer speed with smooth fine positioning.

### What the daemon adds

Compared with simply scaling the stock relative pointer stream, the daemon provides:

- raw sensor-coordinate input rather than already-quantized `REL_X` / `REL_Y`;
- fractional accumulation, so sub-unit gain is preserved across reports;
- constant base gain;
- an optional low-speed precision/deceleration region without generic adaptive acceleration;
- one-finger tap-to-left-click;
- two-finger tap-to-right-click;
- double-tap recognition;
- tap-and-drag;
- two-finger vertical scrolling;
- optional horizontal scrolling and scroll-axis locking;
- short multi-touch entry/exit grace periods to tolerate fingers landing or lifting on different reports;
- seamless two-finger-scroll -> one-finger-pointer handoff;
- stock-style post-release scroll coasting;
- legacy and high-resolution wheel events (`REL_WHEEL` + `REL_WHEEL_HI_RES`);
- automatic recovery after K400 power-cycle or Unifying receiver unplug/replug;
- optional enforcement of normal F1-F12 behavior.

The virtual pointer works below the desktop protocol layer, through Linux `uinput`, so the same daemon can be used under both **Wayland and X11**.

---

## Stock behavior that is intentionally reproduced

Several behaviors in this project were derived from observed stock K400 Plus behavior rather than invented from scratch.

### Low-speed pointer behavior

The raw HID++ stream exposes much finer movement than the normal relative-pointer path. The daemon preserves that raw granularity and applies its own configurable gain curve.

The default public configuration currently uses:

```ini
K400_GAIN=1.0
K400_PRECISION_SPEED=4.0
K400_PRECISION_TRANSITION=5.0
K400_PRECISION_GAIN=0.6
```

At sufficiently high raw speed, the multiplier is exactly the base gain (`1.0`). The precision multiplier is only used below the transition range.

This is not intended to be traditional mouse acceleration. It is a bounded low-speed precision region designed to suppress small finger-stick/jump artifacts while leaving ordinary and fast movement at a constant gain.

### Scroll coasting / flicking

The normal K400 Plus firmware was observed with `evtest` to continue emitting wheel events after both fingers leave the touchpad when the fingers are moving sufficiently fast at release.

The stock event stream uses complete wheel detents:

```text
REL_WHEEL         -1
REL_WHEEL_HI_RES  -120
SYN_REPORT
```

The magnitude remains constant while the interval between events grows as the scroll slows down.

Very slow releases produce no coast. Faster releases produce progressively longer tails, from roughly half a second near the onset threshold to around 1.5 seconds for a strong flick on the tested device.

The daemon reproduces that model from the recent raw two-finger centroid velocity instead of relying on browser-specific scrolling animation.

---

## Files

The repository is expected to contain:

```text
k400-raw-pointer.py
k400-raw-pointer.service
k400-raw-pointer.sysconfig
README.md
```

The filenames in the repository are intentionally unversioned. Use Git tags/releases for project versions.

---

## Hardware scope

### Tested

- Logitech Wireless Touch Keyboard **K400 Plus**
- Logitech WPID **`404D`**
- Logitech HID++ 2.0
- `TOUCHPAD_RAW_XY` feature **`0x6100`**
- Logitech Unifying receiver
- Fedora Linux
- KDE Plasma / Wayland
- X11-compatible through the same Linux `uinput` path

### Not a generic Logitech touchpad driver

The daemon currently assumes the K400 Plus raw-report format and identifies the device by WPID `404D`.

Other Logitech devices that expose HID++ `0x6100` may be similar, but they are **untested**. Do not assume that another K400 revision, standalone Logitech touchpad, Bolt device, Bluetooth device, or unrelated HID++ peripheral will work without changes.

The daemon has a `--wpid` override for development/testing, but using it does not guarantee protocol compatibility.

---

## Requirements

The runtime dependencies are:

- Linux with `hidraw` support;
- Linux `uinput`;
- Python 3.9 or newer;
- `python-evdev`;
- a recent Solaar installation that provides the `logitech_receiver` Python modules;
- systemd for the supplied service file.

The daemon does **not** shell out to the `solaar` CLI. It imports Solaar's Python modules and uses Solaar's HID++ receiver/device transport directly.

### Tested Solaar version

The current code is tested with **Solaar 1.1.20**.

Newer Solaar releases will likely work as long as the internal `logitech_receiver` APIs used by the daemon remain compatible, but those modules should be considered an implementation dependency rather than a guaranteed stable public API.

If a future Solaar update breaks the daemon, check this dependency first.

---

## Fedora installation

On a current Fedora system:

```bash
sudo dnf install solaar python3-evdev
```

The Fedora Solaar package installs both the `solaar` application and the Python modules required by this daemon.

You can verify the Python dependencies with:

```bash
/usr/bin/python3 -c 'import evdev; from logitech_receiver import base, listener, receiver; print("dependencies OK")'
```

If `/dev/uinput` is unavailable, verify that the `uinput` kernel module is available:

```bash
sudo modprobe uinput
```

---

## RHEL / AlmaLinux / Rocky Linux notes

The supplied service and `/etc/sysconfig` layout are also appropriate for RHEL-family systems.

However, enterprise/EPEL repositories may ship a substantially older Solaar release than Fedora. This project is tested with Solaar 1.1.20, so check:

```bash
solaar --version
```

If the packaged version is significantly older, install a current Solaar release using an appropriate method for your system.

If Solaar is installed into a virtual environment rather than the system Python, update the systemd service so `ExecStart` uses that environment's Python interpreter.

For example:

```ini
ExecStart=/opt/k400-raw-pointer-venv/bin/python /usr/local/libexec/k400-raw-pointer.py ...
```

The important requirement is that the Python interpreter running the daemon can import both:

```python
evdev
logitech_receiver
```

---

## Other distributions

The daemon itself is not Fedora-specific.

On Debian/Ubuntu, Arch, openSUSE, and other distributions, install:

- a recent Solaar package;
- the Python `evdev` package;
- Python 3.9+;
- systemd if you want to use the supplied service.

Package names vary by distribution.

The supplied service expects:

```text
/etc/sysconfig/k400-raw-pointer
```

If your distribution conventionally uses `/etc/default`, either keep `/etc/sysconfig` for this service or change:

```ini
EnvironmentFile=/etc/sysconfig/k400-raw-pointer
```

to your preferred path.

---

## Important: Solaar package vs. Solaar process

This project **requires the Solaar package**, but it does **not** require the Solaar GUI/tray application to be running.

These are different things:

```text
Solaar package installed:       required
Solaar Python modules present:  required
Solaar GUI/tray running:        not recommended
```

### Why running both is discouraged

The daemon and Solaar can both open the same Logitech receiver and issue HID++ requests.

A continuously running Solaar process also maintains cached device state and can reapply saved settings when a Logitech device comes online. That means a persistent Solaar GUI/tray process and this daemon can race with one another over receiver notifications or device configuration.

They may appear to coexist successfully for some operations, but concurrent use is not considered a supported configuration.

### Recommended setup

Install Solaar, but disable its desktop autostart/tray process while this daemon is in normal use.

If Solaar is already running:

```bash
pkill -x solaar
```

Disable its autostart through your desktop environment's autostart settings.

Do **not** uninstall the Solaar package; the daemon needs its Python modules.

### Using Solaar for configuration, pairing, or diagnostics

For the safest behavior, stop the daemon before using Solaar interactively:

```bash
sudo systemctl stop k400-raw-pointer.service
solaar
```

After exiting Solaar:

```bash
sudo systemctl start k400-raw-pointer.service
```

This is especially recommended for:

- pairing/unpairing;
- changing Logitech device settings;
- writing configuration with `solaar config`;
- firmware/device-management operations.

A short read-only command such as `solaar show` may work while the daemon is active, but concurrent receiver access is not a supported or necessary operating mode.

### Function-key behavior

One common reason to leave Solaar running with a K400 Plus is to make F1-F12 behave as ordinary function keys rather than media/special keys.

The daemon can set that directly.

The supplied configuration uses:

```ini
K400_FN_MODE=standard
```

Available modes are:

```text
standard  F1-F12 are primary; hold Fn for special actions
special   media/special actions are primary; hold Fn for F1-F12
leave     do not change the device's current Fn mode
```

The setting is reapplied after a K400 reconnect/power-cycle.

---

## Installation

### 1. Stop conflicting pointer workarounds

Disable any older K400 input-remapping service that would process the same device.

For example, if you previously used an `evsieve` service:

```bash
sudo systemctl disable --now evsieve-k400.service
```

Also quit the running Solaar GUI/tray process.

### 2. Install the daemon

From the repository directory:

```bash
sudo install -Dm0755 \
    k400-raw-pointer.py \
    /usr/local/libexec/k400-raw-pointer.py
```

### 3. Install the systemd service

```bash
sudo install -Dm0644 \
    k400-raw-pointer.service \
    /etc/systemd/system/k400-raw-pointer.service
```

### 4. Install the configuration file

```bash
sudo install -Dm0644 \
    k400-raw-pointer.sysconfig \
    /etc/sysconfig/k400-raw-pointer
```

### 5. Device auto-detection

The daemon automatically scans paired device slots on detected Logitech receivers and looks for the expected K400 Plus WPID:

```text
404D
```

The supplied configuration therefore uses:

```ini
K400_SLOT=0
```

where `0` means **auto-scan**.

Auto-detection does not assume that every paired `404D` record is the keyboard currently in use. A receiver can retain multiple same-model pairings, including old/offline K400 Plus devices. The daemon prefers a candidate that is currently responding and, when necessary, probes for the exact HID++ `TOUCHPAD_RAW_XY` feature (`0x6100`) required by this project.

Selection priority is therefore approximately:

```text
online WPID 404D + exposes 0x6100
        ↓
only responding WPID 404D candidate
        ↓
sole paired WPID 404D candidate (may currently be asleep)
```

If several same-model pairings exist but none is currently responding, the daemon waits quietly until the active K400 wakes instead of forcing a slot choice.

If more than one *responding/usable* K400 Plus remains genuinely ambiguous, the daemon refuses to guess. Select one explicitly with `--receiver-path`, `--slot`, or both.

You can inspect receiver pairings with:

```bash
solaar show
```

A positive slot number remains available as an override:

```ini
K400_SLOT=3
```

### 6. Enable the service

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now k400-raw-pointer.service
```

Check status:

```bash
systemctl status k400-raw-pointer.service
```

Follow logs:

```bash
journalctl -u k400-raw-pointer.service -f
```

---

## Manual test

Before installing the service, you can run the daemon directly.

Example using the current tuned profile:

```bash
sudo /usr/bin/python3 ./k400-raw-pointer.py \
    --gain 1.0 \
    --precision-speed 4.0 \
    --precision-transition 5.0 \
    --precision-gain 0.6 \
    -v
```

The receiver/device slot is auto-detected. Use `--slot N` only if you need an explicit override.

Press `Ctrl+C` for a clean shutdown.

The daemon restores the previous HID++ raw-report state when it exits normally.

---

## Desktop mouse settings

The daemon creates a virtual relative pointer named:

```text
Logitech K400 Raw Pointer
```

The desktop still applies its normal mouse policy to that virtual device.

For the tuning in this repository, the intended downstream configuration is:

```text
Acceleration profile: Flat / no acceleration
Pointer speed:        Neutral
```

The daemon already implements the desired low-speed precision shaping. If the compositor or desktop also applies adaptive mouse acceleration, the two behaviors are layered together and the result will no longer match the intended response curve.

### KDE Plasma

In System Settings, locate the virtual **Logitech K400 Raw Pointer** and use a flat/no-acceleration profile with neutral pointer speed if your Plasma version exposes per-device controls.

The exact UI labels vary between Plasma versions.

On the development system, the equivalent libinput intent was:

```text
PointerAcceleration=0.000
PointerAccelerationProfile=Flat
```

Do not compensate for pointer speed in KDE first. Prefer tuning `K400_GAIN` in the daemon so the raw-to-relative conversion remains predictable.

### GNOME and other desktops

The same principle applies:

- disable adaptive acceleration for the virtual pointer if possible;
- leave downstream pointer speed neutral;
- use daemon gain/precision settings for K400-specific tuning.

Some desktops expose mouse acceleration only as a global setting rather than per-device. In that case, decide whether changing the global mouse profile is acceptable before using this daemon as your primary K400 pointer path.

### X11

The daemon does not depend on Wayland.

Under X11, the virtual device is still an ordinary Linux relative pointer. Configure its libinput/Xorg acceleration profile as flat/neutral rather than applying an additional X11 coordinate transform on top of the daemon.

---

## Default configuration

The supplied `k400-raw-pointer.sysconfig` uses:

```ini
K400_SLOT=0
K400_GAIN=1.0

K400_PRECISION_SPEED=4.0
K400_PRECISION_TRANSITION=5.0
K400_PRECISION_GAIN=0.6

K400_MULTITOUCH_ENTRY_GRACE_MS=50
K400_MULTITOUCH_EXIT_GRACE_MS=80

K400_DOUBLE_TAP_WINDOW_MS=140
K400_DOUBLE_TAP_MOVE_UNITS=60

K400_TAP_DRAG_WINDOW_MS=300
K400_TAP_DRAG_MOVE_UNITS=120

K400_SCROLL_AXIS_LOCK=on
K400_SCROLL_AXIS_LOCK_RATIO=1.5

K400_KINETIC_HISTORY_MS=80
K400_KINETIC_START_VELOCITY=0.70
K400_KINETIC_STOP_VELOCITY=0.20
K400_KINETIC_DECAY_MS=400
K400_KINETIC_MAX_MS=1600

K400_LOG_LEVEL=warning

K400_FEATURE_WAIT_SECONDS=15
K400_RECONNECT_DELAY_SECONDS=2

K400_FN_MODE=standard
```

`K400_SLOT=0` enables automatic receiver-slot discovery. Use a positive slot number only as an override.

### Receiver selection override

Normally leave:

```ini
K400_SLOT=0
```

For debugging or multi-device setups, the CLI also supports:

```bash
--slot 3
--receiver-path /dev/hidraw0
```

These filters can be used separately or together. Explicit selection is especially useful on systems that intentionally keep more than one K400 Plus powered on at the same time.

### Not every CLI option is exposed in `sysconfig`

The supplied service/configuration exposes the settings that were most useful
during development, but the Python daemon has additional CLI options.

Examples include:

```text
--horizontal-scroll / --no-horizontal-scroll
--scroll-invert
--hscroll-invert
--scroll-units-per-step
--hscroll-units-per-step
--tap / --no-tap
--two-finger-tap / --no-two-finger-tap
--precision-filter-ms
--invert-x
--invert-y
```

Use:

```bash
/usr/local/libexec/k400-raw-pointer.py --help
```

to see the full set.

To make an additional option persistent, either add a corresponding variable
to `/etc/sysconfig/k400-raw-pointer` and pass it from `ExecStart`, or edit the
service's `ExecStart` directly.

---

## Pointer tuning

### Base gain

```ini
K400_GAIN=1.0
```

This is the constant raw-coordinate -> relative-pointer gain used at ordinary and high movement speeds.

Increase it for more cursor travel per unit of finger motion.

Decrease it for less.

### Low-speed precision region

```ini
K400_PRECISION_SPEED=4.0
K400_PRECISION_TRANSITION=5.0
K400_PRECISION_GAIN=0.6
```

At or below `PRECISION_SPEED`, the base gain is multiplied by `PRECISION_GAIN`.

Between `PRECISION_SPEED` and `PRECISION_TRANSITION`, the daemon smoothly transitions back to full gain.

At or above `PRECISION_TRANSITION`, the multiplier is exactly `1.0`.

Examples:

```text
Fine motion still too fast:
    lower K400_PRECISION_GAIN

Fine motion feels sticky:
    raise K400_PRECISION_GAIN

Precision shaping affects too much ordinary motion:
    lower K400_PRECISION_SPEED and/or K400_PRECISION_TRANSITION
```

The daemon also supports `--precision-filter-ms`; see `--help`.

---

## Tap, double-tap, and tap-and-drag

### One-finger tap

A valid one-finger tap synthesizes `BTN_LEFT`.

### Two-finger tap

A valid two-finger tap synthesizes `BTN_RIGHT`.

### Double tap

A generic input daemon cannot emit the first single click instantaneously and later retract it if a second tap arrives.

To avoid applications acting on the first click in the middle of a double-click, the daemon uses a short configurable double-tap window.

The default is:

```ini
K400_DOUBLE_TAP_WINDOW_MS=140
K400_DOUBLE_TAP_MOVE_UNITS=60
```

The time window is measured from the **first finger-down**, not from first release. A single click waits only for the unused remainder of that interval.

Set:

```ini
K400_DOUBLE_TAP_WINDOW_MS=0
```

if immediate single-tap response is more important than suppressing the first-click side effect of a double tap.

Physical K400 button clicks are never delayed by this logic.

### Tap-and-drag

A completed tap remains eligible for a nearby retouch-and-drag:

```ini
K400_TAP_DRAG_WINDOW_MS=300
K400_TAP_DRAG_MOVE_UNITS=120
```

Gesture:

```text
tap
lift
retouch
move beyond tap slop
    -> BTN_LEFT down
move finger
    -> drag
lift
    -> BTN_LEFT up
```

Double-click behavior is preserved if the second contact is released without becoming a drag.

Drag lock is not currently implemented.

---

## Two-finger scrolling

Two-finger scrolling is reconstructed from the centroid of the two raw contacts.

The daemon includes short transition tolerances because two fingers rarely land or leave on exactly the same sensor report.

### Entry grace

```ini
K400_MULTITOUCH_ENTRY_GRACE_MS=50
```

When the first finger touches the pad, pointer motion is briefly suppressed while the daemon waits to see whether a second finger is arriving.

Movement during the grace period is discarded, not accumulated.

This prevents the first finger of an intended two-finger scroll from producing a cursor jump.

### Exit grace

```ini
K400_MULTITOUCH_EXIT_GRACE_MS=80
```

When a two-finger scroll drops temporarily to one finger, pointer motion remains suppressed for a short period.

If the second finger returns, the scroll continues with a re-anchored centroid.

If both fingers leave, the scroll ends.

If one finger genuinely remains beyond the grace period, the daemon re-anchors that finger and hands it cleanly to normal pointer movement.

### Axis locking

Axis locking is enabled in the supplied configuration:

```ini
K400_SCROLL_AXIS_LOCK=on
K400_SCROLL_AXIS_LOCK_RATIO=1.5
```

With horizontal scrolling enabled, a clearly dominant initial axis remains locked for that scroll gesture to prevent small cross-axis finger motion from generating unwanted orthogonal scrolling.

Horizontal scrolling itself is disabled by default, so axis locking does not change the normal vertical-scroll behavior unless horizontal scrolling is enabled manually.

---

## Kinetic scroll / flick behavior

The daemon estimates release velocity from recent two-finger raw motion.

If release speed is below:

```ini
K400_KINETIC_START_VELOCITY=0.70
```

there is no post-release coast.

Above that threshold, the modeled velocity decays while the daemon emits complete wheel detents at progressively larger intervals, matching the observed K400 firmware behavior.

Defaults:

```ini
K400_KINETIC_HISTORY_MS=80
K400_KINETIC_START_VELOCITY=0.70
K400_KINETIC_STOP_VELOCITY=0.20
K400_KINETIC_DECAY_MS=400
K400_KINETIC_MAX_MS=1600
```

Any new touch cancels an active coast immediately.

A deliberate two-finger-scroll -> one-finger-pointer handoff does not start kinetic scrolling.

The virtual pointer advertises both:

```text
REL_WHEEL
REL_WHEEL_HI_RES
REL_HWHEEL
REL_HWHEEL_HI_RES
```

A full vertical detent is emitted as the stock-style pair:

```text
REL_WHEEL         +/-1
REL_WHEEL_HI_RES  +/-120
```

---

## Physical buttons

The daemon does **not** grab or replace the K400's physical left/right buttons.

Physical click buttons continue to arrive from the real K400 event device.

The virtual pointer advertises `BTN_LEFT` / `BTN_RIGHT` only because tap gestures need to synthesize clicks.

This means pointer movement and tap-generated clicks come from the virtual device, while physical hardware button presses continue to come from the original K400 device. Normal Linux desktops/compositors handle this correctly at the seat level, but software that deliberately isolates individual input devices may behave differently.

---

## Reconnect behavior

The daemon is designed to survive normal device interruptions.

### K400 power-cycle

When the K400 reconnects, the daemon:

- detects the Logitech connection notification;
- discards stale gesture/pointer state;
- re-queries HID++ feature `0x6100`;
- re-enables raw mode;
- reapplies the configured F-key mode.

### Receiver/session recovery

After auto-detection has successfully established a K400 endpoint that exposes `0x6100`, the daemon remembers that receiver/slot as the **preferred endpoint**. A transient HID++ failure therefore retries the last known-good K400 first instead of immediately pinging every paired device again.

Recovery is intentionally hierarchical:

```text
raw-mode / wake-up problem
    -> retry 0x6100 on the existing receiver/listener

peripheral temporarily unreachable
    -> keep the healthy receiver listener open and wait for link-up

receiver listener actually stops
    -> reopen the receiver and try the cached receiver/slot first

cached endpoint no longer matches, or stays unavailable repeatedly
    -> perform a full auto-scan and cache the next endpoint that proves 0x6100 works
```

The full auto-scan uses Solaar's paired-device iterator rather than blindly probing every possible receiver slot. This avoids repeated warnings for unused trailing slots on receivers that are not fully populated.

If the receiver itself is unplugged/replugged, the daemon:

- tears down the failed HID++ session;
- keeps the virtual `uinput` pointer alive;
- first tries the cached K400 endpoint if it still exists;
- falls back to receiver/slot auto-detection when necessary;
- reopens the HID++ session;
- resumes raw-pointer operation.

Keeping the virtual pointer alive prevents the desktop from seeing a new virtual mouse every time the Logitech receiver reconnects.

---

## Clean and unclean shutdown

On normal `SIGTERM`, `Ctrl+C`, or systemd shutdown, the daemon attempts to restore the HID++ raw-report state that was present before it started.

A process cannot perform cleanup after `SIGKILL`, kernel panic, abrupt power loss, or some other hard termination.

If raw mode is left enabled after an unclean termination and the normal K400 pointer appears dead, recover by one of the following:

```text
restart the daemon
power-cycle the K400
unplug/replug the Unifying receiver
```

---

## Logging and diagnostics

Normal systemd operation is intentionally quiet. The supplied configuration uses:

```ini
K400_LOG_LEVEL=warning
```

Available values are:

```text
warning   warnings and real failures only; recommended for normal use
info      startup, device selection, configuration and reconnect lifecycle
DEBUG     gesture activity, candidate probing, sleep/wake transitions and detailed diagnostics
```

Use lowercase `debug` in the configuration file:

```ini
K400_LOG_LEVEL=debug
```

Then restart the service:

```bash
sudo systemctl restart k400-raw-pointer.service
```

and follow the journal:

```bash
journalctl -u k400-raw-pointer.service -f
```

Routine taps, tap-and-drag state changes, two-finger scroll start/end, kinetic-scroll activity and similar high-frequency events are logged at **DEBUG**, not INFO, so they do not grow the journal during normal operation.

Expected wireless idle/sleep is also DEBUG-level. The daemon remains attached to the receiver while the K400 sleeps; a normal link-down notification is not treated as a failed HID++ session.

For manual debugging, `-v` and `-vv` remain convenient overrides:

```bash
# INFO
sudo /usr/bin/python3 ./k400-raw-pointer.py --gain 1.0 -v

# DEBUG
sudo /usr/bin/python3 ./k400-raw-pointer.py --gain 1.0 -vv
```

The equivalent explicit form is:

```bash
sudo /usr/bin/python3 ./k400-raw-pointer.py --log-level debug
```

Raw HID++ frame logging is a DEBUG facility:

```bash
sudo /usr/bin/python3 ./k400-raw-pointer.py \
    --print-raw \
    --log-level debug
```

The daemon also contains an experimental comparison/data-collection mode used during development.

Run:

```bash
./k400-raw-pointer.py --help
```

for the complete CLI.

### About `--dual-mode`

`--dual-mode` requests HID++ state `0x15` (`RAW | ENHANCED | RAW_AND_NATIVE`) and is intended for diagnostics.

On the tested K400 Plus, the device accepted/read back `0x15`, but normal native pointer/scroll output did **not** resume while raw mode was active. Do not treat `--dual-mode` as a way to keep the stock firmware pointer active alongside this daemon.

---

## Troubleshooting

### `Could not find Logitech K400 Plus WPID 404D`

Check:

```bash
solaar show
```

Verify:

- the K400 is awake;
- the receiver is detected;
- Solaar can see the K400 Plus;
- no incorrect explicit `--receiver-path` or positive `--slot` override was specified.

With `K400_SLOT=0`, the daemon scans paired slots automatically and prefers a currently responding candidate that exposes HID++ `0x6100`. Multiple offline `404D` pairing records are not by themselves an error.

### `K400 does not expose HID++ feature 0x6100`

The daemon retries feature discovery while the device wakes.

If the failure persists:

- touch the touchpad or press a key;
- verify the correct K400/slot is selected;
- verify that this is the expected K400 Plus revision;
- verify your Solaar version;
- check `journalctl`.

### Pointer moves but feels accelerated

Check the desktop's mouse settings for **Logitech K400 Raw Pointer**.

Use a flat/no-acceleration downstream profile if you want the daemon's response curve without another acceleration layer.

### Duplicate or strange pointer/gesture behavior

Check for competing processes/services:

```bash
pgrep -af solaar
systemctl --type=service | grep -Ei 'evsieve|k400'
```

Do not run an old K400 scaling/remapping service at the same time.

### Function keys perform special actions instead of F1-F12

Use:

```ini
K400_FN_MODE=standard
```

and restart:

```bash
sudo systemctl restart k400-raw-pointer.service
```

### Single tap feels slightly delayed

This is the double-tap recognition tradeoff.

Reduce:

```ini
K400_DOUBLE_TAP_WINDOW_MS=140
```

or set it to `0` for immediate single-tap clicks.

### Scroll coast starts too easily

Increase:

```ini
K400_KINETIC_START_VELOCITY
```

### Scroll coast is too long

Reduce:

```ini
K400_KINETIC_DECAY_MS
```

and/or:

```ini
K400_KINETIC_MAX_MS
```

---

## Features intentionally out of scope

The goal is to reproduce the useful K400 behaviors without turning this into a general-purpose multitouch framework.

Currently out of scope:

- pinch-to-zoom;
- three-finger gestures;
- palm rejection;
- disable-while-typing;
- edge scrolling;
- drag lock;
- gesture-to-keyboard-shortcut mapping.

The K400 raw interface used here reports a maximum of two contacts, so true three-finger gestures are not available anyway.

Pinch-to-zoom is intentionally omitted. Two-finger scrolling is kept unambiguous, and applications can generally still zoom with `Ctrl` + scroll.

---

## Caveats

### This uses Solaar internals

The project imports `logitech_receiver` modules from Solaar directly.

This is convenient and avoids reimplementing Logitech receiver/HID++ transport, but it also means a future Solaar internal API change can break the daemon even if the `solaar` command itself still works.

Solaar 1.1.20 is the tested baseline.

### Receiver/device selection

The daemon auto-scans paired receiver slots by default (`K400_SLOT=0`). WPID `404D` identifies the K400 Plus model, but it is not unique to one physical keyboard, so auto-detection also considers whether a candidate is currently responding and whether it exposes the required `TOUCHPAD_RAW_XY` (`0x6100`) feature.

This allows stale/offline same-model pairings to coexist on the receiver without forcing a hard-coded slot. Once a candidate has successfully exposed `0x6100`, that receiver/slot is cached as the preferred recovery endpoint so transient failures do not cause repeated full receiver scans.

If no same-model candidate is awake yet, the daemon waits for one to become active. If more than one responding/usable K400 Plus is genuinely present, use `--receiver-path` and/or a positive `--slot` override to disambiguate.

### The service runs as root

The supplied systemd unit runs as root so it can access Logitech `hidraw` devices and `/dev/uinput` without additional local permission rules.

That is simple and reliable, but it means you should review the script before installing it as a system service.

A dedicated unprivileged service account with custom `udev` permissions is possible, but is not currently provided.

### Kinetic scrolling is a behavioral reproduction

The coast model was tuned from observed K400 Plus output and feels indistinguishable from stock on the development hardware, but it is not Logitech firmware source code and is not guaranteed to match every firmware revision exactly.

### The virtual pointer is mouse-like, not a kernel multitouch touchpad

The daemon intentionally presents a relative `uinput` pointer rather than creating a full Linux multitouch touchpad device.

That keeps the architecture simple and predictable, but means downstream libinput does not see a native touchpad. Gesture behavior is therefore implemented in this daemon rather than delegated to libinput.

### Suspend/resume

Power-cycle and receiver unplug/replug recovery have been explicitly tested.

Suspend/resume should normally be handled by the same reconnect machinery if the receiver/device reconnects in the expected way, but it should be treated as less-tested than the explicit reconnect cases.

---

## Architecture

Normal stock path:

```text
K400 touch sensor
    |
    v
K400 firmware gesture / pointer processing
    |
    v
quantized REL_X / REL_Y + wheel events
    |
    v
Linux input stack
    |
    v
desktop / applications
```

Raw-pointer path:

```text
K400 touch sensor
    |
    v
HID++ 0x6100 raw absolute coordinates
    |
    v
k400-raw-pointer.py
    |
    +--> pointer precision/gain
    +--> tap/double-tap/tap-drag
    +--> two-finger scroll
    +--> transition grace
    +--> kinetic scroll
    |
    v
Linux uinput virtual relative pointer
    |
    v
libinput / compositor / Xorg
    |
    v
desktop / applications
```

The physical K400 keyboard and hardware mouse buttons continue through their normal kernel input path.

---

## Development notes

The daemon contains comparison/debugging code used to characterize the stock K400 behavior, including:

- raw HID++ event logging;
- optional physical evdev capture;
- raw/native comparison CSV output;
- raw velocity summaries;
- HID++ raw-state verification.

These facilities are not required for normal operation but are intentionally retained because they are useful when testing other firmware revisions or future changes.

---

## Project scope

This project is focused on one thing:

> Make the K400 Plus touchpad behave like a fast, precise, predictable relative pointing device while preserving the useful stock gestures.

It is not intended to replace Solaar, libinput, or a general multitouch gesture framework.

Solaar remains the receiver/HID++ transport dependency. Linux/libinput/the desktop still handle the resulting virtual pointer. This daemon fills the hardware-specific gap between the K400 Plus raw touch sensor and the relative pointer behavior desired by the user.

---

## Disclaimer

This project is not affiliated with or endorsed by Logitech or the Solaar project.

It directly changes Logitech HID++ device state while running. Use it at your own risk.

This project was developed with substantial assistance from generative AI tools. The source code is provided under the MIT License. Third-party dependencies remain subject to their respective licenses.
