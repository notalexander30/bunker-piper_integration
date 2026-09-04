#!/usr/bin/env python3

"""Recover a stuck RealSense USB/UVC device on the Jetson."""

import fcntl
import glob
import os
import time
from pathlib import Path


VENDOR_ID = '8086'
PRODUCT_IDS = {
    '0b3a': 'D435i',
    '0b56': 'D555',
}
USBDEVFS_RESET = 0x5514


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding='ascii').strip().lower()
    except (FileNotFoundError, PermissionError, OSError):
        return ''


def find_devices() -> list[Path]:
    devices = []
    for raw_path in sorted(glob.glob('/sys/bus/usb/devices/*')):
        path = Path(raw_path)
        if (
            _read(path / 'idVendor') == VENDOR_ID
            and _read(path / 'idProduct') in PRODUCT_IDS
        ):
            devices.append(path)
    return devices


def device_model(device: Path) -> str:
    return PRODUCT_IDS.get(_read(device / 'idProduct'), 'RealSense')


def camera_node_running() -> bool:
    own_pid = os.getpid()
    for raw_path in glob.glob('/proc/[0-9]*/cmdline'):
        try:
            pid = int(Path(raw_path).parts[2])
            if pid == own_pid:
                continue
            command = Path(raw_path).read_bytes().replace(b'\x00', b' ')
        except (FileNotFoundError, PermissionError, OSError, ValueError):
            continue
        if b'realsense2_camera_node' in command:
            return True
    return False


def set_power_on(device: Path) -> None:
    # A pair of cameras shares downstream hubs on the Jetson. Pinning only the
    # camera leaf is insufficient: an autosuspend of either parent hub removes
    # both V4L2 devices at once. Walk from the camera to (but not including) the
    # USB root hub and keep every downstream device/hub powered.
    current = device.resolve()
    while not current.name.startswith('usb'):
        control = current / 'power' / 'control'
        if control.exists():
            try:
                control.write_text('on\n', encoding='ascii')
            except OSError as exc:
                raise RuntimeError(
                    f'cannot disable USB autosuspend through {control}: {exc}; '
                    'run inside the privileged trystan-bunker container'
                ) from exc
        current = current.parent


def device_node(device: Path) -> Path | None:
    try:
        bus = int(_read(device / 'busnum'))
        number = int(_read(device / 'devnum'))
    except ValueError:
        return None
    return Path(f'/dev/bus/usb/{bus:03d}/{number:03d}')


def wait_for_node(
    device_name: str,
    previous_node: Path | None,
    timeout_sec: float = 20.0,
) -> tuple[Path, Path]:
    deadline = time.monotonic() + timeout_sec
    saw_transition = previous_node is None
    stable_node = None
    stable_polls = 0
    while time.monotonic() < deadline:
        device = Path('/sys/bus/usb/devices') / device_name
        if not device.exists():
            device = None
        node = device_node(device) if device is not None else None
        if node is None or not node.exists():
            saw_transition = True
            stable_node = None
            stable_polls = 0
        else:
            if previous_node is not None and node != previous_node:
                saw_transition = True
            if saw_transition and node == stable_node:
                stable_polls += 1
            elif saw_transition:
                stable_node = node
                stable_polls = 1
            if stable_polls >= 4:
                return device, node
        time.sleep(0.25)
    raise RuntimeError('RealSense did not reappear within %.1f seconds' % timeout_sec)


def reset_device(device: Path) -> None:
    set_power_on(device)
    node = device_node(device)
    if node is None or not node.exists():
        raise RuntimeError('RealSense USB device node disappeared before it could be reset.')
    model = device_model(device)
    print(f'Resetting RealSense {model} at {node} ({device.name}) ...', flush=True)
    disconnected_during_reset = False
    try:
        with node.open('rb+', buffering=0) as usb_device:
            fcntl.ioctl(usb_device, USBDEVFS_RESET, 0)
    except OSError as exc:
        # RealSense firmware can disconnect during USBDEVFS_RESET, causing
        # ENODEV or EPIPE even though the host-controller reset succeeded.
        print(f'Reset returned {exc}; waiting for USB re-enumeration.', flush=True)
        disconnected_during_reset = True

    recovered, recovered_node = wait_for_node(
        device.name,
        node if disconnected_during_reset else None,
    )
    set_power_on(recovered)
    model = device_model(recovered)
    print(
        'RealSense %s recovered at %s; autosuspend disabled.'
        % (model, recovered_node),
        flush=True,
    )


def main() -> None:
    if camera_node_running():
        raise SystemExit(
            'A realsense2_camera_node is running. Stop its launch with Ctrl-C, '
            'then run this recovery command again.'
        )

    devices = find_devices()
    if not devices:
        supported = ', '.join(f'8086:{pid} ({name})' for pid, name in PRODUCT_IDS.items())
        raise SystemExit(f'No supported Intel RealSense is connected. Supported: {supported}.')

    for device in devices:
        reset_device(device)
    print('RealSense recovery complete. Start the camera launch now.', flush=True)


if __name__ == '__main__':
    main()
