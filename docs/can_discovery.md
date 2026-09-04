# CAN Discovery

Do not assume that Linux `can0`, `can1`, `can2`, etc. stay attached to the same physical USB-CAN adapter. The public setup should discover the current `canX` names first, then pass those values into launch files.

## List Visible CAN Links

```bash
ip -br link show type can
ip -details link show type can
```

If no CAN links appear, check USB power, drivers, and adapter permissions before launching ROS.

## Resolve By USB Adapter Serial

When adapter serials are known, use:

```bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

PIPER_CAN_USB_SERIAL=YOUR_FRONT_OR_SINGLE_PIPER_ADAPTER_SERIAL \
BUNKER_CAN_USB_SERIAL=YOUR_BUNKER_ADAPTER_SERIAL \
ros2 run bunker_slam_bringup resolve_can_interfaces.sh
```

For shell variables:

```bash
eval "$(
  PIPER_CAN_USB_SERIAL=YOUR_PIPER_ADAPTER_SERIAL \
  BUNKER_CAN_USB_SERIAL=YOUR_BUNKER_ADAPTER_SERIAL \
  ros2 run bunker_slam_bringup resolve_can_interfaces.sh --shell
)"

echo "$ARM_CAN_INTERFACE"
echo "$BUNKER_CAN_INTERFACE"
```

## Configure CAN Bitrates

PiPER uses 1 Mbit/s. Bunker uses 500 kbit/s.

```bash
sudo -v
ros2 run bunker_slam_bringup configure_can.sh FRONT_PIPER_CANX BUNKER_CANX REAR_PIPER_CANX
```

Example shape only:

```bash
ros2 run bunker_slam_bringup configure_can.sh canX canY canZ
```

Replace `canX`, `canY`, and `canZ` with discovered interfaces.

## Validate Traffic

```bash
candump -L FRONT_PIPER_CANX
candump -L REAR_PIPER_CANX
candump -L BUNKER_CANX
```

Move or power one device at a time while watching frames so the front/rear PiPER labels are confirmed before motion is enabled.
