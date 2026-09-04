# Bunker + Dual PiPER URDF File Index

Use this as a documentation index for the Bunker + dual PiPER + D435i robot
description files. The entries are intentionally written without computer-
specific workspace paths so they can be linked later from another document.

## Main Robot Description

*Bunker + Dual PiPER + D435i Xacro*: editable source of truth for the complete
integrated robot.

```text
bunker_dual_piper_d435i.urdf.xacro
```

Use this file for:

```text
Bunker Mini base geometry
upper mounting frame
front PiPER arm placement
rear PiPER arm placement
front D435i camera placement
rear D435i camera placeholder
fixed TF joints
visual meshes
collision geometry
base_link / base_footprint relationship
```

## PiPER Arm Macro

*PiPER arm Xacro macro*: editable reusable arm definition used by the main
robot Xacro.

```text
piper_arm_macro.xacro
```

Use this file for:

```text
PiPER links
PiPER joints
PiPER joint limits
PiPER visual meshes
PiPER collision geometry
front_piper_ and rear_piper_ macro expansion
```

## Generated URDF

*Generated plain URDF*: expanded output produced from the Xacro files.

```text
bunker_dual_piper_d435i.generated.urdf
```

Use this file for:

```text
inspection
debugging
sharing with tools that do not support Xacro
checking the fully expanded link and joint tree
```

Do not edit this as the source of truth. If the Xacro changes, regenerate this
file.

## Relationship

```text
piper_arm_macro.xacro
        +
bunker_dual_piper_d435i.urdf.xacro
        |
        v
bunker_dual_piper_d435i.generated.urdf
```

## Related Launch Files

*URDF preview launch*: loads the robot description and opens RViz for visual
inspection only.

```text
urdf_camera_preview.launch.py
```

*Robot description launch*: starts `robot_state_publisher` and publishes the
combined robot description.

```text
description.launch.py
```

## Related RViz File

*URDF preview RViz config*: RViz display setup for checking the robot model.

```text
urdf_camera_preview.rviz
```

## Short Copy-Paste Index

```text
bunker_dual_piper_d435i.urdf.xacro: editable complete Bunker + dual PiPER + D435i robot description
piper_arm_macro.xacro: editable reusable PiPER arm macro used by the main robot Xacro
bunker_dual_piper_d435i.generated.urdf: generated expanded URDF for inspection/debugging, not the source of truth
urdf_camera_preview.launch.py: preview-only launch for viewing the URDF in RViz
description.launch.py: robot description launch that publishes robot_description and TF
urdf_camera_preview.rviz: RViz config for URDF inspection
```

