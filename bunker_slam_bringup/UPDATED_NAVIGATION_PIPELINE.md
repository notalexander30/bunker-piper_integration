# Updated Bunker Navigation Pipeline

## Camera / Perception

```text
Camera / Perception
├── PiPER / Bunker fixed camera TF
│   └── base_link -> front_camera frames
│
├── RealSense D435i camera node
│   ├── RGB image
│   │   └── OpenCV ArUco detection
│   │       └── Detect marker ID 6
│   │           └── Save ArUco landmark
│   │               └── /ros2_ws/maps/aruco_landmarks.json
│   │
│   ├── Aligned depth image
│   │   └── RTAB-Map RGB-D input
│   │
│   └── Point cloud
│       ├── Nav2 obstacle layer
│       └── Depth safety monitor
```

## Odometry / Localization / Mapping

```text
Odometry / Localization / Mapping
├── Bunker wheel odometry
│   └── /wheel/odom
│
├── H30 / YESENSE IMU
│   └── /yesense/imu_data_ros diagnostic stream
│
├── Bunker raw odometry mode
│   ├── launch: start_ekf:=false
│   ├── output: /odom
│   └── output TF: odom -> base_link
│
├── RTAB-Map mapping mode
│   ├── input: RGB-D image
│   ├── input: /odom
│   ├── output: map database
│   ├── output: /map
│   └── output TF: map -> odom
│
├── RTAB-Map localization mode
│   ├── input: saved .db map
│   ├── input: RGB-D image
│   ├── input: /odom
│   └── output TF: map -> odom
│
└── AMCL optional localization
    ├── input: /map
    ├── input: /scan
    ├── input: /odom
    └── output TF: map -> odom
```

Note: AMCL is integrated, but practical use requires `/scan`. RTAB-Map localization is still the main working localization method for the current setup.

## Navigation / Autonomy

```text
Navigation / Autonomy
├── Nav2 controller
│   ├── input: /map
│   ├── input: map -> odom -> base_link TF
│   ├── input: /odom
│   ├── input: costmaps
│   └── output: /cmd_vel in drive mode or /cmd_vel_debug in dry-run mode
│
├── Optional legacy safety gate / cmd_vel routing
│   ├── enabled only with use_cmd_vel_mux:=true
│   ├── input: /nav2/cmd_vel_raw
│   ├── input: /safety_stop
│   └── output: /cmd_vel_autonomy -> mux -> /cmd_vel
│
└── Bunker movement
    └── Bunker driver receives /cmd_vel
```

## Self-Exploration

```text
Self-Exploration
├── Frontier explorer
│   ├── input: /map
│   ├── input: robot pose in map
│   ├── output: Nav2 frontier goals
│   └── captures home pose when started
│
├── Exploration modes
│   ├── time_limit
│   │   └── explore until timer ends
│   │
│   ├── frontier_ratio
│   │   └── explore until frontier threshold is low
│   │
│   └── aruco_found
│       └── explore until marker ID 6 is detected
│
└── After completion
    ├── return home
    └── save detected ArUco landmarks
```

## Landmark Navigation

```text
Landmark Navigation
├── Saved landmark file
│   └── /ros2_ws/maps/aruco_landmarks.json
│
├── Home landmark
│   └── saved when exploration starts / ArUco node initializes
│
├── ArUco landmark
│   ├── marker ID: 6
│   ├── marker size: 5x5 cm
│   └── saved as approach pose
│
└── Landmark navigator
    ├── go_home
    ├── go_marker aruco_6
    └── go_furthest
```

## Simplified Full Flow

```text
RealSense RGB-D + Bunker raw odom + H30 diagnostic IMU
        |
        v
Bunker driver publishes /odom and odom -> base_link
        |
        v
RTAB-Map mapping/localization publishes map -> odom
        |
        v
Nav2 receives map + odom + TF
        |
        v
Frontier explorer sends exploration goals
        |
        v
OpenCV detects ArUco marker ID 6
        |
        v
System saves home + ArUco landmark
        |
        v
Landmark navigator can command:
    go home
    go to aruco_6
    go to furthest marker
        |
        v
Nav2 publishes cmd_vel
        |
        v
Bunker moves
```

## Main Updates Compared to the Old PPT

```text
YOLO Detection
    replaced by
OpenCV ArUco Detection
```

```text
Bunker raw odometry mode
    uses
Bunker driver -> /odom and odom -> base_link
```

```text
SLAM only
    expanded to
RTAB-Map mapping/localization + optional AMCL
```

```text
Manual navigation only
    expanded to
Nav2 + frontier exploration + ArUco landmark navigation
```
