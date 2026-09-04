# Third-party notices

## PiPER meshes and kinematic values

Derived from `agilexrobotics/piper_ros`, Humble branch:
https://github.com/agilexrobotics/piper_ros

The upstream repository includes the MIT License. A copy is supplied as
`PIPER_LICENSE.txt`.

## Bunker Mini visual mesh

Copied from the public `agilexrobotics/ugv_gazebo_sim` Bunker Mini model:
https://github.com/agilexrobotics/ugv_gazebo_sim

No standalone license file was present in the sparse source checkout used for
this asset. Confirm redistribution permission or replace the visual with your
vendor-provided mesh before publicly redistributing this package. The primitive
collision geometry and Nav2 operation do not depend on that visual mesh.

## RealSense model

The package does not copy RealSense meshes. It depends on the installed
`realsense2_description` package from Intel's `realsense-ros` project, which is
licensed under Apache-2.0:
https://github.com/IntelRealSense/realsense-ros

## User-supplied/custom assets

`meshes/upper_frame/Frame.stl` was supplied by the user. The mounting-plate STL
was generated for this project from the supplied dimensions.

