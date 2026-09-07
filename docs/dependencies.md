# External dependencies

Hardware SDKs and their ROS wrappers are external dependencies. They are not
copied into this repository.

## Source dependencies

The tested source revisions are pinned in
[dependencies.repos](../dependencies.repos). Import them beside this
repository in a ROS 2 workspace:

    cd /ros2_ws
    vcs import src < src/bunker-piper_integration/dependencies.repos

The manifest currently pins:

- AgileX agx_arm_ros for PiPER descriptions and control.
- AgileX bunker_ros2 for the Bunker driver and messages.
- frontier_exploration_ros2 for optional exploration.
- simple_vlm for optional vision-language navigation.

Intel RealSense, Nav2, MoveIt, RTAB-Map, ArUco, PCL, FastAPI, and the remaining
released ROS dependencies are installed through rosdep or the Docker image.

    sudo rosdep init 2>/dev/null || true
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y \
      --skip-keys "bunker_object_follower semantic_memory"

bunker_object_follower and semantic_memory are optional lab demo packages that
currently have no public upstream URL. Their launch files remain available,
but they are not required by the maintained Nav-Man startup.

## OpenClaw integration

The supported gateway is installed by piper_x_aruco_wall_approach and listens
on 127.0.0.1:8893. The supported OpenClaw skill is included at
openclaw/skills/piper-touch-marker/SKILL.md. Set PIPER_TOUCH_MARKER_SKILL to a
different SKILL.md path if you want the test suite to validate another
deployment copy against the same contract.

## Reproducibility

The manifest uses exact Git commit IDs. Update each revision deliberately,
rebuild, and rerun the test suite before changing the pinned value.
