---
name: piper-touch-marker
description: Safely search for, approach, and touch an ArUco marker with the Nav-Man front PiPER-X through the supported localhost gateway.
user-invocable: true
---

# PiPER touch marker

Use this skill when the operator asks to find, approach, or touch the marker
with the front PiPER-X.

## Ownership and safety

- Use the OpenClaw gateway at http://127.0.0.1:8893.
- The gateway calls the lower PiPER API at http://127.0.0.1:8892.
- Never start a PiPER driver, camera, robot-state publisher, MoveIt stack,
  RTAB-Map instance, Nav2 stack, or TF publisher.
- Never generate arbitrary MoveIt poses or trajectories.
- Treat execution_allowed=false as a hard physical-motion block.
- Do not claim contact unless contact_confirmed is true in the API result.
- Stop when health reports joint_state_available: false, unavailable point cloud,
  unavailable marker pose, unavailable MoveIt, or another active task.

## Workflow

1. Call GET http://127.0.0.1:8893/health.
2. Confirm that the returned lower-level /health result is ready.
3. Start with a dry-run request unless the operator explicitly requests motion.
4. For physical motion, require execution_allowed: true and an explicit
   execute request.
5. Report success, stage, message, contact_confirmed, and completion_type.

The lower-level API pauses mapping work, enables the selected arm, and saves
the current front-arm joint pose before arm motion. It then moves to the
supported manipulation pose and executes the requested marker task.

## Gateway endpoints

- POST /openclaw/search_marker
- POST /openclaw/approach_marker
- POST /openclaw/touch_marker
- POST /openclaw/retract
- POST /openclaw/stop

## Lower-level contract

The gateway maps requests to these validated lower-level routes:

- /tools/piper/approach-marker
- /tools/piper/touch-marker
- /tools/piper/go-home
- /tools/piper/save-home
- /tools/piper/clear-active-tasks

For marker motion, preserve return_home_after when the operator supplies it.
The expected successful marker completion type is
single_moveit_marker_touch. A dry run or geometric approach is not physical
contact.
