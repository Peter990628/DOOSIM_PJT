V4.1 HUMBLE ROTATE-FIRST FIX

Purpose:
- Keep V4 Smac + Inflation baseline unchanged.
- Add RotationShimController before DWB.
- ROS 2 Humble compatible parameter layout.

Important fix compared with the broken V4.1:
- Humble expects FollowPath.primary_controller as a STRING.
- DWB parameters remain directly under FollowPath.
- max_cost_threshold was removed because the Humble implementation does not declare it.

Behavior:
- If path heading error > 0.20 rad (~11.5 deg), rotate in place first.
- Handoff back to DWB once error < 0.10 rad (~5.7 deg).
- After handoff, DWB can still make small simultaneous forward+turn corrections.

Run:
1) ./03_run_isaac.sh
2) ./31_run_nav2_trolley_smac.sh
3) ./32_check_trolley_smac.sh
