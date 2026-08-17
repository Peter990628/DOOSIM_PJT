# V2.16 Scan -> Straight -> Attach -> Safe Transport

- AMR1/AMR2 start detached on exact left/right bay centerlines.
- Start center is local x=-1.85m: cart front face -1.10m plus 0.75m camera standoff.
- ArUco is verification only. No lateral/yaw steering.
- Both AMRs then drive straight 1.85m with angular.z=0 and linear.y=0.
- Existing Lift + dual FixedJoint attaches the cart.
- Transport uses /coop/cart/status actual Isaac world x/y/yaw and a route whose center clearance is ~1.70m.
- Old blind timed LAST RESORT is not used.
- Normal RUN_TRAY_* logic is retained.

Run:
1. ./00_SETUP_TRAY_360_INTEGRATED.sh
2. ./RUN_V216_1_ISAAC_SCAN_READY_EXTERNAL_SAFE.sh
3. When Stage is ready, press PLAY
4. In a new terminal: ./RUN_V216_2_SCAN_STRAIGHT_ATTACH_TRANSPORT.sh
