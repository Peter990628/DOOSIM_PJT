# V5.10 ROS 2 Humble Rotation Shim 설정 수정

V5.9에서 RotationShimController를 추가하면서 newer Nav2의 nested `primary_controller:` 파라미터 형식을 사용한 문제가 있었다.
ROS 2 Humble Nav2에서는 `primary_controller`가 문자열 타입이며, MPPI 파라미터는 `FollowPath`와 동일한 namespace에 둔다.
이 설정 오류 때문에 controller_server configure가 실패하면 lifecycle manager가 다음 노드 활성화를 중단할 수 있고, planner_server/global_costmap이 activate되지 않아 `/global_costmap/costmap`이 publish되지 않는다.

V5.10은 Humble 형식으로 수정했다:

```yaml
FollowPath:
  plugin: nav2_rotation_shim_controller::RotationShimController
  primary_controller: nav2_mppi_controller::MPPIController
  angular_dist_threshold: 0.261799
  # MPPI params are also directly under FollowPath
  vx_max: 0.65
  wz_max: 1.20
```
