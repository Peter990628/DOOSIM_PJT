# V2.1 — AMR2 map_server lifecycle 복구

실제 런타임에서 `/amr2/map_server`가 `unconfigured [1]`에 남아 `/amr2/map`이 발행되지 않았고,
AMR2 global costmap이 병원 지도 대신 기본 5x5m 상태에 머물렀다.
그 결과 `Robot is out of bounds`와 `Can't update static costmap layer, no map received`가 반복되었다.

V2.1은 `hospital_total_08091221` 기준본 231개 파일을 수정하지 않는다.
추가 overlay에서 다음만 보강한다.

1. 실행 전 stale Nav2/map_server/lifecycle_manager child process 정리 (Isaac은 유지)
2. AMR1 `/map_server`와 AMR2 `/amr2/map_server`를 실제 lifecycle 서비스로 확인
3. `unconfigured -> configure -> inactive -> activate -> active`를 명시적으로 복구
4. map_server가 `ACTIVE`가 된 뒤에만 `/map` 또는 `/amr2/map` OccupancyGrid를 검사
5. 맵/pose lock/centerline READY가 모두 확인된 뒤에만 충돌회피 및 트레이 미션 시작

정상 로그 핵심:

```
[LIFECYCLE TRANSITION] node=/amr2/map_server action=configure success=True
[LIFECYCLE TRANSITION] node=/amr2/map_server action=activate success=True
[LIFECYCLE ACTIVE] node=/amr2/map_server state=active[3]
[MAP READY] topic=/amr2/map frame=map size=1528x841 resolution=0.050 ...
[PASS] AMR2 base Nav2 ready: map_server ACTIVE + map + pose lock + centerline READY
[BASE READY V2.1]
```

360 LiDAR, 최신 코너 회전, stale goal 수정, 2F elevator 0.35m, AMR1/AMR2 traffic pause,
3-post ArUco gate(40/41 | 44 | 42/43), dual lift/fixed joint, cooperative Nav2 구조는 그대로 유지한다.
