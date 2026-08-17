# V4.5 테스트 메모

이번 버전은 V4.4에서 관찰된 4가지 문제를 한 번에 직접 겨냥합니다.

1. Raw Smac 점들의 순간 tangent 대신 0.8 m 길이의 dominant segment 방향 사용
2. 큰 방향 변화가 0.70 m 이내로 들어오면 PRETURN STOP 후 다음 dominant segment 방향으로 회전
3. 회전 완료한 코너는 1.0 m 반경에서 consumed 처리하여 같은 코너 재검출 방지
4. 회전 속도 최소 0.12 rad/s, 1초 동안 yaw 변화가 0.5도 미만이면 0.18 rad/s까지 최소 속도 boost

중요: 이 버전도 Stage-1 주행 안정화용입니다. 벽과 책상/자판기를 서로 다른 종류의 cost로 분리하는 기능은 아직 넣지 않았습니다.

## 확인

```bash
./36_check_v4_5_dominant_segment.sh
ros2 topic echo /trolley/cmd_vel
```

코너 회전 중에는 linear.x=0 이어야 합니다.
