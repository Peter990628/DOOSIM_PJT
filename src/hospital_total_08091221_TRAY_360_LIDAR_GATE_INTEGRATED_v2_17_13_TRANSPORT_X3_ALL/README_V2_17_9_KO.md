# V2.17.9 EARLY ATTACH + FAST INSERT

현재 실제 화면에서 약 2.90m 위치면 이미 cart capture 범위에 들어옵니다.
따라서 3.00m까지 느리게 밀지 않고 2.90m에서 실제 ATTACH 검사를 시작합니다.

- attach attempt: 2.90m
- retry creep: +0.03m x 최대 3
- 최대 pre-attach travel: 2.99m
- insert speed: 0.150 / 0.125 / 0.100 / 0.075 / 0.055 m/s
- attach 후 transport: /coop/cmd_vel, straight 0.66m/s

신호 흐름:
1) insert: /cmd_vel + /amr2/cmd_vel
2) attach: /coop/cart/command -> ATTACH
3) attached transport: /coop/cmd_vel
4) Isaac runtime에서 commands_from_twist() -> AMR1/AMR2 wheel commands

motion_signal.log에서 실제 Twist 값을 실시간 확인할 수 있습니다.
