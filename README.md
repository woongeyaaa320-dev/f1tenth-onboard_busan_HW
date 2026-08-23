# F1TENTH 온보드 — race_v2

ROS 2 Humble 기반 실차 배포 브랜치입니다. 6개 터미널로 하드웨어/
localization/시각화/자율주행을 나눠서 띄우는 방식을 기준으로 합니다.

## 접속 (터미널마다 반복)

SSH 접속 + 컨테이너 진입을 한 번에 (그대로 복붙):

```bash
ssh -tt jeonbotdae@172.20.10.10 \
  'docker start f1tenth >/dev/null 2>&1 || true; docker exec -it f1tenth bash'
```
IP는 네트워크에 따라 바뀔 수 있음(현재 접속 정보 확인 후 대체).
아래 모든 터미널 블록은 이 접속 이후 실행하는 명령입니다.

## 한 번만 빌드

온보드 컨테이너에서:

```bash
cd /home/misys/shared_dir/autonomy_ws
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
colcon build --symlink-install --packages-select \
  planning control f1tenth_bringup
source install/setup.bash
```

## 세션 시작 전 초기화 (매번)

이전 세션이 비정상 종료됐으면 프로세스가 남아 `/map_server`, `/amcl`이
중복으로 뜨고 충돌합니다. 실행 전 한 번:

```bash
ssh -tt jeonbotdae@172.20.10.10 \
  'docker restart f1tenth; docker exec -it f1tenth bash'
```
컨테이너 안에서:
```bash
rm -f /dev/shm/fastrtps_*
ros2 daemon stop && ros2 daemon start
```

## 실차 실행 — 터미널 6개

각 터미널마다 로봇에 접속:

```bash
ssh -tt jeonbotdae@172.20.10.10 \
  'docker start f1tenth >/dev/null 2>&1 || true; docker exec -it f1tenth bash'
```

컨테이너 안 공통 환경 (**모든 터미널에서 필수** — 특히 `ROS_DOMAIN_ID`
빠뜨리면 노드끼리 서로 안 보입니다):

```bash
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
```

### 터미널 1 — 하드웨어 Bringup

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

`Opened joystick`, `Connected to VESC`, LiDAR `Connected to a network device`
확인. **이 터미널은 절대 두 번 실행하지 마세요** — 같은 시리얼 포트를
두 프로세스가 잡으면 VESC 통신이 깨지고 드라이버가 죽습니다
(`Out-of-sync with VESC`, segfault). 이미 떠 있는지 헷갈리면:
```bash
ps aux | grep bringup_launch | grep -v grep
```

### 터미널 2 — Map Server

```bash
ros2 run nav2_map_server map_server --ros-args \
  -r __node:=map_server \
  -p yaml_filename:=/home/misys/shared_dir/maps/track05.yaml \
  -p topic:=map -p frame_id:=map -p use_sim_time:=false
```

### 터미널 3 — AMCL

```bash
ros2 run nav2_amcl amcl --ros-args \
  -r __node:=amcl \
  --params-file /home/misys/shared_dir/config/amcl.yaml
```

### 터미널 4 — Lifecycle Activation

Terminal 1~3이 다 뜬 뒤:

```bash
ros2 daemon stop && ros2 daemon start
sleep 2

ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle get /map_server   # active [3] 확인

ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
ros2 lifecycle get /amcl         # active [3] 확인
```

### 터미널 5 — RViz (노트북 호스트, SSH 아님)

```bash
xhost +si:localuser:root
docker exec -it -e DISPLAY=$DISPLAY -e ROS_DOMAIN_ID=30 -e ROS_LOCALHOST_ONLY=0 \
  f1tenth_gym_ros_humble-sim-1 \
  bash -lc '
    source /opt/ros/humble/setup.bash
    source /sim_ws/install/setup.bash
    exec rviz2 -d /sim_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/gym_bridge.rviz
  '
```

**2D Pose Estimate**로 실제 차량 위치/방향을 지정합니다 (Terminal 3의
AMCL이 살아있어야 반영됨). LaserScan(빨간 점)이 지도 벽에 맞는지 확인.

### 터미널 6 — Autonomy

```bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
cd /home/misys/shared_dir
./run_autonomy.sh mode:=real track:=track05 controller:=racing_v2_pp \
  speed:=1.0 maximum_speed:=15.0 localization:=false
```

`localization:=false` 필수 — 안 붙이면 Terminal 2/3과 내부 localization이
중복으로 떠서 `/map_server`, `/amcl`이 2개씩 생기고 서로 충돌합니다.

**속도는 1 m/s부터 단계적으로 올리고, `max_longitudinal_deceleration:=`
오버라이드는 넣지 마세요** — `racing_v2_pp` 기본값(3.04)이 그립테스트
실측치라 오버라이드하면 코너 진입시 감속이 오히려 급해집니다.

로그에 `num waypoints: 228` 확인 (track05 raceline).

## 시작/정지 · 킬스위치

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

물리 킬스위치는 조이스틱 **L2 (buttons[6])** — enable 하기 전에 항상 확인:
```bash
ros2 topic echo /safety/kill_switch
```
`false`로 뜨고 L2 누르면 `true`로 바뀌는지 확인. 안 뜨면 `kill_switch_node`가
launch에 안 붙은 것이니 `control/launch/control.launch.py`의 해당 컨트롤러
분기에 `kill_switch_node` Node()가 있는지 확인하세요.

## 킬스위치 단독 시연 (매핑 안 된 환경)

지도/위치추정/경로 전혀 필요 없음 — 고정 속도로 직진하다가 킬스위치에
반응하는 `kill_switch_demo_node` 하나만 사용합니다. Bringup에서
`joy_teleop`은 조이스틱을 실제로 조작할 때만 `/teleop`에 publish하므로,
시연 중 킬스위치 버튼(L2) 외의 스틱/버튼은 건드리지 마세요 — 건드리면
그 순간 mux 우선순위상 teleop이 앞서서 데모 노드의 `/auto` 명령을 덮습니다.

**터미널 1 — Bringup**
```bash
ssh -tt jeonbotdae@172.20.10.10 \
  'docker start f1tenth >/dev/null 2>&1 || true; docker exec -it f1tenth bash'
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

**터미널 2 — 킬스위치**
```bash
ssh -tt jeonbotdae@172.20.10.10 \
  'docker exec -it f1tenth bash'
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
ros2 run control kill_switch_node --ros-args -p kill_switch_button:=6
```

**터미널 3 — 확인 후 데모 실행**
```bash
ssh -tt jeonbotdae@172.20.10.10 \
  'docker exec -it f1tenth bash'
export ROS_DOMAIN_ID=30
source /opt/ros/humble/setup.bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
```
먼저 킬스위치 확인 (`false`→L2→`true`):
```bash
ros2 topic echo /safety/kill_switch
```
확인되면 데모 실행 — `start_delay`초 후 자동 직진 시작, `max_duration`초
후 무조건 자동 정지(안전장치, 킬스위치와 별개):
```bash
ros2 run control kill_switch_demo_node --ros-args \
  -p speed:=1.0 \
  -p start_delay:=3.0 \
  -p max_duration:=10.0
```
차 움직이기 시작하면 원하는 타이밍에 L2로 정지 시연 → 다시 눌러서
재개되는 것도 보여주면 진짜 토글임을 증명 가능 → 끝나면 Ctrl+C.
앞에 최소 3~4m 공간 확보하고 진행.

## 컨트롤러 선택

`controller:=`만 바꿉니다.

| 이름 | 방식 | 비고 |
|---|---|---|
| `pure_pursuit` (별칭 `racing_pp`) | 속도 비례 lookahead + 곡률 기반 감속 | **기본, 가장 검증됨** |
| `racing_v1_pp` | racing_pp 2026-08-23 고정 스냅샷 | 이후 수정 안 함, 항상 되돌아갈 수 있는 기준점 |
| `racing_v2_pp` | racing_v1_pp에서 계속 튜닝 중 | 그립테스트 실측 반영(`max_lateral_acceleration=4.43`, accel=1.8, decel=3.04), 코너 진입 감속 완화(`speed_limit_preview_margin=1.70`), 직선 속도 상한 확장(`maximum_planning_speed=15.0`), 회피 오탐지 완화 |
| `unicorn_l1` | HMCL-UNIST adaptive L1/PP | 곡률 기반 lookahead 상한 추가 패치 적용됨 |
| `woong_pp` | unicorn_l1 + 장애물회피 안정화 포크 | woongeyaaa320-dev/f1tenth-obstacle-tuning 이식, **시뮬 1.5m/s에서만 검증됨** — 저속부터 테스트 |
| `forza_map` | ForzaETH MAP pursuit | 7 m/s LUT 범위 내 |
| `mpc` / `mpcc` | 선형/nonlinear MPC | 실험적, 검증 부족 |

## 트랙 추가 (신규 매핑)

1. slam_toolbox로 매핑 → `maps/<name>.pgm`, `.yaml`
2. 센터라인 생성: `python3 scripts/generate_centerline.py --map-yaml ... --output waypoints/<name>_centerline.csv`
3. `scripts/generate_racetrack_bounds.py` + Raceline-Optimization(min-curvature) +
   `scripts/global_smooth_raceline.py`로 최적화된 raceline 생성 (벽 클리어런스 +
   곡률/조향각 한계 자동 검증)
4. `f1tenth_bringup/config/tracks.yaml`에 항목 추가

## 알려진 이슈 / 주의사항

- **VESC 속도 상한**: `vesc.yaml`의 `speed_max`가 예전엔 23250(≈5.57 m/s)로
  캡되어 있었음 → 20 m/s(83468)로 상향 완료. `speed:=` 값이 실제로 반영 안
  되는 것 같으면 이 값부터 확인.
- **install/ 심볼릭 링크**: `colcon build --symlink-install` 이후 새로 추가된
  파일(raceline csv 등)은 자동으로 install/에 링크되지 않음 — 수동으로
  `cp src/... install/.../share/...`까지 해야 반영됨.
- **DDS 공유메모리 잔여물**: 세션을 여러 번 강제 종료하면 `/dev/shm/fastrtps_*`
  파일이 쌓여서 `RTPS_TRANSPORT_SHM Error`나 AMCL 타임스탬프 오류가 날 수
  있음 → `docker restart f1tenth` + `rm -f /dev/shm/fastrtps_*`로 정리.
- **`max_heading_error`/`max_path_distance` 등 params.yaml 값**: 파일 기반
  파라미터가 런타임에 반영 안 되는 문제가 있어(원인 불명), 급한 경우
  `control.launch.py`의 인라인 파라미터 dict로 직접 오버라이드하는 편이
  확실함.

## 출처

- [F1TENTH Pure Pursuit](https://github.com/f1tenth-dev/pure_pursuit)
- [Nav2 Regulated Pure Pursuit](https://arxiv.org/abs/2305.20026)
- [HMCL-UNIST UNICORN Racing Stack](https://github.com/HMCL-UNIST/unicorn-racing-stack)
- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
- [TUM/CL2-UWaterloo Global Racetrajectory Optimization](https://github.com/CL2-UWaterloo/f1tenth_ws) (raceline 최적화)
