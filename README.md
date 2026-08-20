# F1TENTH 온보드 — 공통 Sim-to-Real 스택

ROS 2 Humble 기반 실차 배포 브랜치입니다. 시뮬레이션 저장소와 동일한
`planning`, `control`, `f1tenth_bringup`, `track03` map/raceline을 사용하며,
실행 시 핵심 변경은 `mode:=sim`/`mode:=real`입니다.

## 공통/환경별 항목

| 항목 | 시뮬레이션 | 실차 |
|---|---|---|
| planning/controller/AMCL 설정 | 공통 | 공통 |
| map/raceline | `track03` | `track03` (동일 해시) |
| 차량 크기/휠베이스/LiDAR 위치 | 실차 측정값 | 실차 측정값 |
| base frame | `ego_racecar/base_link` | `base_link` |
| odometry | `/ego_racecar/odom` | `/odom` |
| Ackermann 출력 | `/drive` | `/auto` → mux → VESC |
| 시작 자세 | raceline에서 자동 | RViz `2D Pose Estimate` |

하드웨어 드라이버, VESC 보정, 실센서 노이즈와 TF namespace는 환경
어댑터입니다. 제어기에서 VESC ERPM/servo를 직접 출력하지 않으므로 기존 차량
보정은 유지됩니다.

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

## 실차 실행

노트북 터미널마다 접속합니다.

```bash
ssh -tt jeonbotdae@192.168.1.7 \
  'docker start f1tenth >/dev/null 2>&1 || true; docker exec -it f1tenth bash'
```

각 온보드 터미널 공통 환경:

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
export ROS2CLI_NO_DAEMON=1
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
```

### 터미널 1 — 하드웨어

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

`Opened joystick`, `Connected to VESC`, `Connected to a network device`를
확인합니다. 이 워크스페이스와 VESC 설정은 제어기 실험 중 변경하지 않습니다.

### 터미널 2 — Localization + Planning + Controller

```bash
cd /home/misys/shared_dir
./run_autonomy.sh \
  mode:=real track:=track03 controller:=pure_pursuit \
  speed:=1.0 maximum_speed:=20.0
```

이 명령 하나가 map server, 공통 AMCL, 전역경로, 장애물 플래너와 선택한
제어기를 실행합니다. 별도의 lifecycle 명령은 필요 없습니다. 모든 제어기는
비활성 상태로 시작합니다.

### 터미널 3 — RViz 및 초기 자세

노트북에서 ROS domain 30으로 RViz를 실행한 뒤 `2D Pose Estimate`를 지정합니다.
`map → odom → base_link → laser`가 연결되고 LaserScan이 지도 벽과 맞아야 합니다.

```bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
rviz2
```

### 터미널 4 — 시작/정지

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

첫 실차 검증은 `speed:=1.0`과 충분한 공간에서 수행합니다. `maximum_speed`는
소프트웨어 입력 검증 상한일 뿐 VESC·모터·배터리의 물리 한계를 높이지 않습니다.

## 시뮬레이션과 동일한 명령 형태

로컬 F1TENTH Gym 컨테이너에서는 같은 launch에 mode만 바꿉니다.

```bash
./run_autonomy.sh \
  mode:=sim track:=track03 controller:=pure_pursuit \
  speed:=1.0 maximum_speed:=20.0 \
  obstacles:=false rviz:=true
```

`obstacles`와 `rviz`는 Gym fixture/UI 옵션이며 제어기 파라미터를 바꾸지 않습니다.

## 제어기 선택

`controller:=`만 바꿉니다.

| 이름 | 방식 | 현재 용도 |
|---|---|---|
| `pure_pursuit` | 속도 비례 lookahead + 조향률 제한 PP | 우선 기준선 |
| `unicorn_l1` | HMCL-UNIST adaptive L1/PP | 비교 |
| `forza_map` | ForzaETH MAP pursuit | 7 m/s LUT 범위 내 비교 |
| `mpc` | 선형 bicycle MPC | 비교 |
| `mpcc` | nonlinear MPCC | 실험 |

기본 PP도 공통 `/planning/path`, `/planning/speed_limit`,
`/planning/avoidance_active`, `/safety/emergency_stop`을 사용합니다. UNICORN L1과
Forza MAP은 MPC가 아닙니다.

## 주행 전 확인

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
timeout 5 ros2 topic hz /scan
timeout 5 ros2 topic hz /odom
timeout 5 ros2 topic hz /planning/path
ros2 run tf2_ros tf2_echo map base_link
```

- map server와 AMCL: `active [3]`
- `/scan`: 약 40 Hz, `/odom`: 약 50 Hz
- Scan/지도 정렬 정상
- 시작 전 지속적인 `AEB_STOP`, scan timeout, TF 오류 없음

## 종료와 재실행

자율주행 터미널에서 `Ctrl+C`를 한 번 누릅니다. `run_autonomy.sh`가 자신이 만든
process group만 종료하고 다음 실행 전에 남은 autonomy 자식 프로세스를 정리합니다.
하드웨어 bringup은 별도 터미널이므로 유지됩니다.

## Sim-to-real 검증 기준

소스 파일 해시, map/raceline 해시, 차량 형상, AMCL 파일과 launch 인자가 같아야
합니다. 그 뒤 동일 controller/speed로 bag을 기록해 다음을 비교합니다.

- lap time, 평균/최대 cross-track error
- 실제 속도와 명령 속도
- 조향 saturation/진동
- AEB 횟수와 원인
- AMCL pose jump와 TF/scan 지연

마찰계수, 조향 지연, 가속 한계, odometry/AMCL 오차는 실차 bag으로 측정해 Gym에
보정해야 합니다. 따라서 같은 소스는 달성 가능하지만, 측정 없이 동일 동역학을
보장할 수는 없습니다.

## 출처

- [F1TENTH Pure Pursuit](https://github.com/f1tenth-dev/pure_pursuit)
- [Nav2 Regulated Pure Pursuit](https://arxiv.org/abs/2305.20026)
- [HMCL-UNIST UNICORN Racing Stack](https://github.com/HMCL-UNIST/unicorn-racing-stack)
- [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack)
