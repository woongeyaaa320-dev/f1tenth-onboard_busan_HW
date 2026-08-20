# F1TENTH 온보드 자율주행

ROS 2 Humble 기반 F1TENTH 실차용 저장소입니다. 현재 `track03` 지도에서
AMCL 위치 추정, 전역 raceline, 정적 장애물 회피와 여러 제어기를 같은
입출력 인터페이스로 비교합니다.

- 시뮬레이션: [Kimz1xq/f1tenth](https://github.com/Kimz1xq/f1tenth)
- 실차: 이 저장소
- 지도: `maps/track03.yaml`
- 출력: `/auto` → Ackermann mux → VESC

## 파일 구성

```text
f1tenth-onboard/
├── autonomy_ws/src/
│   ├── planning/             전역경로 발행, LiDAR 정적 장애물 회피, AEB
│   ├── control/              제어기와 공통 파라미터
│   │   ├── config/           MPC 설정과 ForzaETH 조향 LUT
│   │   ├── control/          제어기 노드
│   │   └── launch/           제어기 선택 launch
│   └── f1tenth_bringup/      sim/real 공통 autonomy launch
├── config/amcl.yaml          실차 AMCL 설정
├── maps/track03.{pgm,yaml}   실차 지도
└── run_autonomy.sh           중복·고아 autonomy 세션 방지 실행기
```

`build/`, `install/`, `log/`, bag과 결과 CSV는 Git에 저장하지 않습니다.
VESC, LiDAR, joystick bringup은 별도 `/home/misys/f1tenth_ws`를 사용하며 이
저장소의 제어기 교체로 해당 하드웨어 보정값은 변경하지 않습니다.

## 사용 가능한 제어기

| `controller` | 종류 | 용도 |
|---|---|---|
| `pure_pursuit` | 기본 Pure Pursuit | 저속 기준선 |
| `unicorn_l1` | 속도·곡률 기반 L1/Pursuit | 기존 기본 제어기 |
| `forza_map` | ForzaETH MAP | 모델·횡가속도 기반 Pursuit |
| `mpc` | Linear MPC | 선형 MPC 비교 |
| `mpcc` | Nonlinear MPCC | 실험용 비교 |

`UNICORN L1`과 `ForzaETH MAP`은 MPC가 아닙니다. Forza MAP 구현은
[ForzaETH Race Stack](https://github.com/ForzaETH/race_stack/tree/ros2-humble)의
Model- and Acceleration-based Pursuit 전략을 현재 ROS 2 인터페이스에 맞춘
것입니다. `steering_lookup_table:=auto`는 약 0.33 m 휠베이스의 선형 자전거
모델 LUT를 초기 모델로 사용합니다. 실차 식별 LUT가 생기면 절대경로로
교체할 수 있습니다.

## 접속과 공통 환경

현재 공유기에서는 노트북 `192.168.1.6`, 온보드 `192.168.1.7`을 사용합니다.
휴대폰 핫스팟을 사용할 때의 온보드 주소는 `172.20.10.10`입니다. 노트북에서
새 터미널을 열 때마다 현재 네트워크에 맞는 주소로 접속합니다.

```bash
ssh -tt jeonbotdae@192.168.1.7 \
  'docker start f1tenth >/dev/null 2>&1 || true; docker exec -it f1tenth bash'
```

핫스팟에서는 IP만 바꿉니다.

```bash
ssh -tt jeonbotdae@172.20.10.10 \
  'docker start f1tenth >/dev/null 2>&1 || true; docker exec -it f1tenth bash'
```

컨테이너에서 다음을 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
```

`docker restart`는 현재 SSH의 `docker exec`를 종료하므로 평상시에는 사용하지
않습니다.

## 처음 한 번 또는 코드 변경 후 빌드

```bash
cd /home/misys/shared_dir/autonomy_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  planning control f1tenth_bringup
source install/setup.bash
```

## 실차 실행 순서

각 터미널에서 위 SSH 접속과 source를 먼저 수행합니다.

### 터미널 1: 차량 Bringup

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

VESC, joystick, URG LiDAR, odometry, Ackermann mux와
`odom → base_link → laser` TF를 실행합니다.

### 터미널 2: Map Server

```bash
ros2 run nav2_map_server map_server --ros-args \
  -r __node:=map_server \
  -p yaml_filename:=/home/misys/shared_dir/maps/track03.yaml \
  -p topic:=map \
  -p frame_id:=map \
  -p use_sim_time:=false
```

### 터미널 3: AMCL

```bash
ros2 run nav2_amcl amcl --ros-args \
  -r __node:=amcl \
  --params-file /home/misys/shared_dir/config/amcl.yaml
```

### 터미널 4: Lifecycle 활성화

터미널 2와 3이 모두 실행된 뒤 입력합니다. 이미 활성화된 노드에는 잘못된
transition을 다시 요청하지 않습니다.

```bash
for node in map_server amcl; do
  until ros2 lifecycle get /$node >/dev/null 2>&1; do sleep 1; done
  state=$(ros2 lifecycle get /$node 2>/dev/null | tail -n1 | awk '{print $1}')
  [ "$state" = "unconfigured" ] && ros2 lifecycle set /$node configure
  state=$(ros2 lifecycle get /$node 2>/dev/null | tail -n1 | awk '{print $1}')
  [ "$state" = "inactive" ] && ros2 lifecycle set /$node activate
done

ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

두 노드 모두 `active [3]`이어야 합니다.

### 터미널 5: ForzaETH MAP 실행

제어기는 비활성 상태로 시작합니다. 첫 실차 검증은 `1.0 m/s`로 합니다.

```bash
cd /home/misys/shared_dir
./run_autonomy.sh \
  mode:=real \
  controller:=forza_map \
  speed:=1.0 \
  steering_lookup_table:=auto
```

직접 launch하려면 다음 명령과 같습니다.

```bash
ros2 launch f1tenth_bringup autonomy.launch.py \
  mode:=real controller:=forza_map speed:=1.0 \
  steering_lookup_table:=auto
```

실차 모드에서는 장애물 플래너가 항상 실행되고 LiDAR 관측에 따라 전역경로와
회피경로를 자동 전환합니다. 충돌 입력과 AEB는 우회하지 않습니다.

다른 제어기를 비교할 때는 `controller`만 변경합니다.

```bash
controller:=pure_pursuit
controller:=unicorn_l1
controller:=mpc
controller:=mpcc
```

### 터미널 6: RViz

```bash
rviz2 -d /home/misys/f1tenth_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/gym_bridge.rviz
```

`Fixed Frame`을 `map`으로 두고 `2D Pose Estimate`로 실제 위치와 방향을
지정합니다. LaserScan이 지도 벽과 겹치는지 확인합니다.

### 터미널 7: 주행 시작과 정지

시작:

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
```

정지:

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

## 주행 전 검증

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
timeout 5 ros2 topic hz /scan
timeout 5 ros2 topic hz /odom
timeout 5 ros2 topic hz /planning/path
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /planning/local_status
ros2 topic echo /planning/avoidance_active
ros2 topic echo /safety/emergency_stop
```

정상 조건:

- `map_server`, `amcl`: `active [3]`
- TF: `map → odom → base_link → laser`
- `/scan`, `/odom`, `/planning/path`: 지속 발행
- RViz의 Scan과 지도 벽이 일치
- `emergency_stop: false`
- 조향 명령 방향과 실제 바퀴 방향이 일치

`AEB_STOP`, `NO_COLLISION_FREE_PATH`, TF/scan timeout이 있으면 원인을 해결한
뒤 주행을 시작합니다. 첫 검증은 바퀴를 띄운 상태에서 출력 방향을 확인하고,
넓은 공간에서 `1.0 m/s` 경로 추종을 확인한 다음 속도를 단계적으로 올립니다.

## Sim-to-real 기준

다음 패키지는 시뮬레이션과 실차에서 동일하게 유지합니다.

```text
planning
control
f1tenth_bringup
```

환경 차이는 launch의 `mode`로 처리합니다.

| 항목 | 시뮬레이션 | 실차 |
|---|---|---|
| base frame | `ego_racecar/base_link` | `base_link` |
| odometry | `/ego_racecar/odom` | `/odom` |
| drive | `/drive` | `/auto` |
| 초기 자세 | 자동 | RViz에서 지정 |

동일한 지도, raceline, 목표속도로 먼저 시뮬레이션하고 실차에서는 CTE, lap
time, safety stop, 충돌, 조향 saturation을 기록해 비교합니다.
