# F1TENTH 온보드

ROS 2 Humble 기반 실차 자율주행 저장소입니다. 시뮬레이션 저장소 [Kimz1xq/f1tenth](https://github.com/Kimz1xq/f1tenth)의 `planning`, `control`, `f1tenth_bringup`과 동일한 알고리즘 코드를 사용합니다.

## 현재 구성

- 위치 추정: Nav2 Map Server + AMCL
- 전역 경로: `track03_raceline.csv`
- 기본 제어기: HMCL-UNIST UNICORN L1 Humble adapter
- 비교 제어기: Linear MPC, Pure Pursuit
- 장애물 대응: LiDAR local planner + AEB, 자동 전환
- 차량 출력: `/auto` → Ackermann mux → VESC

`unicorn_l1`은 MPC가 아닙니다. 속도·곡률 기반 L1/Pure-Pursuit 계열 제어기이며, `mpc`가 kinematic bicycle model 기반 Linear MPC입니다.

## 네트워크와 접속

현재 공유기 환경:

```text
노트북: 192.168.1.6
온보드: 192.168.1.7
Hotspot : 172.20.10.10
```

새 터미널마다 노트북에서 다음을 실행합니다. 비밀번호는 프롬프트에서 입력합니다.

```bash
ssh -tt jeonbotdae@172.20.10.10 \
  'docker start f1tenth >/dev/null 2>&1 || true; docker exec -it f1tenth bash'
```

컨테이너에 들어온 뒤 매번 source 합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
```

Docker를 `restart`하면 현재 SSH의 `docker exec`가 종료될 수 있으므로 평상시에는 위의 `start` 명령을 사용합니다.

## 터미널별 실행 순서

각 터미널에서 위 SSH 접속과 source를 먼저 수행합니다.

### 터미널 1 — 차량 Bringup

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

LiDAR, VESC, joystick, odometry와 `odom -> base_link -> laser` TF를 실행합니다. 한 번만 실행합니다.

### 터미널 2 — Map Server

```bash
ros2 run nav2_map_server map_server --ros-args \
  -r __node:=map_server \
  -p yaml_filename:=/home/misys/shared_dir/maps/track03.yaml \
  -p topic:=map \
  -p frame_id:=map \
  -p use_sim_time:=false
```

`Creating`에서 기다리는 것은 정상입니다.

### 터미널 3 — AMCL

```bash
ros2 run nav2_amcl amcl --ros-args \
  -r __node:=amcl \
  --params-file /home/misys/shared_dir/config/amcl.yaml
```

### 터미널 4 — Lifecycle 활성화

터미널 2와 3이 모두 실행된 뒤 시작합니다.

```bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -r __node:=lifecycle_manager_localization \
  -p autostart:=true \
  -p bond_timeout:=0.0 \
  -p node_names:="['map_server', 'amcl']"
```

확인:

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

둘 다 `active [3]`이어야 합니다.

### 터미널 5 — 자율주행 제어기

기본 UNICORN L1:

```bash
ros2 launch f1tenth_bringup autonomy.launch.py \
  mode:=real controller:=unicorn_l1 speed:=1.0
```

동적 회피 속도 제한 UNICORN L1:

```bash
ros2 launch f1tenth_bringup autonomy.launch.py \
  mode:=real controller:=unicorn_l1_dynamic speed:=1.0
```

Linear MPC 비교:

```bash
ros2 launch f1tenth_bringup autonomy.launch.py \
  mode:=real controller:=mpc speed:=1.0
```

`speed`는 직선 최대속도입니다. 코너에서는 곡률 제한으로 감속합니다. 실차 launch는 장애물 플래너를 항상 실행하며 장애물 유무에 따라 전역/회피 경로를 자동 전환합니다.

### 터미널 6 — RViz

```bash
rviz2 -d /home/misys/f1tenth_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/gym_bridge.rviz
```

`Fixed Frame`을 `map`으로 설정하고 `2D Pose Estimate`로 실제 초기 자세를 지정합니다. Map과 LaserScan이 겹치고 차량 방향이 실제와 같아야 합니다.

### 터미널 7 — 주행 시작/정지

시작:

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
```

정지:

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

## 주행 전 필수 확인

```bash
timeout 5 ros2 topic hz /joy
timeout 5 ros2 topic hz /scan
timeout 5 ros2 topic hz /odom
timeout 5 ros2 topic hz /planning/path
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /planning/local_status
ros2 topic echo /planning/avoidance_active
ros2 topic echo /safety/emergency_stop
```

정상 TF:

```text
map -> odom -> base_link -> laser
```

`AEB_STOP`, `NO_COLLISION_FREE_PATH`, TF/scan timeout에서는 원인을 해결한 뒤 시작합니다. safety 조건을 우회하지 않습니다.

## 사용 중인 모델

- `AMCL`: Nav2 particle-filter 기반 2D map localization
- 전역 경로: 지도 free-space에서 생성한 폐곡선 raceline CSV
- `unicorn_l1`: [HMCL-UNIST UNICORN Racing Stack](https://github.com/HMCL-UNIST/unicorn-racing-stack)의 L1 전략을 `nav_msgs/Path`, TF, Ackermann 인터페이스에 맞춘 ROS 2 Humble adapter
- `unicorn_l1_dynamic`: 같은 제어기에 로컬 플래너의 `/planning/speed_limit`을 적용
- `mpc`: kinematic bicycle model을 선형화한 저장소 내 Linear MPC 비교 구현
- 장애물 회피: LiDAR cluster 추적, map clearance 검사, offset 후보 경로 선택, 별도 AEB

## Sim-to-real 규칙

다음 세 패키지는 시뮬레이션과 실차에서 동일하게 유지합니다.

```text
autonomy_ws/src/planning
autonomy_ws/src/control
autonomy_ws/src/f1tenth_bringup
```

환경 차이는 launch 인자로만 흡수합니다.

| 항목 | 시뮬레이션 | 실차 |
|---|---|---|
| base frame | `ego_racecar/base_link` | `base_link` |
| odom | `/ego_racecar/odom` | `/odom` |
| drive | `/drive` | `/auto` |
| 초기 자세 | 자동 | RViz 수동 지정 |

실차 적용 전에는 동일한 지도·raceline·목표속도로 시뮬레이션에서 먼저 검증하고, 실차에서는 저속부터 올리며 CTE, lap time, 충돌, safety stop, 조향 saturation을 기록합니다.

## 변경 후 빌드

```bash
cd /home/misys/shared_dir/autonomy_ws
colcon build --symlink-install --packages-select planning control f1tenth_bringup
colcon test --packages-select planning control f1tenth_bringup
source install/setup.bash
```

저장소에는 소스와 설정만 보관하고 `build`, `install`, `log`, rosbag과 결과 CSV는 커밋하지 않습니다.

## 주요 경로

```text
/home/misys/f1tenth_ws                          차량 Bringup/VESC workspace
/home/misys/shared_dir/autonomy_ws              Planning/Control workspace
/home/misys/shared_dir/maps/track03.yaml         실차 지도
/home/misys/shared_dir/config/amcl.yaml          실차 AMCL 설정
autonomy_ws/src/planning                         전역 경로와 LiDAR 회피
autonomy_ws/src/control                          제어기
autonomy_ws/src/f1tenth_bringup                  단일 autonomy launch
```
