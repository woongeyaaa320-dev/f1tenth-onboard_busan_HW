# F1TENTH Onboard

F1TENTH 실차의 Bringup, Map 기반 AMCL, 전역경로, Linear MPC와 정적 장애물 회피를 실행하는 ROS 2 Humble 환경입니다.

노트북 시뮬레이션 코드는 [Kimz1xq/f1tenth](https://github.com/Kimz1xq/f1tenth)에서 관리합니다.

## 1. 접속

현재 LAN 주소 기준:

```bash
ssh jeonbotdae@192.168.1.7
docker exec -it f1tenth bash
```

컨테이너의 모든 새 터미널에서 먼저 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
```

IP는 네트워크가 바뀌면 달라질 수 있습니다.

## 2. 최초 빌드

현재 차량에는 `/home/misys/shared_dir/autonomy_ws`로 설치되어 있습니다.

```bash
cd /home/misys/shared_dir/autonomy_ws

colcon build --symlink-install --packages-select \
  planning control f1tenth_bringup

source install/setup.bash
```

코드를 수정하지 않았다면 매번 빌드하지 않습니다.

새 온보드 PC에 복구할 때만 host의 `shared_dir` 아래에 clone하고, 저장된 파일을 현재 실행 경로로 배치합니다.

```bash
cd /home/jeonbotdae/shared_dir
git clone https://github.com/Kimz1xq/f1tenth-onboard.git

mkdir -p autonomy_ws/src config maps
cp -a f1tenth-onboard/autonomy_ws/src/. autonomy_ws/src/
cp f1tenth-onboard/config/amcl.yaml config/
cp f1tenth-onboard/maps/track03.* maps/
```

그다음 컨테이너에서 차량 패키지 수정본을 반영하고 한 번 빌드합니다.

```bash
cp -a /home/misys/shared_dir/f1tenth-onboard/vehicle_overrides/f1tenth_stack/. \
  /home/misys/f1tenth_ws/src/f1tenth_system/f1tenth_stack/
cp -a /home/misys/shared_dir/f1tenth-onboard/vehicle_overrides/vesc_ackermann/. \
  /home/misys/f1tenth_ws/src/vesc/vesc_ackermann/
cp -a /home/misys/shared_dir/f1tenth-onboard/vehicle_overrides/joy_teleop/. \
  /home/misys/f1tenth_ws/src/teleop_tools/joy_teleop/
cp -a /home/misys/shared_dir/f1tenth-onboard/vehicle_overrides/slam_toolbox/. \
  /home/misys/f1tenth_ws/src/slam_toolbox/

cd /home/misys/f1tenth_ws
colcon build --symlink-install --packages-select \
  f1tenth_stack vesc_ackermann joy_teleop slam_toolbox

cd /home/misys/shared_dir/autonomy_ws
colcon build --symlink-install --packages-select \
  planning control f1tenth_bringup
```

## 3. 실차 실행 순서

아래 네 프로세스는 서로 다른 터미널에서 실행합니다.

### 터미널 1: 차량 Bringup

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

이 명령은 조이스틱, LiDAR, VESC, odometry와 `base_link -> laser` TF를 실행합니다. **한 번만 실행합니다.**

### 터미널 2: Map Server

```bash
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/misys/shared_dir/maps/track03.yaml \
  -p topic:=map \
  -p frame_id:=map \
  -p use_sim_time:=false
```

### 터미널 3: AMCL

```bash
ros2 run nav2_amcl amcl --ros-args \
  --params-file /home/misys/shared_dir/config/amcl.yaml \
  -p base_frame_id:=base_link \
  -p set_initial_pose:=false \
  -p use_sim_time:=false
```

### 터미널 4: Map Server와 AMCL 활성화

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate
```

## 4. RViz와 초기 자세

RViz는 GPU가 있는 노트북 Docker 컨테이너에서 실행합니다.

```bash
cd ~/Downloads/f1tenth_gym_ros_humble
docker compose exec sim bash

source /opt/ros/humble/setup.bash
source /sim_ws/install/setup.bash

rviz2 -d /sim_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/gym_bridge.rviz
```

RViz에서:

1. `Fixed Frame`을 `map`으로 설정합니다.
2. `2D Pose Estimate`를 실제 차량 위치에 찍습니다.
3. 화살표 방향을 실제 차량의 앞 방향과 정확히 맞춥니다.
4. Map과 LaserScan이 겹치는지 확인합니다.

온보드에서 TF를 확인합니다.

```bash
ros2 run tf2_ros tf2_echo map base_link
```

정상 TF:

```text
map -> odom -> base_link -> laser
```

차량 위치나 방향이 경로와 맞지 않으면 MPC를 켜지 않습니다.

## 5. 전역경로와 MPC 실행

새 터미널에서 원하는 속도로 실행합니다.

```bash
ros2 launch f1tenth_bringup autonomy.launch.py \
  mode:=real \
  controller:=mpc \
  mpc_profile:=speed_1.0 \
  min_command_speed:=0.30 \
  obstacles:=false
```

속도는 파일을 만들지 않고 숫자만 바꿉니다.

```text
mpc_profile:=speed_0.5
mpc_profile:=speed_1.0
mpc_profile:=speed_1.4
```

처음에는 `speed_0.5`로 검증한 뒤 단계적으로 올립니다. Autonomy를 재실행할 때는 이 launch만 `Ctrl+C`로 종료합니다. 차량 Bringup을 다시 실행하지 않습니다.

## 6. 주행 시작과 정지

Autonomy launch는 MPC가 꺼진 상태로 시작합니다. RViz 정합을 확인한 다음 실행합니다.

```bash
# 주행 시작
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"

# 즉시 정지
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

시작이 거부되면 다음을 확인합니다.

```bash
ros2 param get /linear_mpc_node enabled
ros2 topic echo /mpc/proposed_drive --once
ros2 run tf2_ros tf2_echo map base_link
```

## 7. 장애물 회피

장애물 없는 기본 주행이 정상일 때만 다음처럼 켭니다.

```bash
ros2 launch f1tenth_bringup autonomy.launch.py \
  mode:=real \
  controller:=mpc \
  mpc_profile:=speed_0.5 \
  min_command_speed:=0.30 \
  obstacles:=true
```

```bash
ros2 topic echo /planning/local_status
ros2 topic echo /safety/emergency_stop
```

상태 의미:

```text
GLOBAL_PATH_CLEAR       장애물 없음
AVOIDING ...            회피 경로 생성됨
NO_COLLISION_FREE_PATH  안전한 경로 없음
AEB_STOP                긴급 정지
```

장애물은 차량에서 최소 2 m 이상 앞에 놓고 저속부터 검증합니다.

## 8. 반드시 지킬 것

- `f1tenth_stack bringup_launch.py`를 두 번 실행하지 않습니다.
- 중복 Bringup은 `/odom`과 TF를 중복 발행해 AMCL을 발산시킵니다.
- `2D Pose Estimate`의 화살표 방향을 차량 앞 방향과 맞춥니다.
- MPC가 벽으로 향하면 즉시 `/control/enable`을 `false`로 호출합니다.
- 속도 변경 시 Autonomy launch만 재실행합니다.

## 저장소 구성

```text
autonomy_ws/src/control          Linear MPC, Pure Pursuit, 검증 도구
autonomy_ws/src/planning         전역경로와 정적 장애물 회피
autonomy_ws/src/f1tenth_bringup  단일 Autonomy launch
config/amcl.yaml                 실차 AMCL 설정
maps/track03.*                   실차 지도
vehicle_overrides/               VESC, odometry, joystick, SLAM 변경본
```

`build`, `install`, `log`, rosbag과 외부 vendor 전체 소스는 저장소에서 제외합니다.
