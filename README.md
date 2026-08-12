# F1TENTH Onboard

실차 온보드 PC에서 사용하는 ROS 2 Humble 자율주행 소스와 차량별 설정입니다. 노트북 Docker 시뮬레이션 코드는 [Kimz1xq/f1tenth](https://github.com/Kimz1xq/f1tenth)에서 관리합니다.

## 구성

| 경로 | 내용 |
|---|---|
| `autonomy_ws/src/control` | Pure Pursuit, Linear MPC, 주행 검증 도구 |
| `autonomy_ws/src/planning` | 전역 경로와 LiDAR 기반 정적 장애물 회피 |
| `autonomy_ws/src/f1tenth_bringup` | 실차/시뮬 공용 autonomy launch |
| `config/amcl.yaml` | 실차 AMCL 설정 |
| `maps/track03.*` | 실차에서 작성한 지도 |
| `vehicle_overrides` | VESC odometry, joystick, SLAM Toolbox 및 기본 Bringup 변경본 |

`build`, `install`, `log`, rosbag 및 외부 vendor 저장소는 포함하지 않습니다.

## 빌드

기본 차량 워크스페이스 `/home/misys/f1tenth_ws`가 준비된 온보드 컨테이너를 기준으로 합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash

cd /home/misys/shared_dir/f1tenth-onboard/autonomy_ws
colcon build --symlink-install
source install/setup.bash
```

`vehicle_overrides`는 원본 패키지에 적용한 차량별 변경본입니다. 새 온보드 환경에 복구할 때 대응하는 `/home/misys/f1tenth_ws/src` 경로에 적용한 후 해당 패키지를 다시 빌드합니다.

## 실행 순서

기본 차량 Bringup은 한 번만 실행합니다.

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

지도 서버와 AMCL을 실행하고 RViz의 `2D Pose Estimate`로 초기 자세를 설정합니다. TF는 다음 연결을 만족해야 합니다.

```text
map -> odom -> base_link -> laser
```

Autonomy는 실행 시 속도와 장애물 회피 사용 여부를 지정합니다.

```bash
ros2 launch f1tenth_bringup autonomy.launch.py \
  mode:=real \
  controller:=mpc \
  mpc_profile:=speed_1.0 \
  min_command_speed:=0.30 \
  obstacles:=false
```

장애물 회피를 사용할 때만 다음처럼 변경합니다.

```bash
obstacles:=true
```

MPC는 안전을 위해 비활성 상태로 시작합니다.

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
```

즉시 정지:

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

속도를 변경할 때는 autonomy launch만 `Ctrl+C`로 종료하고 `speed_숫자`를 바꿔 다시 실행합니다. `f1tenth_stack` Bringup을 중복 실행하면 odometry와 TF가 중복 발행되어 AMCL이 발산할 수 있습니다.
