# OMX trajectory record & replay

OpenMANIPULATOR-X 리더/팔로워 로봇의 관절 궤적을 30 Hz로 CSV에 기록하고, 두 로봇 중 하나에 같은 궤적을 전송하는 ROS 2 패키지입니다.

> 재생 명령은 실제 로봇을 움직입니다. 단일 로봇 재생은 현재 자세에서 첫 녹화 자세까지 기본 3초에 걸쳐 접근한 뒤 시작합니다. 이 접근 동작은 장애물을 고려하지 않습니다. 재생 노드의 Ctrl+C만으로 이미 전송한 전체 궤적이 취소되지는 않습니다.


## 현재 권장 사용법: 팔로워 손 녹화 → 팔로워 재생

**[MANUAL_FOLLOWER.md](MANUAL_FOLLOWER.md)**에 터미널별 실행 순서를 정리했습니다.
리더나 teleoperation을 켜지 않고, 토크를 끈 팔로워를 손으로 움직여 관절값을 기록합니다.
그리퍼도 기록된 팔로워 각도 그대로 재생하므로 리더→팔로워 각도 변환을 사용하지 않습니다.

- `omx_follower_record.launch.py`: 팔로워 상태 발행 후 토크 OFF 서비스 호출.
- `record_trajectory.py`: `/joint_states`를 명시하여 30 Hz CSV 저장. 유효한 상태가 0.5초 끊기면 녹화 종료.
- `omx_follower_replay.launch.py`: 팔로워 재생 컨트롤러 실행.
- `replay_trajectory.py`: `start_delay_s`(기본 3초) 동안 첫 자세에 접근한 뒤 재생. 연결과 최신 관절 상태가 없으면 명령을 보내지 않습니다.

팔로워 launch 기본 포트는 개발 장비의 `/dev/serial/by-id/...`입니다.
다른 PC/보드에서는 `port_name:=/dev/serial/by-id/<YOUR_FOLLOWER>`로 변경하세요.
녹화 CSV 및 `build/`, `install/`, `log/`는 저장소에서 제외됩니다.
아래 리더·듀얼 로봇 절차는 별도 운용 방식입니다. 리더 CSV를 팔로워에 그대로 재생하면
그리퍼 각도 변환이 적용되지 않으므로 팔로워 재생에는 팔로워에서 녹화한 CSV를 사용하세요.

## 환경

- Ubuntu 24.04 / ROS 2 Jazzy
- OpenMANIPULATOR-X
- Python 3
- `colcon`, `rosdep`
- `open_manipulator_description`
- `dynamixel_hardware_interface`
- `ros2_control` 및 `joint_trajectory_controller`

## 실행 전 준비 사항

이 저장소에는 `omx_replay` 코드와 설정만 포함됩니다. 팀원 PC에는 ROS 2와 OpenMANIPULATOR-X 드라이버가 별도로 설치되어 있어야 합니다.

### 필수 항목

- Ubuntu 24.04 / ROS 2 Jazzy
- Git, `colcon`, `rosdep`
- `open_manipulator_description`, `dynamixel_hardware_interface`
- `controller_manager`, `joint_trajectory_controller`
- `joint_state_broadcaster`, `robot_state_publisher`, `xacro`

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash

cd ~/omx_replay_ws
sudo rosdep init  # 처음 한 번만; 이미 초기화됐다면 생략
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

주요 ROS 패키지가 준비됐는지 확인합니다.

```bash
ros2 pkg prefix open_manipulator_description
ros2 pkg prefix dynamixel_hardware_interface
ros2 pkg prefix controller_manager
ros2 pkg prefix joint_trajectory_controller
ros2 pkg prefix joint_state_broadcaster
ros2 pkg prefix robot_state_publisher
ros2 pkg prefix xacro
```

각 명령이 설치 경로를 출력하면 준비된 것입니다. `Package not found` 또는 rosdep 오류가 나오면 팀에서 사용 중인 OpenMANIPULATOR-X/ROBOTIS 드라이버를 추가로 설치하세요. 개발 PC에서는 주요 패키지가 `/opt/ros/jazzy`에 설치되어 있습니다.

### 하드웨어 체크리스트

- 관절 이름과 Dynamixel ID가 저장소 설정과 같은지 확인
- 리더 포트 `/dev/ttyACM2`, 재생 로봇 포트 `/dev/ttyACM0` 확인
- 사용자가 `dialout` 그룹에 포함됐는지 확인
- Dynamixel 모델 경로 `/param/dxl_model` 확인
- 재생 전 작업 공간과 비상 정지 수단 확인

```bash
ls -l /dev/ttyACM*
groups
ls -ld /param/dxl_model
```

포트가 다르면 launch의 `port_name`을 바꾸세요. `/param/dxl_model`이 없다면 [ros2_control xacro](src/omx_replay/ros2_control/omx_l_replay.ros2_control.xacro)의 `dynamixel_model_folder`를 설치 환경에 맞게 수정해야 합니다.

> 기록용 launch는 `joint_trajectory_command_broadcaster`를 사용합니다. 해당 컨트롤러를 찾지 못하면 리더 로봇에서 사용하던 동일한 ROBOTIS 드라이버 패키지가 필요합니다.

## 1. 내려받기 및 빌드

```bash
cd ~
git clone https://github.com/DUARO-hanium/replay_final.git omx_replay_ws
cd omx_replay_ws

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

새 터미널을 열 때마다 아래 두 줄을 실행합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/omx_replay_ws/install/setup.bash
```

## 2. USB 포트 확인

연결된 포트를 확인합니다.

```bash
ls -l /dev/ttyACM*
```

기본 포트는 리더(기록) `/dev/ttyACM2`, 재생 로봇 `/dev/ttyACM0`입니다. 환경이 다르면 아래 실행 명령의 `port_name`을 변경하세요.

포트 권한 오류가 발생하면 사용자를 `dialout` 그룹에 추가한 뒤 로그아웃/로그인합니다.

```bash
sudo usermod -aG dialout "$USER"
```

## 3. 궤적 기록

터미널 1에서 리더 로봇을 실행합니다.

```bash
cd ~/omx_replay_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch omx_replay omx_leader_record.launch.py port_name:=/dev/ttyACM2
```

컨트롤러가 모두 올라온 다음, 터미널 2에서 기록을 시작합니다.

```bash
cd ~/omx_replay_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run omx_replay record_trajectory.py
```

팔로워를 기록하려면 하드웨어 launch와 기록 topic만 바꿉니다. 팔로워의 기존
`open_manipulator_description` xacro를 재사용하므로 별도 follower xacro는 필요하지 않습니다.

```bash
ros2 launch omx_replay omx_follower_record.launch.py port_name:=/dev/ttyACM0
ros2 run omx_replay record_trajectory.py --ros-args \
  -p joint_states_topic:=/joint_states
```

팔로워 record launch의 controller update rate는 WSL USB/IP 통신 부하를 줄이고
기록 주기와 맞추기 위해 30 Hz로 설정됩니다.
또한 joint state broadcaster가 시작된 후 모터 토크를 자동으로 끄므로, 팔이 처지지
않도록 받친 상태에서 launch하세요. 기록 중에는 별도의 제어 명령을 보내지 않습니다.

리더 로봇을 움직여 궤적을 기록하고 `Ctrl+C`로 종료합니다. 워크스페이스 루트에 `trajectory.csv`가 생성됩니다. 기록 대상 토픽은 `/leader/joint_states`이며 다음 관절을 저장합니다.

```text
joint1, joint2, joint3, joint4, joint5, gripper_joint_1
```

## 4. 궤적 재생

재생 전에 `~/omx_replay_ws/trajectory.csv`가 존재하는지 확인하세요.

터미널 1에서 재생 로봇의 컨트롤러를 실행합니다.

```bash
cd ~/omx_replay_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch omx_replay omx_leader_replay.launch.py port_name:=/dev/ttyACM0
```

컨트롤러가 활성화된 것을 확인한 뒤, 터미널 2에서 궤적을 한 번 재생합니다.

```bash
cd ~/omx_replay_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run omx_replay replay_trajectory.py
```

팔로워에 재생하려면:

```bash
ros2 launch omx_replay omx_follower_replay.launch.py port_name:=/dev/ttyACM0
ros2 run omx_replay replay_trajectory.py --ros-args \
  -p dxl_resources:="[dxl11, dxl12, dxl13, dxl14, dxl15, dxl16]"
```

stock follower xacro는 `Present Current`/`Present Input Voltage` state interface를 노출하지
않으므로, 위 구성에서 궤적 재생은 정상 동작하지만 `replay_metrics.csv`의
전류·전압·전력 값은 `nan`이 될 수 있습니다. 팔로워 metrics까지 필요하면
ros2_control xacro에 해당 state interface만 추가해야 합니다.

### 팔로워 두 대 동시 재생 + 전류 기록

두 팔로워를 각각 `follower1`, `follower2` namespace로 실행합니다. 포트 번호는
바뀐 수 있으므로 `/dev/serial/by-id` 경로 사용을 권장합니다.

```bash
ros2 launch omx_replay omx_dual_follower_replay.launch.py \
  port1:=/dev/serial/by-id/<FIRST_FOLLOWER> \
  port2:=/dev/serial/by-id/<SECOND_FOLLOWER>

ros2 run omx_replay replay_dual_followers.py
```

같은 `trajectory.csv`가 두 대에 동시 전송되고, 각 로봇 ID 11~16의 전류·전압·전력은
`dual_replay_metrics.csv`에 `follower1_*`, `follower2_*` 열로 저장됩니다.

단일 로봇 재생 노드는 최신 `/joint_states`와 구독자 연결을 기다린 뒤 `/replay_controller/joint_trajectory`로 시작 접근 구간을 포함한 전체 궤적을 한 번 전송하고, 동작이 끝나면 종료됩니다.

## 문제 해결

컨트롤러 상태 확인:

```bash
ros2 control list_controllers
```

관절 상태 토픽 확인:

```bash
ros2 topic echo /leader/joint_states --once
```

코드를 수정한 뒤 다시 빌드:

```bash
cd ~/omx_replay_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`trajectory.csv`는 로봇별 로컬 실행 데이터이므로 Git에는 포함되지 않습니다.
