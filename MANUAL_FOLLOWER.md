# 팔로워 손 녹화 → 같은 팔로워 재생

기존 teleoperation, replay, record launch를 모두 종료합니다. 리더는 실행하지 않습니다.
아래 파일명은 `manual_take1.csv`이며 같은 파일명으로 녹화하면 덮어씁니다.
모터나 통신 오류가 나오면 재생으로 넘어가지 않습니다.

## 각 터미널에서 환경 적용

```bash
cd ~/omx_replay_ws
source /opt/ros/jazzy/setup.bash
source ~/omx_replay_ws/install/setup.bash
```

## 녹화

터미널 1: 팔이 처지지 않도록 받친 상태에서 실행합니다.
팔로워의 USB 고유 ID가 기본값이며 ACM 번호가 바뀌어도 같은 보드를 사용합니다.
다른 보드에는 `port_name:=...`를 명시합니다.

```bash
ros2 launch omx_replay omx_follower_record.launch.py
```

자동 토크 OFF 서비스 응답이 `success=True`인지 확인합니다.
관절이 토크 때문에 버티는 경우 억지로 움직이지 않습니다.

터미널 2:

```bash
ros2 run omx_replay record_trajectory.py --ros-args \
  -p joint_states_topic:=/joint_states \
  -p output_file:=$HOME/omx_replay_ws/manual_take1.csv
```

`Recorded ... samples`를 확인하고 팔로워와 그리퍼를 손으로 천천히 움직입니다.
관절 상태가 0.5초 넘게 끊기면 녹화를 종료합니다. 오류 없이 끝낸 녹화를 사용합니다.
터미널 2에서 Ctrl+C로 녹화를 종료한 후, 터미널 1도 Ctrl+C로 종료합니다.

## 재생

팔로워를 첫 녹화 자세에 가깝게 둡니다. 터미널 1:

```bash
ros2 launch omx_replay omx_follower_replay.launch.py
```

`replay_controller`가 `Configured and activated`됐는지 확인합니다.
이때 모터 토크가 켜집니다.

터미널 2: 아래 명령은 실제로 로봇을 움직입니다.

```bash
ros2 run omx_replay replay_trajectory.py --ros-args \
  -p input_file:=$HOME/omx_replay_ws/manual_take1.csv \
  -p start_delay_s:=3.0 \
  -p dxl_resources:="[dxl11, dxl12, dxl13, dxl14, dxl15, dxl16]"
```

현재 관절값에서 첫 녹화 자세까지 3초에 걸쳐 접근한 뒤 한 번 재생합니다.
이 접근 동작은 장애물을 고려한 경로 계획이 아닙니다.
CSV는 팔로워 관절값 그대로이며 그리퍼 각도 변환이나 teleoperation 보정은 적용하지 않습니다.
녹화에 닫힘 동작이 없으면 재생에서도 닫히지 않습니다.
재생 노드의 Ctrl+C는 이미 전달한 전체 궤적을 취소하지 않습니다.
재생 완료 후 로봇을 받친 상태에서 터미널 1을 Ctrl+C로 종료합니다.
