# lane_detection_ad — 차선 주행 v1 (ROS 2 Galactic)

선배의 `lane_detection_dl`(커밋 f1a7a74)에서 **YOLO 차선 모델, BEV 변환, 토픽 인터페이스는 그대로** 쓰고, **차선 추정과 제어부만 새로 짠** 패키지다.

- 원본 repo는 수정하지 않았다. 모델은 복사본이다(`models/lane_yolo26n_seg.pt`, sha256 `b7afc9ee…`).
- 기준 환경은 Ubuntu 20.04 + ROS 2 Galactic + Python 3.8이다. Humble에서도 같은 명령으로 돈다.

목차:

1. 설치 (워크스페이스부터 빌드까지)
2. 실차 실행
3. Gazebo 실행
4. 두 버전 (일반판 / 하드코딩판)
5. 선배 코드 대비 바뀐 점과 오프라인 결과
6. 로그, 피드백, 파라미터

---

## 1. 설치

모든 터미널에서 먼저 실행한다(`~/.bashrc`에 넣어 두면 편하다).

```bash
source /opt/ros/galactic/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=30          # 로봇과 PC가 같은 값이어야 함 (팀에서 쓰던 값으로)
```

### 1-1. 워크스페이스와 패키지 가져오기

이 repo 하나에 패키지 두 개가 들어 있다. clone 한 번이면 끝이다(`ros2 pkg create` 필요 없음).

| 폴더 | 내용 |
| --- | --- |
| `lane_detection_ad/` | 차선 패키지 (이번에 만든 것) |
| `camera_ros/` | 선배 `turtlebot-autorace-2026/camera_ros`를 그대로 복사한 것 (커밋 cf15991, 2026-10-28, 수정 없음). 실차 launch가 여기 있는 `publisher`(카메라 → `/camera`)와 `test`(영상 뷰어)를 띄운다 |

워크스페이스는 기존 `~/turtlebot3_ws`와 **따로** `~/autorace_ad_ws`로 만든다.

- 기존 워크스페이스에 넣으면 선배 `camera_ros`가 두 개가 되어 colcon이 "Duplicate package names"로 멈춘다.
- 기존 워크스페이스는 건드리지 않는다.

```bash
mkdir -p ~/autorace_ad_ws/src
cd ~/autorace_ad_ws/src
git clone https://github.com/turtlebot-autorace-2026/lane_detection_ad
```

선배가 `camera_ros`를 업데이트하면 이 복사본에는 자동으로 반영되지 않는다. 다시 복사해서 커밋해야 한다.

### 1-2. 의존성

```bash
sudo apt install -y python3-numpy python3-opencv python3-yaml ros-galactic-rqt-image-view
pip3 show ultralytics || pip3 install ultralytics
```

- `ultralytics`는 선배 `lane_detection_dl`이 돌던 PC라면 이미 깔려 있다. 그대로 두고 **업그레이드하지 말 것**.
- pip가 numpy를 올리면 ROS의 cv_bridge 등이 깨질 수 있다.

### 1-3. 빌드

```bash
cd ~/autorace_ad_ws
colcon build --symlink-install --packages-select camera_ros lane_detection_ad
source ~/autorace_ad_ws/install/setup.bash
```

`--symlink-install`은 꼭 붙인다. `camera_ros`가 보정 파일(`camera_calibration.pkl`)을 소스 폴더 기준으로 찾기 때문에, 이 옵션이 없으면 카메라 노드가 시작하지 않는다.

### 1-4. 확인

```bash
ros2 pkg executables camera_ros
# camera_ros calibration / publisher / publisher2 / test / test2
ros2 pkg executables lane_detection_ad
# lane_detection_ad detection / section_judge / sim_camera_bridge
```

둘 중 하나라도 안 나오면 `source ~/autorace_ad_ws/install/setup.bash`를 안 했거나 빌드가 실패한 것이다.

`~/.bashrc`에서 이미 `~/turtlebot3_ws/install/setup.bash`를 source하고 있어도 괜찮다. **`~/autorace_ad_ws`를 그 뒤에 source하면** `camera_ros`는 이 repo의 것이 쓰인다(내용은 같음).

선택: ROS 없이 돌아가는 오프라인 테스트다.

```bash
cd ~/autorace_ad_ws/src/lane_detection_ad/lane_detection_ad && python3 -m pytest -q -s test
```

---

## 2. 실차 실행

`lane_robot.launch.py` 하나가 아래 세 개를 같이 띄운다. 그래서 로봇에서는 터미널 2개면 된다.

- 선배 `camera_ros publisher`: 카메라 → `/camera`
- 선배 `camera_ros test`: `/camera` 영상 창
- 차선 노드: 디버그 창 포함

```bash
# 터미널 1 (로봇): 모터, 오도메트리. turtlebot3_bringup이 있는 기존 워크스페이스 그대로
source /opt/ros/galactic/setup.bash
source ~/turtlebot3_ws/install/setup.bash        # turtlebot3를 apt로 깔았으면 이 줄 생략
ros2 launch turtlebot3_bringup robot.launch.py

# 터미널 2 (로봇): 카메라 + 뷰어 + 차선 노드. 기본은 대기 상태라 바로 출발하지 않음. v_max 기본 0.08
source /opt/ros/galactic/setup.bash
source ~/autorace_ad_ws/install/setup.bash
ros2 launch lane_detection_ad lane_robot.launch.py 2>&1 | tee ~/lane_log_$(date +%m%d_%H%M).txt
```

**카메라 보정은 선배 것 그대로다.**

- 차선 노드는 선배 `camera_ros publisher`가 보정 파일(`camera_calibration.pkl`)로 왜곡 보정한 `/camera`(640×480)를 받는다.
- 그 위에 선배 `lane_detection_dl`의 BEV 변환을 그대로 쓴다. 다시 보정하는 코드는 없다.
- 카메라를 다시 캘리브레이션하거나 카메라 각도를 바꾸면, 보정 후 영상의 크롭이 달라진다. 그러면 BEV 기준점(`lane_estimator.py`의 `BEV_SRC`)도 다시 맞춰야 한다.

- `camera_ros publisher`를 이미 따로 띄웠으면 `camera:=false`를 준다. 같은 카메라를 두 번 열면 충돌한다.
- 창이 3개 뜬다(`publisher`의 보정 영상, `test` 영상, 차선 디버그). 로봇에 화면(모니터나 원격 데스크톱)이 있어야 한다. 창을 줄이려면 `viewer:=false` 또는 `show:=false`를 준다.
- YOLO를 노트북에서 돌리려면 로봇에서는 `ros2 run camera_ros publisher`만 띄운다. 노트북에서는 `lane_robot.launch.py camera:=false`를 실행한다(같은 `ROS_DOMAIN_ID`). 이 경우 카메라 JPEG가 Wi-Fi로 넘어오므로 로그의 `age`(지연)를 꼭 확인한다.

### 출발, 정지, 비상 정지

```bash
ros2 topic pub --once /lane_state std_msgs/msg/Bool "{data: true}"     # 출발
ros2 topic pub --once /lane_state std_msgs/msg/Bool "{data: false}"    # 정지 (속도 0 발행)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"             # 비상 정지
```

터미널 3에서 Ctrl+C를 눌러도 정지 명령을 보내고 종료한다.

### launch 옵션 (`lane_robot.launch.py`)

| 인자 | 기본 | 설명 |
| --- | --- | --- |
| `start_enabled` | false | true면 `/lane_state` 없이 바로 출발 |
| `v_max` | 0.08 | 최고 속도 m/s. 첫 주행은 낮게, 안정되면 0.10 → 0.12 |
| `show` | true | 차선 디버그 창 (선, 중심 경로, 로그 숫자) |
| `viewer` | true | 선배 `camera_ros test` 영상 창 (`/camera`) |
| `camera` | true | 선배 `camera_ros publisher`를 같이 띄움. 이미 따로 띄웠으면 false |
| `hardcoded` | false | true면 구간 판단 노드(`section_judge`)도 띄움. 4번 참고 |
| `start_section` | '' | 하드코딩판의 시작 구간 이름 |
| `params_file` | config/lane_params.yaml | 파라미터 파일 교체 |

예시:

```bash
ros2 launch lane_detection_ad lane_robot.launch.py start_enabled:=true v_max:=0.10
```

### 보기와 기록

```bash
ros2 run rqt_image_view rqt_image_view /lane_detection/debug/compressed
ros2 bag record -o lane_test_$(date +%m%d_%H%M) /camera /odom /cmd_vel /lane_detection/debug/compressed
```

디버그 화면 색:

- 노란 점/흰 점: 추적한 선
- 초록 선: 카메라로 본 중심 경로
- 주황 선: 기억한 경로
- 노란 사각형: 관심 영역(ROI)

### 첫 실차 체크리스트

1. 로봇을 차선 가운데에 놓고, 출발 전에 rqt_image_view로 양쪽 선이 잡히는지 확인한다.
2. 비상 정지 명령을 다른 터미널에 미리 입력해 둔다.
3. 직진 → 커브 → 지그재그 순서로, `v_max:=0.08`부터 시작한다.
4. 직진에서 로그 `lw=`가 안정되면 그 값을 기록한다(차선 폭 학습값).
5. 동아리 맵은 시뮬 맵과 다를 수 있다. **일반판(기본)으로 돌린다**. 하드코딩판 구간 표는 시뮬/규정 맵 기준이다.

---

## 3. Gazebo 실행

`autorace_gazebo`(hakee 맵)도 같은 `~/autorace_ad_ws`에 넣고 빌드한다.

```bash
cp -r autorace_gazebo ~/autorace_ad_ws/src/        # hakee가 준 Gazebo 패키지
cd ~/autorace_ad_ws && colcon build --symlink-install --packages-select autorace_gazebo lane_detection_ad
source ~/autorace_ad_ws/install/setup.bash

# 터미널 1
ros2 launch autorace_gazebo autorace.launch.py
# 터미널 2: 일반판
ros2 launch lane_detection_ad lane_sim.launch.py
# 터미널 2: 하드코딩판
ros2 launch lane_detection_ad lane_sim.launch.py hardcoded:=true
```

- `sim_camera_bridge`가 Gazebo의 `/camera/image_raw`(1280×720 Image)를 실로봇과 같은 `/camera`(640×480 JPEG)로 바꿔 준다. 선배 `lane_detection_dl`도 이 브리지만 띄우면 시뮬에서 돈다(비교용).
- 시뮬은 바로 출발한다(`start_enabled` 기본 true). 신호등은 아직 안 보므로 `race_manager`가 출발 실패로 기록하는데, 차선 테스트에서는 무시하면 된다.
- 시뮬의 도로 단차는 0.5 mm라서 선을 넘어도 떨어지지 않는다. 로그의 `margin`이 음수면 선을 밟은 것이다.

### 구간만 따로 테스트할 때 스폰 위치

텍스처에서 계산한 추정 좌표다. 안 맞으면 알려줘.

| 구간 | 터미널 1 인자 | 하드코딩판 터미널 2 인자 |
| --- | --- | --- |
| 지그재그 | `x:=0.0 y:=2.01 yaw:=3.1416` | `hardcoded:=true start_section:=after_parking` |
| U자 연속 커브 | `x:=-1.58 y:=2.01 yaw:=3.1416` | `hardcoded:=true start_section:=serpentine` |
| 출발 커브 | 기본 | `hardcoded:=true` |

---

## 4. 두 버전

| 버전 | 실행 | 구간 판단 |
| --- | --- | --- |
| 일반판 | 기본 (`hardcoded:=false`) | 카메라로 본 경로 모양(곡률)으로 커브 전에 자동 감속 |
| 하드코딩판 | `hardcoded:=true` | `section_judge`가 `config/section_table.yaml`의 구간 순서와 오도메트리 거리로 현재 구간을 정해 `/lane/section`으로 보냄. 차선 노드는 그 구간의 `config/section_profiles.yaml` 값을 적용 |

차선 인식과 제어 코어는 하나다. 두 버전은 "구간을 누가 정하느냐"만 다르다.

하드코딩판 주의:

- 이 맵은 교차로가 좌/우 랜덤이고 주차에서 왕복 주행이 생긴다. 그래서 출발점부터 거리를 세는 방식으로는 구간을 맞출 수 없다.
- 그래서 미션 구간(`length_m: -1`)은 **기준점(anchor)** 이 와야 다음 구간으로 넘어간다. 미션 상태머신이 생기기 전까지는 손으로 보낸다.

```bash
ros2 topic pub --once /lane/section_anchor std_msgs/msg/String "{data: after_parking}"
```

구간 표의 길이는 텍스처에서 잰 추정값이다. 프로파일의 오프라인 이득은 작다(5절 참고).

---

## 5. 선배 코드 대비 바뀐 점

| 문제 | 선배 코드 | v1 |
| --- | --- | --- |
| 직진 와리가리 | 22.6 cm 앞 점 하나를 향해 조향하고, 명령에 평활 필터(지연 생김). 한쪽 선만 보이면 선에서 0.6×차선폭 지점을 중심으로 써서, 양쪽↔한쪽이 바뀔 때마다 중심이 26 mm 튐 | 축 위치의 횡오차 + 헤딩 오차 + 곡률로 조향하고, 거리 기준으로 감쇠를 맞춤(`control_length`). 명령 평활 제거, 지연은 예측으로 보상. 한쪽 선일 때도 0.5×차선폭으로 써서 중심이 튀지 않게 함 |
| 커브에서 선 밟음 | 이미 꺾을 때 감속 | 앞 55 cm 곡률을 보고 들어가기 전에 감속. 조향이 한계에 걸리면 속도를 줄여 회전 반경을 유지 |
| 지그재그 | 2차 포물선 피팅이라 꺾임이 뭉개짐. 시야 밖 경로를 기억하지 못함 | 행마다 선을 따라가는 추적(꺾임 보존). 오도메트리로 지나온 경로를 기억해서, 카메라에 안 보이는 로봇 바로 앞 구간도 씀 |
| 시야 가장자리 | 잘린 마스크의 중심을 선 중심으로 사용 | BEV에서 실제로 보이는 영역 경계에 잘린 선은 반대쪽 끝에서 선 폭의 절반만큼으로 보정 |
| 차선 폭 | 320 px로 고정 | 양쪽 선이 보이면 학습(로그 `lw=`) |
| 선 놓침 | 즉시 정지 | 0.1 m 또는 1 s 동안 기억한 경로로 감속 주행 → 정지 → 1 s 뒤 마지막 쪽으로 회전 탐색(30초 규칙 대비) |
| 카메라 끊김 | 마지막 명령 유지 | 0.5 s 동안 새 영상이 없으면 정지 |

호환: 선배 노드와 같은 토픽(`/camera`, `/lane_state`, `/lane_mode`, `/intersection_state`, `cmd_vel`, `/lane_detection/debug/compressed`)을 쓴다. 추가로 `/odom`을 구독한다.

미션 FSM(`missions_v1`)용 입력:

| 토픽 | 타입 | 동작 |
| --- | --- | --- |
| `/lane_state` | Bool | 바뀔 때만 반응. 같은 값을 반복해 보내도 추정기를 초기화하지 않는다(새로 시작하려면 false → true) |
| `/lane/follow_line` | String | `both`(기본) / `yellow` / `white`. 지정한 선 하나만 따라간다(교차로 분기용) |
| `/intersection_state` | Bool | 속도 제한(`v_max_intersection`)만 켠다. 선배 코드의 25/12/28초 타이머로 `lane_mode`를 바꾸던 동작은 지웠다 |

참고: BEV 맨 아래(로봇 앞 17 cm)에서 보이는 폭은 ±12 cm뿐이다. 그래서 로봇 바로 앞에서는 두 선이 동시에 거의 안 보이고, 한쪽 선 모드가 많은 건 정상이다.

### 오프라인 결과 (Gazebo, 실차 아님)

`test/test_estimator.py` 조건:

- **가짜로 그린 완벽한 마스크**를 썼다(실제 BEV 시야 사다리꼴로 잘라냄).
- 명령 지연 0.15 s, 속도/조향 잡음을 넣었다.
- 맵 실측 형태를 썼다: 커브 중심 반경 0.28 m, 지그재그 27° 꺾임 0.33/0.65/0.33 m.

| 구간 | 최대 횡오차 |
| --- | --- |
| 직진 | ≤ 13 mm |
| 커브 | ≤ 17 mm |
| 지그재그 | 28~30 mm |

- 실제 YOLO 마스크 잡음, 실제 지연, Gazebo와 실차 동작은 **아직 검증 안 됨**.
- 꺾임에서 멈춰서 제자리 회전하는 옵션(`kink_pivot`)은 오프라인에서 24~76 mm로 들쭉날쭉해서 기본 꺼둠.
- 하드코딩판 프로파일에서 `v_max`는 커브와 지그재그 오차에 거의 영향이 없다. 곡률 감속이 이미 속도를 묶고 있어서, 이득은 직진 가속 정도다.

---

## 6. 로그, 피드백, 파라미터

0.5초마다 로그가 한 줄씩 나온다.

```text
[auto] both v=0.085 w=+0.31 | e_y=+4mm e_th=+2.1deg margin=36mm | k=+3.2 k_prev=3.6 kink=0 lw=286mm | infer=45ms age=120ms
```

| 항목 | 의미 |
| --- | --- |
| `[auto]` | 현재 구간 모드 |
| `both / yellow_only / white_only / memory / lost / search` | 경로를 어디서 얻었는지 |
| `e_y` | 차선 중심 대비 횡오차 (+ = 중심이 왼쪽) |
| `margin` | 바퀴와 안쪽 선 사이 예상 여유. 0 이하 = 선 밟음 |
| `k`, `k_prev` | 지금 곡률 / 앞쪽 최대 곡률 |
| `lw` | 학습된 차선 폭 |
| `infer` | YOLO 추론 시간 |
| `age` | 이미지가 처리될 때 이미 지난 시간 |

선을 밟으면 다음 세 가지를 주면 원인을 바로 좁힐 수 있다.

1. 그 직전 몇 초 로그
2. 디버그 화면 캡처 (또는 위의 bag 파일)
3. 어느 구간이었는지

먼저 만져볼 파라미터는 아래와 같다. `config/lane_params.yaml`을 고치고 다시 빌드하거나, `--ros-args -p` 또는 `params_file:=`로 바꾼다. 실수형 값에는 반드시 소수점을 쓴다(0.1 O, 1 X).

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `control_length` | 0.12 | 작을수록 빨리 붙지만 잡음에 예민 |
| `latency` | 0.15 | 로그의 `age` + `infer`에 맞추기 (초) |
| `a_lat_max` | 0.03 | 작을수록 커브에서 더 느림 |
| `v_max` | 0.12 | 최고 속도. 실차 launch는 0.08로 덮어씀 |
| `lane_width_px` | 320 | 로그 `lw`가 안정되면 그 값(mm ÷ 0.806)으로 고정 |
| `conf` | 0.4 | YOLO 신뢰도 임계값 |

## 구성

```text
lane_detection_ad/
  lane_detection_ad/lane_estimator.py     차선 추정 + 제어 (ROS 없음, 테스트 가능)
  lane_detection_ad/detection.py          ROS 노드 (선배 노드와 같은 토픽)
  lane_detection_ad/section_judge.py      하드코딩판 구간 판단
  lane_detection_ad/sim_camera_bridge.py  Gazebo 카메라 → /camera
  config/lane_params.yaml                 노드 파라미터
  config/section_profiles.yaml            구간별 파라미터
  config/section_table.yaml               하드코딩 구간 순서
  launch/lane_robot.launch.py             실차
  launch/lane_sim.launch.py               Gazebo
  test/test_estimator.py                  pytest (numpy+opencv만 필요)
```
