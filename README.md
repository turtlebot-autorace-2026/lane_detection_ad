# lane_detection_ad — 차선 주행 v1

선배의 `lane_detection_dl`(커밋 f1a7a74)에서 **YOLO 차선 모델, BEV 변환, 노드 인터페이스는 그대로** 쓰고, **차선 추정과 제어부만 새로 짠** 패키지다. 원본 repo는 수정하지 않았다. 모델 파일은 복사본(`models/lane_yolo26n_seg.pt`, sha256 `b7afc9ee…`)이다.

환경: ROS 2 Galactic 기준(Python 3.8 문법). Humble에서도 그대로 돈다. YOLO는 선배 코드와 같이 pip `ultralytics`를 쓴다.

## 두 버전

| 버전 | 실행 | 구간 판단 |
| --- | --- | --- |
| 일반판 | `lane_sim.launch.py` | 카메라로 본 경로 모양(곡률)으로 자동 감속 |
| 하드코딩판 | `lane_sim_hardcoded.launch.py` | `section_judge` 노드가 `config/section_table.yaml`의 구간 순서를 보고 직진/커브/지그재그/미션 모드를 `/lane/section`으로 보냄 → 차선 노드가 `config/section_profiles.yaml` 값으로 바꿈 |

차선 인식과 제어 코어는 하나다. 두 버전은 "구간을 누가 정하느냐"만 다르다.

하드코딩판 주의: 이 맵은 교차로가 좌/우 랜덤이고 주차에서 왕복 주행이 생겨서, 출발점부터 거리를 세는 방식으로는 구간을 맞출 수 없다. 그래서 미션 구간(`length_m: -1`)은 **기준점(anchor)** 이 와야 다음 구간으로 넘어간다. 미션 상태머신이 생기기 전까지는 손으로 보낸다.

```bash
ros2 topic pub --once /lane/section_anchor std_msgs/msg/String "{data: after_parking}"
```

## 빌드·실행 (Gazebo)

```bash
cd ~/autorace/ws/src && git clone https://github.com/turtlebot-autorace-2026/lane_detection_ad
cd .. && colcon build --packages-select lane_detection_ad && source install/setup.bash

# 터미널 1: 시뮬레이션
ros2 launch autorace_gazebo autorace.launch.py
# 터미널 2: 일반판
ros2 launch lane_detection_ad lane_sim.launch.py
#   또는 하드코딩판
ros2 launch lane_detection_ad lane_sim_hardcoded.launch.py
```

- `sim_camera_bridge`가 Gazebo의 `/camera/image_raw`(1280×720 Image)를 실로봇과 같은 `/camera`(640×480 JPEG)로 바꿔 준다. 그래서 선배 코드도 이 브리지만 띄우면 시뮬에서 돈다(비교용).
- 시뮬에서는 `start_enabled:=true`라서 바로 출발한다. 신호등은 아직 안 보므로 `race_manager`가 출발 실패로 기록하는데, 차선 테스트에서는 무시하면 된다.
- 디버그 화면: 창(`show:=true`) 또는 `rqt_image_view /lane_detection/debug/compressed`.
  - 초록 선 = 카메라로 본 경로, 주황 선 = 기억한 경로.

### 구간만 따로 테스트할 때 스폰 위치

텍스처에서 계산한 추정 좌표다. 안 맞으면 알려줘.

| 구간 | 터미널 1 | 하드코딩판 터미널 2 |
| --- | --- | --- |
| 지그재그 | `autorace.launch.py x:=0.0 y:=2.01 yaw:=3.1416` | `start_section:=after_parking` |
| U자 연속 커브 | `autorace.launch.py x:=-1.58 y:=2.01 yaw:=3.1416` | `start_section:=serpentine` |
| 출발 커브 | 기본 스폰 | 기본 |

## 선배 코드 대비 바뀐 점

| 문제 | 선배 코드 | v1 |
| --- | --- | --- |
| 직진 와리가리 | 22.6 cm 앞 점 하나를 향해 조향하고, 명령에 평활 필터(지연 생김). 한쪽 선만 보이면 선에서 0.6×차선폭 지점을 중심으로 써서, 양쪽↔한쪽이 바뀔 때마다 중심이 26 mm 튐 | 축 위치의 횡오차 + 헤딩 오차 + 곡률로 조향하고, 거리 기준으로 감쇠를 맞춤(`control_length`). 명령 평활 제거, 지연 0.15 s는 예측으로 보상. 한쪽 선일 때도 0.5×차선폭으로 써서 중심이 튀지 않게 함 |
| 커브에서 선 밟음 | 이미 꺾을 때 감속 | 앞 55 cm 곡률을 보고 **들어가기 전에** 감속. 조향이 한계에 걸리면 바깥으로 밀리지 않게 속도를 줄여 회전 반경을 유지 |
| 지그재그 | 2차 포물선 피팅이라 꺾임이 뭉개짐. 시야 밖 경로를 기억하지 못함 | 행마다 선을 따라가는 추적(꺾임 보존). 오도메트리로 지나온 경로를 기억해서, 카메라에 안 보이는 로봇 바로 앞 구간도 씀 |
| 시야 가장자리 | 잘린 마스크의 중심을 선 중심으로 사용 | BEV에서 실제로 보이는 영역 경계에 잘린 선은 반대쪽 끝에서 선 폭의 절반만큼으로 보정 |
| 차선 폭 | 320 px로 고정 | 양쪽 선이 보이면 학습(로그 `lw=`에 표시) |

참고: BEV 맨 아래(로봇 앞 17 cm)에서 보이는 폭은 ±12 cm뿐이다. 그래서 로봇 바로 앞에서는 두 선이 동시에 거의 안 보이고, 한쪽 선 모드가 많은 건 정상이다.

## 오프라인 결과 (Gazebo 아님)

`test/test_estimator.py` 조건:

- **가짜로 그린 완벽한 마스크**를 썼다(실제 BEV 시야 사다리꼴로 잘라냄).
- 명령 지연 0.15 s, 속도/조향 잡음을 넣었다.
- 맵 실측 형태를 썼다: 커브 중심 반경 0.28 m, 지그재그 27° 꺾임 0.33/0.65/0.33 m.

| 구간 | 최대 횡오차 |
| --- | --- |
| 직진 | ≤ 13 mm |
| 커브 | ≤ 17 mm |
| 지그재그 | 28~30 mm |
| 랩 | 약 56 s (이 구간들만) |

- 실제 YOLO 마스크 잡음, Gazebo 물리, 실제 지연은 **검증 안 됨**. 이 작업 환경에서는 ROS와 YOLO를 못 돌린다.
- 꺾임에서 멈춰서 제자리 회전하는 옵션(`kink_pivot`)은 오프라인에서 24~76 mm로 들쭉날쭉해서 **기본 꺼둠**.
- 하드코딩판 프로파일의 오프라인 이득은 작다. 커브와 지그재그는 곡률 감속이 이미 속도를 묶고 있어서 `v_max`가 거의 영향이 없다. 실질적인 이득은 직진 가속 정도다.

## 피드백 줄 때

0.5초마다 로그가 한 줄씩 나온다.

```text
[curve] both v=0.085 w=+0.31 | e_y=+4mm e_th=+2.1deg margin=36mm | k=+3.2 k_prev=3.6 kink=0 lw=286mm | infer=45ms age=120ms
```

- `e_y` = 차선 중심 대비 횡오차
- `margin` = 바퀴와 안쪽 선 사이 예상 여유
- `lw` = 학습된 차선 폭
- `age` = 이미지가 처리될 때 이미 지난 시간

선을 밟은 곳이 있으면 다음 세 가지를 주면 원인을 바로 좁힐 수 있다.

1. 그 직전 몇 초 로그
2. 디버그 화면 캡처
3. 어느 구간이었는지

## 먼저 만져볼 파라미터 (`config/lane_params.yaml`)

| 파라미터 | 기본값 | 의미 |
| --- | --- | --- |
| `control_length` | 0.12 | 작을수록 빨리 붙지만 잡음에 예민 |
| `latency` | 0.15 | 로그의 `age` + 추론 시간에 맞추기 |
| `a_lat_max` | 0.03 | 작을수록 커브에서 더 느림 |
| `v_max` | 0.12 | 최고 속도 |
| `lane_width_px` | 320 | 로그 `lw`가 안정되면 그 값으로 고정 |

## 구성

```text
lane_detection_ad/
  lane_detection_ad/lane_estimator.py     차선 추정 + 제어 (ROS 없음, 테스트 가능)
  lane_detection_ad/detection.py          ROS 노드 (선배 노드와 같은 토픽)
  lane_detection_ad/section_judge.py      하드코딩판 구간 판단
  lane_detection_ad/sim_camera_bridge.py  Gazebo 카메라 → /camera
  config/  lane_params.yaml, section_profiles.yaml, section_table.yaml
  launch/  lane.launch.py (실로봇), lane_sim.launch.py, lane_sim_hardcoded.launch.py
  test/test_estimator.py                  pytest (numpy+opencv만 필요)
```
