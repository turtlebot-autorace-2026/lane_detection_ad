# camera_ros 작업 인수인계

작성일: 2026-10-27, 시간대 Asia/Seoul(KST). 다른 채팅에서 이 파일을 읽으면 현재 카메라 구성, 캘리브레이션 결과, 복구 방법을 확인할 수 있습니다.

## 현재 완료 상태

**새 캘리브레이션을 완료하고 보정 파일을 교체했으며, publisher 재시작과 발행 시작 로그까지 확인했습니다.** 이전 보정값도 복구 가능한 상태로 보관했습니다.

| 항목 | 결과 |
| --- | --- |
| 작업 디렉터리 | `/home/atom5/turtlebot3_ws/src/camera_ros` |
| 환경 | ROS 2 Galactic, Python 3.8.10, OpenCV 4.2.0 |
| 카메라 | 장치 인덱스 0, V4L2, MJPEG 1280×720, 30fps 요청 |
| 체커보드 설정 | 내부 코너 7×9, 한 칸 18.2mm |
| 촬영 이미지 | 78장 |
| 유효한 체커보드 이미지 | 43장 |
| 재투영 RMS 오차 | 1.6178150814188486 픽셀 |
| 보정 파일 저장 시각 | 2026-10-27 23:11:23 KST |
| 적용 파일 | `camera_ros/camera_calibration.pkl` |
| 성공한 촬영 폴더 | `camera_ros/calibration_images/20261027_230907_604004/` |

실제 FPS를 측정한 것은 아닙니다. 30fps는 카메라에 요청하는 설정이며, publisher의 타이머 주기는 0.03초입니다.

완료 근거:

- `calibration_backups/20261027_230802_+0900/session.log`: `Valid checkerboard views: 43/78`, RMS 출력, 보정 파일 저장, `Calibration updated: True`.
- 같은 폴더의 `session_result.json`: `calibration_updated: true`, 재시작한 publisher PID `20817` 기록.
- 같은 폴더의 `publisher_after_calibration.log`: 세 영상 토픽의 발행 시작 로그 확인.
- 활성 보정 파일의 SHA-256이 기존 파일과 달라졌고, 저장된 메타데이터도 위 결과와 일치함.

PID와 발행 시작 로그는 당시 실행 기록입니다. 이후 프로세스가 계속 실행 중인지 확인해야 할 때는 현재 상태를 다시 조회하세요. 이번 문서 작성 과정에서는 카메라나 publisher를 재실행하지 않았습니다.

## publisher 동작과 토픽

`publisher`는 실행할 때 캘리브레이션을 새로 계산하지 않습니다. `camera_ros/camera_calibration.pkl`을 한 번 읽고, 저장된 카메라 행렬과 왜곡 계수로 영상 왜곡을 보정합니다. **보정 파일을 교체하거나 복구하면 publisher를 재시작해야 합니다.**

현재 [camera_ros/publisher.py](camera_ros/publisher.py)는 카메라를 한 번 열고 동일한 원본 프레임에서 다음 세 토픽을 만듭니다. 기본 네임스페이스 기준이며, 모두 `sensor_msgs/msg/CompressedImage`, JPEG 형식입니다.

| 토픽 | 영상 처리 |
| --- | --- |
| `/camera_raw_16_9` | 원본 1280×720, 왜곡 보정·크롭·리사이즈 없음 |
| `/camera_raw` | 원본 중앙의 960×720 영역(`frame[:, 160:1120]`)을 잘라 640×480으로 축소, 왜곡 보정 없음 |
| `/camera` | 1280×720 원본에 왜곡 보정 적용 → ROI로 검은 테두리 제거 → 640×480으로 리사이즈 |

- 세 영상은 동일한 타임스탬프를 사용합니다.
- 왜곡 보정은 `getOptimalNewCameraMatrix(alpha=0)`와 `initUndistortRectifyMap`/`remap`을 사용합니다.
- 카메라가 실제 1280×720 프레임을 반환하지 않으면 publisher는 시작을 중단합니다.
- `/camera_raw`는 16:9 원본의 좌우를 자른 영상이므로 카메라의 별도 640×480 센서 모드와 시야가 같다고 가정하면 안 됩니다.
- `publisher2`는 별도 카메라 인덱스 2를 열어 `/camera2`로 발행합니다. 이번 재캘리브레이션 대상은 카메라 0입니다.
- `launch/camera.launch.py`는 `publisher`와 `publisher2`를 모두 실행합니다.

세 토픽을 발행하는 publisher 코드는 재캘리브레이션 작업 시작 시 이미 존재했습니다. 이 채팅의 재캘리브레이션 작업에서는 publisher 코드를 추가 수정하지 않았습니다.

## 이번에 변경한 캘리브레이션 처리

[camera_ros/calibration.py](camera_ros/calibration.py)를 다음과 같이 수정했습니다.

1. publisher와 동일한 V4L2/MJPEG 1280×720, 30fps 요청 모드로 촬영합니다. 다른 크기의 프레임이나 보정 이미지는 거부합니다.
2. 실행마다 `calibration_images/<촬영시각>/` 폴더를 새로 만들어 이전 촬영분과 섞이지 않게 했습니다.
3. 1초마다 원본 프레임을 저장합니다. 영상 창에서 `q`를 누르면 촬영을 끝내고 해당 실행의 이미지로만 계산합니다.
4. 체커보드 검출에 성공한 영상이 10장 미만이면 기존 보정 파일을 유지합니다. 촬영 안내는 위치·거리·각도를 바꾼 20~30장 이상을 권장합니다.
5. RMS 오차를 출력하고, 결과에 NaN/무한대가 있으면 저장하지 않습니다.
6. 결과를 `.tmp` 파일에 쓴 뒤 `os.replace`로 보정 파일을 교체합니다.
7. 종료 시 `finally`에서 카메라·창·ROS 노드를 정리합니다. 카메라를 열지 못한 경우 기존 이미지로 계산하지 않습니다.

기존 `camera_matrix`, `dist_coeffs`, `rvecs`, `tvecs` 키를 유지하고 다음 메타데이터를 추가했습니다.

```text
image_size: (1280, 720)
rms_reprojection_error: 1.6178150814188486
valid_image_count: 43
image_directory: /home/atom5/turtlebot3_ws/src/camera_ros/camera_ros/calibration_images/20261027_230907_604004
```

캘리브레이션 행렬은 **1280×720 원본 좌표계** 기준입니다. `/camera`의 ROI 크롭·640×480 리사이즈를 반영한 행렬이 따로 저장되는 구조는 아닙니다.

로그에 출력된 새 카메라 행렬과 왜곡 계수는 다음과 같습니다. 정확한 값은 활성 `.pkl` 파일을 읽어 사용하세요.

```text
camera_matrix:
[[852.04529052   0.         649.21246448]
 [  0.         803.76239286 430.66004179]
 [  0.           0.           1.        ]]

dist_coeffs:
[[-0.64423013  0.34015099 -0.01677141 -0.00289825 -0.07864560]]
```

## 백업과 복구

아래 경로는 모두 작업 디렉터리 기준입니다.

| 폴더 | 내용 |
| --- | --- |
| `calibration_backups/20261027_225632_+0900/` | 최초 보정 파일, 기존 이미지 158장, 당시 `calibration.py`·`publisher.py`, SHA-256 manifest, 복구 스크립트 |
| `calibration_backups/20261027_230802_+0900/` | 재시작 직전 보정 파일과 두 Python 파일, manifest, 복구 스크립트, 성공한 실행의 launcher·로그·결과 JSON |

두 백업의 `camera_calibration.pkl`은 같은 기존 보정값입니다. 최초 백업의 manifest는 이미지와 코드까지 161개 파일을 포함하고, 두 번째 백업의 manifest는 보정 파일과 코드 3개를 포함합니다.

```text
기존 보정 파일 SHA-256:
891303a6cd054cd6ad2461da4013025569aa45685d9079be13d32450ec9675a6

새 활성 보정 파일 SHA-256:
c22cd925234e39a8fa587a9d9076f73fb4edcfb2b8e8110b406568a3f75a1da8
```

**기존 보정값으로 복구하려면 사용 중인 publisher를 종료한 뒤 다음을 실행하세요.**

```bash
cd /home/atom5/turtlebot3_ws/src/camera_ros
python3 calibration_backups/20261027_230802_+0900/restore.py
```

복구 스크립트는 백업의 체크섬을 확인하고, 복구 직전 활성 보정 파일을 해당 백업 폴더의 `restore_history/`에 보존한 뒤 보정 파일을 교체합니다. 촬영 이미지나 Python 코드를 되돌리는 스크립트는 아닙니다. 복구 후 publisher를 다시 실행해야 적용됩니다.

**백업 폴더와 날짜별 촬영 폴더는 `.gitignore` 대상이며 이 컴퓨터의 로컬 파일입니다.** 다른 컴퓨터나 Git clone에는 자동으로 전달되지 않으므로 필요한 경우 별도로 복사해야 합니다.

## 촬영 세션과 실행 방식

- `camera_ros/calibration_images/20261027_230212_770181/`: 최초 촬영 세션, 220장 보관. 최종 결과에 사용하지 않았습니다.
- `camera_ros/calibration_images/20261027_230907_604004/`: 다시 시작한 세션, 78장 중 43장으로 최종 보정값 생성.
- 성공한 실행은 `calibration_backups/20261027_230802_+0900/run_session.py`로 실행했습니다. 이 launcher는 기존 캘리브레이션 종료, publisher 일시 중지, 데스크톱 촬영 창 실행, 종료 후 publisher 재시작을 처리합니다.
- 재시작한 launcher는 분리된 프로세스로 실행하고 `session.log`에 출력을 기록했습니다. launcher PID 기록은 `launcher.pid`에 있습니다.
- 당시 GUI 연결은 `DISPLAY=:1`, `XAUTHORITY=/run/user/1000/gdm/Xauthority`로 확인했습니다. 새 실행 환경에서는 디스플레이 연결을 다시 확인하세요.
- `run_session.py`는 이번 작업용 실행 기록입니다. 새 촬영 전에는 새 보정 파일 백업을 먼저 만들고 로그 경로도 새로 정하세요. 같은 launcher를 재사용하면 그 폴더의 실행 결과가 갱신됩니다.

## 다시 실행할 때의 명령

환경 설정:

```bash
source /opt/ros/galactic/setup.bash
source /home/atom5/turtlebot3_ws/install/setup.bash
```

카메라 0을 사용하는 기존 publisher를 종료한 뒤 촬영:

```bash
ros2 run camera_ros calibration
```

표준 `calibration` 실행 자체는 백업 생성이나 publisher 자동 재시작을 수행하지 않습니다. 새 촬영 전에 활성 `.pkl`을 별도로 백업하고, 영상 창에서 `q`를 눌러 계산이 끝난 뒤 publisher를 실행하세요.

```bash
ros2 run camera_ros publisher
```

같은 카메라를 중복해서 열지 않도록 기존 실행을 먼저 확인하세요. 이번 환경에서는 설치된 Python 모듈이 소스 디렉터리를 참조하는 것을 확인했습니다.

## 검증 범위와 저장소 상태

- 캘리브레이션 코드의 Python 문법, 백업 체크섬, 촬영 폴더 분리와 해상도 설정을 확인했습니다.
- 임시 mock 검증으로 유효 이미지 부족·해상도 불일치 시 기존 보정 파일 유지, 정상 결과의 메타데이터 저장과 임시 파일 교체를 확인했습니다. 이 검증은 저장소에 테스트 파일로 추가하지 않았습니다.
- 실제 촬영·계산 결과, 활성 보정 파일 교체, publisher 발행 시작 로그를 확인했습니다. 실제 토픽 수신율 측정이나 별도의 정량적 왜곡 품질 평가는 수행하지 않았습니다.
- 문서 작성 시 브랜치: `feature/camera-bev-input`, HEAD: `0b9af6f` (`chore: exclude generated camera artifacts`). 직전 커밋은 `2bf2885` (`baseline: preserve camera state before BEV work`).
- 문서 작성 직전 `git status --short`에는 `camera_ros/camera_calibration.pkl`만 수정 상태였습니다. 코드와 README 변경은 당시 HEAD에 이미 포함돼 있었습니다. 이 문서와 새 보정 파일은 이 채팅에서 커밋하지 않았습니다.
- [README.md](README.md)에도 토픽 구성, 뷰어 실행 예시, 캘리브레이션 및 최초 백업 복구 안내가 있습니다.

다음 채팅에서는 이 파일과 활성 `camera_ros/camera_calibration.pkl`을 기준으로 이어가면 됩니다. 캘리브레이션은 이미 완료됐으므로 별도 요청이 없다면 다시 촬영할 필요는 없습니다.
