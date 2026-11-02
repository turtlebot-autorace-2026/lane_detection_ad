# sub_pub_pkg
sub_pub_pkg for Gumi High school project

## 보정 전 카메라 영상 보기

`publisher`는 카메라 0을 MJPEG 1280×720, 30fps 모드로 열고, 같은 캡처
프레임에서 세 토픽을 발행합니다. 실제 발행 속도는 영상 처리 시간에 따라 달라집니다.

| 토픽 | 메시지 타입 | 영상 |
| --- | --- | --- |
| `/camera_raw_16_9` | `sensor_msgs/msg/CompressedImage` | 16:9, 1280×720 원본 영상. 왜곡 보정·크롭·리사이즈 없음, JPEG 압축 |
| `/camera_raw` | `sensor_msgs/msg/CompressedImage` | 4:3, 중앙 960×720 영역을 640×480으로 축소한 보정 전 영상, JPEG 압축 |
| `/camera` | `sensor_msgs/msg/CompressedImage` | 기존 왜곡 보정·크롭 후 640×480 영상, JPEG 압축 |

세 영상에는 동일한 타임스탬프가 들어갑니다. 4:3 영상은 16:9 원본의 좌우를
각 160픽셀 잘라내므로 가로 시야가 좁아집니다. 카메라를 두 번 열거나
서로 다른 센서 모드로 동시에 캡처하는 방식은 아닙니다. `/camera_raw`는
이전의 카메라 기본 640×480 모드와 시야가 달라질 수 있습니다.

`publisher`는 기존과 같이 `camera_ros/camera_calibration.pkl` 파일이 필요합니다.
현재 파일은 저장된 캘리브레이션 이미지와 같은 1280×720 해상도를 기준으로 합니다.
다른 카메라 또는 센서 모드의 보정 파일로 교체할 때는 다시 캘리브레이션해야 합니다.
카메라가 실제 1280×720 프레임을 반환하지 않으면 오류를 내고 시작을 중단합니다.

빌드 후 카메라 노드를 실행합니다.

```bash
cd /home/atom5/turtlebot3_ws
source /opt/ros/galactic/setup.bash
colcon build --symlink-install --packages-select camera_ros
source install/setup.bash
ros2 run camera_ros publisher
```

다른 터미널에서 기존 뷰어의 구독 토픽을 바꿔 16:9 원본 영상을 확인합니다.
영상 창에서 `q`를 누르면 종료됩니다.

```bash
source /opt/ros/galactic/setup.bash
source /home/atom5/turtlebot3_ws/install/setup.bash
ros2 run camera_ros test --ros-args -r __node:=camera_wide_viewer -r /camera:=/camera_raw_16_9
```

또 다른 터미널에서 아래 명령을 실행하면 4:3 보정 전 영상도 동시에 볼 수 있습니다.

```bash
source /opt/ros/galactic/setup.bash
source /home/atom5/turtlebot3_ws/install/setup.bash
ros2 run camera_ros test --ros-args -r __node:=camera_4_3_viewer -r /camera:=/camera_raw
```

이미 `ros2 launch camera_ros camera.launch.py`로 실행 중이면 해당 실행을
종료하고 다시 실행하면 됩니다. `publisher`를 중복 실행하면 같은 카메라 장치를
열려고 하므로 둘 중 하나만 실행합니다. `/camera2`는 카메라 2의 별도 영상입니다.

## 캘리브레이션 다시 실행하기

카메라 0을 사용하는 publisher를 종료한 뒤 아래 명령을 실행합니다.

```bash
ros2 run camera_ros calibration
```

publisher와 동일한 MJPEG 1280×720, 30fps 모드로 촬영합니다. 체커보드는
내부 코너 7×9, 한 칸 18.2mm 기준입니다. 새 이미지는 매 실행마다 별도의
`camera_ros/calibration_images/<촬영시각>/` 폴더에 저장하며, 예전 촬영분과 섞지 않습니다.
보드의 위치·거리·각도를 바꾸어 20~30장 이상 촬영한 후 영상 창에서 `q`를 누릅니다.
유효한 체커보드 영상이 10장 이상일 때 보정 파일을 교체하고 재투영 RMS 오차를 출력합니다.
종료 후 publisher를 다시 실행해야 새 보정값이 적용됩니다.

2026-10-27 재촬영 전의 보정값과 이미지 158장은
`calibration_backups/20261027_225632_+0900/`에 백업했습니다. 다음 명령으로
기존 보정값을 복구할 수 있습니다. 복구 직전 보정값도 `restore_history/`에 보존합니다.

```bash
python3 calibration_backups/20261027_225632_+0900/restore.py
```

복구 후 publisher를 재시작하세요.
