# Jarabot Simulator (ROS2 Humble)

This repository is the jarabot simulator workspace that 
allows testing of the robot's movements in RViz2 without 
the physical jarabot hardware.

## Package Structure

- src/jarabot_sim
- src/jarabot_sim_interfaces

## Build Procedure

cd jarabot_sim_ws
colcon build
source install/setup.bash
ros2 launch jarabot_sim jarabot_simulator.launch.py 

# arabot Simulator for ROS 2 Humble

Jarabot Simulator는 실제 자라봇(Jarabot)의 주행, 센서, 환경 정보를 **ROS 2 Humble 에서 그대로 재현**할 수 있도록 설계된 2D/3D 하이브리드 시뮬레이터입니다.  
실제 로봇 없이도 **SLAM, Navigation2, 경로추종, 센서 데이터 처리 연습**이 가능하도록 개발되었습니다.

---

## 주요 특징 (Features)

- **LiDAR 시뮬레이션** 
  - YDLIDAR X4 Pro를 모델링한 2D 가상 라이다 데이터 생성
- **Odometry / TF 브로드캐스트** 
  - 로봇의 위치(x, y, θ)를 엔코더 기반으로 적분하여 odom → base_link 전달
- **모터/엔코더 시뮬레이션** 
  - 실제 자라봇의 구동 방식(차동 구동)을 기반으로 속도 계산
- **Marker 기반 시각화 (RViz2)** 
  - 로봇 모델, 궤적(Path), 라이다 포인트, heading 등을 실시간 표시
- **Keyboard Teleop 지원** 
  - 시뮬레이터 내에서 직접 속도 증가/감소, 회전 조작 가능

---

## 패키지 구성

### 1. `jarabot_sim`
Jarabot의 모든 시뮬레이션 노드를 포함한 메인 패키지

| 기능               | 설명                       |
|--------------------|----------------------------|
| `jara_sim_lidar`   | 가상 라이다 데이터 publish |
| `jara_sim_encoder` | 모터/엔코더 시뮬레이션     |
| `jara_sim_marker`  | RViz2 마커 표시            |
| `jara_sim_path`    | 이동 궤적 기록             |
| `jara_keyboard`    | 키보드로 로봇 제어         |

### 2. `jarabot_sim_interfaces`
Jarabot Simulator에서 사용하는 커스텀 메시지 정의 패키지

| 메시지 | 설명                       |
|--------|----------------------------|
| `Cmd`  | 로봇 제어 명령 (속도/회전) |
| `Ecd`  | 엔코더 데이터 메시지       |

---

## 🛠 설치 방법 (Installation)

### 1. 워크스페이스 생성

```bash
mkdir -p ~/jarabot_sim_ws/src
cd ~/jarabot_sim_ws/src

git clone https://github.com/playros/jarabot-simulator.git
cd ~/jarabot_sim_ws
colcon build --symlink-install
source ~/jarabot_sim_ws/install/setup.bash

ros2 launch jarabot_sim simulator.launch.py
ros2 run jarabot_sim keyboard

### 키보드 조작 방

| 키          | 기능                        |
| ----------- | --------------------------  |
| `w` / `x`   | 전진 / 후진 속도 10% 증가   |
| `a` / `d`   | 좌회전 / 우회전 10% 증가    |
| `q` / `e`   | 선속도 유지하며 회전만 조절 |
| `s`         | 정지                        |
| `CTRL + C`  | 종료                        |

### 주요 ROS2 토픽 목록

| 토픽                 | 타입                     | 설명                       |
| -------------------- | ------------------------ | -------------------------- |
| `/scan`              | `sensor_msgs/LaserScan`  | 가상 라이다 데이터         |
| `/odom`              | `nav_msgs/Odometry`      | odom 좌표계 기준 로봇 상태 |
| `/tf`                | TF                       | odom → base_link           |
| `/cmd`               | `jarabot_interfaces/Cmd` | 로봇 속도 명령             |
| `/ecd`               | `jarabot_interfaces/Ecd` | 엔코더 피드백              |
| `/jara_robot_marker` | Marker                   | RViz2 로봇 모델 표시       |
 
