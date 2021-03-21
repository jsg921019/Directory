### 미션통과 자율주행 경주대회 패키지

<br>

#### 0. 실행

* `roslaunch rally main.launch`

<br>

#### 1. 초음파센서 활용 알고리즘기반 장애물 회피 주행

* ultrasonic_drive.py
* 5개의 초음파센서 값을 통해 장애물이 없는 방향으로 주행

<br>

#### 2. 180도 방향 전환

* ultrasonic_drive.py 에 통합
* 후진 각도를 조정

<br>

#### 3. 초음파센서 활용 딥러닝 기반 주행

* dqn_drive.py
* pytorch에서 시뮬레이터를 통해 DQN 알고리즘으로 학습시킴

<br>

#### 4. Yolo기반 객체인식 주행

* yolo_drive.py
* yolov2_tiny 모델/ coco dataset 활용
* 지정한 객체가 인식되면 그 방향으로 이동

<br>

#### 5. AR 기반 주차

* ar_drive.py
* ar_track_alvar 패키지 사용
* x, y, yaw 값을 통한 stanley method로 제어

<br>

#### 6. 통합 주행

* main.py
* QR 코드를 통해 어떤 미션을 수행해야하는지 인식를 인식
* pyzbar 패키지 사용