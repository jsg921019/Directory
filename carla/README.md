## Pure Pursuit using Carla simulator
---
### 1. 실행 방법

<terminal 1>
* At Carla Simulator directory,
* `$ SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

<terminal 2>
* `$ source ~/carla_ws/devel/setup.bash`
* `$ roslaunch pure_pursuit pure_pursuit.launch`

<br>

### 2. 실행영상

[<img src="https://img.youtube.com/vi/EZ3FkREphB8/0.jpg" alt="Watch the video" style="zoom:30%;" />](https://youtu.be/EZ3FkREphB8)

<br>

### 3. record

* reference path를 만들기 위해 사용
* 수동 운행으로 path를 기록하여 loop이 인식되면 pickle file로 저장
* /carla/ego_vehicle/odometry 메시지를 참조
* path resolution은 0.2m로 self.resolution으로 조절 가능

<br>

### 4. pure_pursuit.py

* lincoln사의 mkz2017 사용
* speed controller로 pid 제어 사용
  * target speed는 12m/s
* steer controller로 pure pursuit 사용
  * look ahead distance는 약 5m