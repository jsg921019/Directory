## Simulator 사용법

---

#### 0. 간단 사용법

* 일단 기본 파라미터대로 train.py 을 실행
* 이후 잘 안되시면 파라미터 수정
* reward 수정
* 모델 수정
* 시뮬레이션은 되는데 자이카에서 돌릴 시 안될경우 map이나 car부분을 수정
* 저같은 경우는 기본 파라미터로 800 episode 내로 10바퀴 완주가 되었습니다. (sample.pth, sample.avi 참조)
* 실제 2층에서 실험 결과(시뮬레이터에서 4바퀴 성공한 모델 사용) 직선구간은 왠만하면 잘 갔었던거 같고, 커브구간에서 여유공간이 좀 있으면 잘 갔지만 없을시 자포자기하고 벽에 박아버립니다.

#### 1. assets : Map

* Map은 그림 파일을 인자로 받는 클래스
* 벽은 검은색, 완료지점은 초록색으로 해서 그림판에서 직접 제작 가능
* Map Class의 self.init_positions 에서 초기 시작 위치를 넣을 수 있음 : [x, y, yaw] 형식

#### 2. assets : Car

* 차의 픽셀 크기는 cm와 동일 (자로 재었을때, b3모델은 55cm x 27cm 이므로 시뮬레이터상에서도 55px, 27px로 설정)
* self.hitbox에서 충돌 영역 설정가능
* self.sensor_position 과 self.sensor_angle 에서 초음파 센서의 위치/각도 설정 가능
* self.measure_distance 에 max_dist 인수로 초음파센서 최대 인식 거리 설정 가능
* measure_distance는 왼쪽, 왼쪽앞, 앞, 오른쪽앞, 오른쪽 순의 ndarray를 반환 (자이카 메시지의 순서와 동일), 단위는 px=cm 이고 노이즈는 없도록 설정하였지만 범위형이 아닌 직선형으로 작동
* self.check_collision 과 self.check_goal로 충돌 및 완료 확인 가능
* self.reset()으로 차의 위치를 초기화(기본적으로 init_positions 중 랜덤위치, 지정하고 싶다면 position인자에 숫자를 넣어주면 됨)
* asset 파일을 실행시키면 초기 위치 및 센서/hitbox 확인 가능

#### 3. model_custom

* DQN 강의에서 쓴 모델과 99퍼 일치
* [512,512,512] hidden layer
* relu activation layer
* MSE loss
* Adam optimizer

#### 4. my_reward

* 이건 자유롭게 설정하기 나름이지만 저같은 경우는 reference_path를 설정하고 이 지역에 있으면 달콤한 보상을, 벗어나거나 벽에 충돌하면 엄하게 벌점을 부여하므로써 강하게 훈련시켰습니다.
* reference_path는 처음에는 그림판으로 그리다가 귀찮아져서 opencv의 dilate 함수를 사용하였습니다. 이 파일을 실행시키면 reference path를 확인하실수 있고, kernel size로 두께를 조절할 수 있습니다.
* 경험상 보상 정책은 복잡하게 안하여도, 충돌할 시 벌점을 준다면 그걸로 충분히 돌았습니다.

#### 5. recorder / visual

* recorder는 저장하는 역할
* visual은 visdom 역할
* 원래 시뮬레이터랑 동일한 작동을 하지만, visdom그래프에서 흥미롭지만 사실 별 정보를 뽑기 어려웠던 loss graph와 death position을 뺐습니다. step수와 reward의 변화만 넣어서 이놈이 학습이 되고있는건지 아닌지만 판별하는 용도로 사용하였습니다.

#### 6. train

* 설정 parameters
  * state_size : state의 개수 (저는 센서 5개 값만 사용)
  * action_size : action의 개수 (저는 3개 사용)
  * lr : 학습 속도 (처음엔 크게 설정하고 점점 줄이는 것이 좋은 것 같습니다)
  * discount_factor : 감마 (딱히 다르게 설정해본 적이 없습니다)
  * batch_size : 한번에 몇개 학습시킬 것인지
  * mem_maxlen : Experience Replay memory 크기
  * eps_init : 초기 epsilon 
  * eps_min : 최소 epsilon
  * skip_frame / stack_frame : 프레임 스킵/스탭 (딱히 쓴적은 없습니다)
  * map_file : map 이미지 파일 경로
  * start_train_step : 몇번째부터 학습할 것인지
  * max_episode : 언제까지 학습시킬 것인지
  * target_update_cycle : 타깃 모델 업테이트 주기
  * report_cycle : print/visdom 현황보고 주기
  * autosave_cycle : 자동저장 주기 100
  * end_training_loop : 몇 번 완주할시 끝낼것인지
  * eps_decrease_rate : 매 step 마다의 eps 감소량

* conversion 함수
  * 쓰시는 state나 action에 따라 바꾸셔야됩니다.
  * action2msg(action) : action -> xycar_msg.angle 로 변환\
  저같은 경우 0 : -50, 1 : 0, 2 : 50 으로 설정\
  ros에서 실행시킬때 action값을 받아와 xycar_msg로 변환시키기 위해 사용
  * msg2rad(msg): xycar_msg.angle -> kinetic model의 steer_angle (rad)\
  그냥 시뮬레이션을 시키기 위한 것이고 ros에서는 사용 안함

* load model
  * agent = DQNAgent(param_dict, training=True, load_path=None) : 처음 학습시
  * 이어서 학습시키는 경우(저는 주로 lr 을 바꾸고 싶을시 사용했습니다), load_path에 pth 파일의 경로를 넣으시면 됩니다.

#### 7. viewer
* 학습 시킨 모델의 param_dict로 수정
* load_path 를 원하는 pth의 경로로 수정
* 실행시켜서 오오 감상한다음 spacebar로 종료