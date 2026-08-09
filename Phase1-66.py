"""
2026 전국 대학생 AI 자율주행 경진대회
1단계 시간측정 경기 - Phase1-66

Phase1-58 구성 원칙
  1. Phase1-41의 일반 주행, 조향 계산, 속도 스케줄, S자 통과,
     일반 LOST 처리를 그대로 사용한다.
  2. Phase1-52에서 횡단보도 통과에 직접 필요한 가로선 제거,
     경계 오인 방지, 녹색 외곽 보조 경계, 횡단보도 감속만 가져온다.
  3. 위 보호는 CROSS 및 해제 직후의 짧은 시간에만 제어에 개입한다.
     따라서 횡단보도를 벗어나면 즉시 Phase1-41 제어로 돌아간다.
  4. 우굴곡에서는 정상적인 양의 헤딩 때문에 예측 가드가 필요한
     우조향을 지우지 않도록 실제 실선 여유로 가드를 판단한다.
  5. 우굴곡 속도를 최대 185로 제한하고 핵심 디버그 값만 표시한다.
  6. 네모박스·정지선의 가로 성분은 계속 제거하지만 HLINE이 조향을
     고정하지 않는다.
  7. 횡단보도는 CROSS 확정 전의 안정적인 HLINE 진입부터 녹색 외곽
     경계를 사용하고, 기하값의 물리적인 변화 속도만 제한한다.
  8. S자 좌굴곡에서 우굴곡으로의 전환을 0.10초 확인하고, 확인 중에는
     완만한 좌조향만 유지한다. 확정 뒤에는 거리 비례 가드로 추적한다.

BEV 기반 구조 전환판 - 개정 8   (Phase1-33 기반)

Phase1-39 가 알려준 것
  직진값 177 은 맞았다. t=140 에서 E=+0.6, IN=-0.1 로 적분 도움
  없이 차가 목표에 있었다. 트림 문제는 해결됐다.

  그런데 적분이 트림이 아니라 커브 편향을 학습해 버렸다.
  이 트랙은 좌커브가 압도적이라 좌커브의 E=+5 를 보고 우조향을
  쌓고, Ti=12 초의 긴 기억으로 그 값을 S 자 우커브까지 들고 간다.
      t=70 E= +2.6 IN=-5.8      t=76 E= -5.6 IN=-7.7
      t=73 E=+10.6 IN=-6.8      t=79 E= -8.9 IN=-5.0
  차가 실선으로 밀리는 내내 적분이 우조향을 보태고 있었다.

  그리고 더 근본적으로, 차의 정착 위치가 곡률로 정해진다.
      좌커브 K=-0.0018 -> E = +0.6 ~ +5.8
      우커브 K=+0.0021 -> E = -5.6 ~ -12.0
  좌우 사이에서 12 cm 넘게 옮겨 앉고, 그 우커브 쪽이 실선을 넘는다.

Phase1-40 에서 바뀐 것 두 가지
  1. 적분을 직선(|K| < 0.0006)에서만 쌓는다.
     적분은 상수 트림만 맡는다.
  2. 목표를 곡률에 비례해 옮긴다 (3500 x K, 최대 10 cm).
     곡률 비례 편향은 이쪽이 맡는다.

  둘은 서로 다른 것을 담당하며 간섭하지 않는다.
  게인, 감쇠항, 검출 로직, 가드, 속도 스케줄은 Phase1-39 그대로다.

Phase1-41 에서 바뀐 것
  Phase1-40의 일반 주행, 좌회전 진입, 속도 스케줄은 그대로 유지한다.
  S자 후반에 차량이 우측 실선을 향하는데도 기존 가드가 늦게
  작동하던 문제만 다음 두 단계로 보완한다.

  1. 현재 실선 여유와 헤딩 오차로 가까운 미래의 실선 여유를 예측하고,
     예상 여유가 부족하면 우조향을 미리 부드럽게 제한한다.
  2. 실제 실선 여유가 위험 범위에 들어왔고 차량이 계속 우측을
     향하고 있을 때만 최소 좌조향을 만들어 실선 안쪽으로 복구한다.

  우조향 뒤에 무조건 좌조향하는 시간 기반 동작은 사용하지 않는다.
  따라서 정상 우회전과 일반 주행에서는 보호 조건을 만족할 때만
  개입한다.

Phase1-38 이 알려준 것  ―  이탈의 원인이 확정됐다
  S 자 출구 t=29.4~33.9 의 4.5 초 동안 조향 명령이 170 에 고정된 채
  E 가 -2.5 에서 -14.6 으로 밀렸다.
      t=30.5  E= -7.9  FF -7.9  CT +8.3  PS -0.0  STR=170
      t=31.4  E= -8.4  FF -8.8  CT +9.0  PS -0.0  STR=170
      t=33.9  E=-14.6  FF -9.9  CT+17.0  PS -7.9  STR=170
  FF 와 CT 가 상쇄되어 제어기는 "직진" 에서 안정된다. 제어기는
  제 할 일을 했다. 틀린 것은 직진값이다. 이 차의 실제 직진은 177 인데
  코드는 170 이었으니 4.5 초 내내 7 카운트 우조향을 낸 것이다.
  7 카운트는 반지름 700 cm 이고 4.5 초면 횡방향 20 cm 대다.

  감속은 의도대로 걸렸다 (69 % 프레임, S 자에서 SPD 103~110).
  다만 출발점이 7 카운트 틀려 효과를 볼 수 없었다.

Phase1-39 에서 바뀐 것 두 가지
  1. 직진값 170 -> 177  (주행 중 실측값)
  2. 적분항 추가  ―  트림이 또 흔들려도 스스로 맞춘다

  게인, 감쇠항, 검출 로직, 가드, 속도 스케줄, 아두이노 규약은
  Phase1-38 그대로다.

Phase1-33 을 베이스로 삼은 이유
  실주행 비교에서 Phase1-33 이 가장 부드럽고 S 자 이탈도 가장 적었다.
  Phase1-35 부터 Phase1-37 까지의 변경은 전부 되돌린다.

Phase1-36 과 Phase1-37 이 왜 나빠졌는가
  Phase1-36  그 주행에서 기계 직진값이 169 가 아니라 178 로 틀어져
             있었다. 제어기 탓이 아니다. 거기에 우조향 가드를 -5 로
             옮긴 탓에 차가 문턱 위에 얹혀 가드가 켜졌다 꺼졌다 했다.
  Phase1-37  곡률 필터를 0.15 에서 0.45 로 올렸는데, 곡률은 FF 뿐
             아니라 atan(C x kappa) 로 헤딩 보정에도 들어간다. 즉
             피드백 경로다. 거기에 0.3 초 지연을 넣어 발진했다.

  공통점은 둘 다 제어 루프 안을 건드렸다는 것이다.

Phase1-38 에서 바뀐 것 세 가지

  1. 직진값 169 -> 170.  트림 개념 제거.
     조향 한계 230/100 은 Phase1-33 그대로 둔다.

  2. 감속 스케줄을 실제로 걸리게 고쳤다.   <- 이번 판의 유일한 기능 변경
     Phase1-35, Phase1-36 로그 711 프레임이 전부 SPD=120 이다.
     감속이 한 번도 걸린 적이 없었다. 문턱이 실측 곡률 범위 밖에
     있었기 때문이다. 감속은 제어 루프 밖이라 발진 위험이 없다.

  3. Lane Mask 창 제거.  검출에는 계속 쓰이고 화면 표시만 없앴다.

  게인, 감쇠항, 검출 로직, 가드, 아두이노 규약은 Phase1-33 그대로다.
  아두이노 펌웨어는 기존 파일을 그대로 쓴다.

----------------------------------------------------------------------
아래는 Phase1-28 이후의 이력이다. 참고용으로 남긴다.
======================================================================

Phase1-28 실주행(951프레임, 2바퀴) 결과와 이번 수정
----------------------------------------------------------------------
성공한 것
  droop 이 사라졌다. 횡오차 E 의 중앙값이 -2.0 cm 다.
  Phase1-27 은 같은 값이 -15 cm 수준이었다.
  주행의 78.4%가 속도 120 이고 평균 118.5 로 Phase1-27 보다 빨랐다.

실패한 것과 원인

1) 코너에서 안쪽으로 파고들어 중앙 점선을 침범했다.
   원인은 PS 항이 곡률 피드포워드를 이중으로 계상한 것이다.
   측정된 PSI 를 곡률 구간별로 나눠 보면

       직선   K~0         PSI -1.2도
       보통   K -0.00211  PSI -7.8도
       급코너 K -0.00287  PSI -11.1도

   PSI 가 곡률에 정확히 비례한다. 이것은 제어 오차가 아니라
   카메라가 차량 회전중심보다 앞에 달려 있어서 생기는 기하학적 값이다.
   회귀하면 PSI = C x kappa, C = 54.2 cm 로 아주 잘 맞는다.

   즉 PS 항은 사실상 두 번째 피드포워드였고, 준정상 코너에서
   +5.4 카운트의 좌조향 편향을 상시 더하고 있었다.

2) 피드포워드 게인이 8% 과다했다.
   이번 주행의 준정상 코너 410프레임에서 다시 구하면
   실제 필요 조향은 6691 x (-kappa) 였다. 설정값은 7200 이었다.

   1)과 2)를 합치면 급코너에서 약 7카운트가 과다했다. 그래서 안쪽으로 밀렸다.

3) 그 다음 우측으로 크게 넘어가 이탈했다.
   E 가 +10 cm 까지 안쪽으로 갔다가 1.3초 만에 -20 cm 로 넘어갔다.
   억제하는 항이 없었기 때문이다.
   PS 항은 원래 이 감쇠를 맡아야 했는데, 위 1) 때문에
   기하 성분에 묻혀 감쇠로 기능하지 못했다.

4) 경계가 시야를 벗어나면 곡률 추정이 폭주했다.
   검출점 PTS 가 42에서 4까지 무너지는 동안 K 가 +0.00513 까지 튀었다.
   반지름 195 cm 로, 이 트랙의 최소 반경 250 cm 보다 작아 물리적으로 불가능하다.
   그 값에 7200 을 곱해 FF 가 -36.9 카운트, 즉 강한 우조향이 나갔다.

   검출이 건강한 프레임(PTS>=35)만 보면 |K| 최대가 0.00359 였다.
   즉 폭주는 전부 검출이 무너진 구간에서만 일어났다.

이번 수정
  A. PSI 에서 기하 성분을 뺀다. psi_error = psi + atan(C x kappa).
     이 하나로 코너 편향이 사라지고, 동시에 PS 항이 제 역할인
     감쇠 항으로 돌아온다. 3)의 진동도 여기서 잡힌다.
  B. FF 게인을 실측값 6700 으로 내린다.
  C. 곡률에 물리적 상한을 둔다. 트랙 최소 반경 250 cm 기준.
  D. 곡률을 믿을 수 있는 조건을 강화한다. 점 개수와 전방 관측 길이.
     관측 길이가 짧으면 FF 를 신뢰도만큼 줄인다.
     곡률은 긴 기선이 있어야 구할 수 있지만 횡오차는 그렇지 않다.
  E. 우측 실선에 가까워지면 우조향 허용량을 줄인다.
     실선 이탈은 절대 피해야 하는 사건이므로 안전 포락선을 둔다.
======================================================================

왜 바꾸는가
----------------------------------------------------------------------
Phase1-27 실주행(916프레임, 2바퀴)에서 다음이 확정되었다.

- Phase1-27의 A(b) 수정은 목표대로 동작했다.
  S자 최대 프레임간 조향 점프가 9에서 4로, TGT 최솟값이 148에서 158로 개선됐다.
  조향 명령의 인위적 불연속은 제거되었다.

- 그런데도 이탈했다. 원인은 진동이 아니라 정상편차(droop)였다.
  전 구간 corr(|STR-160|, NE) = -0.938.
  S자 탈출부에서 NE 실측 -78.0은 droop 회귀 예측 -70.7과
  잔차 표준편차 2.0px 안에서 일치한다.
  즉 그 -78은 오차가 아니라 그 조향을 만들기 위해 반드시 지불하는 값이다.

- droop 은 정확히 1/KP_NEAR 다. 없애려면 KP_NEAR 를 0.28에서 0.60으로
  올려야 하는데, 기존 코드가 직선에서 STRAIGHT_KP_NEAR = 0.16 으로
  낮춰 쓰고 있다는 사실 자체가 0.28이 이미 한계라는 뜻이다.

P 제어는 정의상 출력 = K x 오차 다. 코너에서 21카운트가 필요하면
21/K 만큼의 오차를 반드시 유지해야 한다. 빠져나갈 길은 셋뿐이다.
  1) K 를 키운다      -> 2배 필요. 불가.
  2) 적분항을 넣는다  -> 연속 코너 트랙에서 수렴 못 함. 실패 전례 있음.
  3) 그 21카운트를 오차가 아닌 곳에서 공급한다 -> 곡률 피드포워드.

3번만 남는다. 그리고 곡률 피드포워드에는 물리 단위의 곡률이 필요하고,
그것은 원근이 제거된 BEV 에서만 나온다.


무엇이 바뀌었는가
----------------------------------------------------------------------
NE 와 LE 는 사라졌다. 그 둘은 횡방향 오차와 도로 곡률과 헤딩이
분리 불가능하게 섞인 값이었다. BEV 에서 우측 경계를 2차식으로 맞추면
세 가지가 물리량으로 분리된다.

    d(s) = a s^2 + b s + c        s = 전방거리(cm), d = 횡거리(cm)

      c   -> 횡방향 위치        (cross track error 의 재료)
      b   -> 헤딩 오차 psi
      2a  -> 실제 곡률 kappa    (1/cm)

제어식

    counts = FF * (-kappa) + KPSI * (-psi) + KE * (-e)
    steer  = STEER_CENTER + counts

첫 항이 코너에 필요한 조향을 오차 없이 공급한다. 그래서 e 가 0 에
가까운 채로 코너를 돌 수 있다. 이것이 droop 제거의 전부다.

부수적으로 다음이 전부 불필요해져 삭제했다.
  S 게이트 / 횡단보도 뒤 32초 창 / S_ARM / S_BEND_ASSIST /
  접근속도 감쇠 / STRAIGHT 모드 / curve target shift /
  출발 시 원근 오프셋 보정
kappa 의 부호가 바뀌면 피드포워드 부호도 자동으로 바뀌므로
S자를 특별 취급할 이유가 없다.


캘리브레이션 근거
----------------------------------------------------------------------
BEV_Calibration.py 실측 보고서 (검증 통과)
  우측 실선 직선성 1.87 cm (기준 2.0 이내), 샘플 30개
  평행성 비율 1.023 (기준 1.00 +- 0.03), 샘플 16개
  원본 프레임 직선 잔차 1.0 px

이 캘리브레이션으로 실주행 영상 15개 프레임을 BEV 로 펴서 얻은 값
  2차 피팅 잔차 1~2 cm / 전방 137 cm
  급코너 곡률 kappa = -0.0030 ~ -0.0040 /cm  (반지름 250~330 cm)
  차로 폭 실측 96.2 cm
  경계 횡거리 실측 26.3 ~ 44.0 cm  (목표 47.6 cm 대비 3.6~21.3 cm 부족)
                                    <- 이것이 droop 의 실체다


게인을 어떻게 정했는가
----------------------------------------------------------------------
FF_COUNTS_PER_CURVATURE = 7200
  같은 15프레임에서 화면에 찍힌 실제 조향값과 BEV 곡률을 대조했다.
  준정상 상태의 코너에서는 차량이 도로를 따라가고 있으므로
  그때의 조향값이 곧 그 곡률에 필요한 조향값이다.
  |kappa| > 0.002 인 코너 7프레임만 골라 계산했다.
    프레임별 counts/|kappa| : 8970 7983 5495 6019 8679 6750 7985
    중앙값 7983 / 평균 7411 / 원점통과 최소제곱 7216

KE_COUNTS_PER_CM = 1.30
  기존 KP_NEAR = 0.28 counts/px 를 그대로 승계했다.
  P0 행에서 ROI 1px = 0.211 cm 이므로 0.28 x 4.75 = 1.33 counts/cm.
  이 값은 지금까지 안정하게 돌던 게인이다. 루프 안정성을 건드리지 않고
  피드포워드만 얹기 위해 일부러 바꾸지 않았다.

  이 승계가 맞다는 독립 검증이 있다. NE 를 cm 로 환산한 값과
  BEV 로 잰 실제 횡오차가 6개 프레임에서 1.5 cm 이내로 일치했다.
      NE -85.7px -> -18.1cm  |  BEV 실측 -18.2cm
      NE -59.5px -> -12.6cm  |  BEV 실측 -12.1cm
      NE -72.0px -> -15.2cm  |  BEV 실측 -16.3cm
      NE -79.6px -> -16.8cm  |  BEV 실측 -17.3cm
  즉 호모그래피와 목표 47.6cm 가 기존 NE=0 목표와 같은 것을 가리킨다.

KPSI_COUNTS_PER_DEG
  기존 구조에 없던 항이라 승계할 값이 없다. 보수적으로 시작한다.
  실측 헤딩오차는 직선에서 0~3도, 급코너에서 12~16도였다.
  (급코너의 12도는 지금 차량이 코너에서 바깥으로 밀리며 생긴 값이다.
   피드포워드가 들어가면 줄어들어야 정상이다.)

목표 횡위치
  BEV_Calibration.py 가 계산한 47.60 cm 를 그대로 쓴다.
  실측 차로폭 96.2 cm 의 절반인 48.1 cm 와 사실상 같다. 즉 차로 중앙이다.
  이전 BEV 시도에서 좌측으로 붙었던 원인은 목표를 차로 중앙으로
  다시 정의했기 때문일 가능성이 크므로, 여기서는 목표를 바꾸지 않고
  Phase1-27 이 쓰던 값을 그대로 승계한다.


속도
----------------------------------------------------------------------
Phase1-27 의 감속 문제(주행의 54.3%가 속도 110, 랩타임 +4.1초)는
S_GATE_SPEED_CURVATURE_PX = 6.0 이 게이트 열린 동안 거의 모든 코너에
걸렸기 때문이다. 여기서는 게이트가 없어졌고, 실제 곡률로 감속한다.
직선과 완만한 코너는 120 을 유지하고 급코너에서만 내려간다.


남겨둔 것
----------------------------------------------------------------------
검증된 부분은 그대로 둔다.
  ROI, 이진화, 가로 마킹 제거, 마킹 래치 차단 3중 가드,
  아두이노 명령, 경계 상실 시 홀드/정지 동작
가로 마킹 제거는 BEV 에서도 필수다. 정지선과 횡단보도가
우측 경계 검출을 오염시키기 때문이다.
"""

import json
import math
import os
import time

import cv2
import numpy as np

import Function_Library as fl


# ======================================================================
# 1. 장치와 실행 키
# ======================================================================

ARDUINO_PORT = "COM3"
ARDUINO_BAUDRATE = 9600

# 아두이노 방향 코드. Phase1-16 부터 Phase1-27 까지 쓰던 값과 같아야 한다.
#   0 = 정지, 1 = 전진, 2 = 후진
# Phase1-28 최초 배포판에서 전진을 2로 잘못 넣어 차량이 후진했다.
# 숫자를 직접 쓰지 말고 반드시 이 상수를 쓴다.
DIRECTION_STOP = 0
DIRECTION_FORWARD = 1
DIRECTION_REVERSE = 2

CAMERA_PORT = 0

START_KEY = ord("s")
STOP_KEY = ord("q")


# ======================================================================
# 2. 카메라 ROI와 영상 전처리
# ======================================================================

ROI_Y_START = 160
ROI_Y_END = 480

THRESHOLD_VALUE = 190
GAUSSIAN_KERNEL = (5, 5)


# ======================================================================
# 3. BEV 캘리브레이션 실측값
# ======================================================================
# BEV_Calibration.py 가 출력한 값이다.
# 카메라 각도나 높이가 바뀌면 반드시 다시 측정해야 한다.
# 재측정하면 bev_calib.json 을 이 파일 옆에 두기만 해도 자동으로 읽는다.

BEV_CALIB_FILE = "bev_calib.json"

# 클릭한 4점 (ROI 픽셀 좌표)
# 순서: 근-좌, 근-우, 원-우, 원-좌
BEV_SOURCE_POINTS = (
    (132.0, 260.0),
    (549.0, 276.0),
    (464.0, 111.0),
    (226.5, 101.0),
)

BEV_MARKER_WIDTH_CM = 89.0
BEV_MARKER_LENGTH_CM = 120.0

BEV_PIXELS_PER_CM = 3.0
BEV_WIDTH = 640
BEV_HEIGHT = 520
BEV_NEAR_MARGIN_PX = 40

# 기존 원근변환과 차량 기준점은 그대로 두고, 출력 영상의 오른쪽만 넓힌다.
# 따라서 기존 0~639 px의 좌표와 제어값은 바뀌지 않고, 우측 실선이 화면
# 끝으로 밀릴 때 사용할 수 있는 80 px의 여유 영역만 추가된다.
BEV_RIGHT_EXTRA_PX = 80


# ======================================================================
# 4. BEV 경계 검출
# ======================================================================

# BEV 에서 훑을 세로 범위. 아래가 가깝고 위가 멀다.
BEV_SCAN_BOTTOM_PX = 505
BEV_SCAN_TOP_PX = 90
BEV_SCAN_STEP_PX = 10
BEV_SCAN_HALF_HEIGHT_PX = 3

# 흰 선으로 인정할 세로 묶음 폭 (BEV 픽셀). 차선은 약 3 cm = 9 px 이다.
MIN_LINE_WIDTH_PX = 4
MAX_LINE_WIDTH_PX = 80

# 첫 검출 때 우측 경계를 찾을 x 범위
BEV_INIT_LEFT_PX = 300
BEV_INIT_RIGHT_PX = 620

# 이전 프레임 곡선에서 이만큼 안에 있는 후보만 받는다.
BEV_TRACK_WINDOW_PX = 45

# 한 스캔 단계(10px = 3.3cm) 사이에 경계가 움직일 수 있는 최대 거리
BEV_STEP_JUMP_PX = 20

# Phase1-29 D
# 곡률은 긴 기선이 있어야 정해진다. 점 개수만으로는 부족하고
# 전방 관측 길이를 함께 본다.
# Phase1-28 에서 PTS 가 42에서 4로 무너지는 동안 K 가 0.00513 까지 튀었다.
MIN_BOUNDARY_POINTS = 20
MIN_FIT_SPAN_CM = 80.0

# 표시선 구간의 물리 범위와 재확인 조건.
# 일반 주행에서는 검사하지 않고 HLINE/CROSS 보호 중에만 사용한다.
PLAUSIBLE_LATERAL_MIN_CM = 15.0
PLAUSIBLE_LATERAL_MAX_CM = 90.0

CROSSWALK_ENTRY_MAX_HEADING_DEG = 10.0
CROSSWALK_ENTRY_MAX_CURVATURE = 0.0007
CROSSWALK_BOUNDARY_PREDICT_SEC = 0.18

# Phase1-66: 횡단보도 진입 전에 화면 위쪽의 가로선을 먼저 확인하고,
# 통과 중에는 속도가 아니라 조향만 직진 근처로 보호한다.
MARKING_APPROACH_START_RATIO = 0.08
MARKING_APPROACH_END_RATIO = 0.50
MARKING_APPROACH_MIN_PIXELS = 180
MARKING_APPROACH_ARM_SEC = 1.50
MARKING_CROSSWALK_MIN_WHITE_RATIO = 0.085
MARKING_CROSSWALK_CONFIRM_FRAMES = 2
MARKING_PRESTEER_MAX_SEC = 0.75
MARKING_PRESTEER_REFERENCE_BAND_COUNTS = 3.0
MARKING_PRESTEER_RIGHT_LIMIT_COUNTS = 2.0
MARKING_PRESTEER_LEFT_LIMIT_COUNTS = 10.0
MARKING_STEER_MIN_SEC = 1.20
MARKING_STEER_MAX_SEC = 2.50
MARKING_STEER_RELEASE_FRAMES = 4
MARKING_STEER_RIGHT_LIMIT_COUNTS = 2.0
MARKING_STEER_LEFT_BASE_COUNTS = 4.0
MARKING_STEER_LEFT_MAX_COUNTS = 14.0
MARKING_STEER_LEFT_ASSIST_START_CM = -6.0
MARKING_STEER_LEFT_ASSIST_FULL_CM = -18.0
MARKING_RELEASE_MAX_HEADING_DEG = 15.0
MARKING_RELEASE_MAX_CURVATURE = 0.0030

# Phase1-52의 횡단보도용 녹색 외곽 보조 경계.
# 평소에는 흰 우측 실선과의 위치 차이만 학습하고,
# 실제 대체 사용은 CROSS 보호 중으로 한정한다.
GREEN_H_MIN = 28
GREEN_H_MAX = 100
GREEN_S_MIN = 35
GREEN_V_MIN = 25
GREEN_CLOSE_KERNEL_PX = 9
GREEN_MIN_COMPONENT_AREA_RATIO = 0.015
GREEN_COMPONENT_MIN_RIGHT_RATIO = 0.70
GREEN_MIN_BOUNDARY_POINTS = 24

GREEN_OFFSET_MIN_CM = -2.0
GREEN_OFFSET_MAX_CM = 12.0
GREEN_MATCH_MAX_HEADING_DEG = 8.0
GREEN_MATCH_MAX_CURVATURE = 0.0030
GREEN_TRUST_CONFIRM_FRAMES = 12
GREEN_TRUST_LOSS_FRAMES = 8
GREEN_OFFSET_FILTER_SEC = 0.80

GREEN_FALLBACK_MAX_LATERAL_JUMP_CM = 15.0
GREEN_FALLBACK_MAX_HEADING_JUMP_DEG = 12.0
GREEN_FALLBACK_MAX_CURVATURE_JUMP = 0.0045
GREEN_FALLBACK_SPEED_LIMIT = 180

GREEN_CONTROL_LATERAL_RATE_CM_PER_SEC = 35.0
GREEN_CONTROL_HEADING_RATE_DEG_PER_SEC = 30.0
GREEN_CONTROL_CURVATURE_RATE_PER_SEC = 0.0040

S_CURVE_TARGET_FILTER_SEC = 0.12
S_CURVE_TARGET_DEADBAND_COUNTS = 3.5
S_RIGHT_TRANSITION_CURVATURE = 0.0007
S_RIGHT_TRANSITION_HEADING_DEG = -8.0
S_RIGHT_TRANSITION_MAX_LEFT_HEADING_COUNTS = 6.0

# Phase1-64 재작성(Phase1-65에서도 그대로 유지):
# 좌굴곡에서 우굴곡으로 넘어가는 신호가 한 프레임만 나타났다고 즉시
# 좌조향을 풀지 않는다. 아래 시간 동안 곡률과 헤딩 조건이 함께 유지될
# 때만 우굴곡을 확정하며, 확인 중에는 과하지 않은 좌조향을 유지한다.
S_RIGHT_TRANSITION_CONFIRM_SEC = 0.10
S_RIGHT_TRANSITION_HOLD_STEER = 186.0

# Phase1-63: S자 후반 우측 복귀를 한 프레임의 곡률 변화로 해제하지 않는다.
# 우측 굴곡을 한 번 확인하면 짧은 검출 공백 동안 상태를 유지하고,
# 실제 출구처럼 곡률과 방향이 함께 안정된 경우에만 해제한다.
S_RIGHT_RELEASE_CURVATURE = 0.0003
S_RIGHT_RELEASE_HEADING_DEG = 4.0
S_RIGHT_RELEASE_CONFIRM_SEC = 0.20
S_RIGHT_MAX_DROPOUT_SEC = 0.60

# Phase1-65: S자 우굴곡에서 우측 실선이 기존 BEV 끝에 가까워졌을 때만
# 제한적인 우조향을 보조한다. 164보다 강한 우조향은 절대 명령하지 않는다.
S_RIGHT_EDGE_START_X_PX = 610.0
S_RIGHT_EDGE_FULL_X_PX = 635.0
S_RIGHT_EDGE_START_STEER = 170.0
S_RIGHT_EDGE_FULL_STEER = 164.0
S_RIGHT_EDGE_MIN_STEER = 164.0

# 2차 피팅 후 이 값보다 크게 벗어난 점은 버리고 한 번 다시 맞춘다.
FIT_OUTLIER_CM = 4.0
FIT_MAX_RESIDUAL_CM = 6.0

# Phase1-29 C
# 이 트랙의 실측 최소 반경은 250 cm 다. 그보다 급한 곡률은 물리적으로
# 있을 수 없으므로 검출 오류로 보고 잘라낸다.
# 검출이 건강한 프레임(PTS>=35)만 보면 실측 |K| 최대가 0.00359 였다.
# Phase1-33 수정 1 : 이 상한이 S자 이탈의 직접 원인이었다.
#
# 0.0040 으로 잡은 근거는 Phase1-28 로그의 |K| 최대 0.00359 였는데,
# 그 로그는 S자에서 이미 검출이 무너진 상태였다.
# Phase1-30 주행 영상에서 S자 좌굴곡을 상한 없이 다시 재면
#     kappa = -0.00856 (반지름 117 cm), -0.00798 (반지름 125 cm)
# 상한의 2.1배다. 트랙에서 이 구간만 유독 급하다.
#
# 그래서 S자에서
#     필요 조향 = 9 + 4900 x 0.00856 = 51 카운트
#     상한 때문에 실제 = 9 + 4900 x 0.0040 = 29 카운트
# 22 카운트를 버리고 있었다. 이것이 "좌조향 유지 부족" 의 정체다.
#
# 실측 최대 0.00856 위에 여유를 둔다.
# 반지름 91 cm 보다 급한 곡률은 이 트랙에 없으므로 검출 오류로 본다.
CURVATURE_LIMIT_ABS = 0.0110


# ======================================================================
# 5. 제어 게인
# ======================================================================

# Phase1-38 : 직진값을 170 으로 확정하고 트림 개념을 없앴다.
# 직진값 실측 (주행 로그에서 횡오차가 안정된 프레임의 STR 을
# 도로 곡률로 회귀해 K=0 절편을 구한 값)
#     Phase1-30  170.1
#     Phase1-35  169.1
#     Phase1-36  177.7   <- 이 주행만 기계가 틀어져 있었다
# Phase1-38 도 177.2 였다. 정지 상태 실측은 170 이지만 주행 중
# 실제로 직진하는 값은 177 이다. 구동 토크와 타이어 스크럽 때문이며
# 제어에 필요한 것은 주행값이다. Phase1-38 에서 이 7 카운트가
# S 자 이탈을 그대로 만들어 냈다.
# 조향 한계 230/100 은 Phase1-33 그대로 둔다. 잘 달리는 값을 건드리지
# 않는다. 직진 170 기준으로 좌 60 / 우 70 카운트다.
# Phase1-59 : 직진값 170.
STEER_CENTER = 177
# Phase1-59 : 좌우를 조금씩 좁혔다. 직진 170 기준 좌우 각 50 카운트로
# 대칭이 된다. Phase1-58 에서 관측된 최대 요구는 좌 +18 / 우 -37 이라
# 정상 동작에는 걸리지 않고, 조향모터가 못 따라가는 극단 명령만 막는다.
STEER_LEFT_LIMIT = 220
STEER_RIGHT_LIMIT = 120

# ----------------------------------------------------------------------
# Phase1-30 의 핵심 수정: 기계적 조향 트림
# ----------------------------------------------------------------------
# 서보 160 은 실제 직진이 아니었다. 참 직진은 160 + 이 값이다.
#
# Phase1-28 로그(951프레임, 2바퀴)로 확정했다.
#
#  1) 직선 구간(|K|<0.0005) 223프레임에서 조향 중앙값이 STR 170 이었다.
#     즉 똑바로 가는 데 +10 카운트를 계속 쓰고 있었다.
#  2) 그 구간의 횡오차 중앙값은 -7.4 cm 였다.
#     CT = -KE x E = -1.30 x (-7.4) = +9.6 카운트.
#     즉 차량은 트림을 상쇄하려고 일부러 우측으로 7.4 cm 치우쳐 달리고 있었다.
#  3) 한 바퀴를 돌면 차량 방향 변화와 도로 방향 변화가 같아야 한다.
#     도로: 적분(kappa x v x dt) = 12.53 rad = 정확히 2바퀴. 곡률 측정은 정확했다.
#     차량: trim=0, G=6700 을 넣으면 24.09 rad = 3.83 바퀴가 나온다. 불가능하다.
#     trim=+9 로 두면 G=4900 에서 정확히 2바퀴가 된다.
#
# 이 트림이 곧 그동안 반복된 증상의 원인이다.
# 실패 순간(t=69.3~70.1, 좌굴곡 kappa=-0.0022)을 보면
#     명령 STR 164  ->  실효 조향 164-160-9 = -5 카운트, 즉 살짝 우조향
#     그 코너에 필요한 값 +11 카운트
#     16 카운트 부족
# 좌코너 한복판에서 사실상 우조향을 하고 있었다.
# "좌조향을 미리 풀어버린다"는 관측이 정확히 이것이다.
#
# 곡률이 0 에 가까워질수록 부족분이 커진다는 점이 중요하다.
# FF 는 곡률에 비례해 0 으로 가지만 트림은 그대로 남기 때문이다.
# S자 변곡점이 바로 그 지점이라 그곳에서 가장 크게 밀렸다.
#
# 주의: 이 값은 기계적인 것이라 조향 링키지를 만지면 다시 재야 한다.
# 확인법은 직선에서 로그의 E 를 보는 것이다. 0 근처면 맞는 값이다.
# Phase1-38 : STEER_CENTER 가 170 이므로 트림은 0 이다.
# 기계를 만져 직진값이 바뀌면 STEER_CENTER 를 그 값으로 고치면 된다.
STEER_TRIM_COUNTS = 0.0

# ======================================================================
# Phase1-39 : 적분항
# ======================================================================
# 왜 필요한가
#   기계 직진값이 주행마다 달라진다.
#       Phase1-30  170.1   Phase1-35  169.1
#       Phase1-36  177.7   Phase1-38  177.2
#   8 카운트가 하룻밤 사이에 바뀌었고 그대로 유지되고 있다.
#   대회 당일에 또 바뀌면 재측정할 시간이 없다.
#
#   그리고 원리의 문제다. 비례 제어는 플랜트의 상수 오프셋을
#   구조적으로 없앨 수 없다. 지금은 CT 항이 그것을 대신 메우는데,
#   메우려면 반드시 E 가 0 이 아니어야 한다.
#   Phase1-38 직선 구간 E 중앙값 -6.0 cm 가 그 대가다.
#       CT 가 메운 양   1.30 x 6.0 = 7.8 카운트
#       회귀로 잰 오차  177.2 - 170 = 7.2 카운트   (일치)
#
# 크기를 어떻게 정했는가
#   폐루프 고유주파수 wn = V sqrt(KE/G) = 40 sqrt(1.30/4900) = 0.65 rad/s
#   즉 고유주기가 9.6 초다. 적분이 이보다 빠르면 위상을 깎아 발진한다.
#   Ti 별 위상 손실
#       Ti= 6초  14도      Ti= 9초  10도
#       Ti=12초   7도  <- 채택     Ti=18초   5도
#   KI = KE / Ti = 1.30 / 12 = 0.108
#
#   Phase1-37 은 감쇠 경로에 0.3 초 지연을 넣어 발진했다. 적분은
#   위치가 다르고 7 도면 여유 안이다.
#
# 안전장치
#   경계를 못 찾은 프레임에서는 아예 호출되지 않으므로 적분이 멈춘다.
#   조향이 한계에 걸리면 적분을 동결한다 (안티와인드업).
#   누적값은 +-12 카운트로 자른다. 트림이 8 카운트 틀어져도 덮는다.
#
# 끄는 법
#   KI_COUNTS_PER_CM_SEC = 0.0 으로 두면 Phase1-38 과 완전히 같아진다.
KI_COUNTS_PER_CM_SEC = 0.108
INTEGRAL_LIMIT_COUNTS = 12.0

# Phase1-40 : 적분을 직선에서만 쌓는다.
# Phase1-39 실주행에서 적분이 트림이 아니라 커브 편향을 학습해 버렸다.
# 이 트랙은 좌커브가 압도적이라(Phase1-38 기준 좌 254 : 우 36 프레임)
# 좌커브의 E=+5 를 "왼쪽으로 치우쳤다" 로 읽고 우조향을 쌓는다.
# Ti 가 12 초라 기억이 길어 그 값을 그대로 들고 S 자 우커브로 들어간다.
#
#   S 자 구간의 IN 값 (Phase1-39 실측)
#       t=70  E= +2.6  IN=-5.8      t=76  E= -5.6  IN=-7.7
#       t=73  E=+10.6  IN=-6.8      t=79  E= -8.9  IN=-5.0
#   차가 실선으로 밀리는 내내 적분이 우조향 5~8 카운트를 보태고 있었다.
#
# 적분은 상수 오프셋(기계 트림)만 맡아야 한다. 곡률에 비례하는 편향은
# 아래 CURVE_OFFSET 이 맡는다. 둘을 한 항이 떠안으면 이렇게 실패한다.
INTEGRAL_CURVATURE_MAX = 0.0006

# ======================================================================
# Phase1-40 : 곡률 비례 목표 이동
# ======================================================================
# 실측된 사실
#   차의 정착 위치가 도로 곡률로 정해진다. Phase1-39 실주행에서
#       좌커브 K = -0.0018 -> E = +0.6 ~ +5.8
#       우커브 K = +0.0021 -> E = -5.6 ~ -12.0
#   기울기로 정리하면  E = -3050 x K  이고, 좌우 사이에서 차가
#   12 cm 넘게 옮겨 앉는다. 그 우커브 쪽 위치가 실선을 넘는 자리다.
#
#   원인은 구조다. 고정 목표 47.6 cm 로 우측 경계 하나만 추종하는데,
#   우커브에서 우측 실선은 곡선의 안쪽이다. 고정 간격 추종은 차를
#   안쪽으로 눌러 앉힌다. 게인 조정으로는 없앨 수 없다.
#
# 대책
#   목표를 곡률에 비례해 옮겨 그 편향을 상쇄한다. 좌우 대칭이다.
#       우커브 K=+0.0022 -> 목표가 실선에서 +7.7 cm 멀어진다
#       좌커브 K=-0.0018 -> 목표가 중앙선에서 -6.3 cm 멀어진다
#   결과적으로 차의 위치가 곡률과 무관하게 평평해진다.
#
# 왜 Phase1-36 과 Phase1-37 에서는 실패했는가
#   Phase1-36  그 주행의 기계 직진값이 178 인데 코드가 169 였다.
#              게다가 우조향 가드를 -5 로 옮겨 차가 문턱 위에 얹혔다.
#   Phase1-37  곡률 필터 0.45 초로 감쇠 경로에 지연을 넣어 발진했다.
#   둘 다 이 이격 로직이 원인이 아니었다. 그리고 그때는 이격량이
#   추정치였는데 지금은 실측 기울기 -3050 이 있다.
#   현재는 직진값 177 이 확인됐고 가드와 필터는 Phase1-33 그대로다.
#
# 안전 판단은 이격을 뺀 실제 실선 여유(line_margin)로 한다.
# 이격분까지 더해 이중으로 세면 안 된다.
CURVE_OFFSET_CM_PER_CURVATURE = 2500.0
CURVE_OFFSET_LIMIT_CM = 10.0

# ======================================================================
# Phase1-59 : 좌커브 이격만 절반으로  ―  S 자 진입 좌조향 회복
# ======================================================================
# Phase1-58 실주행 S 자 진입부(좌커브)의 항 분해
#     t      E        K       FF     CT     PS     IN   TGT
#   238   -1.8  -0.00179    +8.8   -3.9   +9.6   -4.4   187
#   240   -0.0  -0.00198    +9.7   -7.1   +8.0   -4.4   182
#
# 좌커브에서 이격이 3500 x (-0.00179) = -6.3 cm 로 걸린다. 목표가
# 우측 실선 쪽으로 6.3 cm 옮겨지면서, E 가 -1.8 인데도 lateral_error
# 가 +4.5 가 되어 CT 가 우조향 -3.9 를 낸다. t=240 에서는 E 가 0.0
# 인데 이격 -6.9 때문에 CT 가 -7.1 까지 간다.
#
# 즉 차가 목표에 있는데 코드가 "왼쪽으로 치우쳤다" 고 판단해 우조향을
# 넣고 있었다. 이것이 진입 좌조향 부족의 정체다.
#
# 우커브 이격은 그대로 둔다. 그쪽은 우측 실선 회피라 필요하다.
# 좌커브만 절반으로 줄인다.
#     t=238  이격 -6.3 -> -3.1,  CT -3.9 -> 0.0,   TGT 187 -> 191
#     t=240  이격 -6.9 -> -3.5,  CT -7.1 -> -2.6,  TGT 182 -> 187
#
# 배율을 0.3 이 아니라 0.5 로 한 이유
#   좌커브 이격을 줄이면 차가 그만큼 왼쪽에 앉아 중앙선 여유를 먹는다.
#   차로 폭 96.2 cm, 목표 47.6 cm 이므로 중앙선은 E = +48.6 이고
#   차폭 절반을 30 cm 로 보면 왼쪽 끝이 닿는 지점이 E = +18.6 이다.
#   Phase1-58 에서 E 가 +17.5 까지 간 프레임이 실제로 있었다.
#       배율 1.0  좌커브 정착 E +0.2, 진동시 최대 +7.2
#       배율 0.5  좌커브 정착 E +3.4, 진동시 최대 +10.4  <- 채택
#       배율 0.3  좌커브 정착 E +4.6, 진동시 최대 +11.6
#   t=238 은 데드밴드가 흡수해 0.3 과 0.5 의 효과가 같고, t=240 에서만
#   1.8 카운트 차이다. 중앙선 여유 1.2 cm 를 그것과 바꿀 이유가 없다.
CURVE_OFFSET_LEFT_SCALE = 0.35
CURVE_OFFSET_ARM_CM      = 4.0
CURVE_OFFSET_ARM_SPAN_CM = 8.0

# 실제 직진에 해당하는 서보 값. 코드 안에서 "똑바로"는 전부 이 값이다.
# STEER_CENTER 는 아두이노 프로토콜상의 기준값일 뿐이므로 바꾸지 않는다.
STEER_STRAIGHT = STEER_CENTER + STEER_TRIM_COUNTS

# 목표 횡위치. 차량 중심선에서 우측 경계까지의 거리다.
# Phase1-27 이 쓰던 목표를 BEV 로 환산한 값이며 차로 중앙과 같다.
TARGET_LATERAL_CM = 50.6

# 곡률 피드포워드. counts per (1/cm).
#
# Phase1-28 실주행의 준정상 코너 410프레임에서 다시 구했다.
# BEV 로 곡률을 직접 재면서 동시에 실제 조향을 기록했으므로
# Phase1-27 영상 15프레임으로 추정하던 것보다 훨씬 정확하다.
#   원점 통과 최소제곱 = 6691 counts per (1/cm)
# Phase1-28 의 7200 은 8% 과다였고, PS 편향과 합쳐져 안쪽으로 밀었다.
#
# 튜닝 방향
#   코너에서 바깥(우측 실선) 쪽으로 계속 밀리면 올린다.
#   코너에서 안쪽(중앙 점선)을 파고들면 내린다.
#   1000 을 바꾸면 가장 급한 코너에서 횡위치가 약 2.7 cm 움직인다.
# Phase1-30: 트림을 분리하고 나면 순수 곡률 게인은 이 값이다.
# 랩 방향 적분으로 trim=+9 와 함께 동시에 풀었다.
# 검증 (예측 vs Phase1-28 실측 중앙값)
#     직선     +9.0  vs +10.0
#     보통코너 +19.3 vs +18.0
#     급코너   +23.1 vs +23.0
FF_COUNTS_PER_CURVATURE = 4900.0

# Phase1-29 D
# 곡률은 관측 길이가 짧으면 못 믿는다. 이 길이 아래로 내려가면
# FF 를 비례해서 줄이고 근거리 정보(횡오차, 헤딩)에 더 의존한다.
# Phase1-33 수정 2 : FF_TRUST 를 곡률 유지로 바꾼다.
#
# FF_TRUST 는 관측 길이가 짧으면 FF 를 줄이는 방식이었다.
# 그런데 관측이 짧아지는 곳이 바로 급굴곡이다.
# FF 가 가장 필요한 곳에서 FF 를 줄이고 있었으니 방향이 반대였다.
#
# 게다가 짧은 관측에서는 곡률 자체도 과소평가된다.
# 짧은 호에서는 가까운 평탄한 부분만 보이기 때문이다.
# Phase1-30 영상의 같은 코너에서
#     관측  53 cm -> kappa -0.00192
#     관측  60 cm -> kappa -0.00327
#     관측  83 cm -> kappa -0.00884
#     관측 100 cm -> kappa -0.00751
# 4.6배까지 차이가 났다. 여기에 신뢰계수까지 곱하면 이중으로 깎인다.
#
# 코너는 갑자기 완만해지지 않는다. 관측이 짧아졌다는 것은 굴곡이 급해서
# 경계선이 시야를 벗어났다는 뜻이지, 굴곡이 풀렸다는 뜻이 아니다.
# 그래서 신뢰할 수 있는 관측에서 얻은 곡률을 잠시 유지한다.
# 부호가 바뀌면(S자 변곡) 즉시 새 관측을 따른다.
CURVATURE_TRUST_SPAN_CM = 95.0
CURVATURE_HOLD_SEC = 1.2

# 횡방향 오차. counts per cm.
# 기존 KP_NEAR 0.28 counts/px x 4.75 px/cm = 1.33 을 승계했다.
#
# Phase1-59 : 1.30 -> 0.95
# 게인은 SPD 120 에서 잡았는데 지금 180~210 으로 달린다.
# wn = V sqrt(KE/G) 는 속도에 비례해 오르는데 지연(필터와 조향모터)은
# 시간 단위로 고정이라 위상 여유를 그만큼 까먹는다.
#     SPD 120 -> 고유주기 7.9 초
#     SPD 210 -> 고유주기 4.5 초
# Phase1-58 S 자에서 관측된 진동 주기가 약 4 초로 후자와 일치한다.
# 고유주파수에서 진동한다는 것은 위상 여유가 0 이라는 뜻이다.
# KE 를 0.95 로 낮추면 wn 이 0.855 배가 되어 여유가 돌아온다.
KE_COUNTS_PER_CM = 0.95

# 헤딩 오차. counts per deg.
#
# Phase1-29 A
# Phase1-28 에서 이 항은 감쇠가 아니라 두 번째 피드포워드로 작동했다.
# 카메라가 차량 회전중심보다 앞에 있어서, 코너를 정상적으로 돌기만 해도
# PSI 에 기하학적 성분이 생기기 때문이다. 실측 회귀로
#     PSI = C x kappa,  C = 54.2 cm
# 가 매우 잘 맞았다. 구간별로 보면
#     보통 코너  -9.2도  ->  보정 후 -2.3도
#     급코너    -12.0도  ->  보정 후 -2.6도
# 이 성분을 빼고 나면 남는 값이 진짜 헤딩 오차이고,
# 그것은 곧 횡오차의 거리 미분이므로 제대로 된 감쇠 항이 된다.
HEADING_KINEMATIC_CM = 54.0

# 이 항의 크기는 감쇠비가 정한다. 폐루프 오차 동역학은
#
#     d2y/dt2 + (V*KPSI_rad/G)*dy/dt + (V^2*KE/G)*y = 0
#
# 여기서 G = 6691 counts/(1/cm), V = 60 cm/s, KE = 1.30 counts/cm 다.
#   고유진동수 wn = V*sqrt(KE/G) = 0.836 rad/s (주기 7.5초, 주행 451cm)
#   감쇠비    zeta = V*KPSI_rad / (2*G*wn)
#
# Phase1-28 과 Phase1-29 초판의 0.60 은 zeta = 0.18 이다.
# 심각한 부족감쇠이고, 이것이 E 가 +10 에서 -20 cm 로 2초 만에
# 넘어가며 이탈한 진짜 이유였다.
#   zeta 0.70 이 되는 값 = 2.28 counts/deg
#
# 궤적 시뮬레이션(S자, 우측 10cm 치우쳐 진입)
#   KPSI 0.6 -> 최대 우측 편위 27.8 cm (여유 0.3 cm)
#   KPSI 2.3 -> 최대 우측 편위 12.6 cm (여유 15.5 cm)
#
# 이 값을 올릴 수 있게 된 것은 위의 기하 보정 덕분이다.
# 보정 전에는 KPSI 를 올리면 코너 편향이 같이 커져서 올릴 수 없었다.
# G 가 6691 에서 4900 으로 재동정되어 감쇠비를 다시 맞춘다.
#   wn = V*sqrt(KE/G) = 0.977 rad/s
#   KPSI 0.60 -> zeta 0.22   1.95 -> zeta 0.70   2.30 -> zeta 0.83
#
# Phase1-59 : 1.95 -> 1.67
# KE 를 낮추면 zeta = KPSI_rad / (2 sqrt(KE G)) 가 커진다.
# sqrt(0.95/1.30) = 0.855 이므로 KPSI 도 같은 비율로 낮춰야
# zeta 0.70 이 그대로 유지된다.  1.95 x 0.855 = 1.67
# 감쇠비는 건드리지 않고 루프 속도만 늦추는 조합이다.
KPSI_COUNTS_PER_DEG = 1.67

# 각 항이 낼 수 있는 최대 조향량
FF_COUNTS_LIMIT = 45.0
CTE_COUNTS_LIMIT = 30.0
# 정상 주행에서 감쇠항이 포화되면 안 된다. 시뮬레이션에서 통상 최대는
# 17 카운트였다. 30 이면 평상시 포화가 없고 큰 이탈 상황에서만 걸린다.
PSI_COUNTS_LIMIT = 30.0

# 미세 떨림 방지
CTE_DEADBAND_CM = 1.5
PSI_DEADBAND_DEG = 1.0

# Phase1-29 E
# 우측 실선에 가까워지면 우조향 허용량을 줄인다.
# 실선 이탈은 재위치와 페널티로 직결되므로 안전 포락선을 둔다.
# E 가 START 보다 작아지면 우조향을 줄이기 시작하고
# FULL 이하에서는 순수 우조향을 내지 않는다. 좌조향은 제한하지 않는다.
# Phase1-41: 헤딩 오차로 약 45 cm 앞의 우측 실선 여유를 예측한다.
# 양의 heading_error는 현재 제어 부호에서 우조향 방향의 오차이므로,
# 예측 여유를 계산할 때 현재 line_margin에서 차감한다.
RIGHT_GUARD_LOOKAHEAD_CM = 45.0
RIGHT_GUARD_HEADING_LIMIT_DEG = 12.0

# 예측 여유가 -2 cm보다 작아지면 우조향을 줄이기 시작하고,
# -10 cm 이하에서는 추가 우조향을 허용하지 않는다.
RIGHT_GUARD_START_CM = -2.0
RIGHT_GUARD_FULL_CM = -10.0

# 우굴곡은 BEV 곡률만으로 자동 판단한다. 횡단보도 상태를 사용하지 않는다.
RIGHT_CURVE_GUARD_CURVATURE = 0.0009

# 우굴곡에서는 양의 헤딩이 자연스럽게 나타나므로 예측 여유 대신
# 실제 실선 여유가 이 범위에 들어왔을 때만 우조향을 제한한다.
RIGHT_CURVE_GUARD_START_CM = 4.0
RIGHT_CURVE_GUARD_FULL_CM = -2.0

# Phase1-61 최소 수정:
# S자 우굴곡에서 우측 실선 여유가 4 cm 이하가 되면 우조향을 제한하고,
# 여유가 5 cm 이상으로 회복될 때까지 제한을 유지한다.
# STEER_STRAIGHT=177 기준 170은 약한 우조향이므로, 커브 추종은 남기면서
# 영상에서 확인된 150~160대의 과도한 우조향만 차단한다.
RIGHT_CURVE_GUARD_RELEASE_CM = 5.0
RIGHT_CURVE_GUARD_MIN_STEER = 170.0
RIGHT_CURVE_GUIDE_STEER = 162.0

# 예측 제한 이후에도 실제 여유가 계속 줄어들 경우 사용하는 최종 복구 장치.
# -10 cm부터 좌조향을 만들기 시작해 -16 cm에서 최대 7카운트를 사용한다.
RIGHT_RECOVERY_START_CM = -10.0
RIGHT_RECOVERY_FULL_CM = -16.0
RIGHT_RECOVERY_MAX_COUNTS = 7.0

# 저역통과 필터 시정수
CTE_FILTER_SEC = 0.10
PSI_FILTER_SEC = 0.12
CURVATURE_FILTER_SEC = 0.15

# 조향 명령 변화 속도 제한.
# Phase1-26 에서 낮은 복귀율(12/s)이 위상지연을 만들어 이탈로 이어졌다.
# 여기서는 서보 보호 목적으로만 두고 좌우 대칭으로 높게 잡는다.
STEER_RATE_PER_SEC = 150.0
STEER_COMMAND_DEADBAND = 2


# ======================================================================
# 6. 속도 스케줄
# ======================================================================
# 실제 곡률로 감속한다. Phase1-27 은 게이트가 열린 동안 거의 모든 코너에서
# 감속해 주행의 54.3%가 속도 110 이었고 랩타임이 4.1초 늘었다.
#
# 이 트랙의 실측 곡률 분포 (주행 영상 90프레임을 BEV 로 펴서 측정)
#   10백분위 0.00005 (R=21722cm)   25백분위 0.00098 (R=1025cm)
#   50백분위 0.00206 (R=  486cm)   75백분위 0.00242 (R= 413cm)
#   90백분위 0.00303 (R=  330cm)  최대     0.00366 (R= 273cm)
# 이 트랙은 중앙값이 R=486cm 로 거의 전 구간이 코너다.
# 그래서 임계를 낮게 잡으면 주행 대부분이 감속 상태가 된다.
#   START=0.0010 -> 74.4% 감속 (1-27 보다 나쁨)
#   START=0.0020 -> 52.2% 감속
#   START=0.0025 -> 21.1% 감속  <- 채택. 가장 급한 코너에서만 줄인다.

# ======================================================================
# Phase1-38 : 감속 스케줄을 실제로 걸리게 고쳤다
# ======================================================================
# Phase1-35(284프레임), Phase1-36(427프레임) 실주행 로그를 보면
# 전 프레임이 SPD=120 이다. 감속이 단 한 번도 걸리지 않았다.
#
# 원인은 문턱이다. START 가 0.0025 인데 실측 곡률 분포가
#     50%  0.00131 (R 763cm)    90%  0.00187 (R 535cm)
#     95%  0.00204 (R 490cm)   100%  0.00252 (R 397cm)
# 라서 문턱에 닿지를 않는다. FULL 0.0036 은 실측 최대보다도 높아
# SPEED_MIN 에는 애초에 도달할 수 없었다.
#
# 위 값들의 근거였던 옛 분석은 곡률 상한이 0.0040 이던 시절 것이다.
# Phase1-33 에서 상한을 0.0110 으로 올린 뒤 곡률 척도가 바뀌었는데
# 속도 문턱은 같이 옮기지 않았다.
#
# 새 값으로 같은 로그를 다시 계산하면
#     감속이 걸리는 프레임 66%, 평균 속도 114, 최저 95
# 즉 가장 급한 코너에서 21% 줄이고 랩 전체로는 5% 손해다.
#
# 왜 감속인가
#   커브 안쪽 파고듦이 실측 -4.7 cm 인데 이는 타이어 슬립이고
#   속도 제곱에 비례한다. 그리고 프레임률이 4.72 fps 로 고정이라
#   SPD 120 에서는 제어 한 번에 10.3 cm 를 진행한다. 느릴수록
#   제어가 촘촘해지고 카메라와 조향 모터의 지연이 거리로 환산될 때
#   작아진다.
#
#   무엇보다 감속은 제어 루프 밖이다. Phase1-36 과 Phase1-37 을
#   망친 것은 전부 루프 안을 건드린 것이었다. 감속은 그 위험이 없다.
#
# 랩타임에 대해
#   지금 차는 전 구간 120 고정이라 속도 차등이 아예 없다. 랩타임을
#   줄이려면 직선에서 빨라져야 하는데, 곡률 감속이 없는 상태로
#   SPEED_MAX 만 올리면 모든 커브에서 이탈한다. 이 스케줄을 살리는
#   것이 속도를 올리기 위한 선행조건이다.
SPEED_MAX = 255
SPEED_MIN = 220

# 우굴곡에서 DC 조향모터가 반응할 시간을 확보하는 자동 속도 상한.
RIGHT_CURVE_SPEED_CURVATURE = 0.0007
RIGHT_CURVE_SPEED_LIMIT = 185

# 반지름 1000 cm 부터 줄이기 시작한다.
SPEED_CURVATURE_START = 0.0012
# 반지름 400 cm 에서 SPEED_MIN 이 된다. 실측 최대 곡률이 여기다.
SPEED_CURVATURE_FULL = 0.0020

LOST_DRIVE_SPEED = 80


# ======================================================================
# 7. 정지선·횡단보도 (Phase1-27 의 래치 차단을 그대로 유지)
# ======================================================================

HORIZONTAL_KERNEL_WIDTH = 35
HORIZONTAL_KERNEL_HEIGHT = 3
# 가로선 위아래의 잔여 픽셀도 함께 지우는 Phase1-52 값.
HORIZONTAL_REMOVAL_DILATE_HEIGHT = 5
VERTICAL_RECONNECT_HEIGHT = 19

MARKING_REGION_START_RATIO = 0.38
MARKING_REGION_END_RATIO = 0.82

ABSOLUTE_LEFT_RATIO = 0.42
ABSOLUTE_RIGHT_RATIO = 0.98

HORIZONTAL_MIN_PIXELS = 250

BOUNDARY_MIN_LATERAL_CM = 15.0
CROSSWALK_MIN_BARS = 3
CROSSWALK_BAR_MIN_WIDTH = 35
CROSSWALK_BAR_MIN_HEIGHT = 12
# A true crosswalk bar is a tall, filled strip in the driving direction.
# Thin stop lines and the horizontal edges of hollow boxes fail these checks.
CROSSWALK_BAR_MIN_ASPECT_RATIO = 1.25
CROSSWALK_BAR_MIN_FILL_RATIO = 0.55

CROSSWALK_WHITE_RATIO = 0.10
CROSSWALK_CLEAR_WHITE_RATIO = 0.08
CROSSWALK_CONFIRM_FRAMES = 8
CROSSWALK_CLEAR_FRAMES = 6
HORIZONTAL_CLEAR_FRAMES = 4

HORIZONTAL_MAX_CONTINUOUS_SEC = 3.0
CROSSWALK_MAX_ACTIVE_SEC = 4.0
CROSSWALK_REARM_BLOCK_SEC = 6.0

# Phase1-52의 횡단보도 속도 상한.
# SPEED_MAX=210, SPEED_MIN=150이면 160으로 계산된다.
CROSSWALK_SPEED_RATIO = 0.88
CROSSWALK_SPEED_FLOOR = 120
CROSSWALK_SPEED_HARD_MAX = 200
MARKING_SPEED_RELEASE_HOLD_SEC = 0.50
CROSSWALK_TRACK_RELEASE_HOLD_SEC = 0.20

# 횡단보도에서 경계를 잃었을 때만 강한 직전 조향을 제한한다.
# 일반 구간의 LOST 처리는 Phase1-41을 그대로 사용한다.
CROSSWALK_LOST_TRANSITION_SEC = 0.20
CROSSWALK_LOST_STEER_LIMIT_COUNTS = 8.0
CROSSWALK_LOST_SPEED = 120

# 횡단보도 감속 명령만 급변하지 않게 만든다.
CROSSWALK_SPEED_DECEL_PER_SEC = 110.0
CROSSWALK_SPEED_ACCEL_PER_SEC = 90.0


# ======================================================================
# 8. 상실 처리와 시간
# ======================================================================

LOST_COMMAND_HOLD_FRAMES = 20

# Phase1-65: 카메라 FPS 변화와 무관하도록 프레임 수가 아니라 시간으로
# S자 우굴곡 LOST를 처리한다. 처음 0.25초는 마지막 우조향을 164~168로
# 유지하고, 이어지는 0.55초는 170의 약한 우조향으로 재검출을 기다린다.
S_RIGHT_LOST_STRONG_SEC = 0.25
S_RIGHT_LOST_TOTAL_SEC = 0.80
S_RIGHT_LOST_STRONG_MIN_STEER = 164.0
S_RIGHT_LOST_STRONG_MAX_STEER = 168.0
S_RIGHT_LOST_WEAK_STEER = 170.0

MIN_CONTROL_DT = 0.005
MAX_CONTROL_DT = 0.100

MAX_CAMERA_FAILURES = 3

DEBUG_PRINT_INTERVAL_SEC = 0.20


# ======================================================================
# 9. 공통 계산
# ======================================================================

def clamp(value, minimum, maximum):
    """값을 지정된 최소·최대 범위로 제한한다."""

    return max(minimum, min(value, maximum))


def move_toward(value, target, maximum_step):
    """현재 값을 목표값 방향으로 일정량만 이동한다."""

    if value < target:
        return min(value + maximum_step, target)

    if value > target:
        return max(value - maximum_step, target)

    return value


def low_pass(previous_value, measured_value, delta_time, time_constant):
    """시간 기반 저역통과 필터를 적용한다."""

    safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))
    alpha = 1.0 - math.exp(-safe_dt / time_constant)

    return previous_value + alpha * (measured_value - previous_value)


def apply_deadband(value, deadband):
    """작은 값은 0으로 만든다."""

    if abs(value) <= deadband:
        return 0.0

    if value > 0.0:
        return value - deadband

    return value + deadband


# ======================================================================
# 10. 아두이노 명령
# ======================================================================

def send_command(comm, direction, speed, steer):
    """아두이노에 방향·속도·조향 명령을 전송한다."""

    safe_direction = int(clamp(direction, DIRECTION_STOP, DIRECTION_REVERSE))
    safe_speed = int(clamp(speed, 0, 255))
    safe_steer = int(clamp(steer, STEER_RIGHT_LIMIT, STEER_LEFT_LIMIT))

    command = "%d,%d,%d\n" % (safe_direction, safe_speed, safe_steer)
    comm.write(command.encode("utf-8"))


def send_stop(comm):
    """차량을 정지하고 조향을 중앙으로 보낸다."""

    send_command(comm, DIRECTION_STOP, 0, STEER_STRAIGHT)


# ======================================================================
# 11. 영상 전처리와 가로선 제거
# ======================================================================

def preprocess_roi(roi):
    """컬러 ROI를 흰색 차선용 이진 영상으로 변환한다."""

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, GAUSSIAN_KERNEL, 0)
    _, binary = cv2.threshold(
        blurred, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY
    )

    return binary


def remove_horizontal_markings(binary_image):
    """
    정지선·횡단보도·네모박스의 긴 가로 성분을 제거한다.

    BEV 로 펴도 이 처리는 필요하다. 가로 마킹이 우측 경계 검출을
    오염시키는 것은 원근이든 BEV 든 마찬가지다.
    """

    horizontal_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (HORIZONTAL_KERNEL_WIDTH, HORIZONTAL_KERNEL_HEIGHT),
    )

    horizontal_mask = cv2.morphologyEx(
        binary_image, cv2.MORPH_OPEN, horizontal_kernel
    )

    removal_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (1, HORIZONTAL_REMOVAL_DILATE_HEIGHT)
    )
    removal_mask = cv2.dilate(
        horizontal_mask, removal_kernel, iterations=1
    )

    lane_binary = cv2.bitwise_and(
        binary_image, cv2.bitwise_not(removal_mask)
    )

    vertical_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, (3, VERTICAL_RECONNECT_HEIGHT)
    )

    lane_binary = cv2.morphologyEx(
        lane_binary, cv2.MORPH_CLOSE, vertical_kernel
    )

    return lane_binary, horizontal_mask


# ======================================================================
# 12. BEV 변환
# ======================================================================

def load_calibration():
    """
    BEV 호모그래피와 차량 기준점을 만든다.

    옆에 bev_calib.json 이 있으면 그것을 우선 읽는다.
    없으면 이 파일에 적힌 실측값으로 계산한다.
    두 경로 모두 BEV_Calibration.py 와 완전히 같은 방식이다.
    """

    source_points = list(BEV_SOURCE_POINTS)
    lane_width_cm = BEV_MARKER_WIDTH_CM
    length_cm = BEV_MARKER_LENGTH_CM
    pixels_per_cm = BEV_PIXELS_PER_CM
    bev_width = BEV_WIDTH
    bev_height = BEV_HEIGHT
    origin = "코드 내장값"

    if os.path.exists(BEV_CALIB_FILE):
        try:
            with open(BEV_CALIB_FILE, "r", encoding="utf-8") as handle:
                data = json.load(handle)

            source_points = [tuple(p) for p in data["source_points"]]
            lane_width_cm = float(data["lane_width_cm"])
            length_cm = float(data["marker_length_cm"])
            pixels_per_cm = float(data["pixels_per_cm"])
            bev_width = int(data["bev_width"])
            bev_height = int(data["bev_height"])
            origin = BEV_CALIB_FILE

        except Exception as error:
            print("경고: %s 를 읽지 못해 내장값을 씁니다. (%s)"
                  % (BEV_CALIB_FILE, error))

    half_width = lane_width_cm * pixels_per_cm / 2.0
    length_px = length_cm * pixels_per_cm

    centre_x = bev_width / 2.0
    near_y = bev_height - BEV_NEAR_MARGIN_PX
    far_y = near_y - length_px

    destination = np.float32([
        [centre_x - half_width, near_y],
        [centre_x + half_width, near_y],
        [centre_x + half_width, far_y],
        [centre_x - half_width, far_y],
    ])

    transform = cv2.getPerspectiveTransform(
        np.float32(source_points), destination
    )

    # 차량 기준점. ROI 하단 중앙이 카메라의 전방 축이다.
    roi_height = ROI_Y_END - ROI_Y_START
    vehicle = cv2.perspectiveTransform(
        np.float32([[[640 / 2.0, roi_height - 1]]]), transform
    )

    calibration = {
        "transform": transform,
        "pixels_per_cm": pixels_per_cm,
        "bev_width": bev_width,
        "bev_height": bev_height,
        "vehicle_x": float(vehicle[0][0][0]),
        "vehicle_y": float(vehicle[0][0][1]),
        "origin": origin,
    }

    return calibration


def warp_to_bev(image, calibration):
    """ROI 를 BEV 로 펴준다."""

    return cv2.warpPerspective(
        image,
        calibration["transform"],
        (
            calibration["bev_width"] + BEV_RIGHT_EXTRA_PX,
            calibration["bev_height"],
        ),
        flags=cv2.INTER_LINEAR,
    )


# ======================================================================
# 13. BEV 경계 검출
# ======================================================================

def find_white_runs(binary_image, center_y):
    """지정한 높이 주변에서 세로 방향 흰색 묶음의 중심 x 를 찾는다."""

    height, width = binary_image.shape

    y_start = max(0, int(center_y) - BEV_SCAN_HALF_HEIGHT_PX)
    y_end = min(height, int(center_y) + BEV_SCAN_HALF_HEIGHT_PX + 1)

    if y_end <= y_start:
        return []

    band = binary_image[y_start:y_end]
    needed = max(2, int(math.ceil(band.shape[0] * 0.5)))
    active = np.count_nonzero(band, axis=0) >= needed

    runs = []
    start = None

    for x in range(len(active)):
        if active[x] and start is None:
            start = x

        last = x == len(active) - 1

        if start is not None and (not active[x] or last):
            end = x if (active[x] and last) else x - 1
            run_width = end - start + 1

            if MIN_LINE_WIDTH_PX <= run_width <= MAX_LINE_WIDTH_PX:
                runs.append((start + end) / 2.0)

            start = None

    return runs


def detect_bev_boundary(bev_binary, previous_poly, calibration):
    """
    BEV 에서 우측 외곽 실선을 훑는다.

    아래(가까운 쪽)에서 위(먼 쪽)로 올라가며 한 행씩 고른다.
    첫 행은 이전 프레임 곡선이 있으면 그것을, 없으면 가장 오른쪽을 쓴다.
    이후 행은 바로 아래 행에서 크게 벗어나지 않는 후보만 받는다.
    BEV 는 미터법이라 한 단계에 경계가 움직일 수 있는 거리가
    물리적으로 정해져 있고, 그래서 이 제한이 원근 영상보다 훨씬 잘 듣는다.
    """

    points = []
    previous_x = None

    # 새로 늘어난 오른쪽 영역에서도 첫 검출과 재검출이 가능하게 한다.
    # 왼쪽 시작 범위는 바꾸지 않으므로 중앙점선 선택 가능성은 늘지 않는다.
    initial_right_limit = min(
        bev_binary.shape[1] - 1,
        BEV_INIT_RIGHT_PX + BEV_RIGHT_EXTRA_PX,
    )

    scan_top = max(BEV_SCAN_TOP_PX, 12)

    for y in range(BEV_SCAN_BOTTOM_PX, scan_top, -BEV_SCAN_STEP_PX):
        runs = find_white_runs(bev_binary, y)

        if not runs:
            continue

        if previous_x is not None:
            expected = previous_x
            window = BEV_STEP_JUMP_PX

        elif previous_poly is not None:
            expected = evaluate_boundary_x(previous_poly, y, calibration)
            window = BEV_TRACK_WINDOW_PX

        else:
            expected = None
            window = None

        if expected is None:
            candidates = [
                x for x in runs
                if BEV_INIT_LEFT_PX <= x <= initial_right_limit
            ]

            if not candidates:
                continue

            selected = max(candidates)

        else:
            candidates = [
                x for x in runs if abs(x - expected) <= window
            ]

            if not candidates:
                continue

            selected = min(candidates, key=lambda x: abs(x - expected))

        points.append((y, selected))
        previous_x = selected

    return points


def extract_right_green_component(bev_colour):
    """BEV 오른쪽까지 이어진 큰 녹색 연결영역만 남긴다."""

    if bev_colour is None or bev_colour.ndim != 3:
        return None

    hsv = cv2.cvtColor(bev_colour, cv2.COLOR_BGR2HSV)
    green_mask = cv2.inRange(
        hsv,
        np.array([GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN], dtype=np.uint8),
        np.array([GREEN_H_MAX, 255, 255], dtype=np.uint8),
    )

    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        (GREEN_CLOSE_KERNEL_PX, GREEN_CLOSE_KERNEL_PX),
    )
    green_mask = cv2.morphologyEx(
        green_mask, cv2.MORPH_CLOSE, close_kernel
    )

    label_count, labels, stats, _ = cv2.connectedComponentsWithStats(
        green_mask, connectivity=8
    )

    image_height, image_width = green_mask.shape
    minimum_area = int(round(
        image_height * image_width * GREEN_MIN_COMPONENT_AREA_RATIO
    ))
    minimum_right_x = image_width * GREEN_COMPONENT_MIN_RIGHT_RATIO

    best_label = 0
    best_area = 0

    for label in range(1, label_count):
        left = int(stats[label, cv2.CC_STAT_LEFT])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        area = int(stats[label, cv2.CC_STAT_AREA])
        right = left + width - 1

        if area < minimum_area or right < minimum_right_x:
            continue

        if area > best_area:
            best_area = area
            best_label = label

    if best_label == 0:
        return None

    component = np.zeros_like(green_mask)
    component[labels == best_label] = 255

    return component


def detect_green_boundary(bev_colour):
    """오른쪽 녹색 연결영역의 왼쪽 끝을 행별 보조 경계점으로 만든다."""

    component = extract_right_green_component(bev_colour)

    if component is None:
        return []

    points = []
    scan_top = max(BEV_SCAN_TOP_PX, 12)

    for y in range(BEV_SCAN_BOTTOM_PX, scan_top, -BEV_SCAN_STEP_PX):
        y_start = max(0, y - BEV_SCAN_HALF_HEIGHT_PX)
        y_end = min(
            component.shape[0],
            y + BEV_SCAN_HALF_HEIGHT_PX + 1,
        )

        band = component[y_start:y_end]

        if band.size == 0:
            continue

        needed = max(2, int(math.ceil(band.shape[0] * 0.45)))
        active = np.count_nonzero(band, axis=0) >= needed
        active_x = np.flatnonzero(active)

        if active_x.size == 0:
            continue

        points.append((y, float(active_x[0])))

    return points


def evaluate_boundary_x(poly, bev_y, calibration):
    """미터 단위 2차식을 BEV 픽셀 x 로 되돌린다."""

    pixels_per_cm = calibration["pixels_per_cm"]
    forward_cm = (calibration["vehicle_y"] - bev_y) / pixels_per_cm
    lateral_cm = np.polyval(poly, forward_cm)

    return calibration["vehicle_x"] + lateral_cm * pixels_per_cm


# ======================================================================
# 14. 기하 추정
# ======================================================================

def fit_boundary(points, calibration):
    """
    BEV 점들을 미터 단위 2차식으로 맞춘다.

    반환하는 값의 정의
      lateral_cm : 차량 기준선에서 우측 경계까지의 거리. 오른쪽이 양수.
      heading_deg: 경계 접선이 차량 전방축과 이루는 각. 왼쪽으로 휘면 음수.
      curvature  : 1/cm. 좌회전이 음수.
    """

    if len(points) < MIN_BOUNDARY_POINTS:
        return None

    pixels_per_cm = calibration["pixels_per_cm"]

    forward = np.array(
        [(calibration["vehicle_y"] - y) / pixels_per_cm for y, _ in points],
        dtype=np.float64,
    )

    lateral = np.array(
        [(x - calibration["vehicle_x"]) / pixels_per_cm for _, x in points],
        dtype=np.float64,
    )

    span_cm = float(np.max(forward) - np.min(forward))

    if span_cm < MIN_FIT_SPAN_CM:
        return None

    poly = np.polyfit(forward, lateral, 2)
    residual = lateral - np.polyval(poly, forward)

    # 이상치를 한 번 걸러내고 다시 맞춘다.
    keep = np.abs(residual) <= FIT_OUTLIER_CM

    if int(np.count_nonzero(keep)) >= MIN_BOUNDARY_POINTS:
        poly = np.polyfit(forward[keep], lateral[keep], 2)
        residual = lateral[keep] - np.polyval(poly, forward[keep])

    max_residual = float(np.max(np.abs(residual)))

    if max_residual > FIT_MAX_RESIDUAL_CM:
        return None

    slope = float(poly[1])

    # 곡선 x = d(s) 의 곡률. 기울기가 클 때의 보정을 포함한다.
    curvature = float(
        2.0 * poly[0] / math.pow(1.0 + slope * slope, 1.5)
    )

    # Phase1-29 C: 물리적으로 불가능한 곡률은 잘라낸다.
    curvature = float(clamp(
        curvature, -CURVATURE_LIMIT_ABS, CURVATURE_LIMIT_ABS
    ))

    return {
        "poly": poly,
        "lateral_cm": float(poly[2]),
        "heading_deg": math.degrees(math.atan(slope)),
        "curvature": curvature,
        "max_residual_cm": max_residual,
        "point_count": len(points),
        "span_cm": span_cm,
        "forward_max_cm": float(np.max(forward)),
    }


def crosswalk_entry_is_qualified(geometry_state):
    """직진에 가까운 정상 진입에서만 횡단보도 경계 고정을 허용한다."""

    return (
        geometry_state["ready"]
        and abs(geometry_state["heading_deg"])
        <= CROSSWALK_ENTRY_MAX_HEADING_DEG
        and abs(geometry_state["curvature"])
        <= CROSSWALK_ENTRY_MAX_CURVATURE
    )


def copy_geometry(geometry):
    """짧은 경계 예측용으로 기하 정보를 독립 복사한다."""

    if geometry is None:
        return None

    copied = dict(geometry)
    copied["poly"] = np.array(geometry["poly"], dtype=np.float64).copy()
    return copied


def stabilize_green_geometry(measured, previous, delta_time):
    """녹색 경계 기하값을 버리지 않고 물리적인 변화 속도만 제한한다."""

    if measured is None:
        return None

    if previous is None:
        return copy_geometry(measured)

    safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))
    stabilized = copy_geometry(measured)

    stabilized["lateral_cm"] = move_toward(
        previous["lateral_cm"],
        measured["lateral_cm"],
        GREEN_CONTROL_LATERAL_RATE_CM_PER_SEC * safe_dt,
    )
    stabilized["heading_deg"] = move_toward(
        previous["heading_deg"],
        measured["heading_deg"],
        GREEN_CONTROL_HEADING_RATE_DEG_PER_SEC * safe_dt,
    )
    stabilized["curvature"] = move_toward(
        previous["curvature"],
        measured["curvature"],
        GREEN_CONTROL_CURVATURE_RATE_PER_SEC * safe_dt,
    )

    # 제한된 횡거리·헤딩·곡률과 일치하도록 2차 경계식도 다시 만든다.
    slope = math.tan(math.radians(stabilized["heading_deg"]))
    quadratic = (
        0.5
        * stabilized["curvature"]
        * math.pow(1.0 + slope * slope, 1.5)
    )
    stabilized["poly"] = np.array(
        [quadratic, slope, stabilized["lateral_cm"]],
        dtype=np.float64,
    )

    return stabilized


def create_green_boundary_state():
    """녹색 외곽 경계의 흰 실선 대비 오프셋과 신뢰 상태를 만든다."""

    return {
        "offset_cm": 2.0,
        "match_frames": 0,
        "missing_frames": 0,
        "ready": False,
    }


def update_green_boundary_state(state, white_geometry, green_geometry,
                                delta_time):
    """흰 실선과 녹색 외곽 경계가 여러 프레임 일치할 때만 신뢰한다."""

    if green_geometry is None:
        state["missing_frames"] += 1

        if state["missing_frames"] >= GREEN_TRUST_LOSS_FRAMES:
            state["ready"] = False
            state["match_frames"] = 0

        return

    state["missing_frames"] = 0

    if white_geometry is None:
        return

    measured_offset = (
        green_geometry["lateral_cm"] - white_geometry["lateral_cm"]
    )
    heading_difference = abs(
        green_geometry["heading_deg"] - white_geometry["heading_deg"]
    )
    curvature_difference = abs(
        green_geometry["curvature"] - white_geometry["curvature"]
    )

    matches = (
        GREEN_OFFSET_MIN_CM <= measured_offset <= GREEN_OFFSET_MAX_CM
        and heading_difference <= GREEN_MATCH_MAX_HEADING_DEG
        and curvature_difference <= GREEN_MATCH_MAX_CURVATURE
    )

    if matches:
        state["offset_cm"] = low_pass(
            state["offset_cm"],
            measured_offset,
            delta_time,
            GREEN_OFFSET_FILTER_SEC,
        )
        state["match_frames"] = min(
            GREEN_TRUST_CONFIRM_FRAMES,
            state["match_frames"] + 1,
        )

        if state["match_frames"] >= GREEN_TRUST_CONFIRM_FRAMES:
            state["ready"] = True
    else:
        state["match_frames"] = max(0, state["match_frames"] - 2)

        if state["match_frames"] == 0:
            state["ready"] = False


def correct_green_geometry(green_geometry, offset_cm):
    """녹색 영역의 왼쪽 끝을 흰 우측 실선 중심 위치로 보정한다."""

    if green_geometry is None:
        return None

    corrected = dict(green_geometry)
    corrected_poly = np.array(green_geometry["poly"], dtype=np.float64)
    corrected_poly[2] -= float(offset_cm)

    corrected["poly"] = corrected_poly
    corrected["lateral_cm"] = (
        green_geometry["lateral_cm"] - float(offset_cm)
    )

    return corrected


def green_geometry_is_safe(green_geometry, geometry_state):
    """검증된 녹색 보조 경계를 현재 상태에서 안전하게 쓸 수 있는지 본다."""

    if green_geometry is None:
        return False

    if green_geometry["point_count"] < GREEN_MIN_BOUNDARY_POINTS:
        return False

    lateral_cm = green_geometry["lateral_cm"]

    if not (
        PLAUSIBLE_LATERAL_MIN_CM
        <= lateral_cm
        <= PLAUSIBLE_LATERAL_MAX_CM
    ):
        return False

    if not geometry_state["ready"]:
        return True

    lateral_jump = abs(
        lateral_cm - geometry_state["lateral_cm"]
    )
    heading_jump = abs(
        green_geometry["heading_deg"] - geometry_state["heading_deg"]
    )
    curvature_jump = abs(
        green_geometry["curvature"] - geometry_state["curvature"]
    )

    return (
        lateral_jump <= GREEN_FALLBACK_MAX_LATERAL_JUMP_CM
        and heading_jump <= GREEN_FALLBACK_MAX_HEADING_JUMP_DEG
        and curvature_jump <= GREEN_FALLBACK_MAX_CURVATURE_JUMP
    )


def create_geometry_state():
    """필터링된 기하값 상태를 만든다."""

    return {
        "lateral_cm": TARGET_LATERAL_CM,
        "heading_deg": 0.0,
        "curvature": 0.0,
        "held_curvature": 0.0,
        "span_cm": 0.0,
        "ready": False,
        # Phase1-39 : 적분 누적값. 단위는 조향 카운트다.
        "integral": 0.0,
        # Phase1-61 : S자 우굴곡의 조건부 우조향 제한 상태.
        "right_curve_guard_active": False,
        # Phase1-63: 짧은 곡률 검출 공백에도 우측 복귀를 유지한다.
        "s_right_phase_active": False,
        # 좌굴곡 -> 우굴곡 전환을 짧게 확인하기 위한 상태다.
        "s_right_transition_sec": 0.0,
        "s_right_transition_pending": False,
        "s_right_dropout_sec": 0.0,
        "s_right_release_sec": 0.0,
    }


def update_geometry_state(state, geometry, delta_time):
    """측정된 기하값에 저역통과 필터를 적용한다."""

    state["span_cm"] = geometry["span_cm"]

    # Phase1-33 : 관측 길이가 충분할 때의 곡률만 신뢰하고, 짧아지면 유지한다.
    measured = geometry["curvature"]

    if geometry["span_cm"] >= CURVATURE_TRUST_SPAN_CM:
        state["held_curvature"] = measured
        effective = measured

    else:
        held = state["held_curvature"] * math.exp(
            -float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))
            / CURVATURE_HOLD_SEC
        )

        if measured * held > 0.0 and abs(held) > abs(measured):
            effective = held
            state["held_curvature"] = held

        else:
            effective = measured
            state["held_curvature"] = measured

    if not state["ready"]:
        state["lateral_cm"] = geometry["lateral_cm"]
        state["heading_deg"] = geometry["heading_deg"]
        state["curvature"] = effective
        state["ready"] = True
        return

    state["lateral_cm"] = low_pass(
        state["lateral_cm"], geometry["lateral_cm"],
        delta_time, CTE_FILTER_SEC,
    )

    state["heading_deg"] = low_pass(
        state["heading_deg"], geometry["heading_deg"],
        delta_time, PSI_FILTER_SEC,
    )

    state["curvature"] = low_pass(
        state["curvature"], effective,
        delta_time, CURVATURE_FILTER_SEC,
    )


# ======================================================================
# 15. 제어
# ======================================================================

def calculate_steering(state, delta_time):
    """
    BEV 기하값에서 조향 명령을 만든다.

    부호 규약
      counts 가 양수면 좌조향이다. steer = STEER_CENTER + counts.
      curvature 가 음수면 좌회전이므로 -curvature 를 쓴다.
      lateral 이 목표보다 작으면 차량이 우측 실선에 붙은 것이므로
      e = lateral - 목표 가 음수가 되고, -e 가 좌조향을 만든다.
      heading 이 음수면 경계가 앞에서 왼쪽으로 가므로 -heading 이 좌조향이다.
    """

    curvature = state["curvature"]

    # ------------------------------------------------------------------
    # Phase1-40 : 두 가지 횡오차를 구분한다
    # ------------------------------------------------------------------
    #   line_margin   실제 우측 실선까지의 여유. 안전 판단은 전부 이
    #                 값으로 한다. 로그에 찍히는 E 와 같은 값이다.
    #   lateral_error 추종할 목표까지의 오차. 곡률에 따라 목표를
    #                 옮기므로 line_margin 과 달라진다.
    line_margin = state["lateral_cm"] - TARGET_LATERAL_CM

    # Phase1-59 : 우커브는 그대로, 좌커브만 절반으로 줄인다.
    raw_offset = CURVE_OFFSET_CM_PER_CURVATURE * curvature

    if curvature < 0.0:
        raw_offset *= CURVE_OFFSET_LEFT_SCALE
    else:
        # 이미 중앙선 쪽에 있으면 목표를 더 왼쪽으로 밀지 않는다.
        raw_offset *= clamp(
            (CURVE_OFFSET_ARM_CM - line_margin)
            / CURVE_OFFSET_ARM_SPAN_CM,
            0.0, 1.0,
        )

    curve_offset = clamp(
        raw_offset,
        -CURVE_OFFSET_LIMIT_CM, CURVE_OFFSET_LIMIT_CM,
    )

    lateral_error = line_margin - curve_offset

    # Phase1-29 A
    # 측정된 PSI 에서 기하 성분을 뺀다. 코너를 정상적으로 돌기만 해도
    # 카메라가 회전중심보다 앞에 있어 atan(C x kappa) 만큼 각이 생긴다.
    # 그 성분을 빼야 남는 값이 진짜 헤딩 오차이고, 그것이 감쇠로 작동한다.
    kinematic_deg = math.degrees(
        math.atan(HEADING_KINEMATIC_CM * curvature)
    )

    heading_error = state["heading_deg"] - kinematic_deg

    # Phase1-29 D
    # 관측 길이가 짧으면 곡률을 덜 믿는다.
    # Phase1-33 : 관측 길이에 따른 감쇠를 없앴다.
    # 곡률 신뢰도는 update_geometry_state 의 유지 로직이 담당한다.
    feedforward = clamp(
        FF_COUNTS_PER_CURVATURE * (-curvature),
        -FF_COUNTS_LIMIT, FF_COUNTS_LIMIT,
    )

    cross_track = clamp(
        KE_COUNTS_PER_CM * (-apply_deadband(lateral_error, CTE_DEADBAND_CM)),
        -CTE_COUNTS_LIMIT, CTE_COUNTS_LIMIT,
    )

    heading_term = clamp(
        KPSI_COUNTS_PER_DEG
        * (-apply_deadband(heading_error, PSI_DEADBAND_DEG)),
        -PSI_COUNTS_LIMIT, PSI_COUNTS_LIMIT,
    )

    # S자에서 곡률이 우굴곡으로 바뀐 뒤에도 이전 좌굴곡의 큰 음수
    # 헤딩 오차가 잠시 남는다. 이 값이 우회전 시작을 막지 않도록
    # 좌조향 성분만 제한하고, 우조향 및 우측 실선 복구는 건드리지 않는다.
    if (
        curvature >= S_RIGHT_TRANSITION_CURVATURE
        and heading_error <= S_RIGHT_TRANSITION_HEADING_DEG
        and heading_term > S_RIGHT_TRANSITION_MAX_LEFT_HEADING_COUNTS
    ):
        heading_term = S_RIGHT_TRANSITION_MAX_LEFT_HEADING_COUNTS

    # ------------------------------------------------------------------
    # Phase1-39 : 적분항
    # ------------------------------------------------------------------
    # 직전 프레임까지 쌓인 값을 이번 조향에 그대로 쓴다.
    # 누적 갱신은 아래에서 포화 여부를 본 뒤에 한다.
    integral_term = clamp(
        state["integral"], -INTEGRAL_LIMIT_COUNTS, INTEGRAL_LIMIT_COUNTS
    )

    total = feedforward + cross_track + heading_term + integral_term

    # ------------------------------------------------------------------
    # Phase1-41 : 예측 기반 우측 실선 보호
    # ------------------------------------------------------------------
    # 기존 가드는 실제 E가 -8 cm 아래로 내려간 뒤에야 작동했다.
    # S자에서는 그 시점보다 먼저 차량의 헤딩이 우측 실선을 향하므로,
    # 현재 헤딩 오차를 이용해 약 45 cm 앞의 여유를 먼저 계산한다.
    closing_heading_deg = clamp(
        heading_error, 0.0, RIGHT_GUARD_HEADING_LIMIT_DEG
    )

    predicted_margin = (
        line_margin
        - RIGHT_GUARD_LOOKAHEAD_CM
        * math.tan(math.radians(closing_heading_deg))
    )

    # 우굴곡에서는 양의 heading_error가 정상적으로 발생한다.
    # 기존 predicted_margin을 그대로 쓰면 이 헤딩을 실선 접근으로
    # 오인해 필요한 우조향까지 지우므로 실제 line_margin을 사용한다.
    # Phase1-63: 우측 굴곡은 매 프레임 다시 판단하지 않고 잠금 상태로 관리한다.
    # 곡률이 잠깐 작아져도 바로 직진으로 풀지 않으며, 실제 출구처럼 곡률과
    # 방향이 함께 안정되거나 검출 공백이 충분히 길 때만 우측 복귀를 끝낸다.
    right_curve_candidate = (
        curvature >= S_RIGHT_TRANSITION_CURVATURE
        and heading_error <= S_RIGHT_TRANSITION_HEADING_DEG
    )
    right_curve_measured = (
        curvature >= S_RIGHT_TRANSITION_CURVATURE
    )

    if not state["s_right_phase_active"]:
        if right_curve_candidate:
            state["s_right_transition_sec"] += delta_time
            state["s_right_transition_pending"] = True

            if (
                state["s_right_transition_sec"]
                >= S_RIGHT_TRANSITION_CONFIRM_SEC
            ):
                state["s_right_phase_active"] = True
                state["s_right_transition_pending"] = False
                state["s_right_transition_sec"] = 0.0
                state["s_right_dropout_sec"] = 0.0
                state["s_right_release_sec"] = 0.0
        else:
            state["s_right_transition_sec"] = 0.0
            state["s_right_transition_pending"] = False
            state["s_right_dropout_sec"] = 0.0
            state["s_right_release_sec"] = 0.0

    elif right_curve_measured:
        state["s_right_transition_sec"] = 0.0
        state["s_right_transition_pending"] = False
        state["s_right_dropout_sec"] = 0.0
        state["s_right_release_sec"] = 0.0

    else:
        state["s_right_transition_pending"] = False
        state["s_right_dropout_sec"] += delta_time

        stable_right_exit = (
            abs(curvature) <= S_RIGHT_RELEASE_CURVATURE
            and abs(heading_error) <= S_RIGHT_RELEASE_HEADING_DEG
        )

        if stable_right_exit:
            state["s_right_release_sec"] += delta_time
        else:
            state["s_right_release_sec"] = 0.0

        if (
            state["s_right_release_sec"] >= S_RIGHT_RELEASE_CONFIRM_SEC
            or state["s_right_dropout_sec"] >= S_RIGHT_MAX_DROPOUT_SEC
        ):
            state["s_right_phase_active"] = False
            state["s_right_dropout_sec"] = 0.0
            state["s_right_release_sec"] = 0.0

    right_curve_confirmed = state["s_right_phase_active"]

    # Phase1-61: S자 우굴곡에서만 사용하는 짧은 히스테리시스다.
    # 여유 4 cm 이하에서 켜고 5 cm 이상에서 끄므로, 경계 부근에서
    # 프레임마다 제한이 켜졌다 꺼지며 조향이 떨리는 현상을 막는다.
    if right_curve_confirmed:
        if state["right_curve_guard_active"]:
            if line_margin >= RIGHT_CURVE_GUARD_RELEASE_CM:
                state["right_curve_guard_active"] = False
        elif line_margin <= RIGHT_CURVE_GUARD_START_CM:
            state["right_curve_guard_active"] = True
    else:
        state["right_curve_guard_active"] = False

    if right_curve_confirmed:
        guard_margin = line_margin
        guard_start = RIGHT_CURVE_GUARD_START_CM
        guard_full = RIGHT_CURVE_GUARD_FULL_CM
    else:
        # 우굴곡이 아닌 곳은 검증된 Phase1-41 예측 가드를 유지한다.
        guard_margin = predicted_margin
        guard_start = RIGHT_GUARD_START_CM
        guard_full = RIGHT_GUARD_FULL_CM

    guard_is_active = (
        state["right_curve_guard_active"]
        if right_curve_confirmed
        else guard_margin < guard_start
    )

    if total < 0.0 and guard_is_active:
        allowance = clamp(
            (guard_margin - guard_full) / max(guard_start - guard_full, 1e-6),
            0.0, 1.0,
        )
        guarded = total * allowance

        if right_curve_confirmed:
            # Phase1-61의 유일한 주행 변경이다.
            # 우굴곡 전체를 제한하지 않고 실제 우측 실선 여유가 부족한 동안만
            # 최종 목표를 170 이상으로 제한한다. 여유가 회복되면 위 상태가
            # 해제되어 기존 라인트레이싱 우조향을 다시 그대로 사용한다.
            total = max(
                guarded,
                RIGHT_CURVE_GUARD_MIN_STEER - STEER_STRAIGHT,
            )
        # 일반 주행은 Phase1-60의 검증된 피드포워드 보호를 유지한다.
        elif feedforward < 0.0:
            total = min(guarded, feedforward)
        else:
            total = guarded
    # (기존 total *= allowance 는 삭제)

    # 우조향을 모두 제한했는데도 실제 차체가 이미 실선에 가까우면서
    # 여전히 우측을 향한다면, 단순 직진만으로는 복구가 늦을 수 있다.
    # 이때만 거리에 비례한 최소 좌조향을 만든다.
    # max()를 사용하므로 기존 제어기가 더 큰 좌조향을 내고 있다면
    # 그 값을 약화시키거나 덧붙이지 않는다.
    if (
        line_margin < RIGHT_RECOVERY_START_CM
        and heading_error > 0.0
    ):
        recovery_ratio = clamp(
            (RIGHT_RECOVERY_START_CM - line_margin)
            / max(
                RIGHT_RECOVERY_START_CM - RIGHT_RECOVERY_FULL_CM,
                1e-6,
            ),
            0.0, 1.0,
        )
        minimum_left = RIGHT_RECOVERY_MAX_COUNTS * recovery_ratio
        total = max(total, minimum_left)

    # Phase1-30: 기계적 트림을 더한다. 이것이 참 직진 기준이다.
    raw = STEER_STRAIGHT + total

    steer = int(round(clamp(raw, STEER_RIGHT_LIMIT, STEER_LEFT_LIMIT)))

    # ------------------------------------------------------------------
    # Phase1-39 : 적분 누적 갱신
    # ------------------------------------------------------------------
    # 조향이 한계에 걸린 프레임에서는 쌓지 않는다 (안티와인드업).
    # 경계를 못 찾은 프레임에서는 이 함수 자체가 호출되지 않으므로
    # 그때도 자동으로 멈춘다.
    saturated = (raw <= STEER_RIGHT_LIMIT) or (raw >= STEER_LEFT_LIMIT)

    # Phase1-40 : 직선에서만 쌓는다. 커브에서 쌓으면 커브 편향을
    # 학습해 버리고, 그것을 다음 반대 커브까지 들고 간다.
    on_straight = abs(curvature) < INTEGRAL_CURVATURE_MAX

    if not saturated and on_straight:
        safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))

        state["integral"] = float(clamp(
            state["integral"]
            + KI_COUNTS_PER_CM_SEC
            * (-apply_deadband(lateral_error, CTE_DEADBAND_CM))
            * safe_dt,
            -INTEGRAL_LIMIT_COUNTS, INTEGRAL_LIMIT_COUNTS,
        ))

    return (steer, feedforward, cross_track, heading_term, heading_error,
            integral_term, curve_offset)


def calculate_speed(curvature):
    """곡률이 클수록 속도를 낮춘다."""

    magnitude = abs(curvature)

    if magnitude <= SPEED_CURVATURE_START:
        return SPEED_MAX

    progress = clamp(
        (magnitude - SPEED_CURVATURE_START)
        / max(SPEED_CURVATURE_FULL - SPEED_CURVATURE_START, 1e-9),
        0.0, 1.0,
    )

    return int(round(SPEED_MAX - (SPEED_MAX - SPEED_MIN) * progress))


def calculate_crosswalk_speed_limit():
    """SPEED_MAX에 비례한 Phase1-52 횡단보도 통과 상한을 계산한다."""

    proportional = int(round(SPEED_MAX * CROSSWALK_SPEED_RATIO))
    protected = max(CROSSWALK_SPEED_FLOOR, SPEED_MIN, proportional)

    return int(min(SPEED_MAX, CROSSWALK_SPEED_HARD_MAX, protected))


def update_crosswalk_speed(target_speed, speed_state, delta_time):
    """횡단보도 보호 중의 속도 명령만 부드럽게 목표값으로 이동시킨다."""

    safe_target = float(clamp(target_speed, 0, 255))
    safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))

    if safe_target < speed_state:
        rate = CROSSWALK_SPEED_DECEL_PER_SEC
    else:
        rate = CROSSWALK_SPEED_ACCEL_PER_SEC

    new_state = move_toward(speed_state, safe_target, rate * safe_dt)
    command = int(round(clamp(new_state, 0.0, 255.0)))

    return new_state, command


def update_steering_command(raw_steer, steer_state, previous_command,
                            delta_time):
    """조향 명령의 변화 속도를 제한한다. 좌우 대칭이다."""

    safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))

    new_state = move_toward(
        steer_state, float(raw_steer), STEER_RATE_PER_SEC * safe_dt
    )

    new_command = int(round(new_state))

    if abs(new_command - previous_command) < STEER_COMMAND_DEADBAND:
        new_command = previous_command

    new_command = int(clamp(
        new_command, STEER_RIGHT_LIMIT, STEER_LEFT_LIMIT
    ))

    return new_state, new_command


def filter_s_curve_target(raw_steer, geometry_state, filter_state,
                          previous_curvature, delta_time):
    """우굴곡 전환을 짧게 확인하고, 전환 뒤의 작은 진동을 완화한다."""

    target = float(raw_steer)
    curvature = geometry_state["curvature"]
    line_margin = geometry_state["lateral_cm"] - TARGET_LATERAL_CM

    def finish_target(value):
        """Phase1-61 가드가 켜진 동안의 최종 S자 조향 목표를 보장한다."""

        value = float(value)

        if not geometry_state["right_curve_guard_active"]:
            return value

        # Phase1-63의 거리 비례 가드를 복원한다.
        # E<0에서 무조건 170으로 고정하면 필요한 좌복귀까지 지워져
        # 우측 실선을 물 수 있으므로, 거리 여유에 따라 우조향을 풀어준다.
        if line_margin < 0.0:
            if line_margin <= RIGHT_CURVE_GUARD_FULL_CM:
                return max(value, RIGHT_CURVE_GUARD_MIN_STEER)

            near_ratio = clamp(
                (line_margin - RIGHT_CURVE_GUARD_FULL_CM)
                / max(-RIGHT_CURVE_GUARD_FULL_CM, 1e-6),
                0.0, 1.0,
            )
            safe_guide = (
                STEER_STRAIGHT
                + (RIGHT_CURVE_GUARD_MIN_STEER - STEER_STRAIGHT)
                * near_ratio
            )
            return max(value, safe_guide)

        # 여유가 다시 생기기 시작한 구간(0<=E<해제값)에서는 직진으로
        # 풀지 않고 170의 약한 우조향으로 실선을 계속 따라간다.
        # 동시에 170보다 강한 우조향도 허용하지 않아 우측 이탈을 막는다.
        safe_ratio = clamp(
            line_margin / max(RIGHT_CURVE_GUARD_START_CM, 1e-6),
            0.0, 1.0,
        )
        return (
            RIGHT_CURVE_GUARD_MIN_STEER
            + (RIGHT_CURVE_GUIDE_STEER - RIGHT_CURVE_GUARD_MIN_STEER)
            * safe_ratio
        )

    # 실제 명령이 좌조향에서 우조향으로 넘어가는 S자 전환에만 적용한다.
    # 직진에서 시작하는 일반 우회전은 이 조건에 들어오지 않으므로 기존
    # 일반 주행은 바꾸지 않는다. 186은 과한 좌이탈을 막는 완만한 유지값이다.
    left_to_right_pending = (
        geometry_state["s_right_transition_pending"]
        and filter_state > STEER_STRAIGHT
        and target < STEER_STRAIGHT
    )
    if left_to_right_pending:
        held_target = float(S_RIGHT_TRANSITION_HOLD_STEER)
        return held_target, held_target, previous_curvature

    right_curve_active = geometry_state["s_right_phase_active"]
    # 방향 전환 자체는 아래 direction_changed 조건이 즉시 반영한다.
    # 잠금 상태 중의 짧은 곡률 공백을 새로운 진입으로 오인하지 않는다.
    right_curve_started = False

    if not right_curve_active:
        target = finish_target(target)
        return target, target, curvature

    # 좌굴곡에서 우굴곡으로 바뀐 첫 순간에는 이전 좌조향 값을 섞지 않는다.
    if right_curve_started:
        target = finish_target(target)
        return target, target, curvature

    # 목표가 직진값의 반대편으로 넘어가면 실제 방향 전환 또는 안전 복구다.
    # 이때도 과거 방향을 섞지 않고 새 목표를 즉시 반영한다.
    direction_changed = (
        (target - STEER_STRAIGHT)
        * (filter_state - STEER_STRAIGHT)
        < 0.0
    )

    if direction_changed:
        target = finish_target(target)
        return target, target, curvature

    # 우측 실선에 가까워져 좌조향 복구값이 커지는 경우에는 안전 명령을
    # 필터로 늦추지 않는다.
    safety_recovery = (
        line_margin < RIGHT_RECOVERY_START_CM
        and target > filter_state
    )

    if safety_recovery:
        target = finish_target(target)
        return target, target, curvature

    if abs(target - filter_state) <= S_CURVE_TARGET_DEADBAND_COUNTS:
        target = finish_target(filter_state)
        return target, target, curvature

    filtered = low_pass(
        filter_state,
        target,
        delta_time,
        S_CURVE_TARGET_FILTER_SEC,
    )
    filtered = finish_target(filtered)
    return filtered, filtered, curvature


def apply_s_right_edge_assist(raw_steer, geometry_state, points):
    """S자 우굴곡에서 우측 실선이 BEV 끝에 접근할 때만 약한 우조향을 보탠다."""

    if not geometry_state["s_right_phase_active"] or not points:
        return raw_steer

    rightmost_x = max(float(x) for _, x in points)

    if rightmost_x < S_RIGHT_EDGE_START_X_PX:
        return raw_steer

    edge_ratio = clamp(
        (rightmost_x - S_RIGHT_EDGE_START_X_PX)
        / max(S_RIGHT_EDGE_FULL_X_PX - S_RIGHT_EDGE_START_X_PX, 1e-6),
        0.0,
        1.0,
    )
    guide_steer = (
        S_RIGHT_EDGE_START_STEER
        + (S_RIGHT_EDGE_FULL_STEER - S_RIGHT_EDGE_START_STEER)
        * edge_ratio
    )

    # 숫자가 작을수록 강한 우조향이다. 기존 목표가 부족할 때만 guide까지
    # 보강하되, 기존 목표가 너무 강해도 164 아래로 내려가지 못하게 한다.
    assisted_steer = max(
        S_RIGHT_EDGE_MIN_STEER,
        min(float(raw_steer), guide_steer),
    )
    return int(round(assisted_steer))


# ======================================================================
# 16. 정지선·횡단보도 상태 (Phase1-27 과 동일)
# ======================================================================

def calculate_marking_metrics(binary_image, horizontal_mask):
    """흰색 비율과 긴 가로선 픽셀 수를 계산한다."""

    image_height, image_width = binary_image.shape

    y_start = int(image_height * MARKING_REGION_START_RATIO)
    y_end = int(image_height * MARKING_REGION_END_RATIO)

    if y_end <= y_start:
        y_end = image_height

    x_start = int(image_width * ABSOLUTE_LEFT_RATIO)
    x_end = int(image_width * ABSOLUTE_RIGHT_RATIO)

    white_region = binary_image[y_start:y_end, x_start:x_end]
    horizontal_region = horizontal_mask[y_start:y_end, x_start:x_end]

    if white_region.size == 0:
        white_ratio = 0.0
    else:
        white_ratio = (
            float(cv2.countNonZero(white_region)) / float(white_region.size)
        )

    horizontal_pixels = int(cv2.countNonZero(horizontal_region))
    horizontal_detected = horizontal_pixels >= HORIZONTAL_MIN_PIXELS

    return white_ratio, horizontal_pixels, horizontal_detected


def detect_marking_approach(horizontal_mask):
    """기존 마킹 영역에 닿기 전, 화면 위쪽의 긴 가로선을 먼저 감지한다."""

    image_height, image_width = horizontal_mask.shape
    y_start = int(image_height * MARKING_APPROACH_START_RATIO)
    y_end = int(image_height * MARKING_APPROACH_END_RATIO)
    x_start = int(image_width * ABSOLUTE_LEFT_RATIO)
    x_end = int(image_width * ABSOLUTE_RIGHT_RATIO)

    approach_region = horizontal_mask[y_start:y_end, x_start:x_end]

    if approach_region.size == 0:
        return False

    approach_pixels = int(cv2.countNonZero(approach_region))
    return approach_pixels >= MARKING_APPROACH_MIN_PIXELS


def marking_release_boundary_is_valid(geometry):
    """마킹 통과 후 일반 TRACK으로 복귀해도 되는 우측 실선인지 확인한다."""

    if geometry is None:
        return False

    return (
        geometry["point_count"] >= MIN_BOUNDARY_POINTS
        and PLAUSIBLE_LATERAL_MIN_CM
        <= geometry["lateral_cm"]
        <= PLAUSIBLE_LATERAL_MAX_CM
        and abs(geometry["heading_deg"])
        <= MARKING_RELEASE_MAX_HEADING_DEG
        and abs(geometry["curvature"])
        <= MARKING_RELEASE_MAX_CURVATURE
    )


def limit_marking_steer(raw_steer, geometry_state):
    """횡단보도에서는 우조향을 억제하고 필요한 좌복귀만 단계적으로 허용한다."""

    line_margin = (
        geometry_state["lateral_cm"] - TARGET_LATERAL_CM
    )
    left_assist_ratio = clamp(
        (MARKING_STEER_LEFT_ASSIST_START_CM - line_margin)
        / max(
            MARKING_STEER_LEFT_ASSIST_START_CM
            - MARKING_STEER_LEFT_ASSIST_FULL_CM,
            1e-6,
        ),
        0.0,
        1.0,
    )
    left_limit = (
        MARKING_STEER_LEFT_BASE_COUNTS
        + (
            MARKING_STEER_LEFT_MAX_COUNTS
            - MARKING_STEER_LEFT_BASE_COUNTS
        )
        * left_assist_ratio
    )

    return int(round(clamp(
        raw_steer,
        STEER_STRAIGHT - MARKING_STEER_RIGHT_LIMIT_COUNTS,
        STEER_STRAIGHT + left_limit,
    )))


def count_crosswalk_bars(horizontal_mask):
    """네모박스의 얇은 선을 제외하고 굵은 횡단보도 띠만 센다."""

    image_height, image_width = horizontal_mask.shape

    y_start = int(image_height * MARKING_REGION_START_RATIO)
    y_end = int(image_height * MARKING_REGION_END_RATIO)
    x_start = int(image_width * ABSOLUTE_LEFT_RATIO)
    x_end = int(image_width * ABSOLUTE_RIGHT_RATIO)

    marking_region = horizontal_mask[y_start:y_end, x_start:x_end]

    if marking_region.size == 0:
        return 0

    component_count, _, stats, _ = cv2.connectedComponentsWithStats(
        marking_region,
        connectivity=8,
    )

    valid_bar_count = 0

    # 0번 성분은 검은 배경이다. 폭과 높이가 모두 충분한 띠만 인정한다.
    for component_index in range(1, component_count):
        component_width = int(
            stats[component_index, cv2.CC_STAT_WIDTH]
        )
        component_height = int(
            stats[component_index, cv2.CC_STAT_HEIGHT]
        )
        component_area = int(
            stats[component_index, cv2.CC_STAT_AREA]
        )

        component_aspect_ratio = (
            component_height / max(component_width, 1)
        )
        component_fill_ratio = (
            component_area
            / max(component_width * component_height, 1)
        )

        if (
            component_width >= CROSSWALK_BAR_MIN_WIDTH
            and component_height >= CROSSWALK_BAR_MIN_HEIGHT
            and component_aspect_ratio >= CROSSWALK_BAR_MIN_ASPECT_RATIO
            and component_fill_ratio >= CROSSWALK_BAR_MIN_FILL_RATIO
        ):
            valid_bar_count += 1

    return valid_bar_count


def update_horizontal_state(clear_frames, detect_since,
                            horizontal_detected, now):
    """오래 연속 검출되는 가로 성분은 배경으로 보고 마킹에서 제외한다."""

    if horizontal_detected:
        if detect_since <= 0.0:
            detect_since = now
    else:
        detect_since = 0.0

    background_like = (
        horizontal_detected
        and detect_since > 0.0
        and (now - detect_since) >= HORIZONTAL_MAX_CONTINUOUS_SEC
    )

    effective_detected = horizontal_detected and not background_like

    if effective_detected:
        clear_frames = 0
    else:
        clear_frames += 1

    active = clear_frames < HORIZONTAL_CLEAR_FRAMES

    return active, clear_frames, detect_since, effective_detected


def update_crosswalk_state(active, evidence_frames, clear_frames,
                           active_since, block_until,
                           white_ratio, horizontal_detected, now, bar_count,
                           crosswalk_signature_confirmed):
    """횡단보도 상태를 갱신한다. 활성 최대 지속시간을 둔다."""

    if (crosswalk_signature_confirmed
            and white_ratio >= CROSSWALK_WHITE_RATIO
            and bar_count >= CROSSWALK_MIN_BARS):
        evidence_frames += 1
    else:
        evidence_frames = 0

    if (not active and now >= block_until
            and evidence_frames >= CROSSWALK_CONFIRM_FRAMES):
        active = True
        clear_frames = 0
        active_since = now

    if active:
        still_present = (
            white_ratio >= CROSSWALK_CLEAR_WHITE_RATIO or horizontal_detected
        )

        if still_present:
            clear_frames = 0
        else:
            clear_frames += 1

        timed_out = (
            active_since > 0.0
            and (now - active_since) >= CROSSWALK_MAX_ACTIVE_SEC
        )

        if clear_frames >= CROSSWALK_CLEAR_FRAMES or timed_out:
            active = False
            evidence_frames = 0
            clear_frames = 0
            active_since = 0.0

            if timed_out:
                block_until = now + CROSSWALK_REARM_BLOCK_SEC
                print("[MARK-TIMEOUT] 횡단보도 상태를 강제 해제했습니다. "
                      "카메라에 정지된 흰색 물체가 있는지 확인하세요.")

    return active, evidence_frames, clear_frames, active_since, block_until


def select_mode(crosswalk_active, horizontal_active, boundary_valid,
                lost_frames):
    """현재 추적 상태를 문자열로 만든다."""

    if crosswalk_active:
        return "CROSS-TRACK" if boundary_valid else "CROSS-PRED"

    if horizontal_active and boundary_valid:
        return "HLINE-MASK"

    if boundary_valid:
        return "TRACK"

    if lost_frames <= LOST_COMMAND_HOLD_FRAMES:
        return "HOLD"

    return "LOST"


# ======================================================================
# 17. 디버그 화면
# ======================================================================

def draw_bev_view(bev_image, points, geometry, calibration):
    """BEV 화면에 검출점과 목표선을 그린다."""

    view = bev_image.copy()
    pixels_per_cm = calibration["pixels_per_cm"]

    target_x = int(round(
        calibration["vehicle_x"] + TARGET_LATERAL_CM * pixels_per_cm
    ))

    cv2.line(view, (target_x, 0), (target_x, view.shape[0]), (0, 255, 0), 2)
    cv2.putText(view, "TARGET", (target_x + 6, 46),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

    vehicle_x = int(round(calibration["vehicle_x"]))
    cv2.line(view, (vehicle_x, 0), (vehicle_x, view.shape[0]),
             (255, 0, 255), 2)
    cv2.putText(view, "CAR", (vehicle_x + 6, 68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

    for y, x in points:
        cv2.circle(view, (int(round(x)), int(y)), 3, (0, 255, 255), -1)

    if geometry is not None:
        curve = []

        for y in range(BEV_SCAN_BOTTOM_PX, BEV_SCAN_TOP_PX, -6):
            x = evaluate_boundary_x(geometry["poly"], y, calibration)

            if 0 <= x < view.shape[1]:
                curve.append((int(round(x)), int(y)))

        if len(curve) >= 2:
            cv2.polylines(view, [np.asarray(curve, np.int32)], False,
                          (0, 140, 255), 2)

    cv2.putText(view, "BEV", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 0), 2)

    return view


def draw_main_view(
        roi, mode, geometry_state,
        raw_steer, steer_command, drive_speed,
        white_ratio, horizontal_pixels,
        lost_frames, point_count,
        heading_error, integral_term):
    """전반적인 주행 판단에 필요한 핵심 값만 화면에 표시한다."""

    lateral_error = geometry_state["lateral_cm"] - TARGET_LATERAL_CM

    cv2.putText(
        roi,
        "MODE:%s P:%d SP:%d WR:%.3f HP:%d LOST:%d" % (
            mode,
            point_count,
            int(geometry_state["span_cm"]),
            white_ratio,
            horizontal_pixels,
            lost_frames,
        ),
        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 2,
    )

    cv2.putText(
        roi,
        "E:%+.1f PE:%+.1f K:%+.5f IN:%+.1f" % (
            lateral_error,
            heading_error,
            geometry_state["curvature"],
            integral_term,
        ),
        (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 255), 2,
    )

    cv2.putText(
        roi,
        "TGT:%d STR:%d SPD:%d" % (
            raw_steer,
            steer_command,
            drive_speed,
        ),
        (8, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2,
    )


def build_combined_view(tracking_image, bev_image):
    """주행 화면과 BEV 를 하나의 창으로 합친다."""

    tracking_view = tracking_image.copy()
    bev_view = bev_image.copy()

    height = max(tracking_view.shape[0], bev_view.shape[0])

    canvas = np.full(
        (height, tracking_view.shape[1] + bev_view.shape[1] + 4, 3),
        60, dtype=np.uint8,
    )

    canvas[:tracking_view.shape[0], :tracking_view.shape[1]] = tracking_view
    canvas[:bev_view.shape[0], tracking_view.shape[1] + 4:] = bev_view

    return canvas


# ======================================================================
# 18. 출발 대기
# ======================================================================

def wait_for_start(camera_environment, camera_channel, calibration):
    """
    's' 키를 기다린다.

    Phase1-27 까지 있던 원근 오프셋 보정은 사라졌다.
    BEV 는 캘리브레이션된 호모그래피가 기하를 정의하므로
    출발 위치에 따라 기준을 다시 잡을 필요가 없다.
    """

    print("")
    print("STAND BY: 차량을 차로와 평행하게 정렬하세요.")
    print("BEV 창에서 주황색 곡선이 우측 실선을 따라가면 's' 키를 누르세요.")
    print("초록 세로선이 목표 위치, 자홍 세로선이 차량 중심선입니다.")
    print("'q' 키를 누르면 종료합니다.")
    print("")

    while True:
        _, frame = camera_environment.camera_read(camera_channel)

        if frame is None:
            if cv2.waitKey(1) & 0xFF == STOP_KEY:
                return False
            continue

        roi = frame[ROI_Y_START:ROI_Y_END, :].copy()

        if roi.size == 0:
            raise RuntimeError("ROI를 만들 수 없습니다.")

        binary = preprocess_roi(roi)
        lane_binary, _ = remove_horizontal_markings(binary)

        bev_binary = warp_to_bev(lane_binary, calibration)
        bev_colour = warp_to_bev(roi, calibration)

        points = detect_bev_boundary(bev_binary, None, calibration)
        geometry = fit_boundary(points, calibration)

        bev_view = draw_bev_view(bev_colour, points, geometry, calibration)

        if geometry is None:
            message = "STAND BY  PTS:%d  (검출 부족)" % len(points)
        else:
            message = "STAND BY  PTS:%d  LAT:%+.1fcm  PSI:%+.1fdeg" % (
                len(points), geometry["lateral_cm"], geometry["heading_deg"]
            )

        cv2.putText(roi, message, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    (0, 0, 255), 2)

        cv2.imshow("Phase1-66 Standby", build_combined_view(roi, bev_view))

        key = cv2.waitKey(1) & 0xFF

        if key == START_KEY:
            cv2.destroyWindow("Phase1-66 Standby")
            return True

        if key == STOP_KEY:
            return False


# ======================================================================
# 19. 메인 주행 루프
# ======================================================================

def main():
    """BEV 기하와 곡률 피드포워드로 주행한다."""

    print("Phase1-66: Phase1-65 주행 + 횡단보도 조향 래치")
    print("OpenCV Version: %s" % cv2.__version__)

    calibration = load_calibration()

    print("BEV 캘리브레이션 출처: %s" % calibration["origin"])
    print("  스케일 %.2f px/cm, 차량 기준점 BEV (%.1f, %.1f)"
          % (calibration["pixels_per_cm"],
             calibration["vehicle_x"], calibration["vehicle_y"]))
    print("  목표 횡거리 %.1f cm" % TARGET_LATERAL_CM)
    print("  FF %.0f counts/(1/cm), KE %.2f counts/cm, KPSI %.2f counts/deg"
          % (FF_COUNTS_PER_CURVATURE, KE_COUNTS_PER_CM,
             KPSI_COUNTS_PER_DEG))
    print("  조향 트림 %+.1f counts (참 직진 서보 %.0f)"
          % (STEER_TRIM_COUNTS, STEER_STRAIGHT))
    print("  헤딩 기하보정 C = %.1f cm, 곡률 상한 %.4f"
          % (HEADING_KINEMATIC_CM, CURVATURE_LIMIT_ABS))
    print("  예측 우조향 가드 %.0f~%.0f cm, 좌복구 %.0f~%.0f cm (최대 %.1f counts)"
          % (RIGHT_GUARD_START_CM, RIGHT_GUARD_FULL_CM,
             RIGHT_RECOVERY_START_CM, RIGHT_RECOVERY_FULL_CM,
             RIGHT_RECOVERY_MAX_COUNTS))
    print("  횡단보도 속도 상한 %d, 녹색 보조 상한 %d"
          % (calculate_crosswalk_speed_limit(),
             GREEN_FALLBACK_SPEED_LIMIT))
    print("  우굴곡 가드 K>=%.4f, 실제 여유 %.0f~%.0f cm, 속도 상한 %d"
          % (RIGHT_CURVE_GUARD_CURVATURE,
             RIGHT_CURVE_GUARD_START_CM,
             RIGHT_CURVE_GUARD_FULL_CM,
             RIGHT_CURVE_SPEED_LIMIT))

    serial_environment = fl.libARDUINO()
    camera_environment = fl.libCAMERA()

    comm = None
    camera_channel = None

    try:
        comm = serial_environment.init(ARDUINO_PORT, ARDUINO_BAUDRATE)

        camera_channel, _ = camera_environment.initial_setting(
            cam0port=CAMERA_PORT, capnum=1
        )

        if camera_channel is None or not camera_channel.isOpened():
            raise RuntimeError("카메라 포트 %d를 열 수 없습니다." % CAMERA_PORT)

        send_stop(comm)

        if not wait_for_start(camera_environment, camera_channel,
                              calibration):
            return

        cv2.namedWindow("Phase1-66 Driving View", cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow("Phase1-66 Driving View", 0, 0)

        steer_state = float(STEER_STRAIGHT)
        steer_command = STEER_STRAIGHT
        raw_steer = STEER_STRAIGHT

        geometry_state = create_geometry_state()
        green_state = create_green_boundary_state()
        previous_poly = None

        feedforward = 0.0
        cross_track = 0.0
        heading_term = 0.0
        integral_term = 0.0
        curve_offset = 0.0
        heading_error = 0.0
        drive_speed = SPEED_MAX
        speed_state = float(SPEED_MAX)
        sent_speed = SPEED_MAX
        point_count = 0
        green_point_count = 0

        crosswalk_active = False
        crosswalk_evidence_frames = 0
        crosswalk_clear_frames = 0
        crosswalk_active_since = 0.0
        crosswalk_block_until = 0.0

        horizontal_clear_frames = HORIZONTAL_CLEAR_FRAMES
        horizontal_detect_since = 0.0

        # 52의 보호 범위가 S자까지 이어지지 않도록 두 타이머를
        # 실제 CROSS 상태에서만 갱신한다.
        marking_speed_until = 0.0
        crosswalk_tracking_until = 0.0

        # 횡단보도에서는 한 번 선택한 녹색 외곽 경계를 유지한다.
        crosswalk_was_active = False
        crosswalk_boundary_latched = False
        crosswalk_last_geometry = None
        crosswalk_last_geometry_time = 0.0
        crosswalk_prediction_used = False
        marking_steer_active = False
        marking_steer_active_since = 0.0
        marking_steer_release_frames = 0
        marking_approach_armed_until = 0.0
        marking_crosswalk_evidence_frames = 0
        marking_presteer_active = False
        marking_presteer_active_since = 0.0
        marking_presteer_reference = float(STEER_STRAIGHT)
        green_control_geometry = None
        last_valid_geometry = None
        crosswalk_lost_since = 0.0
        s_right_lost_since = 0.0
        s_curve_target_state = float(STEER_STRAIGHT)
        s_curve_previous_curvature = 0.0

        white_ratio = 0.0
        horizontal_pixels = 0

        lost_frames = 0
        camera_failures = 0

        last_control_time = time.monotonic()
        last_debug_time = 0.0

        while True:
            _, frame = camera_environment.camera_read(camera_channel)

            if frame is None:
                camera_failures += 1

                if camera_failures >= MAX_CAMERA_FAILURES:
                    send_stop(comm)

                if cv2.waitKey(1) & 0xFF == STOP_KEY:
                    break

                continue

            camera_failures = 0

            now = time.monotonic()
            delta_time = float(clamp(
                now - last_control_time, MIN_CONTROL_DT, MAX_CONTROL_DT
            ))
            last_control_time = now

            roi = frame[ROI_Y_START:ROI_Y_END, :].copy()

            if roi.size == 0:
                raise RuntimeError("주행 ROI를 만들 수 없습니다.")

            binary = preprocess_roi(roi)
            lane_binary, horizontal_mask = remove_horizontal_markings(binary)

            bar_count = count_crosswalk_bars(horizontal_mask)

            (white_ratio,
             horizontal_pixels,
             horizontal_detected) = calculate_marking_metrics(
                binary, horizontal_mask
            )
            approach_horizontal_detected = detect_marking_approach(
                horizontal_mask
            )

            # 한 개의 굵은 정지선이 들어오면 직전 안정 조향을 짧게 기준으로
            # 사용한다. 얇은 네모박스 선은 bar_count 조건을 통과하지 못한다.
            # A thin horizontal line only arms the detector. MARK-GREEN is
            # allowed only after separated, thick crosswalk bars are seen.
            if approach_horizontal_detected and not marking_steer_active:
                marking_approach_armed_until = (
                    now + MARKING_APPROACH_ARM_SEC
                )

            marking_approach_armed = (
                now <= marking_approach_armed_until
            )
            marking_crosswalk_signature = (
                marking_approach_armed
                and bar_count >= CROSSWALK_MIN_BARS
                and white_ratio >= MARKING_CROSSWALK_MIN_WHITE_RATIO
            )

            if marking_crosswalk_signature:
                marking_crosswalk_evidence_frames += 1
            else:
                marking_crosswalk_evidence_frames = 0

            marking_crosswalk_confirmed = (
                marking_crosswalk_evidence_frames
                >= MARKING_CROSSWALK_CONFIRM_FRAMES
            )

            # Pre-steer is allowed only for confirmed, tall and filled
            # crosswalk bars. Stop lines and hollow boxes remain mask-only.
            if (
                not marking_presteer_active
                and not marking_steer_active
                and marking_crosswalk_confirmed
            ):
                marking_presteer_active = True
                marking_presteer_active_since = now
                marking_presteer_reference = float(clamp(
                    steer_command,
                    STEER_STRAIGHT - MARKING_PRESTEER_RIGHT_LIMIT_COUNTS,
                    STEER_STRAIGHT + MARKING_PRESTEER_LEFT_LIMIT_COUNTS,
                ))

            if (
                marking_presteer_active
                and (
                    marking_steer_active
                    or now - marking_presteer_active_since
                    >= MARKING_PRESTEER_MAX_SEC
                )
            ):
                marking_presteer_active = False

            (horizontal_active,
             horizontal_clear_frames,
             horizontal_detect_since,
             effective_horizontal) = update_horizontal_state(
                horizontal_clear_frames, horizontal_detect_since,
                horizontal_detected, now,
            )

            (crosswalk_active,
             crosswalk_evidence_frames,
             crosswalk_clear_frames,
             crosswalk_active_since,
             crosswalk_block_until) = update_crosswalk_state(
                crosswalk_active, crosswalk_evidence_frames,
                crosswalk_clear_frames, crosswalk_active_since,
                crosswalk_block_until, white_ratio, effective_horizontal,
                 now, bar_count, marking_crosswalk_confirmed
             )

            if crosswalk_active:
                marking_speed_until = (
                    now + MARKING_SPEED_RELEASE_HOLD_SEC
                )
                crosswalk_tracking_until = (
                    now + CROSSWALK_TRACK_RELEASE_HOLD_SEC
                )

            speed_protection_active = now <= marking_speed_until
            tracking_protection_active = now <= crosswalk_tracking_until
            crosswalk_started = (
                crosswalk_active and not crosswalk_was_active
            )

            marking_visible = (
                approach_horizontal_detected
                or effective_horizontal
                or crosswalk_active
                or white_ratio >= CROSSWALK_CLEAR_WHITE_RATIO
            )

            # Phase1-66: CROSS 확정을 기다리지 않고 화면 위쪽의 첫 가로선부터
            # 조향 보호를 시작한다. 속도 보호 상태는 기존 로직 그대로 둔다.
            if (
                not marking_steer_active
                and (
                    marking_crosswalk_confirmed
                    or crosswalk_started
                )
                and crosswalk_entry_is_qualified(geometry_state)
            ):
                marking_steer_active = True
                marking_steer_active_since = now
                marking_steer_release_frames = 0
                marking_approach_armed_until = 0.0
                marking_crosswalk_evidence_frames = 0
                marking_presteer_active = False
                crosswalk_boundary_latched = True
                crosswalk_last_geometry = copy_geometry(
                    last_valid_geometry
                )
                crosswalk_last_geometry_time = now
                green_control_geometry = copy_geometry(
                    last_valid_geometry
                )

            if marking_steer_active and crosswalk_started:
                crosswalk_last_geometry = copy_geometry(
                    crosswalk_last_geometry
                    if crosswalk_last_geometry is not None
                    else last_valid_geometry
                )
                crosswalk_last_geometry_time = now

                if green_control_geometry is None:
                    green_control_geometry = copy_geometry(
                        last_valid_geometry
                    )

            green_tracking_active = (
                marking_steer_active
                and crosswalk_boundary_latched
            )

            if not green_tracking_active:
                crosswalk_boundary_latched = False
                crosswalk_last_geometry = None
                crosswalk_last_geometry_time = 0.0
                green_control_geometry = None

            crosswalk_was_active = crosswalk_active

            # 가로선 검출은 마스크 제거와 표시에만 사용한다.
            if not tracking_protection_active:
                crosswalk_lost_since = 0.0

            # ----------------------------------------------------------
            # BEV 기하 추정
            # ----------------------------------------------------------
            bev_binary = warp_to_bev(lane_binary, calibration)
            bev_colour = warp_to_bev(roi, calibration)

            points = detect_bev_boundary(bev_binary, previous_poly,
                                         calibration)
            candidate_geometry = fit_boundary(points, calibration)

            green_points = detect_green_boundary(bev_colour)
            raw_green_geometry = fit_boundary(green_points, calibration)
            green_point_count = len(green_points)

            # 횡단보도 밖의 정상 흰 실선에서만 녹색 오프셋을 학습한다.
            update_green_boundary_state(
                green_state,
                (
                    candidate_geometry
                    if candidate_geometry is not None
                    and not speed_protection_active
                    and not green_tracking_active
                    else None
                ),
                raw_green_geometry,
                delta_time,
            )

            corrected_green_geometry = correct_green_geometry(
                raw_green_geometry, green_state["offset_cm"]
            )
            green_boundary_safe = (
                green_tracking_active
                and green_state["ready"]
                and green_geometry_is_safe(
                    corrected_green_geometry, geometry_state
                )
            )

            geometry = None
            boundary_valid = False
            green_fallback_used = False
            crosswalk_prediction_used = False
            point_count = len(points)

            # 마킹이 사라진 뒤 정상 우측 실선을 연속 확인해야만 조향 보호를
            # 해제한다. 단, 잘못된 검출로 상태가 고착되지 않도록 최대 시간을 둔다.
            if marking_steer_active:
                marking_steer_elapsed = max(
                    0.0, now - marking_steer_active_since
                )

                if marking_steer_elapsed >= MARKING_STEER_MAX_SEC:
                    marking_steer_active = False
                    marking_steer_release_frames = 0

                elif (
                    marking_steer_elapsed < MARKING_STEER_MIN_SEC
                    or marking_visible
                ):
                    marking_steer_release_frames = 0

                elif marking_release_boundary_is_valid(
                    candidate_geometry
                ):
                    marking_steer_release_frames += 1

                    if (
                        marking_steer_release_frames
                        >= MARKING_STEER_RELEASE_FRAMES
                    ):
                        marking_steer_active = False
                        marking_steer_release_frames = 0

                else:
                    marking_steer_release_frames = 0

            if not green_tracking_active:
                # 가로 성분은 이미 제거되었으며 HLINE은 조향을 고정하지 않는다.
                geometry = candidate_geometry
                boundary_valid = geometry is not None

            elif green_boundary_safe:
                # 녹색 소스는 고정하되 매 프레임 기하값의 변화 속도만
                # 물리적으로 가능한 범위로 제한한다.
                geometry = stabilize_green_geometry(
                    corrected_green_geometry,
                    green_control_geometry,
                    delta_time,
                )
                green_control_geometry = copy_geometry(geometry)
                boundary_valid = True
                green_fallback_used = True
                crosswalk_last_geometry = copy_geometry(geometry)
                crosswalk_last_geometry_time = now

            elif (
                crosswalk_last_geometry is not None
                and (now - crosswalk_last_geometry_time)
                <= CROSSWALK_BOUNDARY_PREDICT_SEC
            ):
                # 조향 명령 대신 마지막 정상 경계를 짧게 유지해 매 프레임
                # 조향을 다시 계산한다.
                geometry = copy_geometry(crosswalk_last_geometry)
                boundary_valid = True
                crosswalk_prediction_used = True

            elif candidate_geometry is not None:
                # 녹색을 못 믿고 예측도 만료됐지만 흰 실선 피팅이
                # 정상이면 그것을 쓴다. 멀쩡한 측정을 버리지 않는다.
                geometry = candidate_geometry
                boundary_valid = True

            # 우측 실선이 차량 중심에서 차폭 절반(24cm)보다 가깝게 보이면
            # 이미 선을 넘었거나 엉뚱한 선을 잡은 것이다.
            if (boundary_valid
                    and geometry["lateral_cm"] < BOUNDARY_MIN_LATERAL_CM):
                boundary_valid = False

            if boundary_valid:
                lost_frames = 0
                crosswalk_lost_since = 0.0
                s_right_lost_since = 0.0
                previous_poly = geometry["poly"]
                last_valid_geometry = copy_geometry(geometry)

                update_geometry_state(geometry_state, geometry, delta_time)

                (raw_steer,
                 feedforward,
                 cross_track,
                 heading_term,
                 heading_error,
                 integral_term,
                 curve_offset) = calculate_steering(geometry_state,
                                                    delta_time)

                (raw_steer,
                 s_curve_target_state,
                 s_curve_previous_curvature) = filter_s_curve_target(
                    raw_steer,
                    geometry_state,
                    s_curve_target_state,
                    s_curve_previous_curvature,
                    delta_time,
                )

                raw_steer = apply_s_right_edge_assist(
                    raw_steer,
                    geometry_state,
                    points,
                )

                # 굵은 정지선부터 실제 횡단보도 확정 전까지는 직전의 안정
                # 조향 주변만 허용하여 199→161 같은 급격한 반전을 막는다.
                if marking_presteer_active:
                    presteer_minimum = max(
                        STEER_STRAIGHT
                        - MARKING_PRESTEER_RIGHT_LIMIT_COUNTS,
                        marking_presteer_reference
                        - MARKING_PRESTEER_REFERENCE_BAND_COUNTS,
                    )
                    presteer_maximum = min(
                        STEER_STRAIGHT
                        + MARKING_PRESTEER_LEFT_LIMIT_COUNTS,
                        marking_presteer_reference
                        + MARKING_PRESTEER_REFERENCE_BAND_COUNTS,
                    )
                    raw_steer = int(round(clamp(
                        raw_steer,
                        presteer_minimum,
                        presteer_maximum,
                    )))

                # MARK-GREEN에서는 우조향은 좁게 막고, E가 우측 이탈 방향으로
                # 커질 때만 좌조향 허용량을 직진+4에서 최대 직진+14로 늘린다.
                if marking_steer_active or green_tracking_active:
                    raw_steer = limit_marking_steer(
                        raw_steer,
                        geometry_state,
                    )

                drive_speed = calculate_speed(geometry_state["curvature"])

                # 횡단보도와 관계없이 BEV가 확인한 우굴곡에서만 적용한다.
                if (
                    geometry_state["s_right_phase_active"]
                    or geometry_state["curvature"]
                    >= RIGHT_CURVE_SPEED_CURVATURE
                ):
                    drive_speed = min(
                        drive_speed,
                        RIGHT_CURVE_SPEED_LIMIT,
                    )

                if green_fallback_used and tracking_protection_active:
                    drive_speed = min(
                        drive_speed, GREEN_FALLBACK_SPEED_LIMIT
                    )

                if speed_protection_active:
                    drive_speed = min(
                        drive_speed, calculate_crosswalk_speed_limit()
                    )
                    speed_state, sent_speed = update_crosswalk_speed(
                        drive_speed, speed_state, delta_time
                    )
                else:
                    # 보호가 끝나면 Phase1-41의 속도 명령으로 즉시 복귀한다.
                    speed_state = float(drive_speed)
                    sent_speed = int(drive_speed)

                # 검출이 나빠 green fallback 을 쓰는 동안에는 기하값이
                # 오염되므로 조향을 직진 근처로 묶는다.
                if green_fallback_used:
                    raw_steer = int(round(clamp(
                        raw_steer,
                        STEER_STRAIGHT - CROSSWALK_LOST_STEER_LIMIT_COUNTS,
                        STEER_STRAIGHT + CROSSWALK_LOST_STEER_LIMIT_COUNTS,
                    )))

                steer_state, steer_command = update_steering_command(
                    raw_steer, steer_state, steer_command, delta_time
                )

                send_command(comm, DIRECTION_FORWARD,
                             sent_speed, steer_command)

            else:
                lost_frames += 1

                if geometry_state["s_right_phase_active"]:
                    if s_right_lost_since <= 0.0:
                        s_right_lost_since = now
                    s_right_lost_elapsed = max(
                        0.0, now - s_right_lost_since
                    )
                else:
                    s_right_lost_since = 0.0
                    s_right_lost_elapsed = S_RIGHT_LOST_TOTAL_SEC + 1.0

                if (
                    geometry_state["s_right_phase_active"]
                    and s_right_lost_elapsed <= S_RIGHT_LOST_STRONG_SEC
                ):
                    # 첫 0.25초: 선을 놓치기 직전의 우조향을 안전 범위에서 유지한다.
                    safe_s_steer = int(round(clamp(
                        steer_command,
                        S_RIGHT_LOST_STRONG_MIN_STEER,
                        S_RIGHT_LOST_STRONG_MAX_STEER,
                    )))
                    steer_state, steer_command = update_steering_command(
                        safe_s_steer, steer_state,
                        steer_command, delta_time
                    )
                    sent_speed = min(
                        int(round(speed_state)),
                        RIGHT_CURVE_SPEED_LIMIT,
                    )
                    speed_state = float(sent_speed)
                    send_command(
                        comm, DIRECTION_FORWARD,
                        sent_speed, steer_command
                    )

                elif (
                    geometry_state["s_right_phase_active"]
                    and s_right_lost_elapsed <= S_RIGHT_LOST_TOTAL_SEC
                ):
                    # 다음 0.55초: 170의 약한 우조향으로 재검출을 기다린다.
                    steer_state, steer_command = update_steering_command(
                        S_RIGHT_LOST_WEAK_STEER,
                        steer_state,
                        steer_command,
                        delta_time,
                    )
                    sent_speed = min(
                        int(round(speed_state)),
                        RIGHT_CURVE_SPEED_LIMIT,
                    )
                    speed_state = float(sent_speed)
                    send_command(
                        comm, DIRECTION_FORWARD,
                        sent_speed, steer_command
                    )

                elif tracking_protection_active:
                    # 횡단보도에서는 잘못된 강한 조향을 유지하지 않는다.
                    if crosswalk_lost_since <= 0.0:
                        crosswalk_lost_since = now

                    crosswalk_lost_elapsed = max(
                        0.0, now - crosswalk_lost_since
                    )

                    if (
                        crosswalk_lost_elapsed
                        <= CROSSWALK_LOST_TRANSITION_SEC
                    ):
                        safe_steer = int(round(clamp(
                            steer_command,
                            STEER_STRAIGHT
                            - CROSSWALK_LOST_STEER_LIMIT_COUNTS,
                            STEER_STRAIGHT
                            + CROSSWALK_LOST_STEER_LIMIT_COUNTS,
                        )))
                    else:
                        safe_steer = STEER_STRAIGHT

                    drive_speed = min(
                        int(round(speed_state)),
                        CROSSWALK_LOST_SPEED,
                    )
                    speed_state, sent_speed = update_crosswalk_speed(
                        drive_speed, speed_state, delta_time
                    )
                    steer_state, steer_command = update_steering_command(
                        safe_steer, steer_state,
                        steer_command, delta_time
                    )

                    send_command(comm, DIRECTION_FORWARD,
                                 sent_speed, steer_command)

                elif marking_steer_active or green_tracking_active:
                    # CROSS가 확정되지 않았더라도 마킹 통과 중에는 일반 LOST로
                    # 풀지 않는다. 속도에는 새 상한을 걸지 않고 조향만 보호한다.
                    safe_marking_steer = limit_marking_steer(
                        steer_command,
                        geometry_state,
                    )
                    steer_state, steer_command = update_steering_command(
                        safe_marking_steer,
                        steer_state,
                        steer_command,
                        delta_time,
                    )
                    sent_speed = int(round(speed_state))
                    send_command(
                        comm, DIRECTION_FORWARD,
                        sent_speed, steer_command
                    )

                elif lost_frames <= LOST_COMMAND_HOLD_FRAMES:
                    # 짧은 상실은 직전 명령을 유지한다.
                    send_command(comm, DIRECTION_FORWARD, drive_speed, steer_command)

                else:
                    # 오래 못 찾으면 감속하고 조향을 중앙으로 되돌린다.
                    previous_poly = None
                    geometry_state["ready"] = False

                    steer_state, steer_command = update_steering_command(
                        STEER_STRAIGHT, steer_state,
                        steer_command, delta_time
                    )

                    send_command(comm, DIRECTION_FORWARD, LOST_DRIVE_SPEED, steer_command)

            if crosswalk_prediction_used:
                mode = (
                    "CROSS-PRED"
                    if tracking_protection_active
                    else "MARK-PRED"
                )
            elif green_fallback_used:
                mode = (
                    "CROSS-GREEN"
                    if tracking_protection_active
                    else "MARK-GREEN"
                )
            elif tracking_protection_active and not boundary_valid:
                mode = "CROSS-LOST"
            elif (
                (marking_steer_active or green_tracking_active)
                and not boundary_valid
            ):
                mode = "MARK-HOLD"
            elif (
                marking_presteer_active
                or (marking_steer_active and not crosswalk_active)
            ):
                mode = "CROSS-PRESTEER"
            else:
                mode = select_mode(
                    crosswalk_active, horizontal_active,
                    boundary_valid, lost_frames,
                )

            # ----------------------------------------------------------
            # 화면과 로그
            # ----------------------------------------------------------
            draw_main_view(
                roi,
                mode,
                geometry_state,
                raw_steer,
                steer_command,
                sent_speed,
                white_ratio,
                horizontal_pixels,
                lost_frames,
                point_count,
                heading_error,
                integral_term,
            )

            bev_view = draw_bev_view(bev_colour, points, geometry,
                                     calibration)

            cv2.imshow("Phase1-66 Driving View",
                       build_combined_view(roi, bev_view))

            if now - last_debug_time >= DEBUG_PRINT_INTERVAL_SEC:
                last_debug_time = now

                lateral_error = (
                    geometry_state["lateral_cm"] - TARGET_LATERAL_CM
                )

                print(
                    "[%-11s] P:%2d SP:%3d WR:%.3f HP:%4d "
                    "E:%+5.1f PE:%+5.1f K:%+.5f IN:%+5.1f "
                    "TGT:%3d STR:%3d SPD:%3d LOST:%2d"
                    % (
                        mode,
                        point_count,
                        int(geometry_state["span_cm"]),
                        white_ratio,
                        horizontal_pixels,
                        lateral_error,
                        heading_error,
                        geometry_state["curvature"],
                        integral_term,
                        raw_steer,
                        steer_command,
                        sent_speed,
                        lost_frames,
                    )
                )

            if cv2.waitKey(1) & 0xFF == STOP_KEY:
                print("종료 키 입력: 차량을 정지합니다.")
                break

    finally:
        if comm is not None:
            try:
                send_stop(comm)
                time.sleep(0.1)
                comm.close()
            except Exception:
                pass

        if camera_channel is not None:
            try:
                camera_channel.release()
            except Exception:
                pass

        cv2.destroyAllWindows()

        print("Phase1-66 프로그램을 안전하게 종료했습니다.")


if __name__ == "__main__":
    main()
