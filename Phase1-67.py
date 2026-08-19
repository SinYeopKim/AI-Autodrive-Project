# ======================================================================
# Phase1-67 전체 로직 요약
# ======================================================================
# 1단계: 라이브러리와 주행·카메라·BEV·조향·속도 설정값을 준비한다.
# 2단계: 아두이노 통신 명령과 기본 수학 필터 함수를 준비한다.
# 3단계: 카메라 ROI를 이진화하고 정지선·횡단보도 같은 가로 성분을 제거한다.
# 4단계: ROI를 BEV로 변환하고 여러 높이에서 우측 외곽 실선 후보를 찾는다.
# 5단계: 후보점으로 우측 경계 곡선을 적합하고 위치·헤딩·곡률을 계산한다.
# 6단계: 횡단보도에서는 녹색 외곽 경계를 보조로 추적하고 기하값을 안정화한다.
# 7단계: 오차·헤딩·곡률의 피드백과 예측 가드로 목표 조향값을 계산한다.
# 8단계: S자 좌굴곡과 우굴곡 전환을 확인하고 필요한 조향 보조를 제한적으로 적용한다.
# 9단계: 곡률과 주행 상태에 따라 속도를 정하고 명령 변화량을 부드럽게 제한한다.
# 10단계: 차선을 잠시 놓치면 이전 조향을 유지하고, 오래 놓치면 감속·정지한다.
# 11단계: 디버그 화면과 핵심 로그를 표시하고 키 입력으로 출발·종료한다.
# 12단계: 종료 또는 오류 발생 시 차량·카메라·화면을 안전하게 정리한다.
# ======================================================================

import json  # 필요한 외부 모듈을 불러온다.
import math  # 필요한 외부 모듈을 불러온다.
import os  # 필요한 외부 모듈을 불러온다.
import time  # 필요한 외부 모듈을 불러온다.

import cv2  # 필요한 외부 모듈을 불러온다.
import numpy as np  # 필요한 외부 모듈을 불러온다.

import Function_Library as fl  # 필요한 외부 모듈을 불러온다.


ARDUINO_PORT = "COM3"  # ARDUINO_PORT 설정값을 지정한다.
ARDUINO_BAUDRATE = 9600  # ARDUINO_BAUDRATE 설정값을 지정한다.


DIRECTION_STOP = 0  # DIRECTION_STOP 설정값을 지정한다.
DIRECTION_FORWARD = 1  # DIRECTION_FORWARD 설정값을 지정한다.
DIRECTION_REVERSE = 2  # DIRECTION_REVERSE 설정값을 지정한다.

CAMERA_PORT = 0  # CAMERA_PORT 설정값을 지정한다.

START_KEY = ord("s")  # START_KEY 설정값을 지정한다.
STOP_KEY = ord("q")  # STOP_KEY 설정값을 지정한다.


ROI_Y_START = 160  # ROI_Y_START 설정값을 지정한다.
ROI_Y_END = 480  # ROI_Y_END 설정값을 지정한다.

THRESHOLD_VALUE = 190  # THRESHOLD_VALUE 설정값을 지정한다.
GAUSSIAN_KERNEL = (5, 5)  # GAUSSIAN_KERNEL 설정값을 지정한다.


BEV_CALIB_FILE = "bev_calib.json"  # BEV_CALIB_FILE 설정값을 지정한다.


BEV_SOURCE_POINTS = (  # BEV_SOURCE_POINTS 설정값을 지정한다.
    (132.0, 260.0),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    (549.0, 276.0),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    (464.0, 111.0),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    (226.5, 101.0),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
)  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

BEV_MARKER_WIDTH_CM = 89.0  # BEV_MARKER_WIDTH_CM 설정값을 지정한다.
BEV_MARKER_LENGTH_CM = 120.0  # BEV_MARKER_LENGTH_CM 설정값을 지정한다.

BEV_PIXELS_PER_CM = 3.0  # BEV_PIXELS_PER_CM 설정값을 지정한다.
BEV_WIDTH = 640  # BEV_WIDTH 설정값을 지정한다.
BEV_HEIGHT = 520  # BEV_HEIGHT 설정값을 지정한다.
BEV_NEAR_MARGIN_PX = 40  # BEV_NEAR_MARGIN_PX 설정값을 지정한다.


BEV_RIGHT_EXTRA_PX = 80  # BEV_RIGHT_EXTRA_PX 설정값을 지정한다.


BEV_SCAN_BOTTOM_PX = 505  # BEV_SCAN_BOTTOM_PX 설정값을 지정한다.
BEV_SCAN_TOP_PX = 90  # BEV_SCAN_TOP_PX 설정값을 지정한다.
BEV_SCAN_STEP_PX = 10  # BEV_SCAN_STEP_PX 설정값을 지정한다.
BEV_SCAN_HALF_HEIGHT_PX = 3  # BEV_SCAN_HALF_HEIGHT_PX 설정값을 지정한다.


MIN_LINE_WIDTH_PX = 4  # MIN_LINE_WIDTH_PX 설정값을 지정한다.
MAX_LINE_WIDTH_PX = 80  # MAX_LINE_WIDTH_PX 설정값을 지정한다.


BEV_INIT_LEFT_PX = 300  # BEV_INIT_LEFT_PX 설정값을 지정한다.
BEV_INIT_RIGHT_PX = 620  # BEV_INIT_RIGHT_PX 설정값을 지정한다.


BEV_TRACK_WINDOW_PX = 45  # BEV_TRACK_WINDOW_PX 설정값을 지정한다.


BEV_STEP_JUMP_PX = 20  # BEV_STEP_JUMP_PX 설정값을 지정한다.


MIN_BOUNDARY_POINTS = 20  # MIN_BOUNDARY_POINTS 설정값을 지정한다.
MIN_FIT_SPAN_CM = 80.0  # MIN_FIT_SPAN_CM 설정값을 지정한다.


PLAUSIBLE_LATERAL_MIN_CM = 15.0  # PLAUSIBLE_LATERAL_MIN_CM 설정값을 지정한다.
PLAUSIBLE_LATERAL_MAX_CM = 90.0  # PLAUSIBLE_LATERAL_MAX_CM 설정값을 지정한다.

CROSSWALK_ENTRY_MAX_HEADING_DEG = 10.0  # CROSSWALK_ENTRY_MAX_HEADING_DEG 설정값을 지정한다.
CROSSWALK_ENTRY_MAX_CURVATURE = 0.0007  # CROSSWALK_ENTRY_MAX_CURVATURE 설정값을 지정한다.
CROSSWALK_BOUNDARY_PREDICT_SEC = 0.18  # CROSSWALK_BOUNDARY_PREDICT_SEC 설정값을 지정한다.


MARKING_APPROACH_START_RATIO = 0.08  # MARKING_APPROACH_START_RATIO 설정값을 지정한다.
MARKING_APPROACH_END_RATIO = 0.50  # MARKING_APPROACH_END_RATIO 설정값을 지정한다.
MARKING_APPROACH_MIN_PIXELS = 180  # MARKING_APPROACH_MIN_PIXELS 설정값을 지정한다.
MARKING_APPROACH_ARM_SEC = 1.50  # MARKING_APPROACH_ARM_SEC 설정값을 지정한다.
MARKING_CROSSWALK_MIN_WHITE_RATIO = 0.085  # MARKING_CROSSWALK_MIN_WHITE_RATIO 설정값을 지정한다.
MARKING_CROSSWALK_CONFIRM_FRAMES = 2  # MARKING_CROSSWALK_CONFIRM_FRAMES 설정값을 지정한다.
MARKING_PRESTEER_MAX_SEC = 0.75  # MARKING_PRESTEER_MAX_SEC 설정값을 지정한다.
MARKING_PRESTEER_REFERENCE_BAND_COUNTS = 3.0  # MARKING_PRESTEER_REFERENCE_BAND_COUNTS 설정값을 지정한다.
MARKING_PRESTEER_RIGHT_LIMIT_COUNTS = 2.0  # MARKING_PRESTEER_RIGHT_LIMIT_COUNTS 설정값을 지정한다.
MARKING_PRESTEER_LEFT_LIMIT_COUNTS = 10.0  # MARKING_PRESTEER_LEFT_LIMIT_COUNTS 설정값을 지정한다.
MARKING_STEER_MIN_SEC = 1.20  # MARKING_STEER_MIN_SEC 설정값을 지정한다.
MARKING_STEER_MAX_SEC = 2.50  # MARKING_STEER_MAX_SEC 설정값을 지정한다.
MARKING_STEER_RELEASE_FRAMES = 4  # MARKING_STEER_RELEASE_FRAMES 설정값을 지정한다.
MARKING_STEER_RIGHT_LIMIT_COUNTS = 2.0  # MARKING_STEER_RIGHT_LIMIT_COUNTS 설정값을 지정한다.
MARKING_STEER_LEFT_BASE_COUNTS = 4.0  # MARKING_STEER_LEFT_BASE_COUNTS 설정값을 지정한다.
MARKING_STEER_LEFT_MAX_COUNTS = 14.0  # MARKING_STEER_LEFT_MAX_COUNTS 설정값을 지정한다.
MARKING_STEER_LEFT_ASSIST_START_CM = -6.0  # MARKING_STEER_LEFT_ASSIST_START_CM 설정값을 지정한다.
MARKING_STEER_LEFT_ASSIST_FULL_CM = -18.0  # MARKING_STEER_LEFT_ASSIST_FULL_CM 설정값을 지정한다.
MARKING_RELEASE_MAX_HEADING_DEG = 15.0  # MARKING_RELEASE_MAX_HEADING_DEG 설정값을 지정한다.
MARKING_RELEASE_MAX_CURVATURE = 0.0030  # MARKING_RELEASE_MAX_CURVATURE 설정값을 지정한다.


GREEN_H_MIN = 28  # GREEN_H_MIN 설정값을 지정한다.
GREEN_H_MAX = 100  # GREEN_H_MAX 설정값을 지정한다.
GREEN_S_MIN = 35  # GREEN_S_MIN 설정값을 지정한다.
GREEN_V_MIN = 25  # GREEN_V_MIN 설정값을 지정한다.
GREEN_CLOSE_KERNEL_PX = 9  # GREEN_CLOSE_KERNEL_PX 설정값을 지정한다.
GREEN_MIN_COMPONENT_AREA_RATIO = 0.015  # GREEN_MIN_COMPONENT_AREA_RATIO 설정값을 지정한다.
GREEN_COMPONENT_MIN_RIGHT_RATIO = 0.70  # GREEN_COMPONENT_MIN_RIGHT_RATIO 설정값을 지정한다.
GREEN_MIN_BOUNDARY_POINTS = 24  # GREEN_MIN_BOUNDARY_POINTS 설정값을 지정한다.

GREEN_OFFSET_MIN_CM = -2.0  # GREEN_OFFSET_MIN_CM 설정값을 지정한다.
GREEN_OFFSET_MAX_CM = 12.0  # GREEN_OFFSET_MAX_CM 설정값을 지정한다.
GREEN_MATCH_MAX_HEADING_DEG = 8.0  # GREEN_MATCH_MAX_HEADING_DEG 설정값을 지정한다.
GREEN_MATCH_MAX_CURVATURE = 0.0030  # GREEN_MATCH_MAX_CURVATURE 설정값을 지정한다.
GREEN_TRUST_CONFIRM_FRAMES = 12  # GREEN_TRUST_CONFIRM_FRAMES 설정값을 지정한다.
GREEN_TRUST_LOSS_FRAMES = 8  # GREEN_TRUST_LOSS_FRAMES 설정값을 지정한다.
GREEN_OFFSET_FILTER_SEC = 0.80  # GREEN_OFFSET_FILTER_SEC 설정값을 지정한다.

GREEN_FALLBACK_MAX_LATERAL_JUMP_CM = 15.0  # GREEN_FALLBACK_MAX_LATERAL_JUMP_CM 설정값을 지정한다.
GREEN_FALLBACK_MAX_HEADING_JUMP_DEG = 12.0  # GREEN_FALLBACK_MAX_HEADING_JUMP_DEG 설정값을 지정한다.
GREEN_FALLBACK_MAX_CURVATURE_JUMP = 0.0045  # GREEN_FALLBACK_MAX_CURVATURE_JUMP 설정값을 지정한다.
GREEN_FALLBACK_SPEED_LIMIT = 180  # GREEN_FALLBACK_SPEED_LIMIT 설정값을 지정한다.

GREEN_CONTROL_LATERAL_RATE_CM_PER_SEC = 35.0  # GREEN_CONTROL_LATERAL_RATE_CM_PER_SEC 설정값을 지정한다.
GREEN_CONTROL_HEADING_RATE_DEG_PER_SEC = 30.0  # GREEN_CONTROL_HEADING_RATE_DEG_PER_SEC 설정값을 지정한다.
GREEN_CONTROL_CURVATURE_RATE_PER_SEC = 0.0040  # GREEN_CONTROL_CURVATURE_RATE_PER_SEC 설정값을 지정한다.

S_CURVE_TARGET_FILTER_SEC = 0.12  # S_CURVE_TARGET_FILTER_SEC 설정값을 지정한다.
S_CURVE_TARGET_DEADBAND_COUNTS = 3.5  # S_CURVE_TARGET_DEADBAND_COUNTS 설정값을 지정한다.
S_RIGHT_TRANSITION_CURVATURE = 0.0007  # S_RIGHT_TRANSITION_CURVATURE 설정값을 지정한다.
S_RIGHT_TRANSITION_HEADING_DEG = -8.0  # S_RIGHT_TRANSITION_HEADING_DEG 설정값을 지정한다.
S_RIGHT_TRANSITION_MAX_LEFT_HEADING_COUNTS = 6.0  # S_RIGHT_TRANSITION_MAX_LEFT_HEADING_COUNTS 설정값을 지정한다.


S_RIGHT_TRANSITION_CONFIRM_SEC = 0.10  # S_RIGHT_TRANSITION_CONFIRM_SEC 설정값을 지정한다.
S_RIGHT_TRANSITION_HOLD_STEER = 186.0  # S_RIGHT_TRANSITION_HOLD_STEER 설정값을 지정한다.


S_RIGHT_RELEASE_CURVATURE = 0.0003  # S_RIGHT_RELEASE_CURVATURE 설정값을 지정한다.
S_RIGHT_RELEASE_HEADING_DEG = 4.0  # S_RIGHT_RELEASE_HEADING_DEG 설정값을 지정한다.
S_RIGHT_RELEASE_CONFIRM_SEC = 0.20  # S_RIGHT_RELEASE_CONFIRM_SEC 설정값을 지정한다.
S_RIGHT_MAX_DROPOUT_SEC = 0.60  # S_RIGHT_MAX_DROPOUT_SEC 설정값을 지정한다.


S_RIGHT_EDGE_START_X_PX = 610.0  # S_RIGHT_EDGE_START_X_PX 설정값을 지정한다.
S_RIGHT_EDGE_FULL_X_PX = 635.0  # S_RIGHT_EDGE_FULL_X_PX 설정값을 지정한다.
S_RIGHT_EDGE_START_STEER = 170.0  # S_RIGHT_EDGE_START_STEER 설정값을 지정한다.
S_RIGHT_EDGE_FULL_STEER = 164.0  # S_RIGHT_EDGE_FULL_STEER 설정값을 지정한다.
S_RIGHT_EDGE_MIN_STEER = 164.0  # S_RIGHT_EDGE_MIN_STEER 설정값을 지정한다.


FIT_OUTLIER_CM = 4.0  # FIT_OUTLIER_CM 설정값을 지정한다.
FIT_MAX_RESIDUAL_CM = 6.0  # FIT_MAX_RESIDUAL_CM 설정값을 지정한다.


CURVATURE_LIMIT_ABS = 0.0110  # CURVATURE_LIMIT_ABS 설정값을 지정한다.


STEER_CENTER = 177  # STEER_CENTER 설정값을 지정한다.


STEER_LEFT_LIMIT = 220  # STEER_LEFT_LIMIT 설정값을 지정한다.
STEER_RIGHT_LIMIT = 120  # STEER_RIGHT_LIMIT 설정값을 지정한다.


STEER_TRIM_COUNTS = 0.0  # STEER_TRIM_COUNTS 설정값을 지정한다.


KI_COUNTS_PER_CM_SEC = 0.108  # KI_COUNTS_PER_CM_SEC 설정값을 지정한다.
INTEGRAL_LIMIT_COUNTS = 12.0  # INTEGRAL_LIMIT_COUNTS 설정값을 지정한다.


INTEGRAL_CURVATURE_MAX = 0.0006  # INTEGRAL_CURVATURE_MAX 설정값을 지정한다.


CURVE_OFFSET_CM_PER_CURVATURE = 2500.0  # CURVE_OFFSET_CM_PER_CURVATURE 설정값을 지정한다.
CURVE_OFFSET_LIMIT_CM = 10.0  # CURVE_OFFSET_LIMIT_CM 설정값을 지정한다.


CURVE_OFFSET_LEFT_SCALE = 0.35  # CURVE_OFFSET_LEFT_SCALE 설정값을 지정한다.
CURVE_OFFSET_ARM_CM      = 4.0  # CURVE_OFFSET_ARM_CM 설정값을 지정한다.
CURVE_OFFSET_ARM_SPAN_CM = 8.0  # CURVE_OFFSET_ARM_SPAN_CM 설정값을 지정한다.


STEER_STRAIGHT = STEER_CENTER + STEER_TRIM_COUNTS  # STEER_STRAIGHT 설정값을 지정한다.


TARGET_LATERAL_CM = 50.6  # TARGET_LATERAL_CM 설정값을 지정한다.


FF_COUNTS_PER_CURVATURE = 4900.0  # FF_COUNTS_PER_CURVATURE 설정값을 지정한다.


CURVATURE_TRUST_SPAN_CM = 95.0  # CURVATURE_TRUST_SPAN_CM 설정값을 지정한다.
CURVATURE_HOLD_SEC = 1.2  # CURVATURE_HOLD_SEC 설정값을 지정한다.


KE_COUNTS_PER_CM = 0.95  # KE_COUNTS_PER_CM 설정값을 지정한다.


HEADING_KINEMATIC_CM = 54.0  # HEADING_KINEMATIC_CM 설정값을 지정한다.


KPSI_COUNTS_PER_DEG = 1.67  # KPSI_COUNTS_PER_DEG 설정값을 지정한다.


FF_COUNTS_LIMIT = 45.0  # FF_COUNTS_LIMIT 설정값을 지정한다.
CTE_COUNTS_LIMIT = 30.0  # CTE_COUNTS_LIMIT 설정값을 지정한다.


PSI_COUNTS_LIMIT = 30.0  # PSI_COUNTS_LIMIT 설정값을 지정한다.


CTE_DEADBAND_CM = 1.5  # CTE_DEADBAND_CM 설정값을 지정한다.
PSI_DEADBAND_DEG = 1.0  # PSI_DEADBAND_DEG 설정값을 지정한다.


RIGHT_GUARD_LOOKAHEAD_CM = 45.0  # RIGHT_GUARD_LOOKAHEAD_CM 설정값을 지정한다.
RIGHT_GUARD_HEADING_LIMIT_DEG = 12.0  # RIGHT_GUARD_HEADING_LIMIT_DEG 설정값을 지정한다.


RIGHT_GUARD_START_CM = -2.0  # RIGHT_GUARD_START_CM 설정값을 지정한다.
RIGHT_GUARD_FULL_CM = -10.0  # RIGHT_GUARD_FULL_CM 설정값을 지정한다.


RIGHT_CURVE_GUARD_CURVATURE = 0.0009  # RIGHT_CURVE_GUARD_CURVATURE 설정값을 지정한다.


RIGHT_CURVE_GUARD_START_CM = 4.0  # RIGHT_CURVE_GUARD_START_CM 설정값을 지정한다.
RIGHT_CURVE_GUARD_FULL_CM = -2.0  # RIGHT_CURVE_GUARD_FULL_CM 설정값을 지정한다.


RIGHT_CURVE_GUARD_RELEASE_CM = 5.0  # RIGHT_CURVE_GUARD_RELEASE_CM 설정값을 지정한다.
RIGHT_CURVE_GUARD_MIN_STEER = 170.0  # RIGHT_CURVE_GUARD_MIN_STEER 설정값을 지정한다.
RIGHT_CURVE_GUIDE_STEER = 162.0  # RIGHT_CURVE_GUIDE_STEER 설정값을 지정한다.


RIGHT_RECOVERY_START_CM = -10.0  # RIGHT_RECOVERY_START_CM 설정값을 지정한다.
RIGHT_RECOVERY_FULL_CM = -16.0  # RIGHT_RECOVERY_FULL_CM 설정값을 지정한다.
RIGHT_RECOVERY_MAX_COUNTS = 7.0  # RIGHT_RECOVERY_MAX_COUNTS 설정값을 지정한다.


CTE_FILTER_SEC = 0.10  # CTE_FILTER_SEC 설정값을 지정한다.
PSI_FILTER_SEC = 0.12  # PSI_FILTER_SEC 설정값을 지정한다.
CURVATURE_FILTER_SEC = 0.15  # CURVATURE_FILTER_SEC 설정값을 지정한다.


STEER_RATE_PER_SEC = 150.0  # STEER_RATE_PER_SEC 설정값을 지정한다.
STEER_COMMAND_DEADBAND = 2  # STEER_COMMAND_DEADBAND 설정값을 지정한다.


SPEED_MAX = 255  # SPEED_MAX 설정값을 지정한다.
SPEED_MIN = 220  # SPEED_MIN 설정값을 지정한다.


RIGHT_CURVE_SPEED_CURVATURE = 0.0007  # RIGHT_CURVE_SPEED_CURVATURE 설정값을 지정한다.
RIGHT_CURVE_SPEED_LIMIT = 185  # RIGHT_CURVE_SPEED_LIMIT 설정값을 지정한다.


SPEED_CURVATURE_START = 0.0012  # SPEED_CURVATURE_START 설정값을 지정한다.

SPEED_CURVATURE_FULL = 0.0020  # SPEED_CURVATURE_FULL 설정값을 지정한다.

LOST_DRIVE_SPEED = 80  # LOST_DRIVE_SPEED 설정값을 지정한다.


HORIZONTAL_KERNEL_WIDTH = 35  # HORIZONTAL_KERNEL_WIDTH 설정값을 지정한다.
HORIZONTAL_KERNEL_HEIGHT = 3  # HORIZONTAL_KERNEL_HEIGHT 설정값을 지정한다.

HORIZONTAL_REMOVAL_DILATE_HEIGHT = 5  # HORIZONTAL_REMOVAL_DILATE_HEIGHT 설정값을 지정한다.
VERTICAL_RECONNECT_HEIGHT = 19  # VERTICAL_RECONNECT_HEIGHT 설정값을 지정한다.

MARKING_REGION_START_RATIO = 0.38  # MARKING_REGION_START_RATIO 설정값을 지정한다.
MARKING_REGION_END_RATIO = 0.82  # MARKING_REGION_END_RATIO 설정값을 지정한다.

ABSOLUTE_LEFT_RATIO = 0.42  # ABSOLUTE_LEFT_RATIO 설정값을 지정한다.
ABSOLUTE_RIGHT_RATIO = 0.98  # ABSOLUTE_RIGHT_RATIO 설정값을 지정한다.

HORIZONTAL_MIN_PIXELS = 250  # HORIZONTAL_MIN_PIXELS 설정값을 지정한다.

BOUNDARY_MIN_LATERAL_CM = 15.0  # BOUNDARY_MIN_LATERAL_CM 설정값을 지정한다.
CROSSWALK_MIN_BARS = 3  # CROSSWALK_MIN_BARS 설정값을 지정한다.
CROSSWALK_BAR_MIN_WIDTH = 35  # CROSSWALK_BAR_MIN_WIDTH 설정값을 지정한다.
CROSSWALK_BAR_MIN_HEIGHT = 12  # CROSSWALK_BAR_MIN_HEIGHT 설정값을 지정한다.


CROSSWALK_BAR_MIN_ASPECT_RATIO = 1.25  # CROSSWALK_BAR_MIN_ASPECT_RATIO 설정값을 지정한다.
CROSSWALK_BAR_MIN_FILL_RATIO = 0.55  # CROSSWALK_BAR_MIN_FILL_RATIO 설정값을 지정한다.

CROSSWALK_WHITE_RATIO = 0.10  # CROSSWALK_WHITE_RATIO 설정값을 지정한다.
CROSSWALK_CLEAR_WHITE_RATIO = 0.08  # CROSSWALK_CLEAR_WHITE_RATIO 설정값을 지정한다.
CROSSWALK_CONFIRM_FRAMES = 8  # CROSSWALK_CONFIRM_FRAMES 설정값을 지정한다.
CROSSWALK_CLEAR_FRAMES = 6  # CROSSWALK_CLEAR_FRAMES 설정값을 지정한다.
HORIZONTAL_CLEAR_FRAMES = 4  # HORIZONTAL_CLEAR_FRAMES 설정값을 지정한다.

HORIZONTAL_MAX_CONTINUOUS_SEC = 3.0  # HORIZONTAL_MAX_CONTINUOUS_SEC 설정값을 지정한다.
CROSSWALK_MAX_ACTIVE_SEC = 4.0  # CROSSWALK_MAX_ACTIVE_SEC 설정값을 지정한다.
CROSSWALK_REARM_BLOCK_SEC = 6.0  # CROSSWALK_REARM_BLOCK_SEC 설정값을 지정한다.


CROSSWALK_SPEED_RATIO = 0.88  # CROSSWALK_SPEED_RATIO 설정값을 지정한다.
CROSSWALK_SPEED_FLOOR = 120  # CROSSWALK_SPEED_FLOOR 설정값을 지정한다.
CROSSWALK_SPEED_HARD_MAX = 200  # CROSSWALK_SPEED_HARD_MAX 설정값을 지정한다.
MARKING_SPEED_RELEASE_HOLD_SEC = 0.50  # MARKING_SPEED_RELEASE_HOLD_SEC 설정값을 지정한다.
CROSSWALK_TRACK_RELEASE_HOLD_SEC = 0.20  # CROSSWALK_TRACK_RELEASE_HOLD_SEC 설정값을 지정한다.


CROSSWALK_LOST_TRANSITION_SEC = 0.20  # CROSSWALK_LOST_TRANSITION_SEC 설정값을 지정한다.
CROSSWALK_LOST_STEER_LIMIT_COUNTS = 8.0  # CROSSWALK_LOST_STEER_LIMIT_COUNTS 설정값을 지정한다.
CROSSWALK_LOST_SPEED = 120  # CROSSWALK_LOST_SPEED 설정값을 지정한다.


CROSSWALK_SPEED_DECEL_PER_SEC = 110.0  # CROSSWALK_SPEED_DECEL_PER_SEC 설정값을 지정한다.
CROSSWALK_SPEED_ACCEL_PER_SEC = 90.0  # CROSSWALK_SPEED_ACCEL_PER_SEC 설정값을 지정한다.


LOST_COMMAND_HOLD_FRAMES = 20  # LOST_COMMAND_HOLD_FRAMES 설정값을 지정한다.


S_RIGHT_LOST_STRONG_SEC = 0.25  # S_RIGHT_LOST_STRONG_SEC 설정값을 지정한다.
S_RIGHT_LOST_TOTAL_SEC = 0.80  # S_RIGHT_LOST_TOTAL_SEC 설정값을 지정한다.
S_RIGHT_LOST_STRONG_MIN_STEER = 164.0  # S_RIGHT_LOST_STRONG_MIN_STEER 설정값을 지정한다.
S_RIGHT_LOST_STRONG_MAX_STEER = 168.0  # S_RIGHT_LOST_STRONG_MAX_STEER 설정값을 지정한다.
S_RIGHT_LOST_WEAK_STEER = 170.0  # S_RIGHT_LOST_WEAK_STEER 설정값을 지정한다.

MIN_CONTROL_DT = 0.005  # MIN_CONTROL_DT 설정값을 지정한다.
MAX_CONTROL_DT = 0.100  # MAX_CONTROL_DT 설정값을 지정한다.

MAX_CAMERA_FAILURES = 3  # MAX_CAMERA_FAILURES 설정값을 지정한다.

DEBUG_PRINT_INTERVAL_SEC = 0.20  # DEBUG_PRINT_INTERVAL_SEC 설정값을 지정한다.


def clamp(value, minimum, maximum):  # 값을 지정한 최소·최대 범위로 제한하는 함수이다.


    return max(minimum, min(value, maximum))  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def move_toward(value, target, maximum_step):  # 현재 값을 목표값 쪽으로 제한된 크기만큼 이동시키는 함수이다.


    if value < target:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return min(value + maximum_step, target)  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    if value > target:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return max(value - maximum_step, target)  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    return value  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def low_pass(previous_value, measured_value, delta_time, time_constant):  # 새 측정값의 급격한 변화를 완화하는 저역통과 필터 함수이다.


    safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))  # safe_dt 값을 계산하거나 갱신한다.
    alpha = 1.0 - math.exp(-safe_dt / time_constant)  # alpha 값을 계산하거나 갱신한다.

    return previous_value + alpha * (measured_value - previous_value)  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def apply_deadband(value, deadband):  # 작은 오차를 무시하는 데드밴드 함수이다.


    if abs(value) <= deadband:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return 0.0  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    if value > 0.0:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return value - deadband  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    return value + deadband  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def send_command(comm, direction, speed, steer):  # 속도와 조향 명령을 아두이노로 전송하는 함수이다.


    safe_direction = int(clamp(direction, DIRECTION_STOP, DIRECTION_REVERSE))  # safe_direction 값을 계산하거나 갱신한다.
    safe_speed = int(clamp(speed, 0, 255))  # safe_speed 값을 계산하거나 갱신한다.
    safe_steer = int(clamp(steer, STEER_RIGHT_LIMIT, STEER_LEFT_LIMIT))  # safe_steer 값을 계산하거나 갱신한다.

    command = "%d,%d,%d\n" % (safe_direction, safe_speed, safe_steer)  # command 값을 계산하거나 갱신한다.
    comm.write(command.encode("utf-8"))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.


def send_stop(comm):  # 차량 정지 명령을 전송하는 함수이다.


    send_command(comm, DIRECTION_STOP, 0, STEER_STRAIGHT)  # 계산된 주행 명령을 차량에 전송한다.


def preprocess_roi(roi):  # 카메라 ROI를 흰색 차선 이진 영상으로 변환하는 함수이다.


    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)  # gray 값을 계산하거나 갱신한다.
    blurred = cv2.GaussianBlur(gray, GAUSSIAN_KERNEL, 0)  # blurred 값을 계산하거나 갱신한다.
    _, binary = cv2.threshold(  # 함수 호출 또는 묶음 계산을 시작한다.
        blurred, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    return binary  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def remove_horizontal_markings(binary_image):  # 가로 표시선을 차선 후보에서 제거하는 함수이다.


    horizontal_kernel = cv2.getStructuringElement(  # horizontal_kernel 값을 계산하거나 갱신한다.
        cv2.MORPH_RECT,  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
        (HORIZONTAL_KERNEL_WIDTH, HORIZONTAL_KERNEL_HEIGHT),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    horizontal_mask = cv2.morphologyEx(  # horizontal_mask 값을 계산하거나 갱신한다.
        binary_image, cv2.MORPH_OPEN, horizontal_kernel  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    removal_kernel = cv2.getStructuringElement(  # removal_kernel 값을 계산하거나 갱신한다.
        cv2.MORPH_RECT, (1, HORIZONTAL_REMOVAL_DILATE_HEIGHT)  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    removal_mask = cv2.dilate(  # removal_mask 값을 계산하거나 갱신한다.
        horizontal_mask, removal_kernel, iterations=1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    lane_binary = cv2.bitwise_and(  # lane_binary 값을 계산하거나 갱신한다.
        binary_image, cv2.bitwise_not(removal_mask)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    vertical_kernel = cv2.getStructuringElement(  # vertical_kernel 값을 계산하거나 갱신한다.
        cv2.MORPH_RECT, (3, VERTICAL_RECONNECT_HEIGHT)  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    lane_binary = cv2.morphologyEx(  # lane_binary 값을 계산하거나 갱신한다.
        lane_binary, cv2.MORPH_CLOSE, vertical_kernel  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    return lane_binary, horizontal_mask  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def load_calibration():  # BEV 변환과 차량 기준 좌표를 준비하는 함수이다.


    source_points = list(BEV_SOURCE_POINTS)  # source_points 값을 계산하거나 갱신한다.
    lane_width_cm = BEV_MARKER_WIDTH_CM  # lane_width_cm 값을 계산하거나 갱신한다.
    length_cm = BEV_MARKER_LENGTH_CM  # length_cm 값을 계산하거나 갱신한다.
    pixels_per_cm = BEV_PIXELS_PER_CM  # pixels_per_cm 값을 계산하거나 갱신한다.
    bev_width = BEV_WIDTH  # bev_width 값을 계산하거나 갱신한다.
    bev_height = BEV_HEIGHT  # bev_height 값을 계산하거나 갱신한다.
    origin = "코드 내장값"  # origin 값을 계산하거나 갱신한다.

    if os.path.exists(BEV_CALIB_FILE):  # 조건을 확인해 해당 처리 여부를 결정한다.
        try:  # 오류가 발생할 수 있는 작업을 시작한다.
            with open(BEV_CALIB_FILE, "r", encoding="utf-8") as handle:  # 자원을 안전하게 열어 사용한다.
                data = json.load(handle)  # data 값을 계산하거나 갱신한다.

            source_points = [tuple(p) for p in data["source_points"]]  # source_points 값을 계산하거나 갱신한다.
            lane_width_cm = float(data["lane_width_cm"])  # lane_width_cm 값을 계산하거나 갱신한다.
            length_cm = float(data["marker_length_cm"])  # length_cm 값을 계산하거나 갱신한다.
            pixels_per_cm = float(data["pixels_per_cm"])  # pixels_per_cm 값을 계산하거나 갱신한다.
            bev_width = int(data["bev_width"])  # bev_width 값을 계산하거나 갱신한다.
            bev_height = int(data["bev_height"])  # bev_height 값을 계산하거나 갱신한다.
            origin = BEV_CALIB_FILE  # origin 값을 계산하거나 갱신한다.

        except Exception as error:  # 발생한 오류를 안전하게 처리한다.
            print("경고: %s 를 읽지 못해 내장값을 씁니다. (%s)"  # 현재 상태를 콘솔에 출력한다.
                  % (BEV_CALIB_FILE, error))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    half_width = lane_width_cm * pixels_per_cm / 2.0  # half_width 값을 계산하거나 갱신한다.
    length_px = length_cm * pixels_per_cm  # length_px 값을 계산하거나 갱신한다.

    centre_x = bev_width / 2.0  # centre_x 값을 계산하거나 갱신한다.
    near_y = bev_height - BEV_NEAR_MARGIN_PX  # near_y 값을 계산하거나 갱신한다.
    far_y = near_y - length_px  # far_y 값을 계산하거나 갱신한다.

    destination = np.float32([  # destination 값을 계산하거나 갱신한다.
        [centre_x - half_width, near_y],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        [centre_x + half_width, near_y],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        [centre_x + half_width, far_y],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        [centre_x - half_width, far_y],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    ])  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    transform = cv2.getPerspectiveTransform(  # transform 값을 계산하거나 갱신한다.
        np.float32(source_points), destination  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


    roi_height = ROI_Y_END - ROI_Y_START  # roi_height 값을 계산하거나 갱신한다.
    vehicle = cv2.perspectiveTransform(  # vehicle 값을 계산하거나 갱신한다.
        np.float32([[[640 / 2.0, roi_height - 1]]]), transform  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    calibration = {  # calibration 값을 계산하거나 갱신한다.
        "transform": transform,  # transform 항목의 값을 저장한다.
        "pixels_per_cm": pixels_per_cm,  # pixels_per_cm 항목의 값을 저장한다.
        "bev_width": bev_width,  # bev_width 항목의 값을 저장한다.
        "bev_height": bev_height,  # bev_height 항목의 값을 저장한다.
        "vehicle_x": float(vehicle[0][0][0]),  # vehicle_x 항목의 값을 저장한다.
        "vehicle_y": float(vehicle[0][0][1]),  # vehicle_y 항목의 값을 저장한다.
        "origin": origin,  # origin 항목의 값을 저장한다.
    }  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    return calibration  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def warp_to_bev(image, calibration):  # 카메라 ROI를 조감도 영상으로 변환하는 함수이다.


    return cv2.warpPerspective(  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
        image,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        calibration["transform"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        (  # 함수 호출 또는 묶음 계산을 시작한다.
            calibration["bev_width"] + BEV_RIGHT_EXTRA_PX,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            calibration["bev_height"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        ),  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        flags=cv2.INTER_LINEAR,  # flags 값을 계산하거나 갱신한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


def find_white_runs(binary_image, center_y):  # 한 스캔 행에서 연속된 흰색 구간을 찾는 함수이다.


    height, width = binary_image.shape  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    y_start = max(0, int(center_y) - BEV_SCAN_HALF_HEIGHT_PX)  # y_start 값을 계산하거나 갱신한다.
    y_end = min(height, int(center_y) + BEV_SCAN_HALF_HEIGHT_PX + 1)  # y_end 값을 계산하거나 갱신한다.

    if y_end <= y_start:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return []  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    band = binary_image[y_start:y_end]  # band 값을 계산하거나 갱신한다.
    needed = max(2, int(math.ceil(band.shape[0] * 0.5)))  # needed 값을 계산하거나 갱신한다.
    active = np.count_nonzero(band, axis=0) >= needed  # active 값을 계산하거나 갱신한다.

    runs = []  # runs 값을 계산하거나 갱신한다.
    start = None  # start 값을 계산하거나 갱신한다.

    for x in range(len(active)):  # 대상 항목을 하나씩 반복 처리한다.
        if active[x] and start is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
            start = x  # start 값을 계산하거나 갱신한다.

        last = x == len(active) - 1  # last 값을 계산하거나 갱신한다.

        if start is not None and (not active[x] or last):  # 조건을 확인해 해당 처리 여부를 결정한다.
            end = x if (active[x] and last) else x - 1  # end 값을 계산하거나 갱신한다.
            run_width = end - start + 1  # run_width 값을 계산하거나 갱신한다.

            if MIN_LINE_WIDTH_PX <= run_width <= MAX_LINE_WIDTH_PX:  # 조건을 확인해 해당 처리 여부를 결정한다.
                runs.append((start + end) / 2.0)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

            start = None  # start 값을 계산하거나 갱신한다.

    return runs  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def detect_bev_boundary(bev_binary, previous_poly, calibration):  # BEV에서 우측 경계 후보점을 찾는 함수이다.


    points = []  # points 값을 계산하거나 갱신한다.
    previous_x = None  # previous_x 값을 계산하거나 갱신한다.


    initial_right_limit = min(  # initial_right_limit 값을 계산하거나 갱신한다.
        bev_binary.shape[1] - 1,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        BEV_INIT_RIGHT_PX + BEV_RIGHT_EXTRA_PX,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    scan_top = max(BEV_SCAN_TOP_PX, 12)  # scan_top 값을 계산하거나 갱신한다.

    for y in range(BEV_SCAN_BOTTOM_PX, scan_top, -BEV_SCAN_STEP_PX):  # 대상 항목을 하나씩 반복 처리한다.
        runs = find_white_runs(bev_binary, y)  # runs 값을 계산하거나 갱신한다.

        if not runs:  # 조건을 확인해 해당 처리 여부를 결정한다.
            continue  # 남은 처리를 건너뛰고 다음 반복으로 이동한다.

        if previous_x is not None:  # 조건을 확인해 해당 처리 여부를 결정한다.
            expected = previous_x  # expected 값을 계산하거나 갱신한다.
            window = BEV_STEP_JUMP_PX  # window 값을 계산하거나 갱신한다.

        elif previous_poly is not None:  # 앞 조건이 아니면 다음 조건을 확인한다.
            expected = evaluate_boundary_x(previous_poly, y, calibration)  # expected 값을 계산하거나 갱신한다.
            window = BEV_TRACK_WINDOW_PX  # window 값을 계산하거나 갱신한다.

        else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
            expected = None  # expected 값을 계산하거나 갱신한다.
            window = None  # window 값을 계산하거나 갱신한다.

        if expected is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
            candidates = [  # candidates 값을 계산하거나 갱신한다.
                x for x in runs  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                if BEV_INIT_LEFT_PX <= x <= initial_right_limit  # 조건을 확인해 해당 처리 여부를 결정한다.
            ]  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            if not candidates:  # 조건을 확인해 해당 처리 여부를 결정한다.
                continue  # 남은 처리를 건너뛰고 다음 반복으로 이동한다.

            selected = max(candidates)  # selected 값을 계산하거나 갱신한다.

        else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
            candidates = [  # candidates 값을 계산하거나 갱신한다.
                x for x in runs if abs(x - expected) <= window  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            ]  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            if not candidates:  # 조건을 확인해 해당 처리 여부를 결정한다.
                continue  # 남은 처리를 건너뛰고 다음 반복으로 이동한다.

            selected = min(candidates, key=lambda x: abs(x - expected))  # selected 값을 계산하거나 갱신한다.

        points.append((y, selected))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        previous_x = selected  # previous_x 값을 계산하거나 갱신한다.

    return points  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def extract_right_green_component(bev_colour):  # 우측 녹색 영역의 연결 성분을 선택하는 함수이다.


    if bev_colour is None or bev_colour.ndim != 3:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return None  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    hsv = cv2.cvtColor(bev_colour, cv2.COLOR_BGR2HSV)  # hsv 값을 계산하거나 갱신한다.
    green_mask = cv2.inRange(  # green_mask 값을 계산하거나 갱신한다.
        hsv,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        np.array([GREEN_H_MIN, GREEN_S_MIN, GREEN_V_MIN], dtype=np.uint8),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        np.array([GREEN_H_MAX, 255, 255], dtype=np.uint8),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    close_kernel = cv2.getStructuringElement(  # close_kernel 값을 계산하거나 갱신한다.
        cv2.MORPH_RECT,  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
        (GREEN_CLOSE_KERNEL_PX, GREEN_CLOSE_KERNEL_PX),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    green_mask = cv2.morphologyEx(  # green_mask 값을 계산하거나 갱신한다.
        green_mask, cv2.MORPH_CLOSE, close_kernel  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    label_count, labels, stats, _ = cv2.connectedComponentsWithStats(  # 함수 호출 또는 묶음 계산을 시작한다.
        green_mask, connectivity=8  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    image_height, image_width = green_mask.shape  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    minimum_area = int(round(  # minimum_area 값을 계산하거나 갱신한다.
        image_height * image_width * GREEN_MIN_COMPONENT_AREA_RATIO  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    ))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    minimum_right_x = image_width * GREEN_COMPONENT_MIN_RIGHT_RATIO  # minimum_right_x 값을 계산하거나 갱신한다.

    best_label = 0  # best_label 값을 계산하거나 갱신한다.
    best_area = 0  # best_area 값을 계산하거나 갱신한다.

    for label in range(1, label_count):  # 대상 항목을 하나씩 반복 처리한다.
        left = int(stats[label, cv2.CC_STAT_LEFT])  # left 값을 계산하거나 갱신한다.
        width = int(stats[label, cv2.CC_STAT_WIDTH])  # width 값을 계산하거나 갱신한다.
        area = int(stats[label, cv2.CC_STAT_AREA])  # area 값을 계산하거나 갱신한다.
        right = left + width - 1  # right 값을 계산하거나 갱신한다.

        if area < minimum_area or right < minimum_right_x:  # 조건을 확인해 해당 처리 여부를 결정한다.
            continue  # 남은 처리를 건너뛰고 다음 반복으로 이동한다.

        if area > best_area:  # 조건을 확인해 해당 처리 여부를 결정한다.
            best_area = area  # best_area 값을 계산하거나 갱신한다.
            best_label = label  # best_label 값을 계산하거나 갱신한다.

    if best_label == 0:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return None  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    component = np.zeros_like(green_mask)  # component 값을 계산하거나 갱신한다.
    component[labels == best_label] = 255  # component[labels == best_label] 값을 계산하거나 갱신한다.

    return component  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def detect_green_boundary(bev_colour):  # 녹색 외곽 경계 후보를 검출하는 함수이다.


    component = extract_right_green_component(bev_colour)  # component 값을 계산하거나 갱신한다.

    if component is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return []  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    points = []  # points 값을 계산하거나 갱신한다.
    scan_top = max(BEV_SCAN_TOP_PX, 12)  # scan_top 값을 계산하거나 갱신한다.

    for y in range(BEV_SCAN_BOTTOM_PX, scan_top, -BEV_SCAN_STEP_PX):  # 대상 항목을 하나씩 반복 처리한다.
        y_start = max(0, y - BEV_SCAN_HALF_HEIGHT_PX)  # y_start 값을 계산하거나 갱신한다.
        y_end = min(  # y_end 값을 계산하거나 갱신한다.
            component.shape[0],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            y + BEV_SCAN_HALF_HEIGHT_PX + 1,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        band = component[y_start:y_end]  # band 값을 계산하거나 갱신한다.

        if band.size == 0:  # 조건을 확인해 해당 처리 여부를 결정한다.
            continue  # 남은 처리를 건너뛰고 다음 반복으로 이동한다.

        needed = max(2, int(math.ceil(band.shape[0] * 0.45)))  # needed 값을 계산하거나 갱신한다.
        active = np.count_nonzero(band, axis=0) >= needed  # active 값을 계산하거나 갱신한다.
        active_x = np.flatnonzero(active)  # active_x 값을 계산하거나 갱신한다.

        if active_x.size == 0:  # 조건을 확인해 해당 처리 여부를 결정한다.
            continue  # 남은 처리를 건너뛰고 다음 반복으로 이동한다.

        points.append((y, float(active_x[0])))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    return points  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def evaluate_boundary_x(poly, bev_y, calibration):  # 경계 다항식의 특정 높이 X좌표를 계산하는 함수이다.


    pixels_per_cm = calibration["pixels_per_cm"]  # pixels_per_cm 값을 계산하거나 갱신한다.
    forward_cm = (calibration["vehicle_y"] - bev_y) / pixels_per_cm  # forward_cm 값을 계산하거나 갱신한다.
    lateral_cm = np.polyval(poly, forward_cm)  # lateral_cm 값을 계산하거나 갱신한다.

    return calibration["vehicle_x"] + lateral_cm * pixels_per_cm  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def fit_boundary(points, calibration):  # 경계점으로 차선 곡선과 주행 기하값을 계산하는 함수이다.


    if len(points) < MIN_BOUNDARY_POINTS:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return None  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    pixels_per_cm = calibration["pixels_per_cm"]  # pixels_per_cm 값을 계산하거나 갱신한다.

    forward = np.array(  # forward 값을 계산하거나 갱신한다.
        [(calibration["vehicle_y"] - y) / pixels_per_cm for y, _ in points],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        dtype=np.float64,  # dtype 값을 계산하거나 갱신한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    lateral = np.array(  # lateral 값을 계산하거나 갱신한다.
        [(x - calibration["vehicle_x"]) / pixels_per_cm for _, x in points],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        dtype=np.float64,  # dtype 값을 계산하거나 갱신한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    span_cm = float(np.max(forward) - np.min(forward))  # span_cm 값을 계산하거나 갱신한다.

    if span_cm < MIN_FIT_SPAN_CM:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return None  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    poly = np.polyfit(forward, lateral, 2)  # poly 값을 계산하거나 갱신한다.
    residual = lateral - np.polyval(poly, forward)  # residual 값을 계산하거나 갱신한다.


    keep = np.abs(residual) <= FIT_OUTLIER_CM  # keep 값을 계산하거나 갱신한다.

    if int(np.count_nonzero(keep)) >= MIN_BOUNDARY_POINTS:  # 조건을 확인해 해당 처리 여부를 결정한다.
        poly = np.polyfit(forward[keep], lateral[keep], 2)  # poly 값을 계산하거나 갱신한다.
        residual = lateral[keep] - np.polyval(poly, forward[keep])  # residual 값을 계산하거나 갱신한다.

    max_residual = float(np.max(np.abs(residual)))  # max_residual 값을 계산하거나 갱신한다.

    if max_residual > FIT_MAX_RESIDUAL_CM:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return None  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    slope = float(poly[1])  # slope 값을 계산하거나 갱신한다.


    curvature = float(  # curvature 값을 계산하거나 갱신한다.
        2.0 * poly[0] / math.pow(1.0 + slope * slope, 1.5)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


    curvature = float(clamp(  # curvature 값을 계산하거나 갱신한다.
        curvature, -CURVATURE_LIMIT_ABS, CURVATURE_LIMIT_ABS  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    ))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    return {  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
        "poly": poly,  # poly 항목의 값을 저장한다.
        "lateral_cm": float(poly[2]),  # lateral_cm 항목의 값을 저장한다.
        "heading_deg": math.degrees(math.atan(slope)),  # heading_deg 항목의 값을 저장한다.
        "curvature": curvature,  # curvature 항목의 값을 저장한다.
        "max_residual_cm": max_residual,  # max_residual_cm 항목의 값을 저장한다.
        "point_count": len(points),  # point_count 항목의 값을 저장한다.
        "span_cm": span_cm,  # span_cm 항목의 값을 저장한다.
        "forward_max_cm": float(np.max(forward)),  # forward_max_cm 항목의 값을 저장한다.
    }  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


def crosswalk_entry_is_qualified(geometry_state):  # 횡단보도 보호를 시작해도 안전한 진입 상태인지 확인하는 함수이다.


    return (  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
        geometry_state["ready"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and abs(geometry_state["heading_deg"])  # 여러 판정 조건을 이어서 계산한다.
        <= CROSSWALK_ENTRY_MAX_HEADING_DEG  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and abs(geometry_state["curvature"])  # 여러 판정 조건을 이어서 계산한다.
        <= CROSSWALK_ENTRY_MAX_CURVATURE  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


def copy_geometry(geometry):  # 경계 기하 정보를 안전하게 복사하는 함수이다.


    if geometry is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return None  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    copied = dict(geometry)  # copied 값을 계산하거나 갱신한다.
    copied["poly"] = np.array(geometry["poly"], dtype=np.float64).copy()  # copied["poly"] 값을 계산하거나 갱신한다.
    return copied  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def stabilize_green_geometry(measured, previous, delta_time):  # 녹색 경계 기하값의 순간 변화를 제한하는 함수이다.


    if measured is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return None  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    if previous is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return copy_geometry(measured)  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))  # safe_dt 값을 계산하거나 갱신한다.
    stabilized = copy_geometry(measured)  # stabilized 값을 계산하거나 갱신한다.

    stabilized["lateral_cm"] = move_toward(  # stabilized["lateral_cm"] 값을 계산하거나 갱신한다.
        previous["lateral_cm"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        measured["lateral_cm"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        GREEN_CONTROL_LATERAL_RATE_CM_PER_SEC * safe_dt,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    stabilized["heading_deg"] = move_toward(  # stabilized["heading_deg"] 값을 계산하거나 갱신한다.
        previous["heading_deg"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        measured["heading_deg"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        GREEN_CONTROL_HEADING_RATE_DEG_PER_SEC * safe_dt,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    stabilized["curvature"] = move_toward(  # stabilized["curvature"] 값을 계산하거나 갱신한다.
        previous["curvature"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        measured["curvature"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        GREEN_CONTROL_CURVATURE_RATE_PER_SEC * safe_dt,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


    slope = math.tan(math.radians(stabilized["heading_deg"]))  # slope 값을 계산하거나 갱신한다.
    quadratic = (  # quadratic 값을 계산하거나 갱신한다.
        0.5  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        * stabilized["curvature"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        * math.pow(1.0 + slope * slope, 1.5)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    stabilized["poly"] = np.array(  # stabilized["poly"] 값을 계산하거나 갱신한다.
        [quadratic, slope, stabilized["lateral_cm"]],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        dtype=np.float64,  # dtype 값을 계산하거나 갱신한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    return stabilized  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def create_green_boundary_state():  # 녹색 경계 추적 상태를 초기화하는 함수이다.


    return {  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
        "offset_cm": 2.0,  # offset_cm 항목의 값을 저장한다.
        "match_frames": 0,  # match_frames 항목의 값을 저장한다.
        "missing_frames": 0,  # missing_frames 항목의 값을 저장한다.
        "ready": False,  # ready 항목의 값을 저장한다.
    }  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


def update_green_boundary_state(state, white_geometry, green_geometry,  # 녹색 경계 추적 상태를 매 프레임 갱신하는 함수이다.
                                delta_time):  # 아래에 이어질 처리 블록을 시작한다.


    if green_geometry is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        state["missing_frames"] += 1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

        if state["missing_frames"] >= GREEN_TRUST_LOSS_FRAMES:  # 조건을 확인해 해당 처리 여부를 결정한다.
            state["ready"] = False  # state["ready"] 값을 계산하거나 갱신한다.
            state["match_frames"] = 0  # state["match_frames"] 값을 계산하거나 갱신한다.

        return  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    state["missing_frames"] = 0  # state["missing_frames"] 값을 계산하거나 갱신한다.

    if white_geometry is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    measured_offset = (  # measured_offset 값을 계산하거나 갱신한다.
        green_geometry["lateral_cm"] - white_geometry["lateral_cm"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    heading_difference = abs(  # heading_difference 값을 계산하거나 갱신한다.
        green_geometry["heading_deg"] - white_geometry["heading_deg"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    curvature_difference = abs(  # curvature_difference 값을 계산하거나 갱신한다.
        green_geometry["curvature"] - white_geometry["curvature"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    matches = (  # matches 값을 계산하거나 갱신한다.
        GREEN_OFFSET_MIN_CM <= measured_offset <= GREEN_OFFSET_MAX_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and heading_difference <= GREEN_MATCH_MAX_HEADING_DEG  # 여러 판정 조건을 이어서 계산한다.
        and curvature_difference <= GREEN_MATCH_MAX_CURVATURE  # 여러 판정 조건을 이어서 계산한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    if matches:  # 조건을 확인해 해당 처리 여부를 결정한다.
        state["offset_cm"] = low_pass(  # state["offset_cm"] 값을 계산하거나 갱신한다.
            state["offset_cm"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            measured_offset,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            delta_time,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            GREEN_OFFSET_FILTER_SEC,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        state["match_frames"] = min(  # state["match_frames"] 값을 계산하거나 갱신한다.
            GREEN_TRUST_CONFIRM_FRAMES,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            state["match_frames"] + 1,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        if state["match_frames"] >= GREEN_TRUST_CONFIRM_FRAMES:  # 조건을 확인해 해당 처리 여부를 결정한다.
            state["ready"] = True  # state["ready"] 값을 계산하거나 갱신한다.
    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
        state["match_frames"] = max(0, state["match_frames"] - 2)  # state["match_frames"] 값을 계산하거나 갱신한다.

        if state["match_frames"] == 0:  # 조건을 확인해 해당 처리 여부를 결정한다.
            state["ready"] = False  # state["ready"] 값을 계산하거나 갱신한다.


def correct_green_geometry(green_geometry, offset_cm):  # 녹색 경계로 기존 차선 기하값을 보정하는 함수이다.


    if green_geometry is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return None  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    corrected = dict(green_geometry)  # corrected 값을 계산하거나 갱신한다.
    corrected_poly = np.array(green_geometry["poly"], dtype=np.float64)  # corrected_poly 값을 계산하거나 갱신한다.
    corrected_poly[2] -= float(offset_cm)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    corrected["poly"] = corrected_poly  # corrected["poly"] 값을 계산하거나 갱신한다.
    corrected["lateral_cm"] = (  # corrected["lateral_cm"] 값을 계산하거나 갱신한다.
        green_geometry["lateral_cm"] - float(offset_cm)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    return corrected  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def green_geometry_is_safe(green_geometry, geometry_state):  # 녹색 보조 기하값을 사용해도 안전한지 확인하는 함수이다.


    if green_geometry is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return False  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    if green_geometry["point_count"] < GREEN_MIN_BOUNDARY_POINTS:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return False  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    lateral_cm = green_geometry["lateral_cm"]  # lateral_cm 값을 계산하거나 갱신한다.

    if not (  # 조건을 확인해 해당 처리 여부를 결정한다.
        PLAUSIBLE_LATERAL_MIN_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        <= lateral_cm  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        <= PLAUSIBLE_LATERAL_MAX_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    ):  # 아래에 이어질 처리 블록을 시작한다.
        return False  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    if not geometry_state["ready"]:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return True  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    lateral_jump = abs(  # lateral_jump 값을 계산하거나 갱신한다.
        lateral_cm - geometry_state["lateral_cm"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    heading_jump = abs(  # heading_jump 값을 계산하거나 갱신한다.
        green_geometry["heading_deg"] - geometry_state["heading_deg"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    curvature_jump = abs(  # curvature_jump 값을 계산하거나 갱신한다.
        green_geometry["curvature"] - geometry_state["curvature"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    return (  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
        lateral_jump <= GREEN_FALLBACK_MAX_LATERAL_JUMP_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and heading_jump <= GREEN_FALLBACK_MAX_HEADING_JUMP_DEG  # 여러 판정 조건을 이어서 계산한다.
        and curvature_jump <= GREEN_FALLBACK_MAX_CURVATURE_JUMP  # 여러 판정 조건을 이어서 계산한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


def create_geometry_state():  # 일반 차선 기하 상태를 초기화하는 함수이다.


    return {  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
        "lateral_cm": TARGET_LATERAL_CM,  # lateral_cm 항목의 값을 저장한다.
        "heading_deg": 0.0,  # heading_deg 항목의 값을 저장한다.
        "curvature": 0.0,  # curvature 항목의 값을 저장한다.
        "held_curvature": 0.0,  # held_curvature 항목의 값을 저장한다.
        "span_cm": 0.0,  # span_cm 항목의 값을 저장한다.
        "ready": False,  # ready 항목의 값을 저장한다.

        "integral": 0.0,  # integral 항목의 값을 저장한다.

        "right_curve_guard_active": False,  # right_curve_guard_active 항목의 값을 저장한다.

        "s_right_phase_active": False,  # s_right_phase_active 항목의 값을 저장한다.

        "s_right_transition_sec": 0.0,  # s_right_transition_sec 항목의 값을 저장한다.
        "s_right_transition_pending": False,  # s_right_transition_pending 항목의 값을 저장한다.
        "s_right_dropout_sec": 0.0,  # s_right_dropout_sec 항목의 값을 저장한다.
        "s_right_release_sec": 0.0,  # s_right_release_sec 항목의 값을 저장한다.
    }  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


def update_geometry_state(state, geometry, delta_time):  # 검출 결과와 이전 기억으로 기하 상태를 갱신하는 함수이다.


    state["span_cm"] = geometry["span_cm"]  # state["span_cm"] 값을 계산하거나 갱신한다.


    measured = geometry["curvature"]  # measured 값을 계산하거나 갱신한다.

    if geometry["span_cm"] >= CURVATURE_TRUST_SPAN_CM:  # 조건을 확인해 해당 처리 여부를 결정한다.
        state["held_curvature"] = measured  # state["held_curvature"] 값을 계산하거나 갱신한다.
        effective = measured  # effective 값을 계산하거나 갱신한다.

    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
        held = state["held_curvature"] * math.exp(  # held 값을 계산하거나 갱신한다.
            -float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))  # 값의 변화 범위를 안정적으로 제한한다.
            / CURVATURE_HOLD_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        if measured * held > 0.0 and abs(held) > abs(measured):  # 조건을 확인해 해당 처리 여부를 결정한다.
            effective = held  # effective 값을 계산하거나 갱신한다.
            state["held_curvature"] = held  # state["held_curvature"] 값을 계산하거나 갱신한다.

        else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
            effective = measured  # effective 값을 계산하거나 갱신한다.
            state["held_curvature"] = measured  # state["held_curvature"] 값을 계산하거나 갱신한다.

    if not state["ready"]:  # 조건을 확인해 해당 처리 여부를 결정한다.
        state["lateral_cm"] = geometry["lateral_cm"]  # state["lateral_cm"] 값을 계산하거나 갱신한다.
        state["heading_deg"] = geometry["heading_deg"]  # state["heading_deg"] 값을 계산하거나 갱신한다.
        state["curvature"] = effective  # state["curvature"] 값을 계산하거나 갱신한다.
        state["ready"] = True  # state["ready"] 값을 계산하거나 갱신한다.
        return  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    state["lateral_cm"] = low_pass(  # state["lateral_cm"] 값을 계산하거나 갱신한다.
        state["lateral_cm"], geometry["lateral_cm"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        delta_time, CTE_FILTER_SEC,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    state["heading_deg"] = low_pass(  # state["heading_deg"] 값을 계산하거나 갱신한다.
        state["heading_deg"], geometry["heading_deg"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        delta_time, PSI_FILTER_SEC,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    state["curvature"] = low_pass(  # state["curvature"] 값을 계산하거나 갱신한다.
        state["curvature"], effective,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        delta_time, CURVATURE_FILTER_SEC,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


def calculate_steering(state, delta_time):  # 차선 오차·헤딩·곡률로 원시 조향값을 계산하는 함수이다.


    curvature = state["curvature"]  # curvature 값을 계산하거나 갱신한다.


    line_margin = state["lateral_cm"] - TARGET_LATERAL_CM  # line_margin 값을 계산하거나 갱신한다.


    raw_offset = CURVE_OFFSET_CM_PER_CURVATURE * curvature  # raw_offset 값을 계산하거나 갱신한다.

    if curvature < 0.0:  # 조건을 확인해 해당 처리 여부를 결정한다.
        raw_offset *= CURVE_OFFSET_LEFT_SCALE  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.

        raw_offset *= clamp(  # 함수 호출 또는 묶음 계산을 시작한다.
            (CURVE_OFFSET_ARM_CM - line_margin)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            / CURVE_OFFSET_ARM_SPAN_CM,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            0.0, 1.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    curve_offset = clamp(  # curve_offset 값을 계산하거나 갱신한다.
        raw_offset,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        -CURVE_OFFSET_LIMIT_CM, CURVE_OFFSET_LIMIT_CM,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    lateral_error = line_margin - curve_offset  # lateral_error 값을 계산하거나 갱신한다.


    kinematic_deg = math.degrees(  # kinematic_deg 값을 계산하거나 갱신한다.
        math.atan(HEADING_KINEMATIC_CM * curvature)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    heading_error = state["heading_deg"] - kinematic_deg  # heading_error 값을 계산하거나 갱신한다.


    feedforward = clamp(  # feedforward 값을 계산하거나 갱신한다.
        FF_COUNTS_PER_CURVATURE * (-curvature),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        -FF_COUNTS_LIMIT, FF_COUNTS_LIMIT,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    cross_track = clamp(  # cross_track 값을 계산하거나 갱신한다.
        KE_COUNTS_PER_CM * (-apply_deadband(lateral_error, CTE_DEADBAND_CM)),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        -CTE_COUNTS_LIMIT, CTE_COUNTS_LIMIT,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    heading_term = clamp(  # heading_term 값을 계산하거나 갱신한다.
        KPSI_COUNTS_PER_DEG  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        * (-apply_deadband(heading_error, PSI_DEADBAND_DEG)),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        -PSI_COUNTS_LIMIT, PSI_COUNTS_LIMIT,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


    if (  # 조건을 확인해 해당 처리 여부를 결정한다.
        curvature >= S_RIGHT_TRANSITION_CURVATURE  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and heading_error <= S_RIGHT_TRANSITION_HEADING_DEG  # 여러 판정 조건을 이어서 계산한다.
        and heading_term > S_RIGHT_TRANSITION_MAX_LEFT_HEADING_COUNTS  # 여러 판정 조건을 이어서 계산한다.
    ):  # 아래에 이어질 처리 블록을 시작한다.
        heading_term = S_RIGHT_TRANSITION_MAX_LEFT_HEADING_COUNTS  # heading_term 값을 계산하거나 갱신한다.


    integral_term = clamp(  # integral_term 값을 계산하거나 갱신한다.
        state["integral"], -INTEGRAL_LIMIT_COUNTS, INTEGRAL_LIMIT_COUNTS  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    total = feedforward + cross_track + heading_term + integral_term  # total 값을 계산하거나 갱신한다.


    closing_heading_deg = clamp(  # closing_heading_deg 값을 계산하거나 갱신한다.
        heading_error, 0.0, RIGHT_GUARD_HEADING_LIMIT_DEG  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    predicted_margin = (  # predicted_margin 값을 계산하거나 갱신한다.
        line_margin  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        - RIGHT_GUARD_LOOKAHEAD_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        * math.tan(math.radians(closing_heading_deg))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


    right_curve_candidate = (  # right_curve_candidate 값을 계산하거나 갱신한다.
        curvature >= S_RIGHT_TRANSITION_CURVATURE  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and heading_error <= S_RIGHT_TRANSITION_HEADING_DEG  # 여러 판정 조건을 이어서 계산한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    right_curve_measured = (  # right_curve_measured 값을 계산하거나 갱신한다.
        curvature >= S_RIGHT_TRANSITION_CURVATURE  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    if not state["s_right_phase_active"]:  # 조건을 확인해 해당 처리 여부를 결정한다.
        if right_curve_candidate:  # 조건을 확인해 해당 처리 여부를 결정한다.
            state["s_right_transition_sec"] += delta_time  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            state["s_right_transition_pending"] = True  # state["s_right_transition_pending"] 값을 계산하거나 갱신한다.

            if (  # 조건을 확인해 해당 처리 여부를 결정한다.
                state["s_right_transition_sec"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                >= S_RIGHT_TRANSITION_CONFIRM_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            ):  # 아래에 이어질 처리 블록을 시작한다.
                state["s_right_phase_active"] = True  # state["s_right_phase_active"] 값을 계산하거나 갱신한다.
                state["s_right_transition_pending"] = False  # state["s_right_transition_pending"] 값을 계산하거나 갱신한다.
                state["s_right_transition_sec"] = 0.0  # state["s_right_transition_sec"] 값을 계산하거나 갱신한다.
                state["s_right_dropout_sec"] = 0.0  # state["s_right_dropout_sec"] 값을 계산하거나 갱신한다.
                state["s_right_release_sec"] = 0.0  # state["s_right_release_sec"] 값을 계산하거나 갱신한다.
        else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
            state["s_right_transition_sec"] = 0.0  # state["s_right_transition_sec"] 값을 계산하거나 갱신한다.
            state["s_right_transition_pending"] = False  # state["s_right_transition_pending"] 값을 계산하거나 갱신한다.
            state["s_right_dropout_sec"] = 0.0  # state["s_right_dropout_sec"] 값을 계산하거나 갱신한다.
            state["s_right_release_sec"] = 0.0  # state["s_right_release_sec"] 값을 계산하거나 갱신한다.

    elif right_curve_measured:  # 앞 조건이 아니면 다음 조건을 확인한다.
        state["s_right_transition_sec"] = 0.0  # state["s_right_transition_sec"] 값을 계산하거나 갱신한다.
        state["s_right_transition_pending"] = False  # state["s_right_transition_pending"] 값을 계산하거나 갱신한다.
        state["s_right_dropout_sec"] = 0.0  # state["s_right_dropout_sec"] 값을 계산하거나 갱신한다.
        state["s_right_release_sec"] = 0.0  # state["s_right_release_sec"] 값을 계산하거나 갱신한다.

    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
        state["s_right_transition_pending"] = False  # state["s_right_transition_pending"] 값을 계산하거나 갱신한다.
        state["s_right_dropout_sec"] += delta_time  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

        stable_right_exit = (  # stable_right_exit 값을 계산하거나 갱신한다.
            abs(curvature) <= S_RIGHT_RELEASE_CURVATURE  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            and abs(heading_error) <= S_RIGHT_RELEASE_HEADING_DEG  # 여러 판정 조건을 이어서 계산한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        if stable_right_exit:  # 조건을 확인해 해당 처리 여부를 결정한다.
            state["s_right_release_sec"] += delta_time  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
            state["s_right_release_sec"] = 0.0  # state["s_right_release_sec"] 값을 계산하거나 갱신한다.

        if (  # 조건을 확인해 해당 처리 여부를 결정한다.
            state["s_right_release_sec"] >= S_RIGHT_RELEASE_CONFIRM_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            or state["s_right_dropout_sec"] >= S_RIGHT_MAX_DROPOUT_SEC  # 여러 판정 조건을 이어서 계산한다.
        ):  # 아래에 이어질 처리 블록을 시작한다.
            state["s_right_phase_active"] = False  # state["s_right_phase_active"] 값을 계산하거나 갱신한다.
            state["s_right_dropout_sec"] = 0.0  # state["s_right_dropout_sec"] 값을 계산하거나 갱신한다.
            state["s_right_release_sec"] = 0.0  # state["s_right_release_sec"] 값을 계산하거나 갱신한다.

    right_curve_confirmed = state["s_right_phase_active"]  # right_curve_confirmed 값을 계산하거나 갱신한다.


    if right_curve_confirmed:  # 조건을 확인해 해당 처리 여부를 결정한다.
        if state["right_curve_guard_active"]:  # 조건을 확인해 해당 처리 여부를 결정한다.
            if line_margin >= RIGHT_CURVE_GUARD_RELEASE_CM:  # 조건을 확인해 해당 처리 여부를 결정한다.
                state["right_curve_guard_active"] = False  # state["right_curve_guard_active"] 값을 계산하거나 갱신한다.
        elif line_margin <= RIGHT_CURVE_GUARD_START_CM:  # 앞 조건이 아니면 다음 조건을 확인한다.
            state["right_curve_guard_active"] = True  # state["right_curve_guard_active"] 값을 계산하거나 갱신한다.
    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
        state["right_curve_guard_active"] = False  # state["right_curve_guard_active"] 값을 계산하거나 갱신한다.

    if right_curve_confirmed:  # 조건을 확인해 해당 처리 여부를 결정한다.
        guard_margin = line_margin  # guard_margin 값을 계산하거나 갱신한다.
        guard_start = RIGHT_CURVE_GUARD_START_CM  # guard_start 값을 계산하거나 갱신한다.
        guard_full = RIGHT_CURVE_GUARD_FULL_CM  # guard_full 값을 계산하거나 갱신한다.
    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.

        guard_margin = predicted_margin  # guard_margin 값을 계산하거나 갱신한다.
        guard_start = RIGHT_GUARD_START_CM  # guard_start 값을 계산하거나 갱신한다.
        guard_full = RIGHT_GUARD_FULL_CM  # guard_full 값을 계산하거나 갱신한다.

    guard_is_active = (  # guard_is_active 값을 계산하거나 갱신한다.
        state["right_curve_guard_active"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        if right_curve_confirmed  # 조건을 확인해 해당 처리 여부를 결정한다.
        else guard_margin < guard_start  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    if total < 0.0 and guard_is_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
        allowance = clamp(  # allowance 값을 계산하거나 갱신한다.
            (guard_margin - guard_full) / max(guard_start - guard_full, 1e-6),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            0.0, 1.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        guarded = total * allowance  # guarded 값을 계산하거나 갱신한다.

        if right_curve_confirmed:  # 조건을 확인해 해당 처리 여부를 결정한다.


            total = max(  # total 값을 계산하거나 갱신한다.
                guarded,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                RIGHT_CURVE_GUARD_MIN_STEER - STEER_STRAIGHT,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        elif feedforward < 0.0:  # 앞 조건이 아니면 다음 조건을 확인한다.
            total = min(guarded, feedforward)  # total 값을 계산하거나 갱신한다.
        else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
            total = guarded  # total 값을 계산하거나 갱신한다.


    if (  # 조건을 확인해 해당 처리 여부를 결정한다.
        line_margin < RIGHT_RECOVERY_START_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and heading_error > 0.0  # 여러 판정 조건을 이어서 계산한다.
    ):  # 아래에 이어질 처리 블록을 시작한다.
        recovery_ratio = clamp(  # recovery_ratio 값을 계산하거나 갱신한다.
            (RIGHT_RECOVERY_START_CM - line_margin)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            / max(  # 함수 호출 또는 묶음 계산을 시작한다.
                RIGHT_RECOVERY_START_CM - RIGHT_RECOVERY_FULL_CM,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                1e-6,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            ),  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            0.0, 1.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        minimum_left = RIGHT_RECOVERY_MAX_COUNTS * recovery_ratio  # minimum_left 값을 계산하거나 갱신한다.
        total = max(total, minimum_left)  # total 값을 계산하거나 갱신한다.


    raw = STEER_STRAIGHT + total  # raw 값을 계산하거나 갱신한다.

    steer = int(round(clamp(raw, STEER_RIGHT_LIMIT, STEER_LEFT_LIMIT)))  # steer 값을 계산하거나 갱신한다.


    saturated = (raw <= STEER_RIGHT_LIMIT) or (raw >= STEER_LEFT_LIMIT)  # saturated 값을 계산하거나 갱신한다.


    on_straight = abs(curvature) < INTEGRAL_CURVATURE_MAX  # on_straight 값을 계산하거나 갱신한다.

    if not saturated and on_straight:  # 조건을 확인해 해당 처리 여부를 결정한다.
        safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))  # safe_dt 값을 계산하거나 갱신한다.

        state["integral"] = float(clamp(  # state["integral"] 값을 계산하거나 갱신한다.
            state["integral"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            + KI_COUNTS_PER_CM_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            * (-apply_deadband(lateral_error, CTE_DEADBAND_CM))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            * safe_dt,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            -INTEGRAL_LIMIT_COUNTS, INTEGRAL_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        ))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    return (steer, feedforward, cross_track, heading_term, heading_error,  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
            integral_term, curve_offset)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.


def calculate_speed(curvature):  # 곡률에 맞는 기본 주행 속도를 계산하는 함수이다.


    magnitude = abs(curvature)  # magnitude 값을 계산하거나 갱신한다.

    if magnitude <= SPEED_CURVATURE_START:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return SPEED_MAX  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    progress = clamp(  # progress 값을 계산하거나 갱신한다.
        (magnitude - SPEED_CURVATURE_START)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        / max(SPEED_CURVATURE_FULL - SPEED_CURVATURE_START, 1e-9),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        0.0, 1.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    return int(round(SPEED_MAX - (SPEED_MAX - SPEED_MIN) * progress))  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def calculate_crosswalk_speed_limit():  # 횡단보도 통과 속도 상한을 계산하는 함수이다.


    proportional = int(round(SPEED_MAX * CROSSWALK_SPEED_RATIO))  # proportional 값을 계산하거나 갱신한다.
    protected = max(CROSSWALK_SPEED_FLOOR, SPEED_MIN, proportional)  # protected 값을 계산하거나 갱신한다.

    return int(min(SPEED_MAX, CROSSWALK_SPEED_HARD_MAX, protected))  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def update_crosswalk_speed(target_speed, speed_state, delta_time):  # 횡단보도 속도 변화를 부드럽게 갱신하는 함수이다.


    safe_target = float(clamp(target_speed, 0, 255))  # safe_target 값을 계산하거나 갱신한다.
    safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))  # safe_dt 값을 계산하거나 갱신한다.

    if safe_target < speed_state:  # 조건을 확인해 해당 처리 여부를 결정한다.
        rate = CROSSWALK_SPEED_DECEL_PER_SEC  # rate 값을 계산하거나 갱신한다.
    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
        rate = CROSSWALK_SPEED_ACCEL_PER_SEC  # rate 값을 계산하거나 갱신한다.

    new_state = move_toward(speed_state, safe_target, rate * safe_dt)  # new_state 값을 계산하거나 갱신한다.
    command = int(round(clamp(new_state, 0.0, 255.0)))  # command 값을 계산하거나 갱신한다.

    return new_state, command  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def update_steering_command(raw_steer, steer_state, previous_command,  # 조향 명령의 프레임 간 변화량을 제한하는 함수이다.
                            delta_time):  # 아래에 이어질 처리 블록을 시작한다.


    safe_dt = float(clamp(delta_time, MIN_CONTROL_DT, MAX_CONTROL_DT))  # safe_dt 값을 계산하거나 갱신한다.

    new_state = move_toward(  # new_state 값을 계산하거나 갱신한다.
        steer_state, float(raw_steer), STEER_RATE_PER_SEC * safe_dt  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    new_command = int(round(new_state))  # new_command 값을 계산하거나 갱신한다.

    if abs(new_command - previous_command) < STEER_COMMAND_DEADBAND:  # 조건을 확인해 해당 처리 여부를 결정한다.
        new_command = previous_command  # new_command 값을 계산하거나 갱신한다.

    new_command = int(clamp(  # new_command 값을 계산하거나 갱신한다.
        new_command, STEER_RIGHT_LIMIT, STEER_LEFT_LIMIT  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    ))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    return new_state, new_command  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def filter_s_curve_target(raw_steer, geometry_state, filter_state,  # S자 구간의 조향 목표를 단계에 맞게 안정화하는 함수이다.
                          previous_curvature, delta_time):  # 아래에 이어질 처리 블록을 시작한다.


    target = float(raw_steer)  # target 값을 계산하거나 갱신한다.
    curvature = geometry_state["curvature"]  # curvature 값을 계산하거나 갱신한다.
    line_margin = geometry_state["lateral_cm"] - TARGET_LATERAL_CM  # line_margin 값을 계산하거나 갱신한다.

    def finish_target(value):  # finish_target 함수를 정의한다.


        value = float(value)  # value 값을 계산하거나 갱신한다.

        if not geometry_state["right_curve_guard_active"]:  # 조건을 확인해 해당 처리 여부를 결정한다.
            return value  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


        if line_margin < 0.0:  # 조건을 확인해 해당 처리 여부를 결정한다.
            if line_margin <= RIGHT_CURVE_GUARD_FULL_CM:  # 조건을 확인해 해당 처리 여부를 결정한다.
                return max(value, RIGHT_CURVE_GUARD_MIN_STEER)  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

            near_ratio = clamp(  # near_ratio 값을 계산하거나 갱신한다.
                (line_margin - RIGHT_CURVE_GUARD_FULL_CM)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                / max(-RIGHT_CURVE_GUARD_FULL_CM, 1e-6),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                0.0, 1.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            safe_guide = (  # safe_guide 값을 계산하거나 갱신한다.
                STEER_STRAIGHT  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                + (RIGHT_CURVE_GUARD_MIN_STEER - STEER_STRAIGHT)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                * near_ratio  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            return max(value, safe_guide)  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


        safe_ratio = clamp(  # safe_ratio 값을 계산하거나 갱신한다.
            line_margin / max(RIGHT_CURVE_GUARD_START_CM, 1e-6),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            0.0, 1.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        return (  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
            RIGHT_CURVE_GUARD_MIN_STEER  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            + (RIGHT_CURVE_GUIDE_STEER - RIGHT_CURVE_GUARD_MIN_STEER)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            * safe_ratio  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


    left_to_right_pending = (  # left_to_right_pending 값을 계산하거나 갱신한다.
        geometry_state["s_right_transition_pending"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and filter_state > STEER_STRAIGHT  # 여러 판정 조건을 이어서 계산한다.
        and target < STEER_STRAIGHT  # 여러 판정 조건을 이어서 계산한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    if left_to_right_pending:  # 조건을 확인해 해당 처리 여부를 결정한다.
        held_target = float(S_RIGHT_TRANSITION_HOLD_STEER)  # held_target 값을 계산하거나 갱신한다.
        return held_target, held_target, previous_curvature  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    right_curve_active = geometry_state["s_right_phase_active"]  # right_curve_active 값을 계산하거나 갱신한다.


    right_curve_started = False  # right_curve_started 값을 계산하거나 갱신한다.

    if not right_curve_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
        target = finish_target(target)  # target 값을 계산하거나 갱신한다.
        return target, target, curvature  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


    if right_curve_started:  # 조건을 확인해 해당 처리 여부를 결정한다.
        target = finish_target(target)  # target 값을 계산하거나 갱신한다.
        return target, target, curvature  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


    direction_changed = (  # direction_changed 값을 계산하거나 갱신한다.
        (target - STEER_STRAIGHT)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        * (filter_state - STEER_STRAIGHT)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        < 0.0  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    if direction_changed:  # 조건을 확인해 해당 처리 여부를 결정한다.
        target = finish_target(target)  # target 값을 계산하거나 갱신한다.
        return target, target, curvature  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


    safety_recovery = (  # safety_recovery 값을 계산하거나 갱신한다.
        line_margin < RIGHT_RECOVERY_START_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and target > filter_state  # 여러 판정 조건을 이어서 계산한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    if safety_recovery:  # 조건을 확인해 해당 처리 여부를 결정한다.
        target = finish_target(target)  # target 값을 계산하거나 갱신한다.
        return target, target, curvature  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    if abs(target - filter_state) <= S_CURVE_TARGET_DEADBAND_COUNTS:  # 조건을 확인해 해당 처리 여부를 결정한다.
        target = finish_target(filter_state)  # target 값을 계산하거나 갱신한다.
        return target, target, curvature  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    filtered = low_pass(  # filtered 값을 계산하거나 갱신한다.
        filter_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        target,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        delta_time,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        S_CURVE_TARGET_FILTER_SEC,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    filtered = finish_target(filtered)  # filtered 값을 계산하거나 갱신한다.
    return filtered, filtered, curvature  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def apply_s_right_edge_assist(raw_steer, geometry_state, points):  # S자 후반 우측 경계 이탈을 방지하는 보조 함수이다.


    if not geometry_state["s_right_phase_active"] or not points:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return raw_steer  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    rightmost_x = max(float(x) for _, x in points)  # rightmost_x 값을 계산하거나 갱신한다.

    if rightmost_x < S_RIGHT_EDGE_START_X_PX:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return raw_steer  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    edge_ratio = clamp(  # edge_ratio 값을 계산하거나 갱신한다.
        (rightmost_x - S_RIGHT_EDGE_START_X_PX)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        / max(S_RIGHT_EDGE_FULL_X_PX - S_RIGHT_EDGE_START_X_PX, 1e-6),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        0.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        1.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    guide_steer = (  # guide_steer 값을 계산하거나 갱신한다.
        S_RIGHT_EDGE_START_STEER  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        + (S_RIGHT_EDGE_FULL_STEER - S_RIGHT_EDGE_START_STEER)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        * edge_ratio  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


    assisted_steer = max(  # assisted_steer 값을 계산하거나 갱신한다.
        S_RIGHT_EDGE_MIN_STEER,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        min(float(raw_steer), guide_steer),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    return int(round(assisted_steer))  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def calculate_marking_metrics(binary_image, horizontal_mask):  # 가로 표시선의 흰색 비율과 픽셀 수를 계산하는 함수이다.


    image_height, image_width = binary_image.shape  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    y_start = int(image_height * MARKING_REGION_START_RATIO)  # y_start 값을 계산하거나 갱신한다.
    y_end = int(image_height * MARKING_REGION_END_RATIO)  # y_end 값을 계산하거나 갱신한다.

    if y_end <= y_start:  # 조건을 확인해 해당 처리 여부를 결정한다.
        y_end = image_height  # y_end 값을 계산하거나 갱신한다.

    x_start = int(image_width * ABSOLUTE_LEFT_RATIO)  # x_start 값을 계산하거나 갱신한다.
    x_end = int(image_width * ABSOLUTE_RIGHT_RATIO)  # x_end 값을 계산하거나 갱신한다.

    white_region = binary_image[y_start:y_end, x_start:x_end]  # white_region 값을 계산하거나 갱신한다.
    horizontal_region = horizontal_mask[y_start:y_end, x_start:x_end]  # horizontal_region 값을 계산하거나 갱신한다.

    if white_region.size == 0:  # 조건을 확인해 해당 처리 여부를 결정한다.
        white_ratio = 0.0  # white_ratio 값을 계산하거나 갱신한다.
    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
        white_ratio = (  # white_ratio 값을 계산하거나 갱신한다.
            float(cv2.countNonZero(white_region)) / float(white_region.size)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    horizontal_pixels = int(cv2.countNonZero(horizontal_region))  # horizontal_pixels 값을 계산하거나 갱신한다.
    horizontal_detected = horizontal_pixels >= HORIZONTAL_MIN_PIXELS  # horizontal_detected 값을 계산하거나 갱신한다.

    return white_ratio, horizontal_pixels, horizontal_detected  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def detect_marking_approach(horizontal_mask):  # 횡단보도 진입 전 가로 표시선을 감지하는 함수이다.


    image_height, image_width = horizontal_mask.shape  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    y_start = int(image_height * MARKING_APPROACH_START_RATIO)  # y_start 값을 계산하거나 갱신한다.
    y_end = int(image_height * MARKING_APPROACH_END_RATIO)  # y_end 값을 계산하거나 갱신한다.
    x_start = int(image_width * ABSOLUTE_LEFT_RATIO)  # x_start 값을 계산하거나 갱신한다.
    x_end = int(image_width * ABSOLUTE_RIGHT_RATIO)  # x_end 값을 계산하거나 갱신한다.

    approach_region = horizontal_mask[y_start:y_end, x_start:x_end]  # approach_region 값을 계산하거나 갱신한다.

    if approach_region.size == 0:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return False  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    approach_pixels = int(cv2.countNonZero(approach_region))  # approach_pixels 값을 계산하거나 갱신한다.
    return approach_pixels >= MARKING_APPROACH_MIN_PIXELS  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def marking_release_boundary_is_valid(geometry):  # 표시선 보호 해제에 필요한 경계 안정성을 확인하는 함수이다.


    if geometry is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return False  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    return (  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
        geometry["point_count"] >= MIN_BOUNDARY_POINTS  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and PLAUSIBLE_LATERAL_MIN_CM  # 여러 판정 조건을 이어서 계산한다.
        <= geometry["lateral_cm"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        <= PLAUSIBLE_LATERAL_MAX_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and abs(geometry["heading_deg"])  # 여러 판정 조건을 이어서 계산한다.
        <= MARKING_RELEASE_MAX_HEADING_DEG  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and abs(geometry["curvature"])  # 여러 판정 조건을 이어서 계산한다.
        <= MARKING_RELEASE_MAX_CURVATURE  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


def limit_marking_steer(raw_steer, geometry_state):  # 횡단보도에서 과도한 조향을 제한하는 함수이다.


    line_margin = (  # line_margin 값을 계산하거나 갱신한다.
        geometry_state["lateral_cm"] - TARGET_LATERAL_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    left_assist_ratio = clamp(  # left_assist_ratio 값을 계산하거나 갱신한다.
        (MARKING_STEER_LEFT_ASSIST_START_CM - line_margin)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        / max(  # 함수 호출 또는 묶음 계산을 시작한다.
            MARKING_STEER_LEFT_ASSIST_START_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            - MARKING_STEER_LEFT_ASSIST_FULL_CM,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            1e-6,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        ),  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        0.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        1.0,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
    left_limit = (  # left_limit 값을 계산하거나 갱신한다.
        MARKING_STEER_LEFT_BASE_COUNTS  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        + (  # 함수 호출 또는 묶음 계산을 시작한다.
            MARKING_STEER_LEFT_MAX_COUNTS  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            - MARKING_STEER_LEFT_BASE_COUNTS  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        * left_assist_ratio  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    return int(round(clamp(  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
        raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        STEER_STRAIGHT - MARKING_STEER_RIGHT_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        STEER_STRAIGHT + left_limit,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.


def count_crosswalk_bars(horizontal_mask):  # 굵고 채워진 횡단보도 막대 수를 세는 함수이다.


    image_height, image_width = horizontal_mask.shape  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    y_start = int(image_height * MARKING_REGION_START_RATIO)  # y_start 값을 계산하거나 갱신한다.
    y_end = int(image_height * MARKING_REGION_END_RATIO)  # y_end 값을 계산하거나 갱신한다.
    x_start = int(image_width * ABSOLUTE_LEFT_RATIO)  # x_start 값을 계산하거나 갱신한다.
    x_end = int(image_width * ABSOLUTE_RIGHT_RATIO)  # x_end 값을 계산하거나 갱신한다.

    marking_region = horizontal_mask[y_start:y_end, x_start:x_end]  # marking_region 값을 계산하거나 갱신한다.

    if marking_region.size == 0:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return 0  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    component_count, _, stats, _ = cv2.connectedComponentsWithStats(  # 함수 호출 또는 묶음 계산을 시작한다.
        marking_region,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        connectivity=8,  # connectivity 값을 계산하거나 갱신한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    valid_bar_count = 0  # valid_bar_count 값을 계산하거나 갱신한다.


    for component_index in range(1, component_count):  # 대상 항목을 하나씩 반복 처리한다.
        component_width = int(  # component_width 값을 계산하거나 갱신한다.
            stats[component_index, cv2.CC_STAT_WIDTH]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        component_height = int(  # component_height 값을 계산하거나 갱신한다.
            stats[component_index, cv2.CC_STAT_HEIGHT]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        component_area = int(  # component_area 값을 계산하거나 갱신한다.
            stats[component_index, cv2.CC_STAT_AREA]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        component_aspect_ratio = (  # component_aspect_ratio 값을 계산하거나 갱신한다.
            component_height / max(component_width, 1)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        component_fill_ratio = (  # component_fill_ratio 값을 계산하거나 갱신한다.
            component_area  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            / max(component_width * component_height, 1)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        if (  # 조건을 확인해 해당 처리 여부를 결정한다.
            component_width >= CROSSWALK_BAR_MIN_WIDTH  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            and component_height >= CROSSWALK_BAR_MIN_HEIGHT  # 여러 판정 조건을 이어서 계산한다.
            and component_aspect_ratio >= CROSSWALK_BAR_MIN_ASPECT_RATIO  # 여러 판정 조건을 이어서 계산한다.
            and component_fill_ratio >= CROSSWALK_BAR_MIN_FILL_RATIO  # 여러 판정 조건을 이어서 계산한다.
        ):  # 아래에 이어질 처리 블록을 시작한다.
            valid_bar_count += 1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    return valid_bar_count  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def update_horizontal_state(clear_frames, detect_since,  # 가로선 마스킹 상태를 갱신하는 함수이다.
                            horizontal_detected, now):  # 아래에 이어질 처리 블록을 시작한다.


    if horizontal_detected:  # 조건을 확인해 해당 처리 여부를 결정한다.
        if detect_since <= 0.0:  # 조건을 확인해 해당 처리 여부를 결정한다.
            detect_since = now  # detect_since 값을 계산하거나 갱신한다.
    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
        detect_since = 0.0  # detect_since 값을 계산하거나 갱신한다.

    background_like = (  # background_like 값을 계산하거나 갱신한다.
        horizontal_detected  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        and detect_since > 0.0  # 여러 판정 조건을 이어서 계산한다.
        and (now - detect_since) >= HORIZONTAL_MAX_CONTINUOUS_SEC  # 여러 판정 조건을 이어서 계산한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    effective_detected = horizontal_detected and not background_like  # effective_detected 값을 계산하거나 갱신한다.

    if effective_detected:  # 조건을 확인해 해당 처리 여부를 결정한다.
        clear_frames = 0  # clear_frames 값을 계산하거나 갱신한다.
    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
        clear_frames += 1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    active = clear_frames < HORIZONTAL_CLEAR_FRAMES  # active 값을 계산하거나 갱신한다.

    return active, clear_frames, detect_since, effective_detected  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def update_crosswalk_state(active, evidence_frames, clear_frames,  # 횡단보도 확정·유지·해제 상태를 갱신하는 함수이다.
                           active_since, block_until,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                           white_ratio, horizontal_detected, now, bar_count,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                           crosswalk_signature_confirmed):  # 아래에 이어질 처리 블록을 시작한다.


    if (crosswalk_signature_confirmed  # 조건을 확인해 해당 처리 여부를 결정한다.
            and white_ratio >= CROSSWALK_WHITE_RATIO  # 여러 판정 조건을 이어서 계산한다.
            and bar_count >= CROSSWALK_MIN_BARS):  # 여러 판정 조건을 이어서 계산한다.
        evidence_frames += 1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
        evidence_frames = 0  # evidence_frames 값을 계산하거나 갱신한다.

    if (not active and now >= block_until  # 조건을 확인해 해당 처리 여부를 결정한다.
            and evidence_frames >= CROSSWALK_CONFIRM_FRAMES):  # 여러 판정 조건을 이어서 계산한다.
        active = True  # active 값을 계산하거나 갱신한다.
        clear_frames = 0  # clear_frames 값을 계산하거나 갱신한다.
        active_since = now  # active_since 값을 계산하거나 갱신한다.

    if active:  # 조건을 확인해 해당 처리 여부를 결정한다.
        still_present = (  # still_present 값을 계산하거나 갱신한다.
            white_ratio >= CROSSWALK_CLEAR_WHITE_RATIO or horizontal_detected  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        if still_present:  # 조건을 확인해 해당 처리 여부를 결정한다.
            clear_frames = 0  # clear_frames 값을 계산하거나 갱신한다.
        else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
            clear_frames += 1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

        timed_out = (  # timed_out 값을 계산하거나 갱신한다.
            active_since > 0.0  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            and (now - active_since) >= CROSSWALK_MAX_ACTIVE_SEC  # 여러 판정 조건을 이어서 계산한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        if clear_frames >= CROSSWALK_CLEAR_FRAMES or timed_out:  # 조건을 확인해 해당 처리 여부를 결정한다.
            active = False  # active 값을 계산하거나 갱신한다.
            evidence_frames = 0  # evidence_frames 값을 계산하거나 갱신한다.
            clear_frames = 0  # clear_frames 값을 계산하거나 갱신한다.
            active_since = 0.0  # active_since 값을 계산하거나 갱신한다.

            if timed_out:  # 조건을 확인해 해당 처리 여부를 결정한다.
                block_until = now + CROSSWALK_REARM_BLOCK_SEC  # block_until 값을 계산하거나 갱신한다.
                print("[MARK-TIMEOUT] 횡단보도 상태를 강제 해제했습니다. "  # 현재 상태를 콘솔에 출력한다.
                      "카메라에 정지된 흰색 물체가 있는지 확인하세요.")  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    return active, evidence_frames, clear_frames, active_since, block_until  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def select_mode(crosswalk_active, horizontal_active, boundary_valid,  # 현재 인식과 제어 상태의 표시 이름을 선택하는 함수이다.
                lost_frames):  # 아래에 이어질 처리 블록을 시작한다.


    if crosswalk_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return "CROSS-TRACK" if boundary_valid else "CROSS-PRED"  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    if horizontal_active and boundary_valid:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return "HLINE-MASK"  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    if boundary_valid:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return "TRACK"  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    if lost_frames <= LOST_COMMAND_HOLD_FRAMES:  # 조건을 확인해 해당 처리 여부를 결정한다.
        return "HOLD"  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

    return "LOST"  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def draw_bev_view(bev_image, points, geometry, calibration):  # BEV 디버그 화면을 그리는 함수이다.


    view = bev_image.copy()  # view 값을 계산하거나 갱신한다.
    pixels_per_cm = calibration["pixels_per_cm"]  # pixels_per_cm 값을 계산하거나 갱신한다.

    target_x = int(round(  # target_x 값을 계산하거나 갱신한다.
        calibration["vehicle_x"] + TARGET_LATERAL_CM * pixels_per_cm  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    ))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    cv2.line(view, (target_x, 0), (target_x, view.shape[0]), (0, 255, 0), 2)  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
    cv2.putText(view, "TARGET", (target_x + 6, 46),  # 디버그 화면에 상태 문자를 표시한다.
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)  # OpenCV 영상 처리 또는 화면 동작을 수행한다.

    vehicle_x = int(round(calibration["vehicle_x"]))  # vehicle_x 값을 계산하거나 갱신한다.
    cv2.line(view, (vehicle_x, 0), (vehicle_x, view.shape[0]),  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
             (255, 0, 255), 2)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    cv2.putText(view, "CAR", (vehicle_x + 6, 68),  # 디버그 화면에 상태 문자를 표시한다.
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)  # OpenCV 영상 처리 또는 화면 동작을 수행한다.

    for y, x in points:  # 대상 항목을 하나씩 반복 처리한다.
        cv2.circle(view, (int(round(x)), int(y)), 3, (0, 255, 255), -1)  # OpenCV 영상 처리 또는 화면 동작을 수행한다.

    if geometry is not None:  # 조건을 확인해 해당 처리 여부를 결정한다.
        curve = []  # curve 값을 계산하거나 갱신한다.

        for y in range(BEV_SCAN_BOTTOM_PX, BEV_SCAN_TOP_PX, -6):  # 대상 항목을 하나씩 반복 처리한다.
            x = evaluate_boundary_x(geometry["poly"], y, calibration)  # x 값을 계산하거나 갱신한다.

            if 0 <= x < view.shape[1]:  # 조건을 확인해 해당 처리 여부를 결정한다.
                curve.append((int(round(x)), int(y)))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

        if len(curve) >= 2:  # 조건을 확인해 해당 처리 여부를 결정한다.
            cv2.polylines(view, [np.asarray(curve, np.int32)], False,  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
                          (0, 140, 255), 2)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    cv2.putText(view, "BEV", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,  # 디버그 화면에 상태 문자를 표시한다.
                (0, 255, 0), 2)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    return view  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def draw_main_view(  # 카메라 디버그 정보를 그리는 함수이다.
        roi, mode, geometry_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        raw_steer, steer_command, drive_speed,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        white_ratio, horizontal_pixels,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        lost_frames, point_count,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        heading_error, integral_term):  # 아래에 이어질 처리 블록을 시작한다.


    lateral_error = geometry_state["lateral_cm"] - TARGET_LATERAL_CM  # lateral_error 값을 계산하거나 갱신한다.

    cv2.putText(  # 디버그 화면에 상태 문자를 표시한다.
        roi,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        "MODE:%s P:%d SP:%d WR:%.3f HP:%d LOST:%d" % (  # 함수 호출 또는 묶음 계산을 시작한다.
            mode,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            point_count,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            int(geometry_state["span_cm"]),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            white_ratio,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            horizontal_pixels,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            lost_frames,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        ),  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 2,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    cv2.putText(  # 디버그 화면에 상태 문자를 표시한다.
        roi,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        "E:%+.1f PE:%+.1f K:%+.5f IN:%+.1f" % (  # 함수 호출 또는 묶음 계산을 시작한다.
            lateral_error,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            heading_error,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            geometry_state["curvature"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            integral_term,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        ),  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        (8, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 255, 255), 2,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    cv2.putText(  # 디버그 화면에 상태 문자를 표시한다.
        roi,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        "TGT:%d STR:%d SPD:%d" % (  # 함수 호출 또는 묶음 계산을 시작한다.
            raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            steer_command,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            drive_speed,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        ),  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
        (8, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


def build_combined_view(tracking_image, bev_image):  # 카메라 화면과 BEV 화면을 하나로 결합하는 함수이다.


    tracking_view = tracking_image.copy()  # tracking_view 값을 계산하거나 갱신한다.
    bev_view = bev_image.copy()  # bev_view 값을 계산하거나 갱신한다.

    height = max(tracking_view.shape[0], bev_view.shape[0])  # height 값을 계산하거나 갱신한다.

    canvas = np.full(  # canvas 값을 계산하거나 갱신한다.
        (height, tracking_view.shape[1] + bev_view.shape[1] + 4, 3),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
        60, dtype=np.uint8,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

    canvas[:tracking_view.shape[0], :tracking_view.shape[1]] = tracking_view  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    canvas[:bev_view.shape[0], tracking_view.shape[1] + 4:] = bev_view  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    return canvas  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def wait_for_start(camera_environment, camera_channel, calibration):  # 사용자의 출발 키 입력을 기다리는 함수이다.


    print("")  # 현재 상태를 콘솔에 출력한다.
    print("STAND BY: 차량을 차로와 평행하게 정렬하세요.")  # 현재 상태를 콘솔에 출력한다.
    print("BEV 창에서 주황색 곡선이 우측 실선을 따라가면 's' 키를 누르세요.")  # 현재 상태를 콘솔에 출력한다.
    print("초록 세로선이 목표 위치, 자홍 세로선이 차량 중심선입니다.")  # 현재 상태를 콘솔에 출력한다.
    print("'q' 키를 누르면 종료합니다.")  # 현재 상태를 콘솔에 출력한다.
    print("")  # 현재 상태를 콘솔에 출력한다.

    while True:  # 조건이 유지되는 동안 처리를 반복한다.
        _, frame = camera_environment.camera_read(camera_channel)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

        if frame is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
            if cv2.waitKey(1) & 0xFF == STOP_KEY:  # 조건을 확인해 해당 처리 여부를 결정한다.
                return False  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.
            continue  # 남은 처리를 건너뛰고 다음 반복으로 이동한다.

        roi = frame[ROI_Y_START:ROI_Y_END, :].copy()  # roi 값을 계산하거나 갱신한다.

        if roi.size == 0:  # 조건을 확인해 해당 처리 여부를 결정한다.
            raise RuntimeError("ROI를 만들 수 없습니다.")  # 계속 실행할 수 없는 상태를 오류로 알린다.

        binary = preprocess_roi(roi)  # binary 값을 계산하거나 갱신한다.
        lane_binary, _ = remove_horizontal_markings(binary)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

        bev_binary = warp_to_bev(lane_binary, calibration)  # bev_binary 값을 계산하거나 갱신한다.
        bev_colour = warp_to_bev(roi, calibration)  # bev_colour 값을 계산하거나 갱신한다.

        points = detect_bev_boundary(bev_binary, None, calibration)  # points 값을 계산하거나 갱신한다.
        geometry = fit_boundary(points, calibration)  # geometry 값을 계산하거나 갱신한다.

        bev_view = draw_bev_view(bev_colour, points, geometry, calibration)  # bev_view 값을 계산하거나 갱신한다.

        if geometry is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
            message = "STAND BY  PTS:%d  (검출 부족)" % len(points)  # message 값을 계산하거나 갱신한다.
        else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
            message = "STAND BY  PTS:%d  LAT:%+.1fcm  PSI:%+.1fdeg" % (  # message 값을 계산하거나 갱신한다.
                len(points), geometry["lateral_cm"], geometry["heading_deg"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        cv2.putText(roi, message, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.52,  # 디버그 화면에 상태 문자를 표시한다.
                    (0, 0, 255), 2)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

        cv2.imshow("Phase1-66 Standby", build_combined_view(roi, bev_view))  # OpenCV 영상 처리 또는 화면 동작을 수행한다.

        key = cv2.waitKey(1) & 0xFF  # key 값을 계산하거나 갱신한다.

        if key == START_KEY:  # 조건을 확인해 해당 처리 여부를 결정한다.
            cv2.destroyWindow("Phase1-66 Standby")  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
            return True  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

        if key == STOP_KEY:  # 조건을 확인해 해당 처리 여부를 결정한다.
            return False  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.


def main():  # 카메라 인식부터 차량 명령까지 전체 주행 루프를 실행하는 함수이다.


    print("Phase1-66: Phase1-65 주행 + 횡단보도 조향 래치")  # 현재 상태를 콘솔에 출력한다.
    print("OpenCV Version: %s" % cv2.__version__)  # 현재 상태를 콘솔에 출력한다.

    calibration = load_calibration()  # calibration 값을 계산하거나 갱신한다.

    print("BEV 캘리브레이션 출처: %s" % calibration["origin"])  # 현재 상태를 콘솔에 출력한다.
    print("  스케일 %.2f px/cm, 차량 기준점 BEV (%.1f, %.1f)"  # 현재 상태를 콘솔에 출력한다.
          % (calibration["pixels_per_cm"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             calibration["vehicle_x"], calibration["vehicle_y"]))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    print("  목표 횡거리 %.1f cm" % TARGET_LATERAL_CM)  # 현재 상태를 콘솔에 출력한다.
    print("  FF %.0f counts/(1/cm), KE %.2f counts/cm, KPSI %.2f counts/deg"  # 현재 상태를 콘솔에 출력한다.
          % (FF_COUNTS_PER_CURVATURE, KE_COUNTS_PER_CM,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             KPSI_COUNTS_PER_DEG))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    print("  조향 트림 %+.1f counts (참 직진 서보 %.0f)"  # 현재 상태를 콘솔에 출력한다.
          % (STEER_TRIM_COUNTS, STEER_STRAIGHT))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    print("  헤딩 기하보정 C = %.1f cm, 곡률 상한 %.4f"  # 현재 상태를 콘솔에 출력한다.
          % (HEADING_KINEMATIC_CM, CURVATURE_LIMIT_ABS))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    print("  예측 우조향 가드 %.0f~%.0f cm, 좌복구 %.0f~%.0f cm (최대 %.1f counts)"  # 현재 상태를 콘솔에 출력한다.
          % (RIGHT_GUARD_START_CM, RIGHT_GUARD_FULL_CM,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             RIGHT_RECOVERY_START_CM, RIGHT_RECOVERY_FULL_CM,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             RIGHT_RECOVERY_MAX_COUNTS))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    print("  횡단보도 속도 상한 %d, 녹색 보조 상한 %d"  # 현재 상태를 콘솔에 출력한다.
          % (calculate_crosswalk_speed_limit(),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             GREEN_FALLBACK_SPEED_LIMIT))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
    print("  우굴곡 가드 K>=%.4f, 실제 여유 %.0f~%.0f cm, 속도 상한 %d"  # 현재 상태를 콘솔에 출력한다.
          % (RIGHT_CURVE_GUARD_CURVATURE,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             RIGHT_CURVE_GUARD_START_CM,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             RIGHT_CURVE_GUARD_FULL_CM,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             RIGHT_CURVE_SPEED_LIMIT))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

    serial_environment = fl.libARDUINO()  # serial_environment 값을 계산하거나 갱신한다.
    camera_environment = fl.libCAMERA()  # camera_environment 값을 계산하거나 갱신한다.

    comm = None  # comm 값을 계산하거나 갱신한다.
    camera_channel = None  # camera_channel 값을 계산하거나 갱신한다.

    try:  # 오류가 발생할 수 있는 작업을 시작한다.
        comm = serial_environment.init(ARDUINO_PORT, ARDUINO_BAUDRATE)  # comm 값을 계산하거나 갱신한다.

        camera_channel, _ = camera_environment.initial_setting(  # 함수 호출 또는 묶음 계산을 시작한다.
            cam0port=CAMERA_PORT, capnum=1  # cam0port 값을 계산하거나 갱신한다.
        )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

        if camera_channel is None or not camera_channel.isOpened():  # 조건을 확인해 해당 처리 여부를 결정한다.
            raise RuntimeError("카메라 포트 %d를 열 수 없습니다." % CAMERA_PORT)  # 계속 실행할 수 없는 상태를 오류로 알린다.

        send_stop(comm)  # 차량에 정지 명령을 전송한다.

        if not wait_for_start(camera_environment, camera_channel,  # 조건을 확인해 해당 처리 여부를 결정한다.
                              calibration):  # 아래에 이어질 처리 블록을 시작한다.
            return  # 계산하거나 갱신한 결과를 호출한 곳에 반환한다.

        cv2.namedWindow("Phase1-66 Driving View", cv2.WINDOW_AUTOSIZE)  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
        cv2.moveWindow("Phase1-66 Driving View", 0, 0)  # OpenCV 영상 처리 또는 화면 동작을 수행한다.

        steer_state = float(STEER_STRAIGHT)  # steer_state 값을 계산하거나 갱신한다.
        steer_command = STEER_STRAIGHT  # steer_command 값을 계산하거나 갱신한다.
        raw_steer = STEER_STRAIGHT  # raw_steer 값을 계산하거나 갱신한다.

        geometry_state = create_geometry_state()  # geometry_state 값을 계산하거나 갱신한다.
        green_state = create_green_boundary_state()  # green_state 값을 계산하거나 갱신한다.
        previous_poly = None  # previous_poly 값을 계산하거나 갱신한다.

        feedforward = 0.0  # feedforward 값을 계산하거나 갱신한다.
        cross_track = 0.0  # cross_track 값을 계산하거나 갱신한다.
        heading_term = 0.0  # heading_term 값을 계산하거나 갱신한다.
        integral_term = 0.0  # integral_term 값을 계산하거나 갱신한다.
        curve_offset = 0.0  # curve_offset 값을 계산하거나 갱신한다.
        heading_error = 0.0  # heading_error 값을 계산하거나 갱신한다.
        drive_speed = SPEED_MAX  # drive_speed 값을 계산하거나 갱신한다.
        speed_state = float(SPEED_MAX)  # speed_state 값을 계산하거나 갱신한다.
        sent_speed = SPEED_MAX  # sent_speed 값을 계산하거나 갱신한다.
        point_count = 0  # point_count 값을 계산하거나 갱신한다.
        green_point_count = 0  # green_point_count 값을 계산하거나 갱신한다.

        crosswalk_active = False  # crosswalk_active 값을 계산하거나 갱신한다.
        crosswalk_evidence_frames = 0  # crosswalk_evidence_frames 값을 계산하거나 갱신한다.
        crosswalk_clear_frames = 0  # crosswalk_clear_frames 값을 계산하거나 갱신한다.
        crosswalk_active_since = 0.0  # crosswalk_active_since 값을 계산하거나 갱신한다.
        crosswalk_block_until = 0.0  # crosswalk_block_until 값을 계산하거나 갱신한다.

        horizontal_clear_frames = HORIZONTAL_CLEAR_FRAMES  # horizontal_clear_frames 값을 계산하거나 갱신한다.
        horizontal_detect_since = 0.0  # horizontal_detect_since 값을 계산하거나 갱신한다.


        marking_speed_until = 0.0  # marking_speed_until 값을 계산하거나 갱신한다.
        crosswalk_tracking_until = 0.0  # crosswalk_tracking_until 값을 계산하거나 갱신한다.


        crosswalk_was_active = False  # crosswalk_was_active 값을 계산하거나 갱신한다.
        crosswalk_boundary_latched = False  # crosswalk_boundary_latched 값을 계산하거나 갱신한다.
        crosswalk_last_geometry = None  # crosswalk_last_geometry 값을 계산하거나 갱신한다.
        crosswalk_last_geometry_time = 0.0  # crosswalk_last_geometry_time 값을 계산하거나 갱신한다.
        crosswalk_prediction_used = False  # crosswalk_prediction_used 값을 계산하거나 갱신한다.
        marking_steer_active = False  # marking_steer_active 값을 계산하거나 갱신한다.
        marking_steer_active_since = 0.0  # marking_steer_active_since 값을 계산하거나 갱신한다.
        marking_steer_release_frames = 0  # marking_steer_release_frames 값을 계산하거나 갱신한다.
        marking_approach_armed_until = 0.0  # marking_approach_armed_until 값을 계산하거나 갱신한다.
        marking_crosswalk_evidence_frames = 0  # marking_crosswalk_evidence_frames 값을 계산하거나 갱신한다.
        marking_presteer_active = False  # marking_presteer_active 값을 계산하거나 갱신한다.
        marking_presteer_active_since = 0.0  # marking_presteer_active_since 값을 계산하거나 갱신한다.
        marking_presteer_reference = float(STEER_STRAIGHT)  # marking_presteer_reference 값을 계산하거나 갱신한다.
        green_control_geometry = None  # green_control_geometry 값을 계산하거나 갱신한다.
        last_valid_geometry = None  # last_valid_geometry 값을 계산하거나 갱신한다.
        crosswalk_lost_since = 0.0  # crosswalk_lost_since 값을 계산하거나 갱신한다.
        s_right_lost_since = 0.0  # s_right_lost_since 값을 계산하거나 갱신한다.
        s_curve_target_state = float(STEER_STRAIGHT)  # s_curve_target_state 값을 계산하거나 갱신한다.
        s_curve_previous_curvature = 0.0  # s_curve_previous_curvature 값을 계산하거나 갱신한다.

        white_ratio = 0.0  # white_ratio 값을 계산하거나 갱신한다.
        horizontal_pixels = 0  # horizontal_pixels 값을 계산하거나 갱신한다.

        lost_frames = 0  # lost_frames 값을 계산하거나 갱신한다.
        camera_failures = 0  # camera_failures 값을 계산하거나 갱신한다.

        last_control_time = time.monotonic()  # last_control_time 값을 계산하거나 갱신한다.
        last_debug_time = 0.0  # last_debug_time 값을 계산하거나 갱신한다.

        while True:  # 조건이 유지되는 동안 처리를 반복한다.
            _, frame = camera_environment.camera_read(camera_channel)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

            if frame is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
                camera_failures += 1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

                if camera_failures >= MAX_CAMERA_FAILURES:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    send_stop(comm)  # 차량에 정지 명령을 전송한다.

                if cv2.waitKey(1) & 0xFF == STOP_KEY:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    break  # 현재 반복문을 종료한다.

                continue  # 남은 처리를 건너뛰고 다음 반복으로 이동한다.

            camera_failures = 0  # camera_failures 값을 계산하거나 갱신한다.

            now = time.monotonic()  # now 값을 계산하거나 갱신한다.
            delta_time = float(clamp(  # delta_time 값을 계산하거나 갱신한다.
                now - last_control_time, MIN_CONTROL_DT, MAX_CONTROL_DT  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            ))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            last_control_time = now  # last_control_time 값을 계산하거나 갱신한다.

            roi = frame[ROI_Y_START:ROI_Y_END, :].copy()  # roi 값을 계산하거나 갱신한다.

            if roi.size == 0:  # 조건을 확인해 해당 처리 여부를 결정한다.
                raise RuntimeError("주행 ROI를 만들 수 없습니다.")  # 계속 실행할 수 없는 상태를 오류로 알린다.

            binary = preprocess_roi(roi)  # binary 값을 계산하거나 갱신한다.
            lane_binary, horizontal_mask = remove_horizontal_markings(binary)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

            bar_count = count_crosswalk_bars(horizontal_mask)  # bar_count 값을 계산하거나 갱신한다.

            (white_ratio,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             horizontal_pixels,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             horizontal_detected) = calculate_marking_metrics(  # 함수 호출 또는 묶음 계산을 시작한다.
                binary, horizontal_mask  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            approach_horizontal_detected = detect_marking_approach(  # approach_horizontal_detected 값을 계산하거나 갱신한다.
                horizontal_mask  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


            if approach_horizontal_detected and not marking_steer_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
                marking_approach_armed_until = (  # marking_approach_armed_until 값을 계산하거나 갱신한다.
                    now + MARKING_APPROACH_ARM_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            marking_approach_armed = (  # marking_approach_armed 값을 계산하거나 갱신한다.
                now <= marking_approach_armed_until  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            marking_crosswalk_signature = (  # marking_crosswalk_signature 값을 계산하거나 갱신한다.
                marking_approach_armed  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                and bar_count >= CROSSWALK_MIN_BARS  # 여러 판정 조건을 이어서 계산한다.
                and white_ratio >= MARKING_CROSSWALK_MIN_WHITE_RATIO  # 여러 판정 조건을 이어서 계산한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            if marking_crosswalk_signature:  # 조건을 확인해 해당 처리 여부를 결정한다.
                marking_crosswalk_evidence_frames += 1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
                marking_crosswalk_evidence_frames = 0  # marking_crosswalk_evidence_frames 값을 계산하거나 갱신한다.

            marking_crosswalk_confirmed = (  # marking_crosswalk_confirmed 값을 계산하거나 갱신한다.
                marking_crosswalk_evidence_frames  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                >= MARKING_CROSSWALK_CONFIRM_FRAMES  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


            if (  # 조건을 확인해 해당 처리 여부를 결정한다.
                not marking_presteer_active  # 여러 판정 조건을 이어서 계산한다.
                and not marking_steer_active  # 여러 판정 조건을 이어서 계산한다.
                and marking_crosswalk_confirmed  # 여러 판정 조건을 이어서 계산한다.
            ):  # 아래에 이어질 처리 블록을 시작한다.
                marking_presteer_active = True  # marking_presteer_active 값을 계산하거나 갱신한다.
                marking_presteer_active_since = now  # marking_presteer_active_since 값을 계산하거나 갱신한다.
                marking_presteer_reference = float(clamp(  # marking_presteer_reference 값을 계산하거나 갱신한다.
                    steer_command,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    STEER_STRAIGHT - MARKING_PRESTEER_RIGHT_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    STEER_STRAIGHT + MARKING_PRESTEER_LEFT_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                ))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

            if (  # 조건을 확인해 해당 처리 여부를 결정한다.
                marking_presteer_active  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                and (  # 여러 판정 조건을 이어서 계산한다.
                    marking_steer_active  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    or now - marking_presteer_active_since  # 여러 판정 조건을 이어서 계산한다.
                    >= MARKING_PRESTEER_MAX_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            ):  # 아래에 이어질 처리 블록을 시작한다.
                marking_presteer_active = False  # marking_presteer_active 값을 계산하거나 갱신한다.

            (horizontal_active,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             horizontal_clear_frames,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             horizontal_detect_since,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             effective_horizontal) = update_horizontal_state(  # 함수 호출 또는 묶음 계산을 시작한다.
                horizontal_clear_frames, horizontal_detect_since,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                horizontal_detected, now,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            (crosswalk_active,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             crosswalk_evidence_frames,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             crosswalk_clear_frames,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             crosswalk_active_since,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
             crosswalk_block_until) = update_crosswalk_state(  # 함수 호출 또는 묶음 계산을 시작한다.
                crosswalk_active, crosswalk_evidence_frames,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                crosswalk_clear_frames, crosswalk_active_since,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                crosswalk_block_until, white_ratio, effective_horizontal,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                 now, bar_count, marking_crosswalk_confirmed  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
             )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            if crosswalk_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
                marking_speed_until = (  # marking_speed_until 값을 계산하거나 갱신한다.
                    now + MARKING_SPEED_RELEASE_HOLD_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                crosswalk_tracking_until = (  # crosswalk_tracking_until 값을 계산하거나 갱신한다.
                    now + CROSSWALK_TRACK_RELEASE_HOLD_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            speed_protection_active = now <= marking_speed_until  # speed_protection_active 값을 계산하거나 갱신한다.
            tracking_protection_active = now <= crosswalk_tracking_until  # tracking_protection_active 값을 계산하거나 갱신한다.
            crosswalk_started = (  # crosswalk_started 값을 계산하거나 갱신한다.
                crosswalk_active and not crosswalk_was_active  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            marking_visible = (  # marking_visible 값을 계산하거나 갱신한다.
                approach_horizontal_detected  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                or effective_horizontal  # 여러 판정 조건을 이어서 계산한다.
                or crosswalk_active  # 여러 판정 조건을 이어서 계산한다.
                or white_ratio >= CROSSWALK_CLEAR_WHITE_RATIO  # 여러 판정 조건을 이어서 계산한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


            if (  # 조건을 확인해 해당 처리 여부를 결정한다.
                not marking_steer_active  # 여러 판정 조건을 이어서 계산한다.
                and (  # 여러 판정 조건을 이어서 계산한다.
                    marking_crosswalk_confirmed  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    or crosswalk_started  # 여러 판정 조건을 이어서 계산한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                and crosswalk_entry_is_qualified(geometry_state)  # 여러 판정 조건을 이어서 계산한다.
            ):  # 아래에 이어질 처리 블록을 시작한다.
                marking_steer_active = True  # marking_steer_active 값을 계산하거나 갱신한다.
                marking_steer_active_since = now  # marking_steer_active_since 값을 계산하거나 갱신한다.
                marking_steer_release_frames = 0  # marking_steer_release_frames 값을 계산하거나 갱신한다.
                marking_approach_armed_until = 0.0  # marking_approach_armed_until 값을 계산하거나 갱신한다.
                marking_crosswalk_evidence_frames = 0  # marking_crosswalk_evidence_frames 값을 계산하거나 갱신한다.
                marking_presteer_active = False  # marking_presteer_active 값을 계산하거나 갱신한다.
                crosswalk_boundary_latched = True  # crosswalk_boundary_latched 값을 계산하거나 갱신한다.
                crosswalk_last_geometry = copy_geometry(  # crosswalk_last_geometry 값을 계산하거나 갱신한다.
                    last_valid_geometry  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                crosswalk_last_geometry_time = now  # crosswalk_last_geometry_time 값을 계산하거나 갱신한다.
                green_control_geometry = copy_geometry(  # green_control_geometry 값을 계산하거나 갱신한다.
                    last_valid_geometry  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            if marking_steer_active and crosswalk_started:  # 조건을 확인해 해당 처리 여부를 결정한다.
                crosswalk_last_geometry = copy_geometry(  # crosswalk_last_geometry 값을 계산하거나 갱신한다.
                    crosswalk_last_geometry  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    if crosswalk_last_geometry is not None  # 조건을 확인해 해당 처리 여부를 결정한다.
                    else last_valid_geometry  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                crosswalk_last_geometry_time = now  # crosswalk_last_geometry_time 값을 계산하거나 갱신한다.

                if green_control_geometry is None:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    green_control_geometry = copy_geometry(  # green_control_geometry 값을 계산하거나 갱신한다.
                        last_valid_geometry  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            green_tracking_active = (  # green_tracking_active 값을 계산하거나 갱신한다.
                marking_steer_active  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                and crosswalk_boundary_latched  # 여러 판정 조건을 이어서 계산한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            if not green_tracking_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
                crosswalk_boundary_latched = False  # crosswalk_boundary_latched 값을 계산하거나 갱신한다.
                crosswalk_last_geometry = None  # crosswalk_last_geometry 값을 계산하거나 갱신한다.
                crosswalk_last_geometry_time = 0.0  # crosswalk_last_geometry_time 값을 계산하거나 갱신한다.
                green_control_geometry = None  # green_control_geometry 값을 계산하거나 갱신한다.

            crosswalk_was_active = crosswalk_active  # crosswalk_was_active 값을 계산하거나 갱신한다.


            if not tracking_protection_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
                crosswalk_lost_since = 0.0  # crosswalk_lost_since 값을 계산하거나 갱신한다.


            bev_binary = warp_to_bev(lane_binary, calibration)  # bev_binary 값을 계산하거나 갱신한다.
            bev_colour = warp_to_bev(roi, calibration)  # bev_colour 값을 계산하거나 갱신한다.

            points = detect_bev_boundary(bev_binary, previous_poly,  # points 값을 계산하거나 갱신한다.
                                         calibration)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            candidate_geometry = fit_boundary(points, calibration)  # candidate_geometry 값을 계산하거나 갱신한다.

            green_points = detect_green_boundary(bev_colour)  # green_points 값을 계산하거나 갱신한다.
            raw_green_geometry = fit_boundary(green_points, calibration)  # raw_green_geometry 값을 계산하거나 갱신한다.
            green_point_count = len(green_points)  # green_point_count 값을 계산하거나 갱신한다.


            update_green_boundary_state(  # 함수 호출 또는 묶음 계산을 시작한다.
                green_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                (  # 함수 호출 또는 묶음 계산을 시작한다.
                    candidate_geometry  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    if candidate_geometry is not None  # 조건을 확인해 해당 처리 여부를 결정한다.
                    and not speed_protection_active  # 여러 판정 조건을 이어서 계산한다.
                    and not green_tracking_active  # 여러 판정 조건을 이어서 계산한다.
                    else None  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                ),  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                raw_green_geometry,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                delta_time,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            corrected_green_geometry = correct_green_geometry(  # corrected_green_geometry 값을 계산하거나 갱신한다.
                raw_green_geometry, green_state["offset_cm"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            green_boundary_safe = (  # green_boundary_safe 값을 계산하거나 갱신한다.
                green_tracking_active  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                and green_state["ready"]  # 여러 판정 조건을 이어서 계산한다.
                and green_geometry_is_safe(  # 여러 판정 조건을 이어서 계산한다.
                    corrected_green_geometry, geometry_state  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            geometry = None  # geometry 값을 계산하거나 갱신한다.
            boundary_valid = False  # boundary_valid 값을 계산하거나 갱신한다.
            green_fallback_used = False  # green_fallback_used 값을 계산하거나 갱신한다.
            crosswalk_prediction_used = False  # crosswalk_prediction_used 값을 계산하거나 갱신한다.
            point_count = len(points)  # point_count 값을 계산하거나 갱신한다.


            if marking_steer_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
                marking_steer_elapsed = max(  # marking_steer_elapsed 값을 계산하거나 갱신한다.
                    0.0, now - marking_steer_active_since  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                if marking_steer_elapsed >= MARKING_STEER_MAX_SEC:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    marking_steer_active = False  # marking_steer_active 값을 계산하거나 갱신한다.
                    marking_steer_release_frames = 0  # marking_steer_release_frames 값을 계산하거나 갱신한다.

                elif (  # 앞 조건이 아니면 다음 조건을 확인한다.
                    marking_steer_elapsed < MARKING_STEER_MIN_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    or marking_visible  # 여러 판정 조건을 이어서 계산한다.
                ):  # 아래에 이어질 처리 블록을 시작한다.
                    marking_steer_release_frames = 0  # marking_steer_release_frames 값을 계산하거나 갱신한다.

                elif marking_release_boundary_is_valid(  # 앞 조건이 아니면 다음 조건을 확인한다.
                    candidate_geometry  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                ):  # 아래에 이어질 처리 블록을 시작한다.
                    marking_steer_release_frames += 1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

                    if (  # 조건을 확인해 해당 처리 여부를 결정한다.
                        marking_steer_release_frames  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                        >= MARKING_STEER_RELEASE_FRAMES  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    ):  # 아래에 이어질 처리 블록을 시작한다.
                        marking_steer_active = False  # marking_steer_active 값을 계산하거나 갱신한다.
                        marking_steer_release_frames = 0  # marking_steer_release_frames 값을 계산하거나 갱신한다.

                else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
                    marking_steer_release_frames = 0  # marking_steer_release_frames 값을 계산하거나 갱신한다.

            if not green_tracking_active:  # 조건을 확인해 해당 처리 여부를 결정한다.

                geometry = candidate_geometry  # geometry 값을 계산하거나 갱신한다.
                boundary_valid = geometry is not None  # boundary_valid 값을 계산하거나 갱신한다.

            elif green_boundary_safe:  # 앞 조건이 아니면 다음 조건을 확인한다.


                geometry = stabilize_green_geometry(  # geometry 값을 계산하거나 갱신한다.
                    corrected_green_geometry,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    green_control_geometry,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    delta_time,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                green_control_geometry = copy_geometry(geometry)  # green_control_geometry 값을 계산하거나 갱신한다.
                boundary_valid = True  # boundary_valid 값을 계산하거나 갱신한다.
                green_fallback_used = True  # green_fallback_used 값을 계산하거나 갱신한다.
                crosswalk_last_geometry = copy_geometry(geometry)  # crosswalk_last_geometry 값을 계산하거나 갱신한다.
                crosswalk_last_geometry_time = now  # crosswalk_last_geometry_time 값을 계산하거나 갱신한다.

            elif (  # 앞 조건이 아니면 다음 조건을 확인한다.
                crosswalk_last_geometry is not None  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                and (now - crosswalk_last_geometry_time)  # 여러 판정 조건을 이어서 계산한다.
                <= CROSSWALK_BOUNDARY_PREDICT_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            ):  # 아래에 이어질 처리 블록을 시작한다.


                geometry = copy_geometry(crosswalk_last_geometry)  # geometry 값을 계산하거나 갱신한다.
                boundary_valid = True  # boundary_valid 값을 계산하거나 갱신한다.
                crosswalk_prediction_used = True  # crosswalk_prediction_used 값을 계산하거나 갱신한다.

            elif candidate_geometry is not None:  # 앞 조건이 아니면 다음 조건을 확인한다.


                geometry = candidate_geometry  # geometry 값을 계산하거나 갱신한다.
                boundary_valid = True  # boundary_valid 값을 계산하거나 갱신한다.


            if (boundary_valid  # 조건을 확인해 해당 처리 여부를 결정한다.
                    and geometry["lateral_cm"] < BOUNDARY_MIN_LATERAL_CM):  # 여러 판정 조건을 이어서 계산한다.
                boundary_valid = False  # boundary_valid 값을 계산하거나 갱신한다.

            if boundary_valid:  # 조건을 확인해 해당 처리 여부를 결정한다.
                lost_frames = 0  # lost_frames 값을 계산하거나 갱신한다.
                crosswalk_lost_since = 0.0  # crosswalk_lost_since 값을 계산하거나 갱신한다.
                s_right_lost_since = 0.0  # s_right_lost_since 값을 계산하거나 갱신한다.
                previous_poly = geometry["poly"]  # previous_poly 값을 계산하거나 갱신한다.
                last_valid_geometry = copy_geometry(geometry)  # last_valid_geometry 값을 계산하거나 갱신한다.

                update_geometry_state(geometry_state, geometry, delta_time)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

                (raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                 feedforward,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                 cross_track,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                 heading_term,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                 heading_error,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                 integral_term,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                 curve_offset) = calculate_steering(geometry_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                                                    delta_time)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

                (raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                 s_curve_target_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                 s_curve_previous_curvature) = filter_s_curve_target(  # 함수 호출 또는 묶음 계산을 시작한다.
                    raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    geometry_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    s_curve_target_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    s_curve_previous_curvature,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    delta_time,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                raw_steer = apply_s_right_edge_assist(  # raw_steer 값을 계산하거나 갱신한다.
                    raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    geometry_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    points,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


                if marking_presteer_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    presteer_minimum = max(  # presteer_minimum 값을 계산하거나 갱신한다.
                        STEER_STRAIGHT  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                        - MARKING_PRESTEER_RIGHT_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        marking_presteer_reference  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                        - MARKING_PRESTEER_REFERENCE_BAND_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    presteer_maximum = min(  # presteer_maximum 값을 계산하거나 갱신한다.
                        STEER_STRAIGHT  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                        + MARKING_PRESTEER_LEFT_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        marking_presteer_reference  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                        + MARKING_PRESTEER_REFERENCE_BAND_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    raw_steer = int(round(clamp(  # raw_steer 값을 계산하거나 갱신한다.
                        raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        presteer_minimum,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        presteer_maximum,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.


                if marking_steer_active or green_tracking_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    raw_steer = limit_marking_steer(  # raw_steer 값을 계산하거나 갱신한다.
                        raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        geometry_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                drive_speed = calculate_speed(geometry_state["curvature"])  # drive_speed 값을 계산하거나 갱신한다.


                if (  # 조건을 확인해 해당 처리 여부를 결정한다.
                    geometry_state["s_right_phase_active"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    or geometry_state["curvature"]  # 여러 판정 조건을 이어서 계산한다.
                    >= RIGHT_CURVE_SPEED_CURVATURE  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                ):  # 아래에 이어질 처리 블록을 시작한다.
                    drive_speed = min(  # drive_speed 값을 계산하거나 갱신한다.
                        drive_speed,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        RIGHT_CURVE_SPEED_LIMIT,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                if green_fallback_used and tracking_protection_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    drive_speed = min(  # drive_speed 값을 계산하거나 갱신한다.
                        drive_speed, GREEN_FALLBACK_SPEED_LIMIT  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                if speed_protection_active:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    drive_speed = min(  # drive_speed 값을 계산하거나 갱신한다.
                        drive_speed, calculate_crosswalk_speed_limit()  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    speed_state, sent_speed = update_crosswalk_speed(  # 함수 호출 또는 묶음 계산을 시작한다.
                        drive_speed, speed_state, delta_time  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.

                    speed_state = float(drive_speed)  # speed_state 값을 계산하거나 갱신한다.
                    sent_speed = int(drive_speed)  # sent_speed 값을 계산하거나 갱신한다.


                if green_fallback_used:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    raw_steer = int(round(clamp(  # raw_steer 값을 계산하거나 갱신한다.
                        raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        STEER_STRAIGHT - CROSSWALK_LOST_STEER_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        STEER_STRAIGHT + CROSSWALK_LOST_STEER_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

                steer_state, steer_command = update_steering_command(  # 함수 호출 또는 묶음 계산을 시작한다.
                    raw_steer, steer_state, steer_command, delta_time  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                send_command(comm, DIRECTION_FORWARD,  # 계산된 주행 명령을 차량에 전송한다.
                             sent_speed, steer_command)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

            else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
                lost_frames += 1  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

                if geometry_state["s_right_phase_active"]:  # 조건을 확인해 해당 처리 여부를 결정한다.
                    if s_right_lost_since <= 0.0:  # 조건을 확인해 해당 처리 여부를 결정한다.
                        s_right_lost_since = now  # s_right_lost_since 값을 계산하거나 갱신한다.
                    s_right_lost_elapsed = max(  # s_right_lost_elapsed 값을 계산하거나 갱신한다.
                        0.0, now - s_right_lost_since  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
                    s_right_lost_since = 0.0  # s_right_lost_since 값을 계산하거나 갱신한다.
                    s_right_lost_elapsed = S_RIGHT_LOST_TOTAL_SEC + 1.0  # s_right_lost_elapsed 값을 계산하거나 갱신한다.

                if (  # 조건을 확인해 해당 처리 여부를 결정한다.
                    geometry_state["s_right_phase_active"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    and s_right_lost_elapsed <= S_RIGHT_LOST_STRONG_SEC  # 여러 판정 조건을 이어서 계산한다.
                ):  # 아래에 이어질 처리 블록을 시작한다.

                    safe_s_steer = int(round(clamp(  # safe_s_steer 값을 계산하거나 갱신한다.
                        steer_command,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        S_RIGHT_LOST_STRONG_MIN_STEER,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        S_RIGHT_LOST_STRONG_MAX_STEER,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    steer_state, steer_command = update_steering_command(  # 함수 호출 또는 묶음 계산을 시작한다.
                        safe_s_steer, steer_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        steer_command, delta_time  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    sent_speed = min(  # sent_speed 값을 계산하거나 갱신한다.
                        int(round(speed_state)),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        RIGHT_CURVE_SPEED_LIMIT,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    speed_state = float(sent_speed)  # speed_state 값을 계산하거나 갱신한다.
                    send_command(  # 계산된 주행 명령을 차량에 전송한다.
                        comm, DIRECTION_FORWARD,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        sent_speed, steer_command  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                elif (  # 앞 조건이 아니면 다음 조건을 확인한다.
                    geometry_state["s_right_phase_active"]  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    and s_right_lost_elapsed <= S_RIGHT_LOST_TOTAL_SEC  # 여러 판정 조건을 이어서 계산한다.
                ):  # 아래에 이어질 처리 블록을 시작한다.

                    steer_state, steer_command = update_steering_command(  # 함수 호출 또는 묶음 계산을 시작한다.
                        S_RIGHT_LOST_WEAK_STEER,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        steer_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        steer_command,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        delta_time,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    sent_speed = min(  # sent_speed 값을 계산하거나 갱신한다.
                        int(round(speed_state)),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        RIGHT_CURVE_SPEED_LIMIT,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    speed_state = float(sent_speed)  # speed_state 값을 계산하거나 갱신한다.
                    send_command(  # 계산된 주행 명령을 차량에 전송한다.
                        comm, DIRECTION_FORWARD,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        sent_speed, steer_command  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                elif tracking_protection_active:  # 앞 조건이 아니면 다음 조건을 확인한다.

                    if crosswalk_lost_since <= 0.0:  # 조건을 확인해 해당 처리 여부를 결정한다.
                        crosswalk_lost_since = now  # crosswalk_lost_since 값을 계산하거나 갱신한다.

                    crosswalk_lost_elapsed = max(  # crosswalk_lost_elapsed 값을 계산하거나 갱신한다.
                        0.0, now - crosswalk_lost_since  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                    if (  # 조건을 확인해 해당 처리 여부를 결정한다.
                        crosswalk_lost_elapsed  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                        <= CROSSWALK_LOST_TRANSITION_SEC  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    ):  # 아래에 이어질 처리 블록을 시작한다.
                        safe_steer = int(round(clamp(  # safe_steer 값을 계산하거나 갱신한다.
                            steer_command,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                            STEER_STRAIGHT  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                            - CROSSWALK_LOST_STEER_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                            STEER_STRAIGHT  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                            + CROSSWALK_LOST_STEER_LIMIT_COUNTS,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        )))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
                        safe_steer = STEER_STRAIGHT  # safe_steer 값을 계산하거나 갱신한다.

                    drive_speed = min(  # drive_speed 값을 계산하거나 갱신한다.
                        int(round(speed_state)),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        CROSSWALK_LOST_SPEED,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    speed_state, sent_speed = update_crosswalk_speed(  # 함수 호출 또는 묶음 계산을 시작한다.
                        drive_speed, speed_state, delta_time  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    steer_state, steer_command = update_steering_command(  # 함수 호출 또는 묶음 계산을 시작한다.
                        safe_steer, steer_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        steer_command, delta_time  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                    send_command(comm, DIRECTION_FORWARD,  # 계산된 주행 명령을 차량에 전송한다.
                                 sent_speed, steer_command)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

                elif marking_steer_active or green_tracking_active:  # 앞 조건이 아니면 다음 조건을 확인한다.


                    safe_marking_steer = limit_marking_steer(  # safe_marking_steer 값을 계산하거나 갱신한다.
                        steer_command,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        geometry_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    steer_state, steer_command = update_steering_command(  # 함수 호출 또는 묶음 계산을 시작한다.
                        safe_marking_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        steer_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        steer_command,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        delta_time,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                    sent_speed = int(round(speed_state))  # sent_speed 값을 계산하거나 갱신한다.
                    send_command(  # 계산된 주행 명령을 차량에 전송한다.
                        comm, DIRECTION_FORWARD,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        sent_speed, steer_command  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                elif lost_frames <= LOST_COMMAND_HOLD_FRAMES:  # 앞 조건이 아니면 다음 조건을 확인한다.

                    send_command(comm, DIRECTION_FORWARD, drive_speed, steer_command)  # 계산된 주행 명령을 차량에 전송한다.

                else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.

                    previous_poly = None  # previous_poly 값을 계산하거나 갱신한다.
                    geometry_state["ready"] = False  # geometry_state["ready"] 값을 계산하거나 갱신한다.

                    steer_state, steer_command = update_steering_command(  # 함수 호출 또는 묶음 계산을 시작한다.
                        STEER_STRAIGHT, steer_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        steer_command, delta_time  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                    send_command(comm, DIRECTION_FORWARD, LOST_DRIVE_SPEED, steer_command)  # 계산된 주행 명령을 차량에 전송한다.

            if crosswalk_prediction_used:  # 조건을 확인해 해당 처리 여부를 결정한다.
                mode = (  # mode 값을 계산하거나 갱신한다.
                    "CROSS-PRED"  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    if tracking_protection_active  # 조건을 확인해 해당 처리 여부를 결정한다.
                    else "MARK-PRED"  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            elif green_fallback_used:  # 앞 조건이 아니면 다음 조건을 확인한다.
                mode = (  # mode 값을 계산하거나 갱신한다.
                    "CROSS-GREEN"  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    if tracking_protection_active  # 조건을 확인해 해당 처리 여부를 결정한다.
                    else "MARK-GREEN"  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
            elif tracking_protection_active and not boundary_valid:  # 앞 조건이 아니면 다음 조건을 확인한다.
                mode = "CROSS-LOST"  # mode 값을 계산하거나 갱신한다.
            elif (  # 앞 조건이 아니면 다음 조건을 확인한다.
                (marking_steer_active or green_tracking_active)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                and not boundary_valid  # 여러 판정 조건을 이어서 계산한다.
            ):  # 아래에 이어질 처리 블록을 시작한다.
                mode = "MARK-HOLD"  # mode 값을 계산하거나 갱신한다.
            elif (  # 앞 조건이 아니면 다음 조건을 확인한다.
                marking_presteer_active  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                or (marking_steer_active and not crosswalk_active)  # 여러 판정 조건을 이어서 계산한다.
            ):  # 아래에 이어질 처리 블록을 시작한다.
                mode = "CROSS-PRESTEER"  # mode 값을 계산하거나 갱신한다.
            else:  # 앞 조건에 해당하지 않는 나머지 경우를 처리한다.
                mode = select_mode(  # mode 값을 계산하거나 갱신한다.
                    crosswalk_active, horizontal_active,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    boundary_valid, lost_frames,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.


            draw_main_view(  # 함수 호출 또는 묶음 계산을 시작한다.
                roi,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                mode,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                geometry_state,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                steer_command,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                sent_speed,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                white_ratio,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                horizontal_pixels,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                lost_frames,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                point_count,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                heading_error,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                integral_term,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
            )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            bev_view = draw_bev_view(bev_colour, points, geometry,  # bev_view 값을 계산하거나 갱신한다.
                                     calibration)  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

            cv2.imshow("Phase1-66 Driving View",  # OpenCV 영상 처리 또는 화면 동작을 수행한다.
                       build_combined_view(roi, bev_view))  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.

            if now - last_debug_time >= DEBUG_PRINT_INTERVAL_SEC:  # 조건을 확인해 해당 처리 여부를 결정한다.
                last_debug_time = now  # last_debug_time 값을 계산하거나 갱신한다.

                lateral_error = (  # lateral_error 값을 계산하거나 갱신한다.
                    geometry_state["lateral_cm"] - TARGET_LATERAL_CM  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

                print(  # 현재 상태를 콘솔에 출력한다.
                    "[%-11s] P:%2d SP:%3d WR:%.3f HP:%4d "  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    "E:%+5.1f PE:%+5.1f K:%+.5f IN:%+5.1f "  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    "TGT:%3d STR:%3d SPD:%3d LOST:%2d"  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
                    % (  # 함수 호출 또는 묶음 계산을 시작한다.
                        mode,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        point_count,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        int(geometry_state["span_cm"]),  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        white_ratio,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        horizontal_pixels,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        lateral_error,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        heading_error,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        geometry_state["curvature"],  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        integral_term,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        raw_steer,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        steer_command,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        sent_speed,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                        lost_frames,  # 호출 인자 또는 자료 항목을 이어서 전달한다.
                    )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.
                )  # 앞에서 시작한 호출 또는 자료 구조를 마무리한다.

            if cv2.waitKey(1) & 0xFF == STOP_KEY:  # 조건을 확인해 해당 처리 여부를 결정한다.
                print("종료 키 입력: 차량을 정지합니다.")  # 현재 상태를 콘솔에 출력한다.
                break  # 현재 반복문을 종료한다.

    finally:  # 성공 여부와 관계없이 정리 작업을 수행한다.
        if comm is not None:  # 조건을 확인해 해당 처리 여부를 결정한다.
            try:  # 오류가 발생할 수 있는 작업을 시작한다.
                send_stop(comm)  # 차량에 정지 명령을 전송한다.
                time.sleep(0.1)  # 현재 시간 또는 경과 시간을 확인한다.
                comm.close()  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            except Exception:  # 발생한 오류를 안전하게 처리한다.
                pass  # 이 경우에는 별도 동작 없이 넘어간다.

        if camera_channel is not None:  # 조건을 확인해 해당 처리 여부를 결정한다.
            try:  # 오류가 발생할 수 있는 작업을 시작한다.
                camera_channel.release()  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
            except Exception:  # 발생한 오류를 안전하게 처리한다.
                pass  # 이 경우에는 별도 동작 없이 넘어간다.

        cv2.destroyAllWindows()  # OpenCV 영상 처리 또는 화면 동작을 수행한다.

        print("Phase1-66 프로그램을 안전하게 종료했습니다.")  # 현재 상태를 콘솔에 출력한다.


if __name__ == "__main__":  # 이 파일을 직접 실행한 경우에만 메인 함수를 호출한다.
    main()  # 현재 단계에 필요한 계산 또는 상태 갱신을 수행한다.
