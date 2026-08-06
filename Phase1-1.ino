#include <Car_Library.h> // 주최측 제공 헤더 파일

// [1] 하드웨어 핀 번호 할당
int drive1_IN1 = 8;   // 후륜 좌측
int drive1_IN2 = 9;   
int drive2_IN1 = 6;   // 후륜 우측
int drive2_IN2 = 7;   
int steer_IN1 = 11;    // 전륜 조향
int steer_IN2 = 3;    
int potPin = A5;      // 가변저항 센서 입력

// [2] 통신 수신용 변수
int drive_dir = 0; //후륜모터 전진 후진 정지
int drive_speed = 0;  
int target_angle = 177; // 파이썬이 명령할 목표 조향각 (초기값 정중앙)

// [3] 하드웨어 튜닝 파라미터
int STEER_POWER = 180;  // 조향 모터 회전 파워 (100에서 마찰력을 고려해 약간 상향)
int TOLERANCE = 3;      // 조향 헌팅 방지용 오차 허용 범위

void setup() 
{
  Serial.begin(9600);             
  pinMode(drive1_IN1, OUTPUT);     
  pinMode(drive1_IN2, OUTPUT);
  pinMode(drive2_IN1, OUTPUT);     
  pinMode(drive2_IN2, OUTPUT);
  pinMode(steer_IN1, OUTPUT);
  pinMode(steer_IN2, OUTPUT);
}

void loop() 
{
  // [1단계] 시리얼 통신: 파이썬에서 "구동방향,속도,목표조향각\n" 수신
  if (Serial.available() > 0) { 
    drive_dir = Serial.parseInt();   
    drive_speed = Serial.parseInt(); 
    target_angle = Serial.parseInt(); 
    
    // 버퍼 찌꺼기 정리
    while(Serial.available() > 0 && Serial.read() != '\n'); 
  }

  // [2단계] 가변저항 기반 실시간 조향 피드백 제어 (Non-blocking)
  int current_angle = potentiometer_Read(potPin); // 현재 조향각 읽기
  
  // 현재 각도가 목표 각도와의 오차 범위(TOLERANCE) 밖에 있을 때만 모터 가동
  if (abs(current_angle - target_angle) > TOLERANCE) 
  { 
    if (current_angle < target_angle) 
    {
      motor_forward(steer_IN1, steer_IN2, STEER_POWER); // 한쪽 회전
    } 
    
    else 
    {
      motor_backward(steer_IN1, steer_IN2, STEER_POWER); // 반대 회전
    }
  } 

  else 
  {
    // 목표 각도 도달 시 모터 즉시 정지
    motor_hold(steer_IN1, steer_IN2);
  }

  // [3단계] 자율 구동 후륜 모터 속도 제어
  if (drive_dir == 1) 
  { // 전진
    motor_forward(drive1_IN1, drive1_IN2, drive_speed);
    motor_forward(drive2_IN1, drive2_IN2, drive_speed);
  } 

  else if (drive_dir == 2) 
  { // 후진
    motor_backward(drive1_IN1, drive1_IN2, drive_speed);
    motor_backward(drive2_IN1, drive2_IN2, drive_speed);
  }

  else 
  { // 정지
    motor_hold(drive1_IN1, drive1_IN2);
    motor_hold(drive2_IN1, drive2_IN2);
  }

  delay(20); // 통신 안정화 및 과부하 방지 딜레이
}