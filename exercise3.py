import Function_Library as fl

if __name__ == "__main__":
    print("자율주행 융합 시뮬레이션 (Print 출력 모드) 시작...")

    # 1. 센서 객체 생성 (종합 라이브러리인 fl에서 모두 불러옴)
    lidar_env = fl.libLIDAR('COM3')  # 본인 환경의 라이다 포트 번호로 변경
    cam_env = fl.libCAMERA()

    # 2. 센서 초기 셋팅
    lidar_env.init()
    ch0, _ = cam_env.initial_setting(cam0port = 1, capnum=1)  # 단일 카메라 사용

    print("시스템 구동 시작! (종료하려면 영상 창에서 'q' 또는 콘솔에서 Ctrl+C 입력)")

    try: #예외처리 문법
        # 3. 메인 주행 루프 (라이다 스캔 주기에 맞춰서 카메라와 동기화)
        for scan in lidar_env.scanning():
            # 1. lidar_env.scanning()이라는 실시간 데이터 상자에서
            # 2. 이번 바퀴에 측정된 거리/각도 데이터 뭉치를 'scan'이라는 이름으로 한 개 꺼낸다.
            # 3. 상자에서 새로운 데이터가 계속 나오는 한, 아래 판단 로직을 무한히 반복(for)한다.

            # [인지 파트 1] 카메라에서 사진 1장을 읽어오고 신호등 색상 파악
            _, frame0 = cam_env.camera_read(ch0)
            color = cam_env.object_detection(frame0, sample=16, print_enable=False)

            # [인지 파트 2] 라이다 데이터에서 정면 거리 추출 (0 ~ 700mm)
            # getDistanceRange 함수는 설정된 범위 안의 [각도, 거리] 배열만 걸러내어 반환합니다.
            obstacle_points = lidar_env.getDistanceRange(scan, 0, 300)

            # ------------------------------------------------------------------
            # [판단 파트] 카메라(color)와 라이다(obstacle_points) 데이터를 융합하여 제어 명령 결정
            # ------------------------------------------------------------------

            if color == "RED":
                # 1순위: 빨간불일 경우 전방 상황과 무관하게 무조건 정지
                print("명령: STOP (사유: 신호등 빨간불 감지)")

            elif color == "GREEN":
                # 2순위: 초록불일 경우 바로 출발하지 않고 라이다로 전방 장애물 확인
                # 노이즈(먼지 등)로 인한 오작동을 막기 위해 700mm 이내에 찍힌 점이 5개 이상일 때만 장애물로 판단
                if len(obstacle_points) > 5:
                    print("명령: STOP (사유: 초록불이지만 전방 700mm 이내 장애물 감지!)")
                else:
                    print("명령: GO (사유: 초록불 & 전방 장애물 없음)")

            # ------------------------------------------------------------------

            # (선택 사항) 카메라 원본 화면을 띄워서 현재 자동차가 무엇을 보고 있는지 확인
            if frame0 is not None: #만약 카메라 프레임(frame0)이 텅 비어있는 상태가 아니라면(is not None):
                                   #그때만 안전하게 화면에 사진을 띄워라(image_show)!
                cam_env.image_show(frame0)

            # 'q' 키를 누르면 루프 탈출
            if cam_env.loop_break():
                break

    except KeyboardInterrupt:
        print("사용자에 의해 프로그램이 강제 종료되었습니다.")

    except Exception as e: #"어떤 에러가 터지든 프로그램이 뻗게 내버려두지 말고, 도대체 '무슨 에러'인지 그 이유를 화면에 알려줘!"라는 뜻의 디버깅(오류 수정)용 방패
        print(f"구동 중 에러 발생 : {e}") #f - string(포맷 스트링)입니다. 따옴표 앞에 f를 붙이면, 중괄호 { } 안에 변수를 쏙 집어넣어 글자와 함께 출력할 수 있습니다.

    finally:
        # 프로그램이 종료될 때 라이다 모터가 계속 돌지 않도록 안전하게 전원 차단
        lidar_env.stop()
        print("라이다 모터를 정지하고 포트를 닫았습니다.")

    #테스트