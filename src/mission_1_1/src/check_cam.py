import cv2

def test_webcam(index):
    # 카메라 연결 시도
    cap = cv2.VideoCapture(index)
    
    if not cap.isOpened():
        print(f"포트 {index}번을 열 수 없습니다.")
        return False

    print(f"포트 {index}번 연결 성공! 'q'를 누르면 종료합니다.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 가져올 수 없습니다.")
            break
        
        # 화면에 영상 표시
        cv2.imshow(f'Webcam Test - Port {index}', frame)
        
        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

# 4번 포트부터 테스트 (보통 4번이 영상, 5번이 메타데이터일 확률이 높음)
test_webcam(4)