## //문제: 실습과제 2

예제 1번의 섭스크라이버 노드를 수정하여 구독한 영상을 화면에 출력하고 동시에 동영상 파일(mp4)로 저장하는 패키지 camera2-2를 작성하라.

주의사항:
실행시 저장을 시작하고 ctrl+c를 누르면 저장을 종료하도록 하라
퍼블리셔 노드는 Jetson 보드에서 camera_ros2 패키지의 pub 노드를 실행하라
Publisher 노드명: campub_7
Subscriber 노드명: camsub_wsl_7
토픽명: image/compressed_7
발행 주기: 10Hz
저장 파일: save.mp4 (codec: X264/avc1)

## //결과:
<img width="1012" height="166" alt="image" src="https://github.com/user-attachments/assets/5c660e46-0d91-4279-b883-0017844f9bfb" />
