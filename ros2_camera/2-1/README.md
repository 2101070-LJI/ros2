## //문제: 실습과제 1

예제 1번을 수정하여 WSL2의 subscriber 노드에서 영상원본을 그레이영상, 이진영상으로 각각 변환하고 3가지 영상을 모두 출력하는 패키지 camera2-1을 작성하라.

주의사항:
퍼블리셔 노드는 Jetson 보드에서 camera_ros2 패키지의 pub 노드를 실행하라
Publisher 노드명: campub_7
Subscriber 노드명: camsub_wsl_7
토픽명: image/compressed_7
발행 주기: 30Hz
출력: 원본(color), 그레이(gray), 이진(binary) 3가지 영상

## //결과:
<img width="1887" height="405" alt="image" src="https://github.com/user-attachments/assets/f8cbda75-1857-4782-92bb-7ca87794893f" />
