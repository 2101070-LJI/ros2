## //문제:

카메라에서 받은 영상을 ROS2를 통해 발행하는 Publisher 노드를 작성하세요.

주의사항:
- 노드명: camera_pub
- - 토픽명: camera/image
  - - 메시지 타입: sensor_msgs/Image
    - - 카메라 해상도: 640x480 이상
      - - 발행 주기: 30fps 이상
        - - QoS: KeepLast(10)
          - - 카메라 초기화 및 오류 처리 필수
           
            - ## //결과:
