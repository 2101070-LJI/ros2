## //문제: 

Dynamixel 모터와 카메라를 통합으로 제어하는 ROS2 노드를 작성하세요.

주의사항:
Publisher 노드명 (카메라): campub_7
Subscriber 노드명 (카메라): camsub_wsl_7
Publisher 노드명 (모터): node_dxlpub
Subscriber 노드명 (모터): node_dxlsub
카메라 토픽명: image/compressed_7
모터 토픽명: topic_dxlpub
카메라 루프 주기: 30Hz (Jetson Nano 발행)
모터 루프 주기: 20Hz (WSL 발행)
키 입력: f(전진), b(후진), l(좌), r(우), s/스페이스(정지)

## //결과:
<img width="896" height="500" alt="image" src="https://github.com/user-attachments/assets/7edd4279-af33-47d2-b1a7-8e4d0e73ed7d" />
