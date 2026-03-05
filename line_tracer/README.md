## //문제: 실습과제 1 - 라인검출 시뮬레이션

패키지명: linedetect_nano(pub.cpp), linedetect_wsl(sub.cpp)

주의사항:
Publisher 노드는 Jetson nano 보드에서 동영상을 입력 받아 영상 토픽을 발행
camera_ros2 패키지의 pub.cpp에서 카메라 대신 동영상에서 입력 받아 발행하는 것으로 수정
Subscriber 노드는 WSL2에서 영상을 구독하여 라인을 검출하는 노드, 영상처리결과를 모니터에 출력
camera_ros2 패키지의 sub.cpp에서 콜백함수 안에 라인 검출코드 추가
linedetect_wsl 패키지에 라인 검출 알고리즘을 구현하고 2개의 동영상(5_lt_cw_100rpm_out.mp4, 7_lt_ccw_100rpm_in.mp4)을 이용하여 시뮬레이션 수행 후 결과를 동영상으로 저장
모든 결과물은 깃허브에 제출, 동영상은 유튜브에 저장하고 깃허브에는 링크만 작성할 것
영상 전송이 너무 느리거나 집에서 작업할 때는 publisher node와 subscriber node 모두를 WSL2에 구현하여 테스트할 것 (카메라가 필요 없으므로 wsl2에서도 동작함)

# 실행 결과
outline검출

https://youtu.be/fm2HYrkYBcg

inline검출

https://youtu.be/VMHc8AGqXhk
