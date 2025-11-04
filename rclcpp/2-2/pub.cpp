#include "rclcpp/rclcpp.hpp" // ROS 2 C++ 클라이언트 라이브러리
#include "geometry_msgs/msg/vector3.hpp" // 발행할 메시지 타입
#include <chrono> // 시간 관련 기능을 사용
#include <functional> // std::bind, std::function 등을 사용

using namespace std::chrono_literals; // 100ms와 같은 시간 리터럴 사용을 위함
void callback(rclcpp::Node::SharedPtr node,
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mypub) // 100ms마다 호출되어 메시지를 발행하는 콜백 함수
{
    static auto message = geometry_msgs::msg::Vector3(); // 발행할 Vector3 메시지 객체 생성 및 초기화
    std::cout<<"값 입력: "; // 안내메시지 출력
    std::cin>>message.x; // 값 입력
    std::cin>>message.y; // 값 입력
    std::cin>>message.z; // 값 입력
    RCLCPP_INFO(node->get_logger(), "Publish: %lf, %lf, %lf", message.x, message.y, message.z); // ROS 2 로깅 시스템을 사용하여 현재 발행하는 데이터를 출력
    mypub->publish(message); // 준비된 메시지 객체를 mytopic에 발행
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv); // ROS 2 시스템 초기화
    auto node = std::make_shared<rclcpp::Node>("mynode"); //노드 객체 생성
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // Qos설정
    auto pub = node->create_publisher<geometry_msgs::msg::Vector3>("mytopic", qos_profile); // topic설정
    std::function<void()> fn = std::bind(callback, node, pub); // 콜백 함수
    auto timer = node->create_wall_timer(100ms, fn); // 타이머 생성
    rclcpp::spin(node); // 대기
    rclcpp::shutdown(); // 종료
    return 0;
}