// 发送移动距离 = 5.0, 接收并打印服务端响应

#include "rclcpp/rclcpp.hpp"
#include "custom_srv_interfaces/srv/move_robot.hpp"
#include <chrono>
#include <thread>

using MoveRobot = custom_srv_interfaces::srv::MoveRobot;
using namespace std::chrono_literals;

class ServiceClientNode : public rclcpp::Node
{
public:
    ServiceClientNode(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点启动", node_name.c_str());
        // 创建客户端，服务名为move_robot_srv
        client_ = this->create_client<MoveRobot>("move_robot_srv");

        // 创建定时器： 启动后1s发送请求（确保服务端已启动）
        timer_ = this->create_wall_timer(1s, std::bind(&ServiceClientNode::send_move_request, this));
    }

    void send_move_request()
    {
        // 取消定时器，防止重复发送请求
        timer_->cancel();

        // 等待服务端上线（超时1s,最多等10次）

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "等待服务时，节点被中断");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务上线中...");
        }

        // 创建请求数据（移动距离5.0）
        auto request = std::make_shared<MoveRobot::Request>();
        request->distance = 5.0;

        // 发送异步请求，并设置回调函数
        auto result_future = client_->async_send_request(request, std::bind(&ServiceClientNode::response_callback, this, std::placeholders::_1));
    }

    void response_callback(rclcpp::Client<MoveRobot>::SharedFuture future)
    {
        // 获取服务端响应
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "服务端响应：移动后位置=%.2f", response->pose);
    }

    private:
    rclcpp::Client<MoveRobot>::SharedPtr client_; // 客户端指针
    rclcpp::TimerBase::SharedPtr timer_; // 定时器指针
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceClientNode>("move_robot_client");
    rclcpp::spin(node); // 保持节点运行
    rclcpp::shutdown();
    return 0;
}