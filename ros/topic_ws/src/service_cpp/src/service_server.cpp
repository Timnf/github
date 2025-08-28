#include "rclcpp/rclcpp.hpp"
#include "custom_srv_interfaces/srv/move_robot.hpp"
#include <chrono>
#include <thread>

using MoveRobot = custom_srv_interfaces::srv::MoveRobot;
using namespace std::chrono_literals;

class ServiceServerNode : public rclcpp::Node
{
public:
    ServiceServerNode(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已启动", node_name.c_str());
        // 创建服务端 服务名”move_robot_srv“, 绑定请求处理回调
        service_ = this->create_service<MoveRobot>("move_robot_srv",
                                                   std::bind(&ServiceServerNode::handle_move_request, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // 请求处理回调函数
    void handle_move_request(const MoveRobot::Request::SharedPtr request,
                             const MoveRobot::Response::SharedPtr response)
    {
        RCLCPP_INFO(this->get_logger(), "收到客户端请求，移动距离=%.2f, 当前位置=%.2f",
                    request->distance, current_pose_);

        // 模拟机器人移动（每步移动剩余距离的0.1， 休眠500ms）
        float target_pose = current_pose_ + request->distance;
        while (std::fabs(target_pose - current_pose_) > 0.01)
        {
            float step = (target_pose - current_pose_) * 0.1;
            current_pose_ += step;
            RCLCPP_INFO(this->get_logger(), "移动中: 当前位置=%.2f", current_pose_);
            std::this_thread::sleep_for(500ms); // 模拟移动延迟
        }

        // 设置响应 最终位置
        response->pose = current_pose_;
        RCLCPP_INFO(this->get_logger(), "响应请求，最终位置=%.2f", response->pose);
    }

    // 声明服务端指针和当前位置变量
    rclcpp::Service<MoveRobot>::SharedPtr service_;
    float current_pose_ = 0.0f; // 机器人当前位置
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceServerNode>("move_robot_server");
    rclcpp::spin(node); // 保持服务端运行
    rclcpp::shutdown();

    return 0;
}
