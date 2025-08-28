// 代码功能每500ms发布一个std_msgs/msg/String的消息。 消息内容“hello world ros2 topic!”

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//定义发布者节点类，继承自rclcpp::Node
class TopicPublisherNode : public rclcpp::Node
{
    public:
    // 构造函数 初始化节点名、创建发布者、创建定时器
    TopicPublisherNode(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已启动", node_name.c_str());
        // 创建发布者，发布话题为"chatter", 队列长度为10
        publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
        // 创建定时器，每500ms调用一次timer_callback函数
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
         std::bind(&TopicPublisherNode::timer_callback, this));
    }

    private:
    // 定时器回调函数，发布消息
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "hello world ros2 topic!";
        RCLCPP_INFO(this->get_logger(), "发布消息: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // 初始化RCLCPP
    auto node = std::make_shared<TopicPublisherNode>("topic_publisher"); // 创建节点
    rclcpp::spin(node); // 循环运行节点，处理回调
    rclcpp::shutdown(); // 关闭RCLCPP
    return 0;
}

