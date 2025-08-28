#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

//定义订阅节点类
class TopicSubscriberNode : public rclcpp::Node
{
    public:
    TopicSubscriberNode(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点启动", node_name.c_str());
        //创建订阅者
        subscriber_ = this->create_subscription<std_msgs::msg::String>("chatter", 
            10, std::bind(&TopicSubscriberNode::topic_callback, this, std::placeholders::_1));
    }

    private:
    //话题回调函数：处理订阅到的消息
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "收到消息：%s", msg->data.c_str());
        // 处理消息
        // ...
    }

    // 声明订阅指针
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    //初始化ROS2客户端库
    rclcpp::init(argc, argv);
    //创建节点对象
    auto node = std::make_shared<TopicSubscriberNode>("topic_subscriber_node");
    // 保持节点运行
    rclcpp::spin(node);
    // 关闭ROS2客户端库
    rclcpp::shutdown();
    return 0;
}