#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class RescueYolo : public rclcpp::Node
{
public:
    RescueYolo() : Node("rescue_yolo")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/detection_result", 10, std::bind(&RescueYolo::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard something!");
        // TODO: Process the YOLO data
    }
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RescueYolo>());
    rclcpp::shutdown();
    return 0;
}
