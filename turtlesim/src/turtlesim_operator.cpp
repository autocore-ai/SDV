#include <rclcpp/rclcpp.hpp>

class TurtleSimOperator : public rclcpp::Node
{
public:
    TurtleSimOperator() : rclcpp::Node("turtle_sim_operator")
    {
        // twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("output/cmd_vel", 1);
        // pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
        //   "input/pose", 1, std::bind(&MimicNode::poseCallback, this, std::placeholders::_1));
    }

private:
    //   void poseCallback(const turtlesim::msg::Pose::SharedPtr pose)
    //   {
    //     geometry_msgs::msg::Twist twist;
    //     twist.angular.z = pose->angular_velocity;
    //     twist.linear.x = pose->linear_velocity;
    //     twist_pub_->publish(twist);
    //   }

    //   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    //   rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleSimOperator>());
    rclcpp::shutdown();
    return 0;
}
