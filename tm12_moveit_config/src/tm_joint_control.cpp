#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "tm_msgs/srv/set_positions.hpp"

class TmJointControl : public rclcpp::Node
{
public:
    TmJointControl() : Node("joint_trajectory_subscriber")
    {
        subscription_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/tmr_arm_controller/joint_trajectory",
            10,  // Queue size
            std::bind(&TmJointControl::jointTrajectoryCallback, this, std::placeholders::_1)
        );
    }

private:
    bool redundantPoints(const std::vector<double>& pos1, const std::vector<double>& pos2)
    {
        if (pos1.size() != pos2.size())
            return false;

        for (size_t i = 0; i < pos1.size(); ++i)
        {
            if (std::abs(pos1[i] - pos2[i]) > 0.01)  
                return false;
        }

        return true;
    }

    void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {        
        for (const auto& point : msg->points)
        {
            double time_sec = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            
            if (!redundantPoints(prev_positions, point.positions))
            {
                for (size_t i = 0; i < point.positions.size(); ++i)
                {
                    RCLCPP_INFO(this->get_logger(), "Joint %zu: %.2f", i, point.positions[i]);
                }
                prev_positions = point.positions;

                std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("demo_set_positions");
                rclcpp::Client<tm_msgs::srv::SetPositions>::SharedPtr client =
                    node->create_client<tm_msgs::srv::SetPositions>("set_positions");
                
                auto request = std::make_shared<tm_msgs::srv::SetPositions::Request>();
                request->motion_type = tm_msgs::srv::SetPositions::Request::PTP_J;
                request->positions.push_back(msg->points[0].positions[0]);
                request->positions.push_back(msg->points[0].positions[1]);
                request->positions.push_back(msg->points[0].positions[2]);
                request->positions.push_back(msg->points[0].positions[3]);
                request->positions.push_back(msg->points[0].positions[4]);
                request->positions.push_back(msg->points[0].positions[5]);
                request->velocity = 3.0;//rad/s
                request->acc_time = 1.0;
                request->blend_percentage = 10;
                request->fine_goal  = true;

                auto result = client->async_send_request(request);

                
            }
        }
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscription_;
    std::vector<double> prev_positions;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TmJointControl>());
    rclcpp::shutdown();
    return 0;
}