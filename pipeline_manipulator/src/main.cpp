#include "rclcpp/rclcpp.hpp"
#include "pipeline_manipulator/pipeline_fsm.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PipelineFSM>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}