#include <common/msg/body_angles.hpp>
#include <common/msg/head_angles.hpp>
#include <common/srv/get_angles.hpp>
#include <common.hpp>
#include <seumath/math.hpp>
#include "WebotsUtils.hpp"
#include "SimRobot.hpp"
#include <unistd.h>


using namespace std;
using namespace common;
using namespace seumath;
using namespace Eigen;

int main(int argc, char **argv)
{
    string robotName;
    if (!WaitForRobots(robotName)) {
        printf("!!! can not connect to webots\n");
        return 0;
    }
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(robotName + "_controller");

    std::shared_ptr<SimRobot> player = std::make_shared<SimRobot>(robotName);

    rclcpp::Client<common::srv::GetAngles>::SharedPtr client =
        node->create_client<common::srv::GetAngles>(robotName + "/get_angles");
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
    }
    int ret = 0;
    while (rclcpp::ok() && ret >= 0) {
        auto request = std::make_shared<common::srv::GetAngles::Request>();
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = result.get();
            player->mAngles = response->body;
            player->mHAngles = response->head;
        }
        ret = player->myStep();

    }
    rclcpp::shutdown();
    return 0;
}

