#include <common/msg/body_angles.hpp>
#include <common/msg/head_angles.hpp>
#include <common/srv/get_angles.hpp>
#include <common.hpp>
#include <seumath/math.hpp>
#include "WebotsUtils.hpp"
#include "SimRobot.hpp"
#include <unistd.h>
#include <chrono>
#include <cstdint>


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
    using GetAnglesFuture = rclcpp::Client<common::srv::GetAngles>::SharedFuture;
    GetAnglesFuture pendingResult;
    bool requestPending = false;
    int64_t pendingRequestId = 0;
    auto pendingSince = std::chrono::steady_clock::now();
    while (rclcpp::ok() && ret >= 0) {
        // Do not block Webots before its first step. Keep one request in
        // flight and give ROS a small window on each cycle, then advance
        // Webots regardless; this avoids the all-controllers startup
        // deadlock while preserving a bounded controller loop.
        if (!requestPending) {
            auto request = std::make_shared<common::srv::GetAngles::Request>();
            auto requestHandle = client->async_send_request(request);
            pendingResult = requestHandle.future.share();
            pendingRequestId = requestHandle.request_id;
            pendingSince = std::chrono::steady_clock::now();
            requestPending = true;
        }
        const auto resultCode = rclcpp::spin_until_future_complete(
            node, pendingResult, std::chrono::milliseconds(1));
        if (resultCode == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = pendingResult.get();
            player->mAngles = response->body;
            player->mHAngles = response->head;
            requestPending = false;
        } else if (resultCode == rclcpp::FutureReturnCode::INTERRUPTED) {
            break;
        } else if (std::chrono::steady_clock::now() - pendingSince >
                   std::chrono::seconds(1)) {
            // A lost service reply must not leave the robot frozen while its
            // camera and IMU continue to report healthy samples.
            client->remove_pending_request(pendingRequestId);
            requestPending = false;
            RCLCPP_WARN(node->get_logger(),
                "%s/get_angles timed out; retrying motion request", robotName.c_str());
        }
        ret = player->myStep();
    }
    rclcpp::shutdown();
    return 0;
}
