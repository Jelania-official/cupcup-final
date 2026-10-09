// Test-only bounded service wait; compile with UNMODIFIED released SimRobot.
// Sensor sampling, head target feedback and motor transforms stay original.
#include <rclcpp/rclcpp.hpp>
#include <common/srv/get_angles.hpp>
#include "SimRobot.hpp"
#include "WebotsUtils.hpp"
#include "target_trace.hpp"
#include <fstream>
#include <iomanip>

int main(int argc, char **argv) {
    std::string name;
    if (!WaitForRobots(name)) return 2;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared(name + "_controller");
    auto robot = std::make_shared<SimRobot>(name);
    std::ofstream targets;
    const char *output = std::getenv("CUPCUP_MEASURE_OUTPUT");
    const char *color = std::getenv("CUPCUP_MEASURE_COLOR");
    if (output && color && name == std::string(color) + "_1") {
        targets.open(std::string(output) + "/motor_targets.csv");
        if (!targets) return 3;
        targets << std::setprecision(10) << "time,";
        cupcup_measure::header(targets);
    }
    auto client = node->create_client<common::srv::GetAngles>(name + "/get_angles");
    while (rclcpp::ok() && !client->wait_for_service(std::chrono::seconds(1))) {}
    using Future = rclcpp::Client<common::srv::GetAngles>::SharedFuture;
    Future future;
    bool pending = false;
    int64_t requestId = 0;
    auto sentAt = std::chrono::steady_clock::now();
    int result = 0;
    while (rclcpp::ok() && result >= 0) {
        if (!pending) {
            auto handle = client->async_send_request(std::make_shared<common::srv::GetAngles::Request>());
            future = handle.future.share(); requestId = handle.request_id;
            pending = true; sentAt = std::chrono::steady_clock::now();
        }
        const auto code = rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(1));
        if (code == rclcpp::FutureReturnCode::SUCCESS) {
            robot->mAngles = future.get()->body; robot->mHAngles = future.get()->head;
            if (targets.is_open()) {
                targets << robot->getTime(); cupcup_measure::angles(targets, robot->mAngles);
                targets.flush();
            }
            pending = false;
        } else if (code == rclcpp::FutureReturnCode::INTERRUPTED) break;
        else if (std::chrono::steady_clock::now() - sentAt > std::chrono::seconds(1)) {
            client->remove_pending_request(requestId); pending = false;
            RCLCPP_WARN(node->get_logger(), "test adapter retrying %s/get_angles", name.c_str());
        }
        result = robot->myStep();
    }
    rclcpp::shutdown();
    return 0;
}
