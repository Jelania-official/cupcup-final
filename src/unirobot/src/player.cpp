#include "topics.hpp"
#include <common/srv/get_color.hpp>

enum RobotColor {
    COLOR_INVALID,
    COLOR_RED,
    COLOR_BLUE
};

RobotColor GetRobotColor(const std::string &name)
{
    if (name.find("red") != std::string::npos) {
        return COLOR_RED;
    } else if (name.find("blue") != std::string::npos) {
        return COLOR_BLUE;
    } else {
        return COLOR_INVALID;
    }
}

int GetRobotId(const std::string &name)
{
    return static_cast<int>(name.back() - '0');
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string robotName = "maxwell";
    std::string friendName = "maxwell";

    std::string tmp = std::string(argv[1]);
    std::string team = tmp.substr(0, tmp.find_last_of('_'));

    auto playerNode = std::make_shared<rclcpp::Node>(tmp + "_player");
    rclcpp::Client<common::srv::GetColor>::SharedPtr client =
        playerNode->create_client<common::srv::GetColor>("gamectrl/get_color");
    while (!client->wait_for_service(std::chrono::duration<long long>(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(playerNode->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(playerNode->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<common::srv::GetColor::Request>();
    request.get()->team = team;
    auto result = client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(playerNode, result);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        auto resp = result.get();
        if (resp->color == "invalid") {
            RCLCPP_ERROR(playerNode->get_logger(), "Not supportted team name. Exiting.");
            return 0;
        }
        robotName = resp->color + tmp.substr(tmp.find_last_of('_'));
        friendName = resp->color;
        RCLCPP_INFO(playerNode->get_logger(), "robotName: %s", robotName.c_str());
    } else {
        RCLCPP_ERROR(playerNode->get_logger(), "Exiting.");
        return 0;
    }

    RobotColor myColor = GetRobotColor(robotName);
    int myId = GetRobotId(robotName);
    friendName = friendName + "_" + std::to_string(3 - myId);
    common::msg::BodyTask btask;
    common::msg::HeadTask htask;
    common::msg::GameData gameData;
    common::msg::Location location;
    common::msg::Talk talkPub; // 发布的对话
    common::msg::Talk talkSub; // 订阅的对话
    btask.type = btask.TASK_WALK;
    btask.count = 2;
    btask.step = 0.03;
    htask.yaw = 0.0;
    htask.pitch = 45.0;
    auto bodyTaskNode = std::make_shared<BodyTaskPublisher>(robotName);
    auto headTaskNode = std::make_shared<HeadTaskPublisher>(robotName);
    auto imageSubscriber = std::make_shared<ImageSubscriber>(robotName);
    auto imuSubscriber = std::make_shared<ImuDataSubscriber>(robotName);
    auto headSubscriber = std::make_shared<HeadAngleSubscriber>(robotName);
    auto resImgPublisher = std::make_shared<ResultImagePublisher>(robotName);
    auto gameSubscriber = std::make_shared<GameDataSubscriber>(robotName);
    auto locSubscriber = std::make_shared<LocationSubscriber>(robotName);
    auto talkPublisher = std::make_shared<TalkPublisher>(robotName);
    auto talkSubscriber = std::make_shared<TalkSubscriber>(friendName);
    rclcpp::WallRate loop_rate(10.0);
    float initYaw = 0.0;

    while (rclcpp::ok()) {
        rclcpp::spin_some(bodyTaskNode);
        rclcpp::spin_some(headTaskNode);
        rclcpp::spin_some(imageSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(headSubscriber);
        rclcpp::spin_some(resImgPublisher);
        auto imuData = imuSubscriber->GetData();
        auto image = imageSubscriber->GetImage().clone();
        auto headAngle = headSubscriber->GetData();

        rclcpp::spin_some(gameSubscriber);
        gameData = gameSubscriber->GetData();

        rclcpp::spin_some(locSubscriber); // 更新定位
        location = locSubscriber->GetData();

        rclcpp::spin_some(talkPublisher); // 更新对话发布
        rclcpp::spin_some(talkSubscriber); // 更新对话订阅
        talkSub = talkSubscriber->GetData();

        // ----------------- 可以修改的部分 begin--------------------
        // 郑重声明：以下说明仅供参考，实际情况以实际为准
        // 下面提供的是一些可能会用到的相关信息接口说明和示例，可以根据自己的需要去使用，不必要纠结于现有的代码结构
        if (gameData.state == gameData.STATE_INIT){
            // 每次开球时都会进入这里
            // 例如：
            // RCLCPP_INFO(playerNode->get_logger(), talkSub.talk_str);
        }
        if (myColor == COLOR_RED) {
            // 使用的红色的机器人
        } else if (myColor == COLOR_BLUE) {
            // 使用的蓝色的机器人
        }

        if (myId == 1) {
            // 1号机器人，前锋
            talkPub.talk_str = "gogogo!";
            btask.step = 1; // 1 号机器人前进
        }
        else if (myId == 2) {
            // 2号机器人，门将
            btask.step = -1; // 2 号机器人后退
        }


        if (!image.empty()) {
            // 在这里写图像处理
            resImgPublisher->Publish(image); // 处理完的图像可以通过该方式发布出去，然后通过rqt中的image_view工具查看
        }
        // 输出定位信息
        // RCLCPP_INFO(playerNode->get_logger(), "%.2f, %.2f", location.x, location.z);

        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);
        talkPublisher->Publish(talkPub);
        loop_rate.sleep();
        // 郑重声明：以上说明仅供参考，实际情况以实际为准
        // ----------------- 可以修改的部分 end--------------------
    }
    rclcpp::shutdown();
    return 0;
}
