#include <webots/Supervisor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common/msg/game_data.hpp>
#include <common/msg/field_data.hpp>
#include <common/msg/player.hpp>
#include <common/msg/location.hpp>
#include "WebotsUtils.hpp"
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include <cmath>
#include <random>

using namespace std;

struct ObjectInfo {
    Eigen::Vector3d translation;
    Eigen::Vector4d rotation;
};

const double ballR = 0.05;
const double robotR = 0.2;
const double fieldX = 4.5 + ballR;
const double fieldZ = 3.0 + ballR;
const double forbidAreaX = 3.55;
const double forbidAreaZ = 1.45;

double Sign(double v)
{
    return v < 0 ? -1 : 1;
}

bool IsEqual(double v1, double v2)
{
    return fabs(v1 - v2) < 1E-2;
}

bool BallMoved(const double *p1, const double *p2)
{
    return !(IsEqual(p1[0], p2[0]) && IsEqual(p1[2], p2[2]));
}

bool IsRedOutOfField(const double *pos)
{
    if ((fabs(pos[0]) > fieldX + robotR) || (fabs(pos[2]) > fieldZ + robotR)
        || (pos[0] < -forbidAreaX && fabs(pos[2]) < forbidAreaZ)) {
        return true;
    } else {
        return false;
    }
}

bool IsBlueOutOfField(const double *pos)
{
    if ((fabs(pos[0]) > fieldX + robotR) || (fabs(pos[2]) > fieldZ + robotR)
        || (pos[0] > - fieldX + robotR)) {
        return true;
    } else {
        return false;
    }
}

class FieldDataPublisher : public rclcpp::Node
{
public:
    FieldDataPublisher(): Node("field_data_publisher")
    {
        publisher_ = this->create_publisher<common::msg::FieldData>("/sensor/field", 5);
    }

    void Publish(const common::msg::FieldData& data)
    {
        publisher_->publish(data);
    }

    rclcpp::Publisher<common::msg::FieldData>::SharedPtr publisher_;
};

class GameDataSubscriber: public rclcpp::Node
{
public:
    GameDataSubscriber(std::string robotName): Node(robotName + "_game_data_subscriber")
    {
        subscription_ = this->create_subscription<common::msg::GameData>(
                            "/sensor/game", 5, std::bind(&GameDataSubscriber::topic_callback, this,
                                    std::placeholders::_1));
    }

    const common::msg::GameData& GetData()
    {
        return gameData_;
    }

private:
    void topic_callback(const common::msg::GameData::SharedPtr msg)
    {
        gameData_ = *msg;
    }

    common::msg::GameData gameData_;
    rclcpp::Subscription<common::msg::GameData>::SharedPtr subscription_;
};

class LocationPublisher : public rclcpp::Node
{
public:
    LocationPublisher(std::string robotName): Node(robotName + "_location_publisher")
    {
        publisher_ = this->create_publisher<common::msg::Location>("/sensor/" + robotName + "_location", 5);
    }

    void Publish(const common::msg::Location& data)
    {
        publisher_->publish(data);
    }

    rclcpp::Publisher<common::msg::Location>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    string judgeName;
    if (!WaitForRobots(judgeName)) {
        printf("!!! can not connect to webots\n");
        return 0;
    }
    const int basicTime = 20;
    const double goalX = 4.5 + ballR;
    const double goalZ = 1.3 + ballR;
    const double ballPosInit[3] = {-3.0, ballR, 0.0};
    const double zeroVel[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ObjectInfo redInitInfos[] = {
        {Eigen::Vector3d(-2.5, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, M_PI)}
    };
    ObjectInfo blueInitInfos[] = {
        {Eigen::Vector3d(-4.5, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, 0)}
    };

    ObjectInfo redWaitInfos[] = {
        {Eigen::Vector3d(0.75, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)}
    };
    ObjectInfo blueWaitInfos[] = {
        {Eigen::Vector3d(-0.75, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)}
    };

    rclcpp::init(argc, argv);

    const int redNum = 1; // num of red player(s)
    std::vector<std::shared_ptr<LocationPublisher>> locPublisherRed_;
    for (int i = 0; i < redNum; i++){
        string robotName = "red_";
        robotName.append(std::to_string(i + 1));
        locPublisherRed_.push_back(make_shared<LocationPublisher>(robotName));
    }

    const int blueNum = 1; // num of blue player(s)
    std::vector<std::shared_ptr<LocationPublisher>> locPublisherBlue_;
    for (int i = 0; i < blueNum; i++){
        string robotName = "blue_";
        robotName.append(std::to_string(i + 1));
        locPublisherBlue_.push_back(make_shared<LocationPublisher>(robotName));
    }

    auto judgeNode = make_shared<rclcpp::Node>("judge");
    auto fieldPublisher = make_shared<FieldDataPublisher>();
    auto gameSubscriber = make_shared<GameDataSubscriber>("judge");

    common::msg::GameData gameData;
    common::msg::FieldData fieldData;
    std::vector<common::msg::Location> locRed_(redNum);
    std::vector<common::msg::Location> locBlue_(blueNum);
    std::shared_ptr<webots::Supervisor> super = make_shared<webots::Supervisor>();
    webots::Node *ball = super->getFromDef("Ball");
    webots::Node *red_1 = super->getFromDef("red_1");
    webots::Node *blue_1 = super->getFromDef("blue_1");
    webots::Node *redPlayers[1] = {red_1};
    webots::Node *bluePlayers[1] = {blue_1};
    int ballMoveCnt = 0;
    double lastBallPos[3] = {0};
    auto lastGoalTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    fieldData.red_players[0].name = "red_1";
    fieldData.blue_players[0].name = "blue_1";
    bool initSet = false;
    bool redWaitSet[2] = {false, false};
    bool blueWaitSet[2] = {false, false};

    std::default_random_engine random(time(NULL));
    std::uniform_real_distribution<float> randDis(0, 1.0);
    std::uniform_real_distribution<float> randRad(0, 2 * M_PI);

    while (super->step(basicTime) != -1 && rclcpp::ok()) {
        rclcpp::spin_some(judgeNode);
        rclcpp::spin_some(fieldPublisher);
        rclcpp::spin_some(gameSubscriber);
        gameData = gameSubscriber->GetData();
        if (gameData.state == gameData.STATE_INIT) {
            ballMoveCnt = 0;
            fieldData.ball_state = fieldData.BALL_NORMAL;
            fieldData.red_players[0].state = common::msg::Player::PLAYER_NORMAL;
            fieldData.blue_players[0].state = common::msg::Player::PLAYER_NORMAL;
            if (!initSet) {
                ball->setVelocity(zeroVel);
                ball->getField("translation")->setSFVec3f(ballPosInit);
                for (size_t i = 0; i < 1; i++) {
                    redPlayers[i]->getField("translation")->setSFVec3f(redInitInfos[i].translation.data());
                    redPlayers[i]->getField("rotation")->setSFRotation(redInitInfos[i].rotation.data());
                    redPlayers[i]->setVelocity(zeroVel);
                    bluePlayers[i]->getField("translation")->setSFVec3f(blueInitInfos[i].translation.data());
                    bluePlayers[i]->getField("rotation")->setSFRotation(blueInitInfos[i].rotation.data());
                    bluePlayers[i]->setVelocity(zeroVel);
                }
                initSet = true;
            }
        } else if (gameData.state == gameData.STATE_PLAY) {
            initSet = false;
            for (size_t i = 0; i < 1; i++) {
                if (gameData.red_players[i].state == common::msg::Player::PLAYER_WAIT) {
                    if (!redWaitSet[i]) {
                        redWaitSet[i] = true;
                        redPlayers[i]->getField("translation")->setSFVec3f(redWaitInfos[i].translation.data());
                        redPlayers[i]->getField("rotation")->setSFRotation(redWaitInfos[i].rotation.data());
                        redPlayers[i]->setVelocity(zeroVel);
                    }
                } else {
                    redWaitSet[i] = false;
                }
                if (gameData.blue_players[i].state == common::msg::Player::PLAYER_WAIT) {
                    if (!blueWaitSet[i]) {
                        blueWaitSet[i] = true;
                        bluePlayers[i]->getField("translation")->setSFVec3f(blueWaitInfos[i].translation.data());
                        bluePlayers[i]->getField("rotation")->setSFRotation(blueWaitInfos[i].rotation.data());
                        bluePlayers[i]->setVelocity(zeroVel);
                    }
                } else {
                    blueWaitSet[i] = false;
                }
            }

            // 因为点球的罚站点也算出界，因此要判断一下
            if (fieldData.red_players[0].state == common::msg::Player::PLAYER_NORMAL) {
                fieldData.red_players[0].state = IsRedOutOfField(red_1->getPosition()) ?
                                            common::msg::Player::PALYER_OUT : common::msg::Player::PLAYER_NORMAL;
            }
            if (fieldData.blue_players[0].state == common::msg::Player::PLAYER_NORMAL) {
                fieldData.blue_players[0].state = IsBlueOutOfField(blue_1->getPosition()) ?
                                            common::msg::Player::PALYER_OUT : common::msg::Player::PLAYER_NORMAL;
            }

            const double *ballPos = ball->getPosition();
            bool goal = false;
            auto currTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

            if (fabs(ballPos[2]) < goalZ) {
                if (ballPos[0] < -goalX) {
                    if (currTime - lastGoalTime > 10) {
                        fieldData.red_score++;
                    }
                    goal = true;
                } else if (ballPos[0] > goalX) {
                    if (currTime - lastGoalTime > 10) {
                        fieldData.blue_score++;
                    }
                    goal = true;
                }
            }
            if (goal) {
                lastGoalTime = currTime;
                fieldData.ball_state = fieldData.BALL_GOAL;
            } else {
                if (fabs(ballPos[2]) > fieldZ || fabs(ballPos[0]) > fieldX) {
                    fieldData.ball_state = fieldData.BALL_OUT;
                } else {
                    if (BallMoved(lastBallPos, ballPos)) {
                        ballMoveCnt = 0;
                        memcpy(lastBallPos, ballPos, 3 * sizeof(double));
                    } else {
                        ballMoveCnt++;
                    }

                    if (ballMoveCnt * basicTime > 60 * 1000) {
                        fieldData.ball_state = fieldData.BALL_NOMOVE;
                    } else {
                        fieldData.ball_state = fieldData.BALL_NORMAL;
                    }
                }
            }
        }
        // update location
        for (int i = 0; i < redNum; i++) {
            const double* redpos = redPlayers[i]->getPosition();
            float noiseDis = randDis(random);
            float noiseRad = randRad(random);
            locRed_[i].x = redpos[0] + noiseDis * sin(noiseRad);
            locRed_[i].z = redpos[2] + noiseDis * cos(noiseRad);
            locPublisherRed_[i]->Publish(locRed_[i]);
        }

        for (int i = 0; i < blueNum; i++) {
            const double* bluepos = bluePlayers[i]->getPosition();
            float noiseDis = randDis(random);
            float noiseRad = randRad(random);
            locBlue_[i].x = bluepos[0] + noiseDis * sin(noiseRad);
            locBlue_[i].z = bluepos[2] + noiseDis * cos(noiseRad);
            locPublisherBlue_[i]->Publish(locBlue_[i]);
        }    
        fieldPublisher->Publish(fieldData);
    }

    return 0;
}
