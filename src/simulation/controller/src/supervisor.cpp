#include <webots/Supervisor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common/msg/game_data.hpp>
#include <common/msg/field_data.hpp>
#include <common/msg/player.hpp>
#include <common/msg/location.hpp>
#include <common/msg/talk.hpp>
#include "WebotsUtils.hpp"
#include <eigen3/Eigen/Dense>
#include <unistd.h>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>

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
const double StopX = 2.45 + ballR;
const double StopZ = 2.15 + ballR;

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

bool IsOutOfField_Red(const double *pos,int number)
{
    if(number==0)
    {
        if ((fabs(pos[0]) > fieldX + robotR) || (fabs(pos[2]) > fieldZ + robotR)
        ||(pos[0])> StopX + robotR && fabs(pos[2]) < StopZ - robotR) {
            return true;
        } else {
            return false;
        }
    }
    else if(number==1)
    {
        if ((fabs(pos[0]) > fieldX + robotR) || (fabs(pos[2]) > fieldZ + robotR)
        ||(pos[0])<- robotR) {
            return true;
        } else {
            return false;
        }
    }
}

bool IsOutOfField_Blue(const double *pos,int number)
{
    if(number==0)
    {
        if ((fabs(pos[0]) > fieldX + robotR) || (fabs(pos[2]) > fieldZ + robotR)
        ||(pos[0])< -StopX - robotR && fabs(pos[2]) < StopZ - robotR) {
            return true;
        } else {
            return false;
        }
    }
    else if(number==1)
    {
        if ((fabs(pos[0]) > fieldX + robotR) || (fabs(pos[2]) > fieldZ + robotR)
        ||(pos[0])> robotR) {
            return true;
        } else {
            return false;
        }
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

class TalkCapture: public rclcpp::Node
{
public:
    explicit TalkCapture(const std::string &robotName): Node(robotName + "_trace_talk")
    {
        subscription_ = this->create_subscription<common::msg::Talk>(
            "/" + robotName + "/talk/talk_str", 5,
            [this](common::msg::Talk::ConstSharedPtr message) {
                latest_ = message->talk_str;
                ++sequence_;
            });
    }

    const std::string &latest() const { return latest_; }
    unsigned long sequence() const { return sequence_; }

private:
    std::string latest_;
    unsigned long sequence_ = 0;
    rclcpp::Subscription<common::msg::Talk>::SharedPtr subscription_;
};

std::string csvQuote(const std::string &value)
{
    std::string quoted = "\"";
    for (char character : value) {
        if (character == '"') quoted += '"';
        quoted += character;
    }
    quoted += '"';
    return quoted;
}

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
    const double ballPosInit[3] = {0.0, ballR, 0.0};
    const double zeroVel[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ObjectInfo redInitInfos[] = {
        {Eigen::Vector3d(1.5, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)},
        {Eigen::Vector3d(2.3, 0.365, 3.0), Eigen::Vector4d(0, 1, 0, M_PI / 2)},
    };
    const bool perceptionCalibration =
        std::getenv("CUPCUP_PERCEPTION_CALIBRATION") != nullptr;
    const bool calibrationBlue = perceptionCalibration &&
        std::getenv("CUPCUP_MOCK_COLOR") != nullptr &&
        std::string(std::getenv("CUPCUP_MOCK_COLOR")) == "blue";
    if (perceptionCalibration && !calibrationBlue) {
        redInitInfos[0] = {Eigen::Vector3d(1.5, 0.365, 0.0),
            Eigen::Vector4d(0, 1, 0, M_PI)};
    }

    ObjectInfo redWaitInfos[] = {
        {Eigen::Vector3d(1.5, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)},
        {Eigen::Vector3d(2.3, 0.365, 3.0), Eigen::Vector4d(0, 1, 0, M_PI / 2)}
    };

    ObjectInfo redWinInfos[] = {
        {Eigen::Vector3d(0.75, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)},
        {Eigen::Vector3d(3.0, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, M_PI)},
    };

    ObjectInfo redLoseInfos[] = {
        {Eigen::Vector3d(0.75, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, M_PI)},
        {Eigen::Vector3d(3.0, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, M_PI)}
    };

    ObjectInfo redResetInfos[] = {
        {Eigen::Vector3d(0.75, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, M_PI)},
        {Eigen::Vector3d(3.0, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, M_PI)}
    };

    ObjectInfo blueWinInfos[] = {
        {Eigen::Vector3d(-0.75, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)},
        {Eigen::Vector3d(-3.0, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, 0)},
    };

    ObjectInfo blueLoseInfos[] = {
        {Eigen::Vector3d(-0.75, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, 0)},
        {Eigen::Vector3d(-3.0, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, 0)}
    };

    ObjectInfo blueInitInfos[] = {
        {Eigen::Vector3d(-1.5, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)},
        {Eigen::Vector3d(-2.3, 0.365, 3.0), Eigen::Vector4d(0, 1, 0, M_PI / 2)},
    };
    if (calibrationBlue) {
        blueInitInfos[0] = {Eigen::Vector3d(-1.5, 0.365, 0.0),
            Eigen::Vector4d(0, 1, 0, 0.0)};
    }

    ObjectInfo blueWaitInfos[] = {
        {Eigen::Vector3d(-1.5, 0.365, -3.0), Eigen::Vector4d(0, 1, 0, -M_PI / 2)},
        {Eigen::Vector3d(-2.3, 0.365, 3.0), Eigen::Vector4d(0, 1, 0, M_PI / 2)}
    };

    ObjectInfo blueResetInfos[] = {
        {Eigen::Vector3d(-0.75, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, 0)},
        {Eigen::Vector3d(-3.0, 0.365, 0.0), Eigen::Vector4d(0, 1, 0, M_PI)}
    };

    rclcpp::init(argc, argv);

    const int redNum = 2; // num of red player(s)
    std::vector<std::shared_ptr<LocationPublisher>> locPublisherRed_;
    for (int i = 0; i < redNum; i++){
        string robotName = "red_";
        robotName.append(std::to_string(i + 1));
        locPublisherRed_.push_back(make_shared<LocationPublisher>(robotName));
    }

    const int blueNum = 2; // num of blue player(s)
    std::vector<std::shared_ptr<LocationPublisher>> locPublisherBlue_;
    for (int i = 0; i < blueNum; i++){
        string robotName = "blue_";
        robotName.append(std::to_string(i + 1));
        locPublisherBlue_.push_back(make_shared<LocationPublisher>(robotName));
    }

    auto judgeNode = make_shared<rclcpp::Node>("judge");
    auto fieldPublisher = make_shared<FieldDataPublisher>();
    auto gameSubscriber = make_shared<GameDataSubscriber>("judge");
    const std::vector<std::string> robotNames = {"red_1", "red_2", "blue_1", "blue_2"};
    std::vector<std::shared_ptr<TalkCapture>> talkCaptures;

    common::msg::GameData gameData;
    common::msg::FieldData fieldData;
    std::vector<common::msg::Location> locRed_(redNum);
    std::vector<common::msg::Location> locBlue_(blueNum);
    std::shared_ptr<webots::Supervisor> super = make_shared<webots::Supervisor>();
    webots::Node *ball = super->getFromDef("Ball");
    webots::Node *red_1 = super->getFromDef("red_1");
    webots::Node *red_2 = super->getFromDef("red_2");
    webots::Node *blue_1 = super->getFromDef("blue_1");
    webots::Node *blue_2 = super->getFromDef("blue_2");
    webots::Node *redPlayers[2] = {red_1, red_2};
    webots::Node *bluePlayers[2] = {blue_1, blue_2};
    // Evaluation only: the strategy never receives simulator camera truth.
    webots::Node *cameras[4] = {nullptr, nullptr, nullptr, nullptr};
    int ballMoveCnt = 0;
    double lastBallPos[3] = {0};
    auto lastGoalTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    fieldData.red_players[0].name = "red_1";
    fieldData.red_players[1].name = "red_2";
    fieldData.blue_players[0].name = "blue_1";
    fieldData.blue_players[1].name = "blue_2";
    bool initSet = false;
    int goal = 0; // 0: not goaled; 1: red goaled; -1: blue goaled
    bool redWaitSet[2] = {false, false};
    bool blueWaitSet[2] = {false, false};

    unsigned int simulationSeed = static_cast<unsigned int>(time(NULL));
    if (const char* seedText = std::getenv("CUPCUP_SIM_SEED")) {
        char* seedEnd = nullptr;
        const unsigned long parsedSeed = std::strtoul(seedText, &seedEnd, 10);
        if (seedEnd != seedText && *seedEnd == '\0' &&
            parsedSeed <= std::numeric_limits<unsigned int>::max()) {
            simulationSeed = static_cast<unsigned int>(parsedSeed);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("supervisor"),
                "Ignoring invalid CUPCUP_SIM_SEED='%s'", seedText);
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("supervisor"), "location-noise seed=%u", simulationSeed);
    std::default_random_engine random(simulationSeed);
    std::uniform_real_distribution<float> randDis(0, 1.0);
    std::uniform_real_distribution<float> randRad(0, 2 * M_PI);

    std::ofstream trace;
    if (const char *tracePath = std::getenv("CUPCUP_TRACE_PATH")) {
        if (*tracePath != '\0') {
            trace.open(tracePath, std::ios::out | std::ios::trunc);
            if (trace) {
                webots::Node *players[4] = {red_1, red_2, blue_1, blue_2};
                for (size_t i = 0; i < robotNames.size(); ++i) {
                    cameras[i] = players[i]->getFromProtoDef("CupcupCamera");
                    if (!cameras[i]) {
                        RCLCPP_WARN(rclcpp::get_logger("supervisor"),
                            "Camera pose unavailable for %s", robotNames[i].c_str());
                    }
                }
                trace << "time,ball_x,ball_z,ball_vx,ball_vz,ball_y";
                for (const auto &name : robotNames) {
                    trace << ',' << name << "_true_x," << name << "_true_z," << name
                        << "_vx," << name << "_vz," << name << "_omega_y," << name
                        << "_obs_x," << name << "_obs_z," << name << "_talk_seq," << name
                        << "_talk," << name << "_camera_x," << name << "_camera_y," << name
                        << "_camera_z";
                    for (int element = 0; element < 9; ++element) {
                        trace << ',' << name << "_camera_r" << element;
                    }
                }
                trace << '\n';
                RCLCPP_INFO(rclcpp::get_logger("supervisor"),
                    "Writing evaluation-only world trace to %s", tracePath);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("supervisor"),
                    "Could not open evaluation trace path: %s", tracePath);
            }
        }
    }
    if (trace) {
        for (const auto &robotName : robotNames) {
            talkCaptures.push_back(make_shared<TalkCapture>(robotName));
        }
    }
    unsigned long traceRows = 0;

    while (super->step(basicTime) != -1 && rclcpp::ok()) {
        rclcpp::spin_some(judgeNode);
        rclcpp::spin_some(fieldPublisher);
        rclcpp::spin_some(gameSubscriber);
        for (const auto &capture : talkCaptures) rclcpp::spin_some(capture);
        gameData = gameSubscriber->GetData();

        // update player state from gameData
        for (int i = 0; i < redNum; i++) {
            fieldData.red_players[i].state = gameData.red_players[i].state;
            fieldData.red_players[i].wait_time = gameData.red_players[i].wait_time;
        }
        for (int i = 0; i < blueNum; i++) {
            fieldData.blue_players[i].state = gameData.blue_players[i].state;
            fieldData.blue_players[i].wait_time = gameData.blue_players[i].wait_time;
        }

        if (gameData.state == gameData.STATE_INIT) {
            ballMoveCnt = 0;
            fieldData.ball_state = fieldData.BALL_NORMAL;
            fieldData.red_players[0].state = common::msg::Player::PLAYER_NORMAL;
            fieldData.red_players[1].state = common::msg::Player::PLAYER_NORMAL;
            fieldData.blue_players[0].state = common::msg::Player::PLAYER_NORMAL;
            fieldData.blue_players[1].state = common::msg::Player::PLAYER_NORMAL;
            if (!initSet) {
                ball->setVelocity(zeroVel);
                ball->getField("translation")->setSFVec3f(ballPosInit);
                ObjectInfo const *redTargetInfos;
                ObjectInfo const *blueTargetInfos;
                if (goal == 0) {
                    redTargetInfos = redInitInfos;
                    blueTargetInfos = blueInitInfos;
                } else if (goal == 1) {
                    // red won
                    redTargetInfos = redWinInfos;
                    blueTargetInfos = blueLoseInfos;
                } else if (goal == -1) {
                    // red lose
                    redTargetInfos = redLoseInfos;
                    blueTargetInfos = blueWinInfos;
                }
                for (size_t i = 0; i < redNum; i++) {
                    redPlayers[i]->getField("translation")->setSFVec3f(redTargetInfos[i].translation.data());
                    redPlayers[i]->getField("rotation")->setSFRotation(redTargetInfos[i].rotation.data());
                    redPlayers[i]->setVelocity(zeroVel);
                }
                for (size_t i = 0; i < blueNum; i++) {
                    bluePlayers[i]->getField("translation")->setSFVec3f(blueTargetInfos[i].translation.data());
                    bluePlayers[i]->getField("rotation")->setSFRotation(blueTargetInfos[i].rotation.data());
                    bluePlayers[i]->setVelocity(zeroVel);
                }
                initSet = true;
                goal = 0;
            }
        } else if (gameData.state == gameData.STATE_PLAY) {
            initSet = false;
            for (size_t i = 0; i < redNum; i++) {
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
            }

            for (size_t i = 0; i < blueNum; i++) {
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

            for (int i = 0; i < redNum; i++) {
                if (fieldData.red_players[i].state == common::msg::Player::PLAYER_NORMAL) {
                    fieldData.red_players[i].state = IsOutOfField_Red(redPlayers[i]->getPosition(),i) ?
                                                common::msg::Player::PALYER_OUT : common::msg::Player::PLAYER_NORMAL;
                }
            }
            for (int i = 0; i < blueNum; i++) {
                if (fieldData.blue_players[i].state == common::msg::Player::PLAYER_NORMAL) {
                    fieldData.blue_players[i].state = IsOutOfField_Blue(bluePlayers[i]->getPosition(),i) ?
                                                common::msg::Player::PALYER_OUT : common::msg::Player::PLAYER_NORMAL;
                }
            }                  
            const double *ballPos = ball->getPosition();
            auto currTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

            // if ball is between goal poles
            if (fabs(ballPos[2]) < goalZ) {
                if (ballPos[0] < -goalX) {
                    if (currTime - lastGoalTime > 10) {
                        fieldData.red_score++;
                    }
                    goal = 1;
                } else if (ballPos[0] > goalX) {
                    if (currTime - lastGoalTime > 10) {
                        fieldData.blue_score++;
                    }
                    goal = -1;
                }
            }
            if (goal) {
                lastGoalTime = currTime;
                fieldData.ball_state = fieldData.BALL_GOAL;
            } else {
                if (fabs(ballPos[0]) > fieldX) {
                    fieldData.ball_state = fieldData.BALL_OUT;
                }
                else if(fabs(ballPos[2]) > fieldZ)
                {
                    fieldData.ball_state = fieldData.BALL_OUTLINE;
                    ball->setVelocity(zeroVel);
                    ball->getField("translation")->setSFVec3f(ballPosInit);
                    for (size_t i = 0; i < redNum; i++) {
                        redPlayers[i]->getField("translation")->setSFVec3f(redResetInfos[i].translation.data());
                        redPlayers[i]->getField("rotation")->setSFRotation(redResetInfos[i].rotation.data());
                        redPlayers[i]->setVelocity(zeroVel);
                    }
                    for (size_t i = 0; i < blueNum; i++) {
                        bluePlayers[i]->getField("translation")->setSFVec3f(blueResetInfos[i].translation.data());
                        bluePlayers[i]->getField("rotation")->setSFRotation(blueResetInfos[i].rotation.data());
                        bluePlayers[i]->setVelocity(zeroVel);
                    }
                } 
                else {
                    if (BallMoved(lastBallPos, ballPos)) {
                        ballMoveCnt = 0;
                        memcpy(lastBallPos, ballPos, 3 * sizeof(double));
                    } else if (gameData.state == gameData.STATE_PLAY) {
                        ballMoveCnt++;
                    }

                    // ball does not move for 100s
                    if (ballMoveCnt * basicTime > 100 * 1000) {
                        fieldData.ball_state = fieldData.BALL_NOMOVE;
                        ballMoveCnt = 0;
                    } else {
                        fieldData.ball_state = fieldData.BALL_NORMAL;
                    }
                }
            }
        }

        if (perceptionCalibration) {
            // Eighteen two-second stages: 3 ranges x 3 bearings x 2 pitches.
            // Cupcup's trace-only head command uses the same simulation clock.
            const int stage = static_cast<int>(super->getTime() / 2.0) % 18;
            const double ranges[3] = {1.5, 2.5, 3.5};
            const double yaws[3] = {-20.0, 0.0, 20.0};
            const double range = ranges[stage / 6];
            const double bearing = yaws[(stage / 2) % 3] * M_PI / 180.0;
            const double originX = calibrationBlue ? -1.5 : 1.5;
            const double direction = calibrationBlue ? 1.0 : -1.0;
            const double ballPosition[3] = {
                originX + direction * range * std::cos(bearing), ballR,
                -direction * range * std::sin(bearing)};
            ball->setVelocity(zeroVel);
            ball->getField("translation")->setSFVec3f(ballPosition);
        }

        // update location
        for (int i = 0; i < redNum; i++) {
            const double* redpos = redPlayers[i]->getPosition();
            float noiseDis = randDis(random);
            float noiseRad = randRad(random);
            locRed_[i].x = redpos[0] + noiseDis * sin(noiseRad);
            locRed_[i].z = redpos[2] + noiseDis * cos(noiseRad);
            locRed_[i].stamp = static_cast<uint32_t>(super->getTime() * 1000.0);
            locPublisherRed_[i]->Publish(locRed_[i]);
        }

        for (int i = 0; i < blueNum; i++) {
            const double* bluepos = bluePlayers[i]->getPosition();
            float noiseDis = randDis(random);
            float noiseRad = randRad(random);
            locBlue_[i].x = bluepos[0] + noiseDis * sin(noiseRad);
            locBlue_[i].z = bluepos[2] + noiseDis * cos(noiseRad);
            locBlue_[i].stamp = static_cast<uint32_t>(super->getTime() * 1000.0);
            locPublisherBlue_[i]->Publish(locBlue_[i]);
        }
        fieldPublisher->Publish(fieldData);

        if (trace) {
            const double *ballPosition = ball->getPosition();
            const double *ballVelocity = ball->getVelocity();
            std::ostringstream sample;
            sample << super->getTime() << ',' << ballPosition[0] << ',' << ballPosition[2]
                << ',' << ballVelocity[0] << ',' << ballVelocity[2] << ',' << ballPosition[1];
            const auto writeRobot = [&sample](webots::Node *node, webots::Node *camera,
                const common::msg::Location &observation,
                const std::shared_ptr<TalkCapture> &talk) {
                const double *position = node->getPosition();
                const double *velocity = node->getVelocity();
                sample << ',' << position[0] << ',' << position[2]
                    << ',' << velocity[0] << ',' << velocity[2] << ',' << velocity[4]
                    << ',' << observation.x << ',' << observation.z << ',' << talk->sequence()
                    << ',' << csvQuote(talk->latest());
                if (camera) {
                    const double *cameraPosition = camera->getPosition();
                    const double *orientation = camera->getOrientation();
                    sample << ',' << cameraPosition[0] << ',' << cameraPosition[1]
                        << ',' << cameraPosition[2];
                    for (int element = 0; element < 9; ++element) {
                        sample << ',' << orientation[element];
                    }
                } else {
                    for (int element = 0; element < 12; ++element) sample << ',';
                }
            };
            writeRobot(redPlayers[0], cameras[0], locRed_[0], talkCaptures[0]);
            writeRobot(redPlayers[1], cameras[1], locRed_[1], talkCaptures[1]);
            writeRobot(bluePlayers[0], cameras[2], locBlue_[0], talkCaptures[2]);
            writeRobot(bluePlayers[1], cameras[3], locBlue_[1], talkCaptures[3]);
            trace << sample.str() << '\n';
            if (++traceRows % 100 == 0) trace.flush();
        }
    }
    return 0;
}
