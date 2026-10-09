// Controlled test supervisor adapted from our initial-round kick experiment.
// This replaces the judge ONLY in a measurement run. Truth goes to local files.
#include <webots/Supervisor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common/msg/body_task.hpp>
#include <common/msg/head_task.hpp>
#include <common/msg/game_data.hpp>
#include <common/msg/imu_data.hpp>
#include <common/msg/head_angles.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "WebotsUtils.hpp"
#include "field_geometry.hpp"
#include <array>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace {
struct Trial {
    std::string name;
    double forward = 0, left = 0, command = 0;
    int repeat = 0;
    double warmupStep = 0, stopWait = 1.0, pulse = 0.12;
    double headPitch = 20, ballForward = -1;
    double blockerForward = -1, blockerLeft = 0;
};
enum Phase { Reset, Ready, Warmup, Observe, Act, Measure };
const char *phaseName(Phase p) {
    return p == Reset ? "RESET" : p == Ready ? "READY" : p == Observe ? "OBSERVE" :
        p == Warmup ? "WARMUP" : p == Act ? "ACT" : "MEASURE";
}
// Webots buffers are owned by the controller and invalidated on the next step.
std::vector<webots::ContactPoint> contacts(webots::Node *body) {
    int count = 0;
    const auto *points = body->getContactPoints(true, &count);
    return points && count > 0 ? std::vector<webots::ContactPoint>(points, points + count) :
        std::vector<webots::ContactPoint>();
}
int sharedContacts(webots::Node *ball, webots::Node *robot) {
    const auto ballPoints = contacts(ball), robotPoints = contacts(robot);
    int count = 0;
    for (const auto &a : ballPoints) {
        for (const auto &b : robotPoints) {
            double distance2 = 0;
            for (int i = 0; i < 3; ++i) distance2 += std::pow(a.point[i] - b.point[i], 2);
            if (distance2 < 1e-10) { ++count; break; }
        }
    }
    return count;
}
bool saveImage(const std::string &path, const sensor_msgs::msg::Image &image) {
    if (image.encoding != "rgb8" || image.width == 0 || image.height == 0 ||
        image.step < image.width * 3 || image.data.size() < image.step * image.height) return false;
    std::ofstream output(path, std::ios::binary);
    output << "P6\n" << image.width << ' ' << image.height << "\n255\n";
    for (unsigned row = 0; row < image.height; ++row)
        output.write(reinterpret_cast<const char *>(image.data.data() + row * image.step), image.width * 3);
    return static_cast<bool>(output);
}
}

int main(int argc, char **argv) {
    std::string name;
    if (!WaitForRobots(name)) return 2;
    const char *outputEnv = std::getenv("CUPCUP_MEASURE_OUTPUT");
    if (!outputEnv || !*outputEnv) return 2;
    const std::string output(outputEnv);
    const bool blue = std::getenv("CUPCUP_MEASURE_COLOR") &&
        std::string(std::getenv("CUPCUP_MEASURE_COLOR")) == "blue";
    const std::string robotName = blue ? "blue_1" : "red_1";
    const double attack = blue ? 1.0 : -1.0;
    const double yaw = blue ? 0.0 : M_PI;
    const int repeats = std::getenv("CUPCUP_MEASURE_REPEATS") ?
        std::stoi(std::getenv("CUPCUP_MEASURE_REPEATS")) : 3;
    if (repeats < 1 || repeats > 10) return 2;
    const bool blocking = std::getenv("CUPCUP_MEASURE_PROFILE") &&
        std::string(std::getenv("CUPCUP_MEASURE_PROFILE")) == "kick-blocking";
    const bool timing = blocking || (std::getenv("CUPCUP_MEASURE_PROFILE") &&
        std::string(std::getenv("CUPCUP_MEASURE_PROFILE")) == "kick-timing");
    const bool dynamicRobot = std::getenv("CUPCUP_MEASURE_PROFILE") &&
        std::string(std::getenv("CUPCUP_MEASURE_PROFILE")) == "robot-dynamic";
    const bool jointViews = std::getenv("CUPCUP_MEASURE_PROFILE") &&
        std::string(std::getenv("CUPCUP_MEASURE_PROFILE")) == "robot-ball-views";
    const bool views = jointViews || dynamicRobot || (std::getenv("CUPCUP_MEASURE_PROFILE") &&
        std::string(std::getenv("CUPCUP_MEASURE_PROFILE")) == "robot-views");
    const bool ballViews = std::getenv("CUPCUP_MEASURE_PROFILE") &&
        (std::string(std::getenv("CUPCUP_MEASURE_PROFILE")) == "ball-views" ||
         std::string(std::getenv("CUPCUP_MEASURE_PROFILE")) == "ball-dynamic");
    const bool dynamicBall = std::getenv("CUPCUP_MEASURE_PROFILE") &&
        std::string(std::getenv("CUPCUP_MEASURE_PROFILE")) == "ball-dynamic";
    const bool staticViews = views || ballViews;
    std::vector<Trial> trials;
    for (int r = 0; r < repeats; ++r) {
        if (blocking) {
            // Paired unobstructed controls and three passive opponent poses.
            // Keep the production foot positions, pulse and wait unchanged.
            for (bool leftFoot : {true, false}) {
                for (const auto &blocker : std::vector<std::pair<double, double>>{
                        {-1, 0}, {.25, 0}, {.50, 0}, {.25, .18}}) {
                    Trial trial{leftFoot ? "left_kick" : "right_kick", .18,
                                leftFoot ? .08 : -.04, 0, r, 0, .8, .12};
                    trial.blockerForward = blocker.first;
                    trial.blockerLeft = blocker.second;
                    trials.push_back(trial);
                }
            }
            continue;
        }
        if (jointViews) {
            // Same geometry, two fixed views; ball lies halfway to a frontal
            // robot. Labels for BOTH objects accompany the same received RGB.
            for (double range : {.6, 1.2, 2.0, 3.0})
                for (double pitch : {15.0, 35.0})
                    trials.push_back({"joint-front", range, 0, 180, r,
                                      0, 2.5, .12, pitch, range / 2.0});
            continue;
        }
        if (dynamicRobot) {
            // Target yaw remains body-relative 180 degrees; only the observer
            // moves. Head cases switch from 40 to 20 degrees at OBSERVE.
            trials.push_back({"robot-walk-slow", 1.2, 0, 180, r, .025, 2.5});
            trials.push_back({"robot-walk-fast", 1.2, 0, 180, r, .05, 2.5});
            trials.push_back({"robot-turn-left", 1.2, 0, 180, r, .025, 2.5});
            trials.push_back({"robot-turn-right", 1.2, 0, 180, r, .025, 2.5});
            trials.push_back({"robot-head-near-early", .6, 0, 180, r, 0, .12});
            trials.push_back({"robot-head-near-settled", .6, 0, 180, r, 0, 1.5});
            trials.push_back({"robot-head-far-early", 2.0, 0, 180, r, 0, .12});
            trials.push_back({"robot-head-far-settled", 2.0, 0, 180, r, 0, 1.5});
            continue;
        }
        if (dynamicBall) {
            trials.push_back({"ball-walk-slow", 1.2, 0, 25, r, .025, 2.5});
            trials.push_back({"ball-walk-fast", 1.2, 0, 25, r, .05, 2.5});
            trials.push_back({"ball-turn-left", 1.2, 0, 25, r, .025, 2.5});
            trials.push_back({"ball-turn-right", 1.2, 0, 25, r, .025, 2.5});
            trials.push_back({"ball-head-near-early", .30, 0, 60, r, 0, .12});
            trials.push_back({"ball-head-near-settled", .30, 0, 60, r, 0, 1.5});
            trials.push_back({"ball-head-far-early", 1.0, 0, 25, r, 0, .12});
            trials.push_back({"ball-head-far-settled", 1.0, 0, 25, r, 0, 1.5});
            continue;
        }
        if (ballViews) {
            // command is the head pitch, not a body command. No ball truth is
            // delivered to a player; positions are offline measurement labels.
            trials.push_back({"ball-right-foot", .16, -.05, 60, r, 0, 2.5});
            trials.push_back({"ball-left-foot", .18, .08, 60, r, 0, 2.5});
            trials.push_back({"ball-front", .30, 0, 60, r, 0, 2.5});
            trials.push_back({"ball-front", .60, 0, 40, r, 0, 2.5});
            trials.push_back({"ball-front", 1.0, 0, 25, r, 0, 2.5});
            trials.push_back({"ball-front", 2.0, 0, 10, r, 0, 2.5});
            trials.push_back({"ball-left", 1.0, .40, 25, r, 0, 2.5});
            trials.push_back({"ball-right", 2.0, -.60, 10, r, 0, 2.5});
            continue;
        }
        if (views) {
            // Predeclared static poses. command is target yaw relative to the
            // observer, not a motion command. Both teams use rotated copies.
            for (double range : {0.6, 1.2, 2.0, 3.0})
                trials.push_back({"robot-front", range, 0, 180, r, 0, 2.5});
            for (double range : {1.2, 2.0})
                trials.push_back({"robot-back", range, 0, 0, r, 0, 2.5});
            trials.push_back({"robot-side", 1.2, 0.4, 90, r, 0, 2.5});
            trials.push_back({"robot-front-offset", 2.0, 0.6, 180, r, 0, 2.5});
            continue;
        }
        if (timing) {
            // Same eight predeclared cases in both colors. Ball is kept away while
            // walking, then placed at measured body-relative coordinates at ACT.
            trials.push_back({"right_kick", .22, -.04, 0, r, 0, .8, .12});
            trials.push_back({"right_kick", .13, -.05, 0, r, 0, .8, .12});
            trials.push_back({"right_kick", .22, -.04, 0, r, .05, 0, .12});
            trials.push_back({"right_kick", .22, -.04, 0, r, .05, .8, .12});
            trials.push_back({"right_kick", .22, -.04, 0, r, .05, 1.6, .12});
            trials.push_back({"right_kick", .22, -.04, 0, r, .05, .8, .8});
            trials.push_back({"right_kick", .13, -.05, 0, r, .05, .8, .8});
            trials.push_back({"left_kick", .18, .08, 0, r, .05, .8, .12});
            continue;
        }
        trials.push_back({"walk", 0, 0, 0.025, r});
        trials.push_back({"walk", 0, 0, 0.05, r});
        trials.push_back({"turn", 0, 0, 10.0, r});
        trials.push_back({"left_kick", 0.18, 0.08, 0, r});
        trials.push_back({"right_kick", 0.18, -0.04, 0, r});
        trials.push_back({"right_kick", 0.22, -0.04, 0, r});
        trials.push_back({"roll", 0, 0, 0.9, r});
        trials.push_back({"roll", 0, 0, 1.5, r});
    }
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cupcup_measurement_judge");
    auto gamePub = node->create_publisher<common::msg::GameData>("/sensor/game", 5);
    std::array<rclcpp::Publisher<common::msg::BodyTask>::SharedPtr, 4> bodies;
    std::array<rclcpp::Publisher<common::msg::HeadTask>::SharedPtr, 4> heads;
    const std::array<std::string, 4> names{{"red_1", "red_2", "blue_1", "blue_2"}};
    for (int i = 0; i < 4; ++i) {
        bodies[i] = node->create_publisher<common::msg::BodyTask>("/" + names[i] + "/task/body", 5);
        heads[i] = node->create_publisher<common::msg::HeadTask>("/" + names[i] + "/task/head", 5);
    }
    sensor_msgs::msg::Image camera;
    common::msg::ImuData observedImu;
    common::msg::HeadAngles observedHead;
    double imageAt = -99.0, imuAt = -99.0, headAt = -99.0;
    int fall = 0, anyFall = 0, targetFall = 0, anyTargetFall = 0;
    webots::Supervisor supervisor;
    auto imageSub = node->create_subscription<sensor_msgs::msg::Image>(
        "/" + robotName + "/sensor/image", 2, [&](sensor_msgs::msg::Image::ConstSharedPtr image) {
            camera = *image; imageAt = supervisor.getTime();
        });
    auto imuSub = node->create_subscription<common::msg::ImuData>(
        "/" + robotName + "/sensor/imu", 2, [&](common::msg::ImuData::ConstSharedPtr imu) {
            observedImu = *imu; imuAt = supervisor.getTime(); fall = imu->fall;
        });
    auto headSub = node->create_subscription<common::msg::HeadAngles>(
        "/" + robotName + "/sensor/joint/head", 2, [&](common::msg::HeadAngles::ConstSharedPtr head) {
            observedHead = *head; headAt = supervisor.getTime();
        });
    auto targetImuSub = node->create_subscription<common::msg::ImuData>(
        "/" + names[blue ? 0 : 2] + "/sensor/imu", 2,
        [&](common::msg::ImuData::ConstSharedPtr imu) { targetFall = imu->fall; });
    webots::Node *ball = supervisor.getFromDef("Ball");
    std::array<webots::Node *, 4> robots;
    for (int i = 0; i < 4; ++i) robots[i] = supervisor.getFromDef(names[i]);
    const int selected = blue ? 2 : 0;
    const int targetIndex = blue ? 0 : 2;
    if (!ball || std::any_of(robots.begin(), robots.end(), [](webots::Node *r) { return !r; })) return 3;
    auto *robot = robots[selected];
    ball->enableContactPointsTracking(20, true);
    robot->enableContactPointsTracking(20, true);
    if (blocking) robots[targetIndex]->enableContactPointsTracking(20, true);
    const double zero[6] = {0, 0, 0, 0, 0, 0}, rotation[4] = {0, 1, 0, yaw};
    std::ofstream trace(output + "/motion_trace.csv"), summary(output + "/trials.csv");
    if (!trace || !summary) return 3;
    trace << "time,trial,name,repeat,phase,phase_time,robot_x,robot_z,robot_y,yaw_deg,vx,vz,omega_y,"
             "ball_x,ball_z,ball_y,ball_vx,ball_vz,fall,ball_contact_count,body_type,"
             "opponent_contact_count,opponent_x,opponent_z,opponent_y,opponent_fall\n";
    summary << "trial,name,repeat,command,forward,left,actual_forward,actual_left,pre_drift,"
               "max_ball_displacement,final_forward,final_left,max_ball_speed,fall,image,image_age,"
               "warmup_step,stop_wait,pulse,contact_frames,first_contact_delay,act_yaw_deg,"
               "blocker_forward,blocker_left,opponent_contact_frames,opponent_fall\n";
    std::ofstream viewSamples;
    if (staticViews) {
        viewSamples.open(output + (ballViews ? "/ball_views.csv" : "/robot_views.csv"));
        if (!viewSamples) return 3;
        viewSamples << "trial,name,repeat,time,image,image_saved,image_age,imu_age,head_age,"
                       "observer,target,observer_x,observer_z,observer_y,target_x,target_z,target_y,"
                       "target_vx,target_vz,imu_yaw,imu_pitch,imu_roll,head_yaw,head_pitch,fall,"
                       "observer_vx,observer_vz,observer_omega_y,phase_time,"
                       "ball_x,ball_z,ball_y,ball_vx,ball_vz\n";
    }
    std::size_t index = 0;
    Phase phase = Reset;
    double phaseAt = 0, startX = 0, startZ = 0, maxDistance = 0, maxSpeed = 0;
    double actualForward = 0, actualLeft = 0, preDrift = 0, imageAge = 0;
    double actAt = 0, actYaw = yaw, firstContact = -1;
    int contactFrames = 0;
    int opponentContactFrames = 0;
    std::string imageName;
    common::msg::GameData game;
    game.mode = game.MODE_NORM; game.remain_time = 900;
    for (int i = 0; i < 2; ++i) {
        game.red_players[i].name = names[i]; game.blue_players[i].name = names[i + 2];
        game.red_players[i].state = game.blue_players[i].state = common::msg::Player::PLAYER_NORMAL;
    }
    auto begin = [&]() {
        const auto &trial = trials[index];
        const bool kick = trial.name.find("kick") != std::string::npos;
        for (int i = 0; i < 4; ++i) {
            const double point[3] = {i < 2 ? 4.0 : -4.0, 0.365, i % 2 ? 2.5 : -2.5};
            robots[i]->getField("translation")->setSFVec3f(point);
            robots[i]->setVelocity(zero); robots[i]->resetPhysics();
        }
        const double position[3] = {-attack * (kick ? trial.forward : 1.4), 0.365,
                                    attack * (kick ? trial.left : 0.0)};
        double ballPosition[3] = {0.0, 0.07,
            !timing && (kick || trial.name == "roll") ? 0.0 : 2.5};
        robot->getField("translation")->setSFVec3f(position);
        robot->getField("rotation")->setSFRotation(rotation);
        robot->setVelocity(zero); robot->resetPhysics();
        if (ballViews) {
            ballPosition[0] = position[0] + attack * trial.forward;
            ballPosition[2] = position[2] - attack * trial.left;
        }
        if (jointViews) {
            ballPosition[0] = position[0] + attack * trial.ballForward;
            ballPosition[2] = position[2];
        }
        if (views) {
            const double targetPosition[3] = {position[0] + attack * trial.forward,
                .365, position[2] - attack * trial.left};
            const double targetRotation[4] = {0, 1, 0, yaw + trial.command * M_PI / 180};
            robots[targetIndex]->getField("translation")->setSFVec3f(targetPosition);
            robots[targetIndex]->getField("rotation")->setSFRotation(targetRotation);
            robots[targetIndex]->setVelocity(zero); robots[targetIndex]->resetPhysics();
        }
        ball->getField("translation")->setSFVec3f(ballPosition);
        ball->setVelocity(zero); ball->resetPhysics();
        phase = Reset; phaseAt = supervisor.getTime();
        maxDistance = maxSpeed = 0; anyFall = fall = contactFrames = 0; firstContact = -1;
        opponentContactFrames = anyTargetFall = targetFall = 0;
    };
    begin();
    while (supervisor.step(20) != -1 && rclcpp::ok() && index < trials.size()) {
        rclcpp::spin_some(node);
        const auto &trial = trials[index];
        const double now = supervisor.getTime(), elapsed = now - phaseAt;
        game.state = phase == Reset ? game.STATE_INIT : phase == Ready ? game.STATE_READY : game.STATE_PLAY;
        gamePub->publish(game);
        int commandedType = common::msg::BodyTask::TASK_WALK;
        for (int i = 0; i < 4; ++i) {
            common::msg::BodyTask body; body.type = body.TASK_WALK;
            common::msg::HeadTask head; head.yaw = 0; head.pitch = views && i == selected ? 20 : 60;
            if (jointViews && i == selected) head.pitch = trial.headPitch;
            if (ballViews && i == selected) head.pitch = trial.command;
            if (dynamicBall && i == selected) {
                if (trial.name.find("ball-head-near") == 0 && phase != Observe)
                    head.pitch = 20;
                if (trial.name.find("ball-head-far") == 0 && phase != Observe)
                    head.pitch = 60;
                if (phase == Observe && trial.warmupStep > 0) {
                    body.count = 2; body.step = trial.warmupStep;
                    if (trial.name == "ball-turn-left") body.turn = 10;
                    if (trial.name == "ball-turn-right") body.turn = -10;
                }
            }
            if (dynamicRobot && i == selected) {
                if (trial.name.find("robot-head-") == 0 && phase != Observe)
                    head.pitch = 40;
                if (phase == Observe && trial.warmupStep > 0) {
                    body.count = 2; body.step = trial.warmupStep;
                    if (trial.name == "robot-turn-left") body.turn = 10;
                    if (trial.name == "robot-turn-right") body.turn = -10;
                }
            }
            if (i == selected && phase == Warmup && trial.warmupStep > 0) {
                body.count = 2; body.step = trial.warmupStep;
            }
            if (i == selected && phase == Act) {
                if (trial.name == "walk") { body.count = 2; body.step = trial.command; }
                if (trial.name == "turn") { body.count = 2; body.turn = trial.command; }
                if (trial.name.find("kick") != std::string::npos && elapsed < trial.pulse) {
                    body.type = body.TASK_ACT; body.count = 1; body.actname = trial.name;
                }
            }
            if (i == selected) commandedType = body.type;
            bodies[i]->publish(body); heads[i]->publish(head);
        }
        const auto *pos = robot->getPosition(), *velocity = robot->getVelocity();
        const auto *orientation = robot->getOrientation();
        const double angle = std::atan2(-orientation[6], orientation[0]) * 180.0 / M_PI;
        const auto *bp = ball->getPosition(), *bv = ball->getVelocity();
        const int contactCount = sharedContacts(ball, robot);
        const int opponentContacts = blocking ? sharedContacts(ball, robots[targetIndex]) : 0;
        if (phase == Act || phase == Measure) {
            maxDistance = std::max(maxDistance, std::hypot(bp[0] - startX, bp[2] - startZ));
            maxSpeed = std::max(maxSpeed, std::hypot(bv[0], bv[2]));
            if (fall) anyFall = fall;
            if (targetFall) anyTargetFall = targetFall;
            if (opponentContacts > 0) ++opponentContactFrames;
            if (contactCount > 0) {
                ++contactFrames;
                if (firstContact < 0) firstContact = now - actAt;
            }
        }
        trace << now << ',' << index << ',' << trial.name << ',' << trial.repeat << ',' << phaseName(phase)
              << ',' << elapsed << ',' << pos[0] << ',' << pos[2] << ',' << pos[1] << ',' << angle
              << ',' << velocity[0] << ',' << velocity[2] << ',' << velocity[4]
              << ',' << bp[0] << ',' << bp[2] << ',' << bp[1] << ',' << bv[0] << ',' << bv[2] << ',' << fall
              << ',' << contactCount << ',' << commandedType << ',' << opponentContacts;
        const auto *opponentPosition = robots[targetIndex]->getPosition();
        trace << ',' << opponentPosition[0] << ',' << opponentPosition[2]
              << ',' << opponentPosition[1] << ',' << targetFall << '\n';
        if (phase == Reset && elapsed >= 1.5) { phase = Ready; phaseAt = now; }
        else if (phase == Ready && elapsed >= 1.5) {
            phase = timing ? Warmup : Observe; phaseAt = now;
        }
        else if (phase == Warmup && elapsed >= 3.0) { phase = Observe; phaseAt = now; }
        else if (phase == Observe && elapsed >= trial.stopWait) {
            std::ostringstream image; image << "trial-" << std::setw(3) << std::setfill('0') << index << ".ppm";
            imageName = image.str(); imageAge = now - imageAt;
            const bool saved = saveImage(output + "/images/" + imageName, camera);
            if (staticViews) {
                const auto *targetPosition = ballViews ? bp : robots[targetIndex]->getPosition();
                const auto *targetVelocity = ballViews ? bv : robots[targetIndex]->getVelocity();
                viewSamples << index << ',' << trial.name << ',' << trial.repeat << ',' << now
                    << ',' << imageName << ',' << saved << ',' << imageAge << ',' << now - imuAt
                    << ',' << now - headAt << ',' << robotName << ',' << (ballViews ? "ball" : names[targetIndex])
                    << ',' << pos[0] << ',' << pos[2] << ',' << pos[1]
                    << ',' << targetPosition[0] << ',' << targetPosition[2] << ',' << targetPosition[1]
                    << ',' << targetVelocity[0] << ',' << targetVelocity[2]
                    << ',' << observedImu.yaw << ',' << observedImu.pitch << ',' << observedImu.roll
                    << ',' << observedHead.yaw << ',' << observedHead.pitch << ',' << fall
                    << ',' << velocity[0] << ',' << velocity[2] << ',' << velocity[4]
                    << ',' << elapsed << ',' << bp[0] << ',' << bp[2] << ',' << bp[1]
                    << ',' << bv[0] << ',' << bv[2] << '\n';
                viewSamples.flush();
            }
            startX = bp[0]; startZ = bp[2]; maxDistance = maxSpeed = 0;
            actualForward = attack * (startX - pos[0]); actualLeft = -attack * (startZ - pos[2]);
            preDrift = std::hypot(startX, startZ);
            actYaw = angle * M_PI / 180;
            if (timing) {
                const double target[3] = {
                    pos[0] + std::cos(actYaw) * trial.forward - std::sin(actYaw) * trial.left,
                    .07,
                    pos[2] - std::sin(actYaw) * trial.forward - std::cos(actYaw) * trial.left};
                ball->getField("translation")->setSFVec3f(target);
                ball->setVelocity(zero); ball->resetPhysics();
                startX = target[0]; startZ = target[2];
                actualForward = trial.forward; actualLeft = trial.left; preDrift = 0;
                if (blocking && trial.blockerForward >= 0) {
                    const double opponent[3] = {
                        startX + std::cos(actYaw) * trial.blockerForward -
                            std::sin(actYaw) * trial.blockerLeft,
                        .365,
                        startZ - std::sin(actYaw) * trial.blockerForward -
                            std::cos(actYaw) * trial.blockerLeft};
                    const double facing[4] = {0, 1, 0, actYaw + M_PI};
                    robots[targetIndex]->getField("translation")->setSFVec3f(opponent);
                    robots[targetIndex]->getField("rotation")->setSFRotation(facing);
                    robots[targetIndex]->setVelocity(zero); robots[targetIndex]->resetPhysics();
                }
            }
            phase = Act; phaseAt = actAt = now;
            if (trial.name == "roll") {
                const double rolling[6] = {attack * trial.command, 0, 0, 0, 0, 0}; ball->setVelocity(rolling);
            }
        } else if (phase == Act && elapsed >= (staticViews ? .02 :
            trial.name == "walk" || trial.name == "turn" ? 8.0 : 1.5)) {
            phase = Measure; phaseAt = now;
        } else if (phase == Measure && elapsed >= (staticViews ? .02 : 5.0)) {
            const double dx = bp[0] - startX, dz = bp[2] - startZ;
            const double finalForward = timing ? std::cos(actYaw) * dx - std::sin(actYaw) * dz : attack * dx;
            const double finalLeft = timing ? -std::sin(actYaw) * dx - std::cos(actYaw) * dz : -attack * dz;
            summary << index << ',' << trial.name << ',' << trial.repeat << ',' << trial.command << ','
                    << trial.forward << ',' << trial.left << ',' << actualForward << ',' << actualLeft << ',' << preDrift
                    << ',' << maxDistance << ',' << finalForward << ',' << finalLeft
                    << ',' << maxSpeed << ',' << anyFall << ',' << imageName << ',' << imageAge
                    << ',' << trial.warmupStep << ',' << trial.stopWait << ',' << trial.pulse
                    << ',' << contactFrames << ',' << firstContact << ',' << actYaw * 180 / M_PI
                    << ',' << trial.blockerForward << ',' << trial.blockerLeft
                    << ',' << opponentContactFrames << ',' << anyTargetFall << '\n';
            summary.flush(); trace.flush();
            RCLCPP_INFO(node->get_logger(), "trial %zu/%zu %s distance=%.3f speed=%.3f fall=%d",
                index + 1, trials.size(), trial.name.c_str(), maxDistance, maxSpeed, anyFall);
            if (++index < trials.size()) begin();
        }
    }
    trace.close(); summary.close();
    supervisor.simulationQuit(index == trials.size() ? 0 : 4);
    rclcpp::shutdown();
    return index == trials.size() ? 0 : 4;
}
