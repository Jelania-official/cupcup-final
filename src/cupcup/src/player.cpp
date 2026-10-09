#include <rclcpp/rclcpp.hpp>
#include <common/msg/body_task.hpp>
#include <common/msg/game_data.hpp>
#include <common/msg/head_angles.hpp>
#include <common/msg/head_task.hpp>
#include <common/msg/imu_data.hpp>
#include <common/msg/location.hpp>
#include <common/msg/talk.hpp>
#include <common/srv/get_color.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ball_tracker.hpp"  // BallObservation type only; no tracker instance.
#include "ball_perception.hpp"
#include "ball_appearance.hpp"
#include "camera_geometry.hpp"
#include "sensor_time.hpp"
#include "strategy_logic.hpp"
#include "world_model.hpp"
#include "match_policy.hpp"
#include "robot_perception.hpp"

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {

enum class Color { Invalid, Red, Blue };

Color colorFromRobot(const std::string &name)
{
    if (name.find("red") != std::string::npos) return Color::Red;
    if (name.find("blue") != std::string::npos) return Color::Blue;
    return Color::Invalid;
}

int robotId(const std::string &name)
{
    if (name.empty() || name.back() < '0' || name.back() > '9') return 0;
    return name.back() - '0';
}

double nowSeconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

double clampValue(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

double wrapDegrees(double value)
{
    while (value > 180.0) value -= 360.0;
    while (value < -180.0) value += 360.0;
    return value;
}

using Ball = cupcup::BallObservation;

using RobotDetection = cupcup::RobotBox;

enum class ForwardState { Wait, Search, Approach, Orbit, Align, Settle, Kick, Verify, Recover };

const char *tacticalActionName(cupcup::TacticalAction action)
{
    switch (action) {
        case cupcup::TacticalAction::Hold: return "HOLD";
        case cupcup::TacticalAction::Chase: return "CHASE";
        case cupcup::TacticalAction::Support: return "SUPPORT";
        case cupcup::TacticalAction::Defend: return "DEFEND";
        case cupcup::TacticalAction::Clear: return "CLEAR";
    }
    return "UNKNOWN";
}

const char *stateName(ForwardState state)
{
    switch (state) {
        case ForwardState::Wait: return "WAIT";
        case ForwardState::Search: return "SEARCH";
        case ForwardState::Approach: return "APPROACH";
        case ForwardState::Orbit: return "ORBIT";
        case ForwardState::Align: return "ALIGN";
        case ForwardState::Settle: return "SETTLE";
        case ForwardState::Kick: return "KICK";
        case ForwardState::Verify: return "VERIFY";
        case ForwardState::Recover: return "RECOVER";
    }
    return "UNKNOWN";
}

class CupcupStrategy
{
public:
    CupcupStrategy(
        const std::shared_ptr<rclcpp::Node> &node,
        const std::string &robot,
        Color color,
        int id,
        const rclcpp::Publisher<common::msg::BodyTask>::SharedPtr &bodyPublisher,
        const rclcpp::Publisher<common::msg::HeadTask>::SharedPtr &headPublisher,
        const rclcpp::Publisher<common::msg::Talk>::SharedPtr &talkPublisher,
        const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &debugPublisher)
        : node_(node), robot_(robot),
          teammate_(color == Color::Red ? "red_" : "blue_"),
          color_(color), id_(id),
          bodyPublisher_(bodyPublisher), headPublisher_(headPublisher),
          talkPublisher_(talkPublisher), debugPublisher_(debugPublisher)
    {
        cv::setNumThreads(1);

        const auto modelPath = node_->declare_parameter<std::string>("ball_model", "");
        const auto verifierPath = node_->declare_parameter<std::string>("ball_verifier_model", "");
        minScore_ = clampValue(node_->declare_parameter<double>("ball_min_score", 0.30), 0.20, 0.90);
        groundBallEnabled_ = node_->declare_parameter<bool>("ball_ground_projection", true);
        geometryKickEnabled_ = node_->declare_parameter<bool>("kick_geometry_alignment", true);
        ballPatternFilter_ = node_->declare_parameter<bool>("ball_pattern_filter", true);
        robotHeightEstimate_ = clampValue(
            node_->declare_parameter<double>("robot_height_estimate", 0.68), 0.45, 0.90);
        cameraRootHeight_ = clampValue(
            node_->declare_parameter<double>("camera_root_height", 0.345), 0.25, 0.50);
        leftKickX_ = clampValue(node_->declare_parameter<double>("left_kick_x", 0.399), 0.30, 0.49);
        rightKickX_ = clampValue(node_->declare_parameter<double>("right_kick_x", 0.575), 0.51, 0.70);
        kickY_ = clampValue(node_->declare_parameter<double>("kick_y", 0.78), 0.50, 0.88);
        kickPitch_ = clampValue(node_->declare_parameter<double>("kick_pitch", 60.0), 35.0, 70.0);
        forwardBoundaryMargin_ = clampValue(
            node_->declare_parameter<double>("forward_boundary_margin", 0.75), 0.20, 1.20);
        defenderBoundaryMargin_ = clampValue(
            node_->declare_parameter<double>("defender_boundary_margin", 0.75), 0.20, 1.20);
        const int configuredConfirmHits = static_cast<int>(
            node_->declare_parameter<int>("ball_confirm_hits", 2));
        ballConfirmHits_ = std::max(2, std::min(6, configuredConfirmHits));
        lifecycle_.setRestartGrace(clampValue(
            node_->declare_parameter<double>("restart_grace", 1.50), 0.50, 4.0));
        defenderHomeX_ = clampValue(
            node_->declare_parameter<double>("defender_home_x", 1.55), 0.80, 2.60);
        defenderHomeZ_ = clampValue(
            node_->declare_parameter<double>("defender_home_z", 0.0), -2.0, 2.0);
        defenderAnchorGain_ = clampValue(
            node_->declare_parameter<double>("defender_anchor_gain", 0.45), 0.15, 0.80);
        supportX_ = clampValue(
            node_->declare_parameter<double>("support_x", 0.40), 0.10, 1.20);
        claimStaleAfter_ = clampValue(
            node_->declare_parameter<double>("claim_stale_after", 1.20), 0.60, 3.0);
        takeoverAfter_ = clampValue(
            node_->declare_parameter<double>("takeover_after", 2.50), 1.20, 6.0);
        kickDirectionHysteresis_ = clampValue(
            node_->declare_parameter<double>("kick_direction_hysteresis", 0.25), 0.0, 2.0);
        defenderClearEnabled_ = node_->declare_parameter<bool>("defender_clear_enabled", true);
        actionPulseSeconds_ = clampValue(
            node_->declare_parameter<double>("kick_action_pulse", 1.0), 0.75, 1.30);
        kickSettleFrames_ = std::max(3, std::min(20, static_cast<int>(
            node_->declare_parameter<int>("kick_settle_frames", 16))));
        gameTimeout_ = clampValue(
            node_->declare_parameter<double>("game_timeout", 2.50), 1.0, 5.0);
        approachTimeout_ = clampValue(
            node_->declare_parameter<double>("approach_timeout", 60.0), 30.0, 90.0);
        orbitTimeout_ = clampValue(
            node_->declare_parameter<double>("orbit_timeout", 25.0), 15.0, 45.0);
        attackYaw_ = color_ == Color::Red ? 180.0 : 0.0;
        targetYaw_ = attackYaw_;
        yawSign_ = node_->declare_parameter<double>("imu_yaw_sign", 1.0);
        yawOffset_ = node_->declare_parameter<double>("imu_yaw_offset", 0.0);
        teammate_ += std::to_string(3 - id_);

        if (!modelPath.empty()) {
            try {
                ballNet_ = cv::dnn::readNetFromONNX(modelPath);
                RCLCPP_INFO(node_->get_logger(), "cupcup: loaded ball model %s", modelPath.c_str());
            } catch (const cv::Exception &error) {
                RCLCPP_WARN(node_->get_logger(),
                    "cupcup: cannot load ball model (%s); using traditional vision", error.what());
            }
        }

        if (!verifierPath.empty()) {
            try {
                ballVerifier_.load(verifierPath);
                RCLCPP_INFO(node_->get_logger(), "cupcup: loaded optional ball verifier %s",
                    verifierPath.c_str());
            } catch (const std::exception &error) {
                ballVerifier_.disable();
                RCLCPP_WARN(node_->get_logger(), "cupcup: ball verifier disabled: %s", error.what());
            }
        }

        imageSubscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            robot_ + "/sensor/image", 2,
            [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
                if (message->width == 0 || message->height == 0 || message->width > 4096 ||
                    message->height > 4096 || message->step < message->width * 3 ||
                    message->data.size() < static_cast<std::size_t>(message->step) * message->height) {
                    return;
                }
                if (message->encoding != sensor_msgs::image_encodings::RGB8 &&
                    message->encoding != sensor_msgs::image_encodings::BGR8) {
                    return;
                }
                cv::Mat received(static_cast<int>(message->height), static_cast<int>(message->width),
                    CV_8UC3, const_cast<unsigned char *>(message->data.data()), message->step);
                if (message->encoding == sensor_msgs::image_encodings::BGR8) {
                    cv::cvtColor(received, frame_, cv::COLOR_BGR2RGB);
                } else {
                    frame_ = received.clone();
                }
                imageAt_ = nowSeconds();
                imageSimTimeValid_ = cupcup::imageTimeMilliseconds(
                    message->header.stamp.sec, message->header.stamp.nanosec,
                    imageSimTimeMs_);
                imageHeadYaw_ = head_.yaw;
                imageHeadPitch_ = head_.pitch;
                imageImuYaw_ = imu_.yaw;
                imageImuPitch_ = imu_.pitch;
                imageImuRoll_ = imu_.roll;
                imageImuFall_ = imu_.fall;
                imageHeadAge_ = headAt_ > 0.0 ? imageAt_ - headAt_ : 99.0;
                imageHeadTargetDwell_ = headStability_.age(imageAt_);
                imageImuAge_ = imuAt_ > 0.0 ? imageAt_ - imuAt_ : 99.0;
                imageHeadSimAge_ = imageSimTimeValid_ && head_.time != 0U ?
                    cupcup::timestampAgeSeconds(imageSimTimeMs_, head_.time) : 99.0;
                imageImuSimAge_ = imageSimTimeValid_ && imu_.stamp != 0U ?
                    cupcup::timestampAgeSeconds(imageSimTimeMs_, imu_.stamp) : 99.0;
                const uint32_t locationStamp = cupcup::optionalStamp(location_);
                imageLocationSimAge_ = imageSimTimeValid_ && locationStamp != 0U ?
                    cupcup::timestampAgeSeconds(imageSimTimeMs_, locationStamp) : 99.0;
                ++imageSequence_;
            });

        imuSubscription_ = node_->create_subscription<common::msg::ImuData>(
            robot_ + "/sensor/imu", 2,
            [this](common::msg::ImuData::ConstSharedPtr message) {
                if (!std::isfinite(message->yaw) || !std::isfinite(message->pitch) ||
                    !std::isfinite(message->roll)) return;
                imu_ = *message;
                if (std::getenv("CUPCUP_TRACE_PATH") != nullptr)
                    imuHistory_.push(message->stamp, *message);
                imuAt_ = nowSeconds();
            });

        headSubscription_ = node_->create_subscription<common::msg::HeadAngles>(
            robot_ + "/sensor/joint/head", 2,
            [this](common::msg::HeadAngles::ConstSharedPtr message) {
                if (!std::isfinite(message->yaw) || !std::isfinite(message->pitch)) return;
                headStability_.observe(message->yaw, message->pitch, nowSeconds());
                head_ = *message;
                if (std::getenv("CUPCUP_TRACE_PATH") != nullptr)
                    headHistory_.push(message->time, *message);
                headAt_ = nowSeconds();
            });

        gameSubscription_ = node_->create_subscription<common::msg::GameData>(
            "/sensor/game", 2,
            [this](common::msg::GameData::ConstSharedPtr message) {
                gameData_ = *message;
                gameState_ = gameData_.state;
                gameAt_ = nowSeconds();
            });

        locationSubscription_ = node_->create_subscription<common::msg::Location>(
            "/sensor/" + robot_ + "_location", 2,
            [this](common::msg::Location::ConstSharedPtr message) {
                if (!std::isfinite(message->x) || !std::isfinite(message->z)) return;
                location_ = *message;
                locationAt_ = nowSeconds();
                ++locationSequence_;
            });

        talkSubscription_ = node_->create_subscription<common::msg::Talk>(
            teammate_ + "/talk/talk_str", 2,
            [this](common::msg::Talk::ConstSharedPtr message) {
                teammateTalk_ = message->talk_str;
                teammateTalkAt_ = nowSeconds();
                ++teammateTalkSequence_;
            });

        enteredAt_ = uprightAt_ = nowSeconds();
    }

    void tick()
    {
        common::msg::BodyTask body;
        body.type = common::msg::BodyTask::TASK_WALK;
        body.count = 0;
        common::msg::HeadTask headTask;
        headTask.yaw = commandedHeadYaw_;
        headTask.pitch = commandedHeadPitch_;

        const double time = nowSeconds();
        navigationTargetValid_ = false;
        const int score = gameData_.red_score + gameData_.blue_score;
        if (lifecycle_.observe(gameState_, score, time) != cupcup::LifecycleEvent::None) {
            reset(time);
        }

        const bool freshFrame = processedImageSequence_ != imageSequence_;
        freshImage_ = freshFrame;
        bool acceptedBallMeasurement = false;
        if (freshFrame && !frame_.empty()) {
            const Ball detected = detect(frame_);
            // The initial field replay showed occasional one-cell detector
            // jumps during the six-pose search. Keep the three-frame safety
            // confirmation, but do not discard a plausible reacquisition at
            // the edge of that known noise envelope.
            const double confirmationGate = state_ == ForwardState::Search ? 0.28 : 0.20;
            const bool confirmed = visibleHits_ >= ballConfirmHits_;
            const bool consistent = ball_.valid && detected.valid &&
                std::hypot(detected.x - ball_.x, detected.y - ball_.y) < confirmationGate;
            if (detected.valid) {
                // Before confirmation, accept a new candidate as the first
                // sample. After confirmation, hold the last target through a
                // single implausible detector jump instead of steering the
                // body toward a different grid cell.
                if (consistent || !confirmed) {
                    visibleHits_ = consistent ? std::min(visibleHits_ + 1, 20) : 1;
                    ball_ = detected;
                    ballSeenAt_ = time;
                    acceptedBallMeasurement = true;
                    localBallPoint_ = cupcup::camera::GroundPoint();
                    localBallPointAt_ = time;
                    localBallHeadYaw_ = imageHeadYaw_;
                    localBallHeadPitch_ = imageHeadPitch_;
                    if (groundBallEnabled_ && imageImuFall_ == 0 && cupcup::usableBallReceiptGeometry(
                            imageHeadAge_, imageImuAge_, imageHeadTargetDwell_)) {
                        const auto pose = cupcup::camera::seurPose(
                            yawSign_ * imageImuYaw_ + yawOffset_, imageImuPitch_, imageImuRoll_,
                            imageHeadYaw_, imageHeadPitch_, cameraRootHeight_);
                        localBallPoint_ = cupcup::camera::project(
                            pose, ball_.x, ball_.y, frame_.cols, frame_.rows);
                        if (std::hypot(localBallPoint_.x, localBallPoint_.z) > 8.0)
                            localBallPoint_.valid = false;
                    }
                    // Preserve the capture timestamp of this accepted ball,
                    // not the timestamp of a later frame without a detection.
                    ballImageSimTimeMs_ = imageSimTimeValid_ ? imageSimTimeMs_ : 0U;
                    ballImageHeadPitch_ = imageHeadPitch_;
                    ballImageImuPitch_ = imageImuPitch_;
                    ballImageHeadSimAge_ = imageHeadSimAge_;
                    ballImageImuSimAge_ = imageImuSimAge_;
                    if (std::getenv("CUPCUP_TRACE_PATH") != nullptr) {
                        common::msg::HeadAngles captureHead;
                        common::msg::ImuData captureImu;
                        ballGroundPoint_ = cupcup::camera::GroundPoint();
                        ballImageHeadSimAge_ = ballImageImuSimAge_ = 99.0;
                        const bool headMatched = headHistory_.nearest(ballImageSimTimeMs_,
                            captureHead, ballImageHeadSimAge_);
                        const bool imuMatched = imuHistory_.nearest(ballImageSimTimeMs_,
                            captureImu, ballImageImuSimAge_);
                        if (headMatched && imuMatched) {
                            ballImageHeadPitch_ = captureHead.pitch;
                            ballImageImuPitch_ = captureImu.pitch;
                            ballCameraPose_ = cupcup::camera::seurPose(captureImu.yaw,
                                captureImu.pitch, captureImu.roll, captureHead.yaw,
                                captureHead.pitch, cameraRootHeight_);
                            // Normal world's ball radius is 0.07 m. The opt-in
                            // calibration fixture explicitly fixes its center
                            // at 0.05 m; do not confuse that with normal play.
                            const double planeHeight =
                                std::getenv("CUPCUP_PERCEPTION_CALIBRATION") ? 0.05 : 0.07;
                            ballGroundPoint_ = cupcup::camera::project(ballCameraPose_,
                                ball_.x, ball_.y, frame_.cols, frame_.rows, planeHeight);
                        }
                    }
                    lastBearing_ = head_.yaw + std::atan((0.5 - ball_.x) * 2.0 * std::tan(1.3613 / 2.0)) *
                        180.0 / M_PI;
                } else if (time - ballSeenAt_ > 1.0) {
                    ball_.valid = false;
                    visibleHits_ = 0;
                }
            } else {
                // Preserve a short observation grace period. The validated
                // implementation did not reset confirmation on one dropped
                // image, which is common while the head is moving.
                if (time - ballSeenAt_ > 1.0 || !confirmed) {
                    ball_.valid = false;
                    visibleHits_ = 0;
                }
            }
            processedImageSequence_ = imageSequence_;
        }

        const bool sensorsFresh = time - imageAt_ < 0.8 && time - imuAt_ < 0.8 &&
            time - headAt_ < 0.8 && time - gameAt_ < gameTimeout_;
        const bool locationFresh = time - locationAt_ < 2.0;
        updateWorldModel(time, freshFrame && acceptedBallMeasurement);
        const bool fallen = imu_.fall != common::msg::ImuData::FALL_NONE;
        const bool playerActive = isPlayerActive();
        const bool healthReady = lifecycle_.canPlay(gameState_, time) && playerActive &&
            sensorsFresh && locationFresh && !fallen && time - uprightAt_ >= 1.5;
        healthy_ = healthReady;
        if (fallen) uprightAt_ = time;

        if (!healthReady) {
            sharedObstacles_.clear();
            policyMemory_.reset();
            if (time - lastHealthLogAt_ > 5.0) {
                RCLCPP_WARN(node_->get_logger(),
                    "%s waiting: state=%d image=%.2f imu=%.2f head=%.2f game=%.2f location=%.2f fall=%d",
                    robot_.c_str(), gameState_, age(imageAt_), age(imuAt_), age(headAt_),
                    age(gameAt_), age(locationAt_), imu_.fall);
                lastHealthLogAt_ = time;
            }
            stop(body);
            transition(ForwardState::Wait, time);
            visibleHits_ = 0;
            tacticalAction_ = cupcup::TacticalAction::Hold;
            claim_ = false;
            clearMode_ = false;
        } else {
            updateTeammateStatus();
            updateTacticalDecision(time, locationFresh);
            if (cupcup::isBallAction(tacticalAction_)) {
                runForward(body, headTask, time, locationFresh);
            } else {
                // Yielding invalidates near-foot preparation and kick telemetry.
                // Never resume an old SETTLE/KICK after a role handoff.
                transition(ForwardState::Wait, time);
                policyMemory_.reset();
                shotLaneSelected_ = false;
                shotYawOffset_ = 0.0;
                if (tacticalAction_ == cupcup::TacticalAction::Support)
                    runSupport(body, headTask, time, locationFresh);
                else if (tacticalAction_ == cupcup::TacticalAction::Defend)
                    runDefender(body, headTask, time, locationFresh);
                else stop(body);
            }
        }

        if (time - lastStrategyLogAt_ > 2.0) {
            const cupcup::PoseEstimate &mappedSelf = worldModel_.self();
            RCLCPP_INFO(node_->get_logger(), "%s %s tactical=%s claim=%d ball=%.2f radius=%.3f hits=%d bearing=%.1f loc=(%.2f,%.2f) map=(%.2f,%.2f) head=(%.1f,%.1f) image=(%.3f,%.3f) yaw=%.1f target=%.1f cmd=(%.3f,%.3f,%.1f)",
                robot_.c_str(), stateName(state_), tacticalActionName(tacticalAction_), claim_,
                ball_.score, ball_.radius, visibleHits_, lastBearing_,
                location_.x, location_.z, mappedSelf.x, mappedSelf.z,
                head_.yaw, head_.pitch, ball_.x, ball_.y,
                yawSign_ * imu_.yaw + yawOffset_, targetYaw_,
                body.step, body.lateral, body.turn);
            lastStrategyLogAt_ = time;
        }

        if (std::getenv("CUPCUP_PERCEPTION_CALIBRATION") != nullptr &&
            std::getenv("CUPCUP_TRACE_PATH") != nullptr && imageSimTimeValid_) {
            const int stage = static_cast<int>(imageSimTimeMs_ / 2000U) % 18;
            const double yaws[3] = {-20.0, 0.0, 20.0};
            headTask.yaw = yaws[(stage / 2) % 3];
            headTask.pitch = stage % 2 == 0 ? 20.0 : 40.0;
            stop(body);
        }

        if (!frame_.empty()) publishDebug();
        publishTalk(time);
        headPublisher_->publish(headTask);
        bodyPublisher_->publish(body);
        freshImage_ = false;
    }

private:
    double age(double stamp) const
    {
        return nowSeconds() - stamp;
    }

    void reset(double time)
    {
        localBallPoint_ = cupcup::camera::GroundPoint();
        headStability_.reset();
        ball_ = Ball();
        ballImageSimTimeMs_ = 0U;
        worldModel_.reset();
        worldLocationSequence_ = locationSequence_;
        worldTeammateSequence_ = teammateTalkSequence_;
        worldRobotSequence_ = robotMeasurementSequence_;
        keeper_ = RobotDetection();
        robotDetections_.clear();
        robotNumberPatches_.clear();
        keeperHits_ = 0;
        visibleHits_ = 0;
        processedImageSequence_ = imageSequence_;
        commandedHeadYaw_ = 0.0;
        commandedHeadPitch_ = 20.0;
        leftFoot_ = true;
        actionIssued_ = false;
        shotYawOffset_ = 0.0;
        shotLaneSelected_ = false;
        targetYaw_ = attackYaw_;
        teammateStatus_ = cupcup::TeamStatus();
        tacticalDecision_ = cupcup::TacticalDecision();
        matchIntent_ = cupcup::MatchIntent();
        policyMemory_.reset();
        tacticalAction_ = cupcup::TacticalAction::Hold;
        claim_ = false;
        clearMode_ = false;
        healthy_ = false;
        boundaryGuard_.reset();
        transition(ForwardState::Wait, time);
        uprightAt_ = time;
        defenderEnteredAt_ = time;
    }

    void transition(ForwardState next, double time)
    {
        if (next == state_) return;
        if (next == ForwardState::Search) {
            searchLowFirst_ = state_ == ForwardState::Recover;
        }
        if (next == ForwardState::Align || next == ForwardState::Settle) {
            shotLaneSelected_ = false;
        }
        if (next == ForwardState::Verify || next == ForwardState::Recover)
            policyMemory_.reset();
        state_ = next;
        enteredAt_ = time;
        stableFrames_ = 0;
        missedAlignmentFrames_ = 0;
        RCLCPP_INFO(node_->get_logger(), "%s strategy -> %s", robot_.c_str(), stateName(state_));
        if (next == ForwardState::Kick) {
            actionIssued_ = false;
            RCLCPP_INFO(node_->get_logger(), "%s kick foot=%s lane_offset=%.1f",
                robot_.c_str(), leftFoot_ ? "left" : "right", shotYawOffset_);
            if (clearMode_) {
                RCLCPP_INFO(node_->get_logger(), "%s defender clear kick foot=%s",
                    robot_.c_str(), leftFoot_ ? "left" : "right");
            }
        }
    }

    void stop(common::msg::BodyTask &body) const
    {
        body.type = common::msg::BodyTask::TASK_WALK;
        body.count = 0;
        body.step = 0.0;
        body.lateral = 0.0;
        body.turn = 0.0;
    }

    void walk(common::msg::BodyTask &body, double forward, double lateral, double turn)
    {
        body.type = common::msg::BodyTask::TASK_WALK;
        body.count = 1;
        body.step = clampValue(forward, -0.025, 0.05);
        body.lateral = clampValue(lateral, -0.028, 0.028);
        body.turn = clampValue(turn, -15.0, 15.0);
        enforceSafety(body);
    }

    void enforceSafety(common::msg::BodyTask &body)
    {
        if (age(locationAt_) > 2.0) {
            body.step = 0.0;
            body.lateral = 0.0;
            body.count = 0;
            return;
        }

        const cupcup::TeamColor teamColor = color_ == Color::Red ?
            cupcup::TeamColor::Red : cupcup::TeamColor::Blue;
        const cupcup::PlayerRole role = id_ == 1 ? cupcup::PlayerRole::Forward :
            cupcup::PlayerRole::Defender;
        const double margin = id_ == 1 ? forwardBoundaryMargin_ : defenderBoundaryMargin_;
        const cupcup::FieldPoint pose = selfPosition(nowSeconds());
        const cupcup::BoundaryDecision boundary = boundaryGuard_.update(
            teamColor, role, pose.x, pose.z, margin);
        const double guardedMargin = boundary.stopWalking ? margin + 0.18 : margin;
        const bool unsafeStep = !cupcup::FieldGeometry::safeMotion(teamColor, id_, pose,
            yawSign_ * imu_.yaw + yawOffset_, body.step, body.lateral, guardedMargin);
        if (unsafeStep && body.type == common::msg::BodyTask::TASK_WALK) {
            // Stop only the forbidden direction. Stopping both directions can
            // deadlock a forward at kickoff when it starts near its boundary.
            body.step = 0.0;
            body.lateral = 0.0;
            body.count = std::abs(body.turn) > 0.01 ? 2 : 0;
        }
    }

    cupcup::FieldPoint selfPosition(double time) const
    {
        const cupcup::PoseEstimate &self = worldModel_.self();
        if (self.fresh(time, 2.0)) return {self.x, self.z};
        return {location_.x, location_.z};
    }

    bool visible(double time) const
    {
        return ball_.valid && time - ballSeenAt_ < 0.45 && visibleHits_ >= ballConfirmHits_;
    }

    double bearingDegrees() const
    {
        if (frame_.empty()) return 0.0;
        constexpr double horizontalFov = 1.3613;
        return head_.yaw + std::atan((0.5 - ball_.x) * 2.0 * std::tan(horizontalFov / 2.0)) *
            180.0 / M_PI;
    }

    double headingError() const
    {
        return wrapDegrees(targetYaw_ - (yawSign_ * imu_.yaw + yawOffset_));
    }

    bool obstacleAhead() const
    {
        return keeper_.valid && keeperHits_ >= 2 && age(keeperAt_) < 0.45 &&
            keeper_.score >= 0.45 && keeper_.y > 0.18 && keeper_.y < 0.72 &&
            keeper_.x > 0.34 && keeper_.x < 0.66;
    }

    bool hasSharedKickTarget(double time) const
    {
        const auto &ball = worldModel_.ball();
        const auto &target = matchIntent_.kickTarget;
        return matchIntent_.kickTargetValid && ball.fresh(time, 0.65) &&
            std::isfinite(target.x) && std::isfinite(target.z) &&
            std::hypot(target.x - ball.x, target.z - ball.z) > 1e-6;
    }

    void updateTargetYaw(bool locationFresh)
    {
        if (!locationFresh) return;
        const cupcup::TeamColor teamColor = color_ == Color::Red ?
            cupcup::TeamColor::Red : cupcup::TeamColor::Blue;
        const cupcup::PoseEstimate &self = worldModel_.self();
        const double selfX = self.fresh(nowSeconds(), 2.0) ? self.x : location_.x;
        const double selfZ = self.fresh(nowSeconds(), 2.0) ? self.z : location_.z;
        const double goalX = cupcup::FieldGeometry::attackGoalX(teamColor);
        if (std::abs(goalX - selfX) <= 0.7) return;
        const auto &ball = worldModel_.ball();
        const bool planned = hasSharedKickTarget(nowSeconds());
        const double originX = planned ? ball.x : selfX, originZ = planned ? ball.z : selfZ;
        const double kickX = planned ? matchIntent_.kickTarget.x : goalX;
        const double kickZ = planned ? matchIntent_.kickTarget.z : 0.0;
        // A shared direction is the decision, not the baseline for a second
        // visual lane decision. Retain legacy bias only without a valid plan.
        const double desired = wrapDegrees(cupcup::fieldHeading({originX, originZ}, {kickX, kickZ}) +
                                          (planned ? 0.0 : shotYawOffset_));
        // Supervisor localization is deliberately noisy (up to about one
        // metre in the supplied simulator).  Use it only as a slow bias; a
        // fast response makes the striker turn while it is still acquiring
        // the ball.
        targetYaw_ = wrapDegrees(targetYaw_ + 0.03 * wrapDegrees(desired - targetYaw_));
    }

    bool isPlayerActive() const
    {
        if (id_ < 1 || id_ > 2) return false;
        const auto &players = color_ == Color::Red ? gameData_.red_players : gameData_.blue_players;
        const int state = players[static_cast<std::size_t>(id_ - 1)].state;
        return state != common::msg::Player::PLAYER_WAIT &&
            state != common::msg::Player::PALYER_OUT;
    }

    void updateTeammateStatus()
    {
        teammateStatus_ = cupcup::parseTeamStatus(teammateTalk_);
    }

    void updateWorldModel(double time, bool newBallObservation)
    {
        if (worldLocationSequence_ != locationSequence_) {
            const double yaw = yawSign_ * imu_.yaw + yawOffset_;
            worldModel_.updateSelf(location_.x, location_.z, yaw, locationAt_);
            worldLocationSequence_ = locationSequence_;
        }

        if (worldTeammateSequence_ != teammateTalkSequence_) {
            teammateStatus_ = cupcup::parseTeamStatus(teammateTalk_);
            if (teammateStatus_.valid && teammateStatus_.hasPose &&
                teammateStatus_.poseAge <= 1.2) {
                worldModel_.updateTeammate(teammateStatus_.poseX, teammateStatus_.poseZ,
                    teammateStatus_.poseYaw, teammateTalkAt_ - teammateStatus_.poseAge);
            }
            if (teammateStatus_.valid && teammateStatus_.ball &&
                teammateStatus_.hasBallPosition && teammateStatus_.ballMapAge <= 0.45) {
                worldModel_.updateBall(teammateStatus_.ballX, teammateStatus_.ballZ,
                    teammateTalkAt_ - teammateStatus_.ballMapAge,
                    teammateStatus_.ballScore * 0.5, cupcup::BallPositionSource::Teammate);
            }
            worldTeammateSequence_ = teammateTalkSequence_;
        }

        if (newBallObservation && visible(time)) {
            const cupcup::PoseEstimate &self = worldModel_.self();
            if (self.fresh(time, 2.0)) {
                const double estimateTime = nowSeconds();
                const double distance = ballDistanceEstimate(estimateTime);
                const cupcup::FieldPoint position = localBallPointFresh(estimateTime) ?
                    cupcup::FieldPoint{self.x + localBallPoint_.x, self.z + localBallPoint_.z} :
                    cupcup::projectToField(self.x, self.z, yawSign_ * imu_.yaw + yawOffset_,
                                          distance, bearingDegrees());
                worldModel_.updateBall(position.x, position.z, ballSeenAt_, ball_.score,
                    localBallPointFresh(estimateTime) ? cupcup::BallPositionSource::GroundRay :
                        cupcup::BallPositionSource::Radius);
            }
        }

        if (worldRobotSequence_ != robotMeasurementSequence_) {
            const cupcup::PoseEstimate &self = worldModel_.self();
            std::vector<cupcup::RobotObservation> observations;
            // Complete known-size front markers supplement missed body boxes.
            // Original neck messages are targets, not measured extrinsics: keep
            // these diagnostic estimates just as uncertain as other candidates.
            const bool robotPoseUsable = self.fresh(time, 2.0) && imageImuFall_ == 0 &&
                cupcup::usableReceiptPose(imageHeadAge_, imageImuAge_);
            const bool markerGeometryUsable = robotPoseUsable &&
                cupcup::usableBallReceiptGeometry(imageHeadAge_, imageImuAge_, imageHeadTargetDwell_);
            if (markerGeometryUsable) {
                const auto camera = cupcup::camera::seurPose(yawSign_ * imageImuYaw_ + yawOffset_,
                    imageImuPitch_, imageImuRoll_, imageHeadYaw_, imageHeadPitch_);
                for (const auto &marker : robotNumberPatches_) {
                    const auto &t = marker.pose.translation;
                    const auto offset = cupcup::camera::rotate(camera.rotation, {{t[2], -t[0], -t[1]}});
                    const cupcup::FieldPoint point{self.x + camera.origin[0] + offset[0],
                                                  self.z + camera.origin[2] + offset[2]};
                    const double range = std::hypot(point.x - self.x, point.z - self.z);
                    observations.push_back({point, marker.panel.team, .25, 1.0 + range * .35,
                                           cupcup::RobotPositionSource::NumberSquare});
                }
            }
            if (robotPoseUsable && !frame_.empty()) for (const auto &robot : robotDetections_) {
                if (!robot.valid || robot.height <= 0.04 || robot.height >= 0.95) continue;
                // One visible body/landmark must not become two same-frame
                // tracks. A stale-pose marker does not suppress the fallback.
                const bool hasMarker = markerGeometryUsable && std::any_of(
                    robotNumberPatches_.begin(), robotNumberPatches_.end(), [&robot](const cupcup::RobotNumberPatch &m) {
                        return std::abs(m.panel.x - robot.x) <= robot.width / 2 &&
                            std::abs(m.panel.y - robot.y) <= robot.height / 2;
                    });
                if (hasMarker) continue;
                constexpr double horizontalFov = 1.3613;
                const double focalPixels = frame_.cols /
                    (2.0 * std::tan(horizontalFov / 2.0));
                const double range = clampValue(robotHeightEstimate_ * focalPixels /
                    (robot.height * frame_.rows), 0.45, 6.0);
                const double bearing = imageHeadYaw_ +
                    std::atan((0.5 - robot.x) * 2.0 * std::tan(horizontalFov / 2.0)) *
                    180.0 / M_PI;
                const cupcup::FieldPoint position = cupcup::projectToField(
                    self.x, self.z, yawSign_ * imageImuYaw_ + yawOffset_, range, bearing);
                observations.push_back({position, robot.team, robot.score * 0.25, 1.0 + range * 0.35,
                                       cupcup::RobotPositionSource::BoxHeight});
            }
            worldModel_.updateRobots(observations, imageAt_);
            worldRobotSequence_ = robotMeasurementSequence_;
        }

        worldModel_.expire(time);
    }

    bool localBallPointFresh(double time) const
    {
        return imu_.fall == 0 && localBallPoint_.valid && time >= localBallPointAt_ &&
            time - localBallPointAt_ <= .25 && headStability_.age(time) >= .8 &&
            cupcup::sameHeadTarget(localBallHeadYaw_, localBallHeadPitch_, head_.yaw, head_.pitch) &&
            cupcup::sameHeadTarget(localBallHeadYaw_, localBallHeadPitch_,
                                   commandedHeadYaw_, commandedHeadPitch_);
    }

    double ballDistanceEstimate(double time = nowSeconds()) const
    {
        if (!ball_.valid || ball_.radius <= 0.001) return 99.0;
        if (localBallPointFresh(time))
            return clampValue(std::hypot(localBallPoint_.x, localBallPoint_.z), 0.12, 8.0);
        // Unsettled targets/stale associated sensors/grazing rays fall back to
        // the existing coarse proxy, not a fabricated precise point. Neither
        // estimate replaces the near-foot pixel servo.
        return clampValue(0.050 / ball_.radius, 0.12, 8.0);
    }

    bool worldBall(double &x, double &z) const
    {
        const double time = nowSeconds();
        const cupcup::TimedPoint &ball = worldModel_.ball();
        if (!visible(time) || !ball.fresh(time, 0.45)) return false;
        x = ball.x;
        z = ball.z;
        return true;
    }

    void updateTacticalDecision(double time, bool locationFresh)
    {
        cupcup::MatchState input;
        input.color = color_ == Color::Red ? cupcup::TeamColor::Red : cupcup::TeamColor::Blue;
        input.id = id_;
        input.now = time;
        input.healthy = healthy_ && locationFresh;
        input.localBallVisible = visible(time);
        input.map = worldModel_;
        input.obstacles = cupcup::mapRobotObstacles(input.map, time);
        input.teammate = teammateStatus_;
        input.teammateMessageAge = time - teammateTalkAt_;
        const auto localObstacles = input.obstacles.size();
        cupcup::appendTeammateObstacles(input);
        sharedObstacles_.assign(input.obstacles.begin() + localObstacles, input.obstacles.end());
        cupcup::PolicyConfig config;
        config.supportX = supportX_;
        config.defenderHomeX = defenderHomeX_;
        config.defenderHomeZ = defenderHomeZ_;
        config.defenderAnchorGain = defenderAnchorGain_;
        config.boundaryMargin = id_ == 1 ? forwardBoundaryMargin_ : defenderBoundaryMargin_;
        config.claimStaleAfter = claimStaleAfter_;
        config.takeoverAfter = takeoverAfter_;
        config.defenderClearEnabled = defenderClearEnabled_;
        config.kickDirectionHysteresis = kickDirectionHysteresis_;
        matchIntent_ = cupcup::planMatch(input, config,
            kickDirectionHysteresis_ > 0.0 ? &policyMemory_ : nullptr);
        tacticalDecision_ = matchIntent_.tactical;
        tacticalAction_ = tacticalDecision_.action;
        claim_ = tacticalDecision_.claim;
        clearMode_ = tacticalAction_ == cupcup::TacticalAction::Clear;
    }

    void navigateTo(common::msg::BodyTask &body, double targetX, double targetZ,
                    double maxForward = 0.032)
    {
        navigationTargetValid_ = true;
        navigationTargetX_ = targetX;
        navigationTargetZ_ = targetZ;
        const cupcup::PoseEstimate &self = worldModel_.self();
        const double selfX = self.fresh(nowSeconds(), 2.0) ? self.x : location_.x;
        const double selfZ = self.fresh(nowSeconds(), 2.0) ? self.z : location_.z;
        const double dx = targetX - selfX;
        const double dz = targetZ - selfZ;
        const double distance = std::hypot(dx, dz);
        const double currentYaw = (yawSign_ * imu_.yaw + yawOffset_) * M_PI / 180.0;
        const double localForward = dx * std::cos(currentYaw) - dz * std::sin(currentYaw);
        const double localLeft = -dx * std::sin(currentYaw) - dz * std::cos(currentYaw);
        const double desiredYaw = std::atan2(-dz, dx) * 180.0 / M_PI;
        const double error = wrapDegrees(desiredYaw - currentYaw * 180.0 / M_PI);
        const double forward = distance < 0.18 ? 0.0 :
            clampValue(localForward * 0.10, -0.018, maxForward);
        const double lateral = distance < 0.12 ? 0.0 : clampValue(localLeft * 0.10, -0.020, 0.020);
        const double turn = distance < 0.12 ? clampValue(error * 0.20, -7.0, 7.0) :
            clampValue(error * 0.12, -8.0, 8.0);
        walk(body, forward, lateral, turn);
    }

    void runSupport(common::msg::BodyTask &body, common::msg::HeadTask &headTask,
                    double time, bool locationFresh)
    {
        // The ball owner gets the corridor.  The striker waits near the
        // centre line instead of crossing behind the defender and creating a
        // second claimant.  Once the owner loses its claim, normal chasing
        // resumes on the next tick.
        if (cupcup::needsBodySearch(tacticalAction_, worldModel_.ball().fresh(time, .65)))
            walk(body, 0.0, 0.0, 8.0);
        else if (locationFresh && matchIntent_.targetValid)
            navigateTo(body, matchIntent_.target.x, matchIntent_.target.z, 0.020);
        else stop(body);
        const bool hasBall = visible(time);
        if (hasBall) {
            commandedHeadYaw_ = clampValue(head_.yaw + bearingDegrees() / 3.0, -55.0, 55.0);
            commandedHeadPitch_ = clampValue(head_.pitch, 18.0, kickPitch_);
        } else {
            const int index = static_cast<int>((time - enteredAt_) * 0.8) % 4;
            static const double scan[] = {-35.0, 0.0, 35.0, 0.0};
            commandedHeadYaw_ = scan[index];
            commandedHeadPitch_ = 28.0;
        }
        headTask.yaw = commandedHeadYaw_;
        headTask.pitch = commandedHeadPitch_;
    }

    void trackHead(double time)
    {
        if (!visible(time)) return;
        const double correction = std::atan((0.5 - ball_.x) * 2.0 * std::tan(1.3613 / 2.0)) *
            180.0 / M_PI;
        const double recenteredYaw = cupcup::headRecenteringCommand(
            head_.yaw, ball_.x, ball_.radius, state_ == ForwardState::Orbit);
        if (recenteredYaw != head_.yaw) {
            // Once near the ball, gradually align the camera with the calibrated
            // kick view. The wider deadband is safe here because the body keeps
            // correcting the residual bearing while ORBIT is active.
            commandedHeadYaw_ = recenteredYaw;
        } else if (std::abs(ball_.x - 0.5) > 0.10) {
            commandedHeadYaw_ = clampValue(head_.yaw +
                clampValue(correction / 3.5, -2.5, 2.5), -60.0, 60.0);
        }
        const bool handoffView = state_ == ForwardState::Orbit && ball_.radius > 0.045 &&
            std::abs(ball_.x - 0.5) < 0.15 && ball_.y > 0.32;
        if (handoffView && commandedHeadPitch_ < kickPitch_ - 0.5) {
            // Move into the calibrated downward view while ORBIT still has
            // enough range to follow the ball. A direct jump loses near balls.
            commandedHeadPitch_ = std::min(kickPitch_, commandedHeadPitch_ +
                clampValue((kickPitch_ - commandedHeadPitch_) * 0.18, 0.5, 1.5));
        } else if (ball_.y < 0.30 || ball_.y > 0.68) {
            commandedHeadPitch_ = clampValue(head_.pitch +
                clampValue((ball_.y - 0.49) * 4.0, -1.8, 1.8), 8.0, kickPitch_);
        }
    }

    void selectShotLane(bool locationFresh)
    {
        if (hasSharedKickTarget(nowSeconds())) {
            shotYawOffset_ = 0.0;
            shotLaneSelected_ = true;
            return;
        }
        if (shotLaneSelected_) return;
        double worldX = 0.0;
        double worldZ = 0.0;
        if (locationFresh && worldBall(worldX, worldZ)) {
            // Near a touchline, bias the kick back toward the field before
            // considering the opponent.  The sign follows the camera/field
            // convention used by Location and is symmetric for both colors.
            if (worldZ > 1.75) shotYawOffset_ = 12.0;
            else if (worldZ < -1.75) shotYawOffset_ = -12.0;
            else if (clearMode_) {
                // A clearance should leave the central danger corridor.  Use
                // the side opposite the current image bearing when no
                // touchline correction is needed.
                shotYawOffset_ = lastBearing_ >= 0.0 ? -12.0 : 12.0;
            }
        }
        if (!clearMode_ && keeper_.valid && keeperHits_ >= 2 && age(keeperAt_) < 0.5) {
            const double keeperBearing = head_.yaw +
                std::atan((0.5 - keeper_.x) * 2.0 * std::tan(1.3613 / 2.0)) *
                180.0 / M_PI;
            // Positive image bearing is the robot's left.  Pick the opposite
            // lane when a stable opponent/keeper detection blocks the centre.
            shotYawOffset_ = keeperBearing >= 0.0 ? -20.0 : 20.0;
        }
        shotLaneSelected_ = true;
    }

    void runForward(common::msg::BodyTask &body, common::msg::HeadTask &headTask,
                    double time, bool locationFresh)
    {
        if (state_ == ForwardState::Wait) transition(ForwardState::Search, time);
        const double stateAge = time - enteredAt_;
        const bool hasBall = visible(time);
        updateTargetYaw(locationFresh);

        if (hasBall && state_ >= ForwardState::Approach && state_ <= ForwardState::Orbit) {
            trackHead(time);
        }

        if (hasBall && (state_ == ForwardState::Approach || state_ == ForwardState::Orbit ||
                        state_ == ForwardState::Align || state_ == ForwardState::Settle) &&
            !shotLaneSelected_ && locationFresh && std::abs(headingError()) < 25.0) {
            selectShotLane(locationFresh);
        }

        if (state_ == ForwardState::Search) {
            static const double scanYaw[] = {-55.0, 0.0, 55.0, 55.0, 0.0, -55.0};
            static const double highPitch[] = {18.0, 18.0, 18.0, 50.0, 50.0, 50.0};
            static const double lowPitch[] = {50.0, 50.0, 50.0, 18.0, 18.0, 18.0};
            const int index = static_cast<int>(stateAge / 1.0) % 6;
            commandedHeadYaw_ = scanYaw[index];
            commandedHeadPitch_ = searchLowFirst_ ? lowPitch[index] : highPitch[index];
            if (time - ballSeenAt_ < 2.5) {
                // Reacquire around the last bearing before expanding to the
                // full scan. This is the active-head-search pattern used by
                // high-performing teams and avoids an immediate blind sweep.
                commandedHeadYaw_ = clampValue(lastBearing_, -60.0, 60.0);
            }
            if (hasBall) transition(ForwardState::Approach, time);
            else {
                const auto search = cupcup::searchMotion(
                    stateAge, time - ballSeenAt_, locationFresh);
                if (search.forward > 0.0) walk(body, search.forward, 0.0, search.turn);
            }
        } else if (state_ == ForwardState::Approach) {
            if (!hasBall) {
                if (time - ballSeenAt_ > 1.0) transition(ForwardState::Search, time);
            } else {
                // A pure turn task is not reliable in this Webots gait: in
                // some initial poses the commanded yaw is accepted but the
                // robot does not change heading.  Use a small crawl while
                // turning so the approach cannot deadlock at a persistent
                // 20--40 degree bearing.  Once centered, use the calibrated
                // forward speed.
                const auto approach = cupcup::approachMotion(lastBearing_, ball_.radius);
                const double forward = approach.forward;
                // Recover the four-player kickoff view quickly when the ball
                // starts near the edge or behind the camera.
                double turn = approach.turn;
                double lateral = 0.0;
                if (obstacleAhead()) {
                    // The detector's x coordinate is image-left to image-right;
                    // positive lateral motion is robot-left.  Pass the nearer
                    // robot on the side with the shorter visual detour.
                    lateral = keeper_.x >= 0.5 ? 0.018 : -0.018;
                    turn += keeper_.x >= 0.5 ? 3.0 : -3.0;
                }
                walk(body, obstacleAhead() ? forward * 0.55 : forward, lateral,
                    clampValue(turn, -7.0, 7.0));
                // Radius alone is unreliable while the ball is at the image
                // edge. The validated initial run entered ORBIT only after
                // the ball was already roughly centred in the camera.
                if (ball_.radius > 0.045 && ball_.radius < 0.11 &&
                    std::abs(lastBearing_) < 30.0)
                    transition(ForwardState::Orbit, time);
            }
        } else if (state_ == ForwardState::Orbit) {
            if (!hasBall) {
                if (time - ballSeenAt_ > 1.0) transition(ForwardState::Search, time);
            } else {
                if (std::abs(lastBearing_) > 50.0) {
                    transition(ForwardState::Approach, time);
                    walk(body, 0.0, 0.0, clampValue(lastBearing_ * 0.25, -8.0, 8.0));
                }
                double turn = std::abs(headingError()) < 5.0 ? 0.0 :
                    clampValue(headingError() * 0.12, -4.5, 4.5);
                double lateral = clampValue(lastBearing_ * 0.0007 - turn * 0.0018, -0.025, 0.025);
                const double forward = clampValue((0.062 - ball_.radius) * 0.75, -0.015, 0.025);
                const double closingForward = head_.pitch > 25.0 && ball_.y < 0.48 ?
                    std::max(forward, clampValue((0.48 - ball_.y) * 0.08, 0.0, 0.025)) : forward;
                if (obstacleAhead()) {
                    lateral += keeper_.x >= 0.5 ? 0.016 : -0.016;
                    turn += keeper_.x >= 0.5 ? 3.0 : -3.0;
                }
                if (std::abs(lastBearing_) <= 50.0) {
                    walk(body, obstacleAhead() ? closingForward * 0.45 : closingForward,
                        clampValue(lateral, -0.028, 0.028), clampValue(turn, -8.0, 8.0));
                }
                if (cupcup::readyForAlignment(headingError(), lastBearing_, head_.yaw,
                    head_.pitch, kickPitch_, ball_.radius, ball_.y)) {
                    leftFoot_ = lastBearing_ >= 0.0;
                    transition(ForwardState::Align, time);
                }
            }
        } else if (state_ == ForwardState::Align || state_ == ForwardState::Settle) {
            commandedHeadYaw_ = 0.0;
            commandedHeadPitch_ = kickPitch_;
            const double desiredX = leftFoot_ ? leftKickX_ : rightKickX_;
            const bool fixedView = std::abs(head_.yaw) < 6.0 &&
                std::abs(head_.pitch - kickPitch_) < 5.0;
            cupcup::KickPoseControl metric;
            if (geometryKickEnabled_ && hasBall && localBallPointFresh(nowSeconds()))
                metric = cupcup::metricKickPose(localBallPoint_.x, localBallPoint_.z,
                    yawSign_ * imu_.yaw + yawOffset_, leftFoot_);
            const bool linedUp = hasBall && fixedView && std::abs(headingError()) < 13.0 &&
                (geometryKickEnabled_ ? metric.valid && metric.linedUp :
                    std::abs(ball_.x - desiredX) < 0.060 && std::abs(ball_.y - kickY_) < 0.070) &&
                ball_.radius > 0.048;
            const bool holdPose = hasBall && fixedView && std::abs(headingError()) < 19.0 &&
                (geometryKickEnabled_ ? metric.valid && metric.holdPose :
                    std::abs(ball_.x - desiredX) < 0.095 && std::abs(ball_.y - kickY_) < 0.105) &&
                ball_.radius > 0.042;
            const bool keepSettling = cupcup::updateKickAlignment(
                freshImage_, linedUp, holdPose, stableFrames_, missedAlignmentFrames_);
            if (state_ == ForwardState::Settle) {
                if (!keepSettling) transition(ForwardState::Align, time);
                else if (linedUp &&
                         cupcup::settledForKick(stateAge, stableFrames_, kickSettleFrames_))
                    transition(ForwardState::Kick, time);
            } else if (!hasBall) {
                if (time - ballSeenAt_ > 0.8) transition(ForwardState::Recover, time);
            } else if (fixedView) {
                if (std::abs(headingError()) > 27.0) transition(ForwardState::Orbit, time);
                else if (linedUp && stableFrames_ >= 3) transition(ForwardState::Settle, time);
                else if (!geometryKickEnabled_ || metric.valid)
                    walk(body, geometryKickEnabled_ ? metric.forward :
                              clampValue((kickY_ - ball_.y) * 0.16, -0.018, 0.025),
                          geometryKickEnabled_ ? metric.lateral :
                              clampValue((desiredX - ball_.x) * 0.20, -0.028, 0.028),
                          headingError() * 0.24);
            }
        } else if (state_ == ForwardState::Kick) {
            commandedHeadYaw_ = 0.0;
            commandedHeadPitch_ = stateAge < 0.55 ? kickPitch_ : 18.0;
            // The motion node samples the latest task asynchronously only when
            // its joint queue is nearly empty. Keep the action request latched
            // for one bounded acceptance window, then publish the normal stop
            // task after it. A much shorter pulse is routinely missed; keeping
            // it latched forever can enqueue the kick again after completion.
            if (stateAge < actionPulseSeconds_) {
                body.type = common::msg::BodyTask::TASK_ACT;
                body.count = 1;
                body.actname = leftFoot_ ? "left_kick" : "right_kick";
                actionIssued_ = true;
            } else if (stateAge >= actionPulseSeconds_ + 0.25) {
                shotYawOffset_ = 0.0;
                shotLaneSelected_ = false;
                transition(ForwardState::Verify, time);
            }
        } else if (state_ == ForwardState::Verify) {
            commandedHeadYaw_ = 0.0;
            commandedHeadPitch_ = 18.0;
            if (stateAge > 0.70 && hasBall && ball_.radius < 0.060) transition(ForwardState::Approach, time);
            else if (stateAge > 1.20) transition(ForwardState::Search, time);
        } else if (state_ == ForwardState::Recover) {
            if (stateAge < 1.5) walk(body, -0.02, recoveryCount_ % 2 ? 0.018 : -0.018, 0.0);
            else {
                ++recoveryCount_;
                transition(ForwardState::Search, time);
            }
        }

        if (((state_ == ForwardState::Approach || state_ == ForwardState::Align) &&
             time - enteredAt_ > approachTimeout_) ||
            (state_ == ForwardState::Orbit && time - enteredAt_ > orbitTimeout_)) {
            ++recoveryCount_;
            transition(ForwardState::Recover, time);
        }
        headTask.yaw = commandedHeadYaw_;
        headTask.pitch = commandedHeadPitch_;
        freshImage_ = false;
    }

    void runDefender(common::msg::BodyTask &body, common::msg::HeadTask &headTask,
                     double time, bool locationFresh)
    {
        static const double scanYaw[] = {-45.0, 0.0, 45.0, 45.0, 0.0, -45.0};
        static const double scanPitch[] = {25.0, 25.0, 25.0, 45.0, 45.0, 45.0};
        const double ageInState = time - defenderEnteredAt_;
        const bool hasBall = visible(time);
        updateTargetYaw(locationFresh);
        double ballBearing = 0.0;

        if (hasBall) {
            ballBearing = bearingDegrees();
            commandedHeadYaw_ = clampValue(head_.yaw + ballBearing / 3.0, -55.0, 55.0);
            commandedHeadPitch_ = cupcup::defenderTrackingPitch(head_.pitch, ball_.y);
        } else {
            const int index = static_cast<int>(ageInState) % 6;
            commandedHeadYaw_ = scanYaw[index];
            commandedHeadPitch_ = scanPitch[index];
        }

        // Follow the shared goal-side coverage target; the role boundary and
        // image safety layer remain independent of tactical placement.
        if (cupcup::needsBodySearch(tacticalAction_, worldModel_.ball().fresh(time, .65)))
            walk(body, 0.0, 0.0, 8.0);
        else if (locationFresh && matchIntent_.targetValid)
            navigateTo(body, matchIntent_.target.x, matchIntent_.target.z, 0.018);
        else stop(body);

        enforceSafety(body);
        headTask.yaw = commandedHeadYaw_;
        headTask.pitch = commandedHeadPitch_;
    }

    Ball detect(const cv::Mat &rgb)
    {
        if (rgb.empty()) return Ball();
        if (ballNet_.empty()) robotDetections_.clear();
        const auto ball = !ballNet_.empty() ? detectModel(rgb) : detectTraditional(rgb);
        robotNumberPatches_ = cupcup::detectRobotNumberPatches(rgb);
        ++robotMeasurementSequence_;
        return ball;
    }

    Ball detectModel(const cv::Mat &rgb)
    {
        const int side = std::max(rgb.cols, rgb.rows);
        const int left = (side - rgb.cols) / 2;
        const int top = (side - rgb.rows) / 2;
        cv::Mat square, resized;
        cv::copyMakeBorder(rgb, square, top, side - rgb.rows - top, left,
            side - rgb.cols - left, cv::BORDER_CONSTANT, cv::Scalar());
        cv::resize(square, resized, cv::Size(416, 416), 0, 0, cv::INTER_NEAREST);
        ballNet_.setInput(cv::dnn::blobFromImage(resized, 1.0 / 255.0));
        std::vector<cv::Mat> outputs;
        ballNet_.forward(outputs, ballNet_.getUnconnectedOutLayersNames());
        const double currentTime = nowSeconds();
        const Ball prior = state_ != ForwardState::Search &&
            ball_.valid && currentTime - ballSeenAt_ < 0.5 ? ball_ : Ball();
        Ball best;
        try {
            best = cupcup::decodeBall(outputs, rgb, minScore_, ballPatternFilter_, prior, &ballVerifier_);
        } catch (const std::exception &error) {
            ballVerifier_.disable();
            RCLCPP_WARN(node_->get_logger(), "cupcup: ball verifier failed; baseline restored: %s",
                error.what());
            best = cupcup::decodeBall(outputs, rgb, minScore_, ballPatternFilter_, prior);
        }
        robotDetections_ = cupcup::decodeRobotBoxes(outputs, rgb);
        if (!robotDetections_.empty()) {
            const auto &candidate = robotDetections_.front();
            const bool consistent = keeper_.valid &&
                std::abs(candidate.x - keeper_.x) < 0.15;
            keeper_ = candidate;
            keeperAt_ = currentTime;
            keeperHits_ = consistent ? std::min(keeperHits_ + 1, 20) : 1;
        } else if (currentTime - keeperAt_ > 0.5) {
            keeper_ = RobotDetection();
            keeperHits_ = 0;
        }
        return best;
    }

    Ball detectTraditional(const cv::Mat &rgb)
    {
        Ball best;
        cv::Mat small, hsv, white, grass, gray;
        const double scale = 320.0 / std::max(1, rgb.cols);
        cv::resize(rgb, small, cv::Size(), scale, scale, cv::INTER_AREA);
        cv::cvtColor(small, hsv, cv::COLOR_RGB2HSV);
        cv::inRange(hsv, cv::Scalar(0, 0, 105), cv::Scalar(180, 100, 255), white);
        cv::inRange(hsv, cv::Scalar(30, 45, 25), cv::Scalar(95, 255, 255), grass);
        cv::morphologyEx(white, white, cv::MORPH_OPEN,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
        cv::morphologyEx(white, white, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(white, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &contour : contours) {
            const double area = cv::contourArea(contour);
            const double perimeter = cv::arcLength(contour, true);
            if (area < 9.0 || perimeter < 1.0) continue;
            const cv::Rect box = cv::boundingRect(contour);
            const double aspect = static_cast<double>(box.width) / std::max(1, box.height);
            const double circularity = 4.0 * M_PI * area / (perimeter * perimeter);
            if (aspect < 0.5 || aspect > 1.9 || circularity < 0.38) continue;
            cv::Point2f center;
            float radius = 0.0f;
            cv::minEnclosingCircle(contour, center, radius);
            if (radius < 3.8f || radius > 90.0f || center.y < 0.1 * small.rows) continue;
            const int ix = std::max(0, std::min(small.cols - 1, static_cast<int>(center.x)));
            const int iy = std::max(0, std::min(small.rows - 1, static_cast<int>(center.y)));
            const double whiteness = cv::mean(white(cv::Rect(ix, iy, 1, 1)))[0] / 255.0;
            const double grassness = cv::mean(grass(cv::Rect(ix, iy, 1, 1)))[0] / 255.0;
            const double score = 0.65 * circularity + 0.35 * (1.0 - grassness);
            if (score > best.score && score >= minScore_ * 0.8) {
                best.valid = score >= minScore_ * 0.9;
                best.x = center.x / small.cols;
                best.y = center.y / small.rows;
                best.radius = radius / small.cols;
                best.score = score + 0.05 * whiteness;
            }
        }
        return best;
    }

    void publishTalk(double time)
    {
        const double estimateTime = nowSeconds();
        const bool ballVisible = visible(time);
        const char *rangeSource = !ballVisible ? "unknown" :
            localBallPointFresh(estimateTime) ? "ground_ray" : "radius";
        std::ostringstream message;
        double worldX = 0.0;
        double worldZ = 0.0;
        const bool hasWorldBall = worldBall(worldX, worldZ);
        const bool active = claim_ || cupcup::isBallAction(tacticalAction_);
        message << "cupcup|id=" << id_ << "|role=" << (id_ == 1 ? "forward" : "defender")
                << "|state=" << (id_ == 1 ? stateName(state_) :
                    (tacticalAction_ == cupcup::TacticalAction::Clear ? stateName(state_) : "DEFEND"))
                << "|tac=" << tacticalActionName(tacticalAction_)
                << "|why=" << matchIntent_.reason
                << "|ball=" << (visible(time) ? 1 : 0) << "|active=" << (active ? 1 : 0)
                << "|kick=" << ((cupcup::isBallAction(tacticalAction_) &&
                    state_ == ForwardState::Kick) ? 1 : 0)
                << "|claim=" << (claim_ ? 1 : 0)
                << "|healthy=" << (healthy_ ? 1 : 0)
                << "|mapped_ball_fresh=" << (worldModel_.ball().fresh(time, .65) ? 1 : 0)
                << "|mapped_ball_age=" << (worldModel_.ball().valid ?
                    time - worldModel_.ball().observedAt : 99.0)
                << "|peer_message_age=" << clampValue(time - teammateTalkAt_, 0.0, 99.0)
                << "|peer_obstacles=" << sharedObstacles_.size()
                << "|conf=" << (visible(time) ? ball_.score : 0.0)
                << "|bdist=" << (ballVisible ? ballDistanceEstimate(estimateTime) : 99.0)
                << "|age=" << clampValue(time - ballSeenAt_, 0.0, 9.9);
        if (worldModel_.ball().fresh(time, .65)) {
            const auto &mapped = worldModel_.ball();
            // Diagnostic map, NOT direct observations to relay as bx/bz.
            message << "|mbx=" << mapped.x << "|mbz=" << mapped.z
                    << "|mbage=" << time - mapped.observedAt
                    << "|mbsource=" << cupcup::ballPositionSourceName(worldModel_.latestBallSource());
        }
        if (hasWorldBall) {
            const double mapAge = std::max(0.0,
                time - worldModel_.ball().observedAt);
            message << "|bx=" << worldX << "|bz=" << worldZ
                    << "|bmap_age=" << clampValue(mapAge, 0.0, 9.9);
        }
        if (navigationTargetValid_) {
            message << "|tx=" << navigationTargetX_ << "|tz=" << navigationTargetZ_;
        } else if (hasWorldBall &&
                   (tacticalAction_ == cupcup::TacticalAction::Chase ||
                    tacticalAction_ == cupcup::TacticalAction::Clear)) {
            message << "|tx=" << worldX << "|tz=" << worldZ;
        }
        message << "|ball_range_source=" << rangeSource;
        if (matchIntent_.kickTargetValid)
            message << "|kx=" << matchIntent_.kickTarget.x << "|kz=" << matchIntent_.kickTarget.z;
        if (cupcup::isBallAction(tacticalAction_) && state_ >= ForwardState::Approach &&
            state_ <= ForwardState::Kick)
            message << "|aim_yaw=" << targetYaw_ << "|aim_offset=" << shotYawOffset_;
        if (std::getenv("CUPCUP_TRACE_PATH") != nullptr && !frame_.empty()) {
            message << "|camera_root_height=" << cameraRootHeight_;
            if (ballVisible && localBallPointFresh(estimateTime)) {
                message << "|lgdx=" << localBallPoint_.x << "|lgdz=" << localBallPoint_.z
                        << "|lg_age=" << estimateTime - localBallPointAt_;
            }
            if (ball_.valid) {
                message << "|iu=" << ball_.x << "|iv=" << ball_.y
                        << "|ir=" << ball_.radius
                        << "|bstamp_ms=" << ballImageSimTimeMs_
                        << "|bihp=" << ballImageHeadPitch_
                        << "|biip=" << ballImageImuPitch_
                        << "|biha_sim=" << ballImageHeadSimAge_
                        << "|biia_sim=" << ballImageImuSimAge_
                        << "|bgvalid=" << (ballGroundPoint_.valid ? 1 : 0)
                        << "|bgx=" << ballGroundPoint_.x << "|bgz=" << ballGroundPoint_.z
                        << "|bcx=" << ballCameraPose_.origin[0]
                        << "|bcy=" << ballCameraPose_.origin[1]
                        << "|bcz=" << ballCameraPose_.origin[2];
                for (int i = 0; i < 9; ++i) message << "|bcr" << i << '=' << ballCameraPose_.rotation[i];
            }
            message << "|hy=" << head_.yaw << "|hp=" << head_.pitch
                    << "|prep_frames=" << stableFrames_ << "|prep_required=" << kickSettleFrames_
                    << "|phase_age=" << std::max(0.0, time - enteredAt_)
                    << "|iy=" << imu_.yaw << "|ip=" << imu_.pitch << "|irll=" << imu_.roll
                    << "|ihy=" << imageHeadYaw_ << "|ihp=" << imageHeadPitch_
                    << "|iiy=" << imageImuYaw_ << "|iip=" << imageImuPitch_
                    << "|iirll=" << imageImuRoll_ << "|iha=" << imageHeadAge_
                    << "|iia=" << imageImuAge_
                    << "|iha_sim=" << imageHeadSimAge_
                    << "|iia_sim=" << imageImuSimAge_
                    << "|ila_sim=" << imageLocationSimAge_
                    << "|sim_stamp_valid=" << (imageSimTimeValid_ ? 1 : 0)
                    << "|istamp_ms=" << imageSimTimeMs_
                    << "|htstamp_ms=" << head_.time
                    << "|mtstamp_ms=" << imu_.stamp
                    << "|ltstamp_ms=" << cupcup::optionalStamp(location_)
                    << "|iw=" << frame_.cols << "|ih=" << frame_.rows
                    << "|fa=" << age(imageAt_) << "|ha=" << age(headAt_) << "|ma=" << age(imuAt_);
        }
        const cupcup::PoseEstimate &self = worldModel_.self();
        if (self.fresh(time, 2.0)) {
            message << "|px=" << self.x << "|pz=" << self.z << "|pyaw=" << self.yaw
                    << "|pose_age=" << clampValue(time - self.observedAt, 0.0, 9.9);
        }
        const cupcup::TimedPoint &robotCandidate =
            worldModel_.robotCandidate().position;
        if (robotCandidate.fresh(time, 0.65)) {
            message << "|rx=" << robotCandidate.x << "|rz=" << robotCandidate.z
                    << "|rc=" << robotCandidate.confidence
                    << "|ra=" << clampValue(time - robotCandidate.observedAt, 0.0, 9.9);
        }
        std::size_t index = 0;
        for (const auto &robot : worldModel_.robots()) {
            if (!robot.position.fresh(time, 0.65)) continue;
            const std::string prefix = "|r" + std::to_string(index++);
            message << prefix << "id=" << robot.trackId
                    << prefix << "team=" << cupcup::robotTeamName(robot.team)
                    << prefix << "x=" << robot.position.x << prefix << "z=" << robot.position.z
                    << prefix << "conf=" << robot.position.confidence
                    << prefix << "unc=" << robot.uncertainty
                    << prefix << "source=" << cupcup::robotPositionSourceName(robot.latestSource)
                    << prefix << "age=" << time - robot.position.observedAt;
        }
        message << "|robots=" << index;
        // Diagnostic copy of effective policy input, not relayable rN tracks.
        for (std::size_t i = 0; i < sharedObstacles_.size(); ++i) {
            const auto &obstacle = sharedObstacles_[i];
            const std::string prefix = "|pobs" + std::to_string(i);
            message << prefix << "x=" << obstacle.position.x
                    << prefix << "z=" << obstacle.position.z
                    << prefix << "conf=" << obstacle.position.confidence
                    << prefix << "unc=" << obstacle.uncertainty
                    << prefix << "age=" << time - obstacle.position.observedAt
                    << prefix << "team=" << cupcup::robotTeamName(obstacle.team);
        }
        common::msg::Talk talk;
        talk.talk_str = message.str();
        talkPublisher_->publish(talk);
    }

    void publishDebug()
    {
        cv::Mat debug = frame_.clone();
        if (ball_.valid) {
            cv::circle(debug, cv::Point(static_cast<int>(ball_.x * debug.cols),
                static_cast<int>(ball_.y * debug.rows)),
                std::max(2, static_cast<int>(ball_.radius * debug.cols)), cv::Scalar(255, 220, 0), 2);
        }
        const double targetX = leftFoot_ ? leftKickX_ : rightKickX_;
        cv::drawMarker(debug, cv::Point(static_cast<int>(targetX * debug.cols),
            static_cast<int>(kickY_ * debug.rows)), cv::Scalar(255, 0, 0), cv::MARKER_CROSS, 16, 2);
        const std::string label = id_ == 1 ? stateName(state_) : "DEFEND";
        cv::putText(debug, label + (visible(nowSeconds()) ? " ball" : " search"), cv::Point(10, 25),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
        sensor_msgs::msg::Image message;
        message.height = debug.rows;
        message.width = debug.cols;
        message.encoding = sensor_msgs::image_encodings::RGB8;
        message.is_bigendian = false;
        message.step = static_cast<decltype(message.step)>(debug.step);
        message.data.assign(debug.data, debug.data + debug.step * debug.rows);
        debugPublisher_->publish(message);
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::string robot_;
    std::string teammate_;
    Color color_ = Color::Invalid;
    int id_ = 0;
    rclcpp::Publisher<common::msg::BodyTask>::SharedPtr bodyPublisher_;
    rclcpp::Publisher<common::msg::HeadTask>::SharedPtr headPublisher_;
    rclcpp::Publisher<common::msg::Talk>::SharedPtr talkPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugPublisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscription_;
    rclcpp::Subscription<common::msg::ImuData>::SharedPtr imuSubscription_;
    rclcpp::Subscription<common::msg::HeadAngles>::SharedPtr headSubscription_;
    rclcpp::Subscription<common::msg::GameData>::SharedPtr gameSubscription_;
    rclcpp::Subscription<common::msg::Location>::SharedPtr locationSubscription_;
    rclcpp::Subscription<common::msg::Talk>::SharedPtr talkSubscription_;

    cv::dnn::Net ballNet_;
    cupcup::BallCandidateVerifier ballVerifier_;
    cupcup::WorldModel worldModel_;
    cv::Mat frame_;
    common::msg::ImuData imu_;
    common::msg::HeadAngles head_;
    common::msg::GameData gameData_;
    common::msg::Location location_;
    std::string teammateTalk_;
    cupcup::TeamStatus teammateStatus_;
    cupcup::MatchLifecycle lifecycle_;
    cupcup::BoundaryGuard boundaryGuard_;
    int gameState_ = -1;
    double imageAt_ = -100.0;
    double imageHeadYaw_ = 0.0;
    double imageHeadPitch_ = 0.0;
    double imageImuYaw_ = 0.0;
    double imageImuPitch_ = 0.0;
    double imageImuRoll_ = 0.0;
    double imageHeadAge_ = 99.0;
    int imageImuFall_ = 0;
    double imageHeadTargetDwell_ = 0.0;
    cupcup::HeadTargetStability headStability_;
    cupcup::camera::GroundPoint localBallPoint_;
    double localBallPointAt_ = -100.0;
    double localBallHeadYaw_ = 0.0;
    double localBallHeadPitch_ = 0.0;
    bool groundBallEnabled_ = true;
    bool geometryKickEnabled_ = true;
    double imageImuAge_ = 99.0;
    double imageHeadSimAge_ = 99.0;
    double imageImuSimAge_ = 99.0;
    double imageLocationSimAge_ = 99.0;
    uint32_t imageSimTimeMs_ = 0;
    uint32_t ballImageSimTimeMs_ = 0;
    double ballImageHeadPitch_ = 0.0;
    double ballImageImuPitch_ = 0.0;
    double ballImageHeadSimAge_ = 99.0;
    double ballImageImuSimAge_ = 99.0;
    cupcup::camera::Pose ballCameraPose_;
    cupcup::camera::GroundPoint ballGroundPoint_;
    cupcup::SensorHistory<common::msg::HeadAngles> headHistory_;
    cupcup::SensorHistory<common::msg::ImuData> imuHistory_;
    bool imageSimTimeValid_ = false;
    double imuAt_ = -100.0;
    double headAt_ = -100.0;
    double gameAt_ = -100.0;
    double locationAt_ = -100.0;
    double teammateTalkAt_ = -100.0;
    std::vector<cupcup::ObservedObstacle> sharedObstacles_;
    double ballSeenAt_ = -100.0;
    double keeperAt_ = -100.0;
    double lastHealthLogAt_ = -100.0;
    double lastStrategyLogAt_ = -100.0;
    double enteredAt_ = 0.0;
    double defenderEnteredAt_ = 0.0;
    double uprightAt_ = 0.0;
    double attackYaw_ = 0.0;
    double yawSign_ = 1.0;
    double yawOffset_ = 0.0;
    double minScore_ = 0.58;
    bool ballPatternFilter_ = true;
    double robotHeightEstimate_ = 0.68;
    double cameraRootHeight_ = 0.345;
    double leftKickX_ = 0.399;
    double rightKickX_ = 0.575;
    double kickY_ = 0.78;
    double kickPitch_ = 60.0;
    double forwardBoundaryMargin_ = 0.75;
    double defenderBoundaryMargin_ = 0.75;
    double defenderHomeX_ = 1.55;
    double defenderHomeZ_ = 0.0;
    double defenderAnchorGain_ = 0.45;
    double supportX_ = 0.40;
    double claimStaleAfter_ = 1.20;
    double takeoverAfter_ = 2.50;
    double kickDirectionHysteresis_ = 0.25;
    bool defenderClearEnabled_ = true;
    double actionPulseSeconds_ = 1.0;
    int kickSettleFrames_ = 16;
    double gameTimeout_ = 2.50;
    double approachTimeout_ = 60.0;
    double orbitTimeout_ = 25.0;
    int ballConfirmHits_ = 2;
    double commandedHeadYaw_ = 0.0;
    double commandedHeadPitch_ = 20.0;
    double lastBearing_ = 0.0;
    double shotYawOffset_ = 0.0;
    double targetYaw_ = 0.0;
    std::size_t imageSequence_ = 0;
    std::size_t processedImageSequence_ = 0;
    std::size_t locationSequence_ = 0;
    std::size_t worldLocationSequence_ = 0;
    std::size_t teammateTalkSequence_ = 0;
    std::size_t worldTeammateSequence_ = 0;
    std::size_t robotMeasurementSequence_ = 0;
    std::size_t worldRobotSequence_ = 0;
    int visibleHits_ = 0;
    int stableFrames_ = 0;
    int missedAlignmentFrames_ = 0;
    int recoveryCount_ = 0;
    bool freshImage_ = false;
    bool searchLowFirst_ = false;
    bool shotLaneSelected_ = false;
    bool actionIssued_ = false;
    bool leftFoot_ = true;
    Ball ball_;
    RobotDetection keeper_;
    std::vector<RobotDetection> robotDetections_;
    std::vector<cupcup::RobotNumberPatch> robotNumberPatches_;
    int keeperHits_ = 0;
    cupcup::TacticalDecision tacticalDecision_;
    cupcup::MatchIntent matchIntent_;
    cupcup::PolicyMemory policyMemory_;
    cupcup::TacticalAction tacticalAction_ = cupcup::TacticalAction::Hold;
    bool navigationTargetValid_ = false;
    double navigationTargetX_ = 0.0;
    double navigationTargetZ_ = 0.0;
    bool claim_ = false;
    bool clearMode_ = false;
    bool healthy_ = false;
    ForwardState state_ = ForwardState::Wait;
};

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("cupcup"), "usage: cupcup <cupcup_1|cupcup_2>");
        rclcpp::shutdown();
        return 2;
    }

    const std::string requestedName(argv[1]);
    const std::size_t underscore = requestedName.find_last_of('_');
    if (underscore == std::string::npos) {
        RCLCPP_ERROR(rclcpp::get_logger("cupcup"), "robot argument must contain an id: %s", requestedName.c_str());
        rclcpp::shutdown();
        return 2;
    }
    const std::string team = requestedName.substr(0, underscore);
    auto node = std::make_shared<rclcpp::Node>(requestedName + "_player");
    auto client = node->create_client<common::srv::GetColor>("gamectrl/get_color");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            rclcpp::shutdown();
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "cupcup: waiting for gamectrl/get_color");
    }

    std::shared_ptr<common::srv::GetColor::Response> response;
    while (rclcpp::ok() && !response) {
        auto request = std::make_shared<common::srv::GetColor::Request>();
        request->team = team;
        auto future = client->async_send_request(request);
        const auto result = rclcpp::spin_until_future_complete(
            node, future, std::chrono::seconds(2));
        if (result == rclcpp::FutureReturnCode::SUCCESS) {
            response = future.get();
        } else {
            RCLCPP_WARN(node->get_logger(),
                "cupcup: gamectrl/get_color timed out; retrying");
        }
    }
    if (!response) {
        rclcpp::shutdown();
        return 0;
    }
    if (response->color == "invalid") {
        RCLCPP_ERROR(node->get_logger(), "cupcup: unsupported team name %s", team.c_str());
        rclcpp::shutdown();
        return 1;
    }

    const std::string robotName = response->color + requestedName.substr(underscore);
    const Color color = colorFromRobot(robotName);
    const int id = robotId(robotName);
    const std::string teammate = response->color + "_" + std::to_string(3 - id);
    RCLCPP_INFO(node->get_logger(), "cupcup: %s is %s, teammate is %s",
        requestedName.c_str(), robotName.c_str(), teammate.c_str());

    auto bodyPublisher = node->create_publisher<common::msg::BodyTask>(robotName + "/task/body", 5);
    auto headPublisher = node->create_publisher<common::msg::HeadTask>(robotName + "/task/head", 5);
    auto talkPublisher = node->create_publisher<common::msg::Talk>(robotName + "/talk/talk_str", 5);
    auto debugPublisher = node->create_publisher<sensor_msgs::msg::Image>(robotName + "/result/image", 5);
    CupcupStrategy strategy(node, robotName, color, id, bodyPublisher, headPublisher, talkPublisher, debugPublisher);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    rclcpp::WallRate loopRate(10.0);
    while (rclcpp::ok()) {
        // Drain the short sensor queues within a bounded window. spin_some
        // takes one callback per subscription; at a 10 Hz strategy loop this
        // left the 50 Hz head/IMU topics behind the camera timestamp.
        executor.spin_all(std::chrono::milliseconds(5));
        strategy.tick();
        loopRate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
