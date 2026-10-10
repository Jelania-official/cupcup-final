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

#include "ball_tracker.hpp"
#include "strategy_logic.hpp"

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
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

struct RobotDetection {
    bool valid = false;
    double x = 0.0;
    double y = 0.0;
    double width = 0.0;
    double height = 0.0;
    double score = 0.0;
};

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
        minScore_ = clampValue(node_->declare_parameter<double>("ball_min_score", 0.30), 0.20, 0.90);
        leftKickX_ = clampValue(node_->declare_parameter<double>("left_kick_x", 0.399), 0.30, 0.49);
        rightKickX_ = clampValue(node_->declare_parameter<double>("right_kick_x", 0.575), 0.51, 0.70);
        kickY_ = clampValue(node_->declare_parameter<double>("kick_y", 0.78), 0.50, 0.88);
        kickPitch_ = clampValue(node_->declare_parameter<double>("kick_pitch", 60.0), 35.0, 70.0);
        forwardBoundaryMargin_ = clampValue(
            node_->declare_parameter<double>("forward_boundary_margin", 0.75), 0.20, 1.20);
        defenderBoundaryMargin_ = clampValue(
            node_->declare_parameter<double>("defender_boundary_margin", 0.75), 0.20, 1.20);
        ballTrackGate_ = clampValue(
            node_->declare_parameter<double>("ball_track_innovation_gate", 0.30), 0.10, 0.60);
        ballTrackTimeout_ = clampValue(
            node_->declare_parameter<double>("ball_track_timeout", 0.60), 0.20, 1.20);
        const int configuredConfirmHits = static_cast<int>(
            node_->declare_parameter<int>("ball_confirm_hits", 2));
        ballConfirmHits_ = std::max(2, std::min(6, configuredConfirmHits));
        ballTracker_ = cupcup::BallTracker(ballTrackGate_, ballTrackTimeout_);
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
        defenderClearEnabled_ = node_->declare_parameter<bool>("defender_clear_enabled", true);
        defenderClearRadius_ = clampValue(
            node_->declare_parameter<double>("defender_clear_radius", 0.075), 0.055, 0.16);
        defenderClearBearing_ = clampValue(
            node_->declare_parameter<double>("defender_clear_bearing", 28.0), 10.0, 45.0);
        defenderClearHeading_ = clampValue(
            node_->declare_parameter<double>("defender_clear_heading", 18.0), 8.0, 30.0);
        defenderClearStableFrames_ = std::max(2, std::min(8,
            static_cast<int>(node_->declare_parameter<int>("defender_clear_stable_frames", 3))));
        defenderClearCooldown_ = clampValue(
            node_->declare_parameter<double>("defender_clear_cooldown", 4.0), 2.0, 12.0);
        actionPulseSeconds_ = clampValue(
            node_->declare_parameter<double>("kick_action_pulse", 1.0), 0.75, 1.30);
        gameTimeout_ = clampValue(
            node_->declare_parameter<double>("game_timeout", 2.50), 1.0, 5.0);
        approachTimeout_ = clampValue(
            node_->declare_parameter<double>("approach_timeout", 60.0), 30.0, 90.0);
        orbitTimeout_ = clampValue(
            node_->declare_parameter<double>("orbit_timeout", 25.0), 15.0, 45.0);
        settleSeconds_ = clampValue(
            node_->declare_parameter<double>("settle_seconds", 0.08), 0.05, 0.50);
        alignStableFrames_ = std::max(2, std::min(4,
            static_cast<int>(node_->declare_parameter<int>("align_stable_frames", 3))));
        finalShotDistance_ = clampValue(
            node_->declare_parameter<double>("final_shot_distance", 0.82), 0.25, 2.20);
        finalCarrySpeed_ = clampValue(
            node_->declare_parameter<double>("final_carry_speed", 0.050), 0.025, 0.055);
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
                ++imageSequence_;
            });

        imuSubscription_ = node_->create_subscription<common::msg::ImuData>(
            robot_ + "/sensor/imu", 2,
            [this](common::msg::ImuData::ConstSharedPtr message) {
                if (!std::isfinite(message->yaw) || !std::isfinite(message->pitch) ||
                    !std::isfinite(message->roll)) return;
                imu_ = *message;
                imuAt_ = nowSeconds();
            });

        headSubscription_ = node_->create_subscription<common::msg::HeadAngles>(
            robot_ + "/sensor/joint/head", 2,
            [this](common::msg::HeadAngles::ConstSharedPtr message) {
                if (!std::isfinite(message->yaw) || !std::isfinite(message->pitch)) return;
                head_ = *message;
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
                if (!filteredLocationValid_) {
                    filteredLocationX_ = message->x;
                    filteredLocationZ_ = message->z;
                    filteredLocationValid_ = true;
                } else {
                    constexpr double locationAlpha = 0.12;
                    filteredLocationX_ += locationAlpha * (message->x - filteredLocationX_);
                    filteredLocationZ_ += locationAlpha * (message->z - filteredLocationZ_);
                }
                locationAt_ = nowSeconds();
            });

        talkSubscription_ = node_->create_subscription<common::msg::Talk>(
            teammate_ + "/talk/talk_str", 2,
            [this](common::msg::Talk::ConstSharedPtr message) {
                teammateTalk_ = message->talk_str;
                teammateTalkAt_ = nowSeconds();
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
        const int score = gameData_.red_score + gameData_.blue_score;
        if (lifecycle_.observe(gameState_, score, time) != cupcup::LifecycleEvent::None) {
            reset(time);
        }

        const bool freshFrame = processedImageSequence_ != imageSequence_;
        freshImage_ = freshFrame;
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
        const bool fallen = imu_.fall != common::msg::ImuData::FALL_NONE;
        const bool playerActive = isPlayerActive();
        const bool healthReady = lifecycle_.canPlay(gameState_, time) && playerActive &&
            sensorsFresh && locationFresh && !fallen && time - uprightAt_ >= 1.5;
        healthy_ = healthReady;
        if (fallen) uprightAt_ = time;

        if (!healthReady) {
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
        } else if (id_ == 1) {
            updateTeammateStatus(time);
            updateTacticalDecision(time, locationFresh);
            if (tacticalAction_ == cupcup::TacticalAction::Support) {
                runSupport(body, headTask, time, locationFresh);
            } else {
                runForward(body, headTask, time, locationFresh);
            }
        } else {
            updateTeammateStatus(time);
            updateTacticalDecision(time, locationFresh);
            if (tacticalAction_ == cupcup::TacticalAction::Clear ||
                tacticalAction_ == cupcup::TacticalAction::Chase) {
                runForward(body, headTask, time, locationFresh);
            } else {
                runDefender(body, headTask, time, locationFresh);
            }
        }

        if (time - lastStrategyLogAt_ > 2.0) {
            RCLCPP_INFO(node_->get_logger(), "%s %s tactical=%s claim=%d ball=%.2f radius=%.3f hits=%d bearing=%.1f loc=(%.2f,%.2f) yaw=%.1f target=%.1f cmd=(%.3f,%.3f,%.1f)",
                robot_.c_str(), stateName(state_), tacticalActionName(tacticalAction_), claim_,
                ball_.score, ball_.radius, visibleHits_, lastBearing_,
                location_.x, location_.z, yawSign_ * imu_.yaw + yawOffset_, targetYaw_,
                body.step, body.lateral, body.turn);
            lastStrategyLogAt_ = time;
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
        ball_ = Ball();
        ballTracker_.reset();
        keeper_ = RobotDetection();
        keeperCandidate_ = RobotDetection();
        keeperHits_ = 0;
        visibleHits_ = 0;
        processedImageSequence_ = imageSequence_;
        commandedHeadYaw_ = 0.0;
        commandedHeadPitch_ = 20.0;
        leftFoot_ = true;
        footLocked_ = false;
        actionIssued_ = false;
        defenderClearIssued_ = false;
        defenderClearStableFramesSeen_ = 0;
        defenderClearCooldownUntil_ = time;
        shotYawOffset_ = 0.0;
        shotLaneSelected_ = false;
        targetYaw_ = attackYaw_;
        filteredLocationValid_ = false;
        teammateStatus_ = cupcup::TeamStatus();
        tacticalDecision_ = cupcup::TacticalDecision();
        tacticalAction_ = cupcup::TacticalAction::Hold;
        claim_ = false;
        clearMode_ = false;
        attackKickCount_ = 0;
        healthy_ = false;
        boundaryGuard_.reset();
        transition(ForwardState::Wait, time);
        uprightAt_ = time;
        defenderEnteredAt_ = time;
    }

    void transition(ForwardState next, double time)
    {
        if (next == state_) return;
        const bool preserveAlignment = state_ == ForwardState::Align &&
            next == ForwardState::Settle;
        if (next == ForwardState::Search) {
            searchLowFirst_ = state_ == ForwardState::Recover;
        }
        if (next == ForwardState::Align || next == ForwardState::Settle) {
            shotLaneSelected_ = false;
        }
        if (next == ForwardState::Align && state_ != ForwardState::Settle) {
            footLocked_ = false;
        }
        state_ = next;
        enteredAt_ = time;
        if (!preserveAlignment) stableFrames_ = 0;
        RCLCPP_INFO(node_->get_logger(), "%s strategy -> %s", robot_.c_str(), stateName(state_));
        if (next == ForwardState::Kick) {
            actionIssued_ = false;
            if (id_ == 1 && !clearMode_) ++attackKickCount_;
            const double goalX = color_ == Color::Red ? -4.5 : 4.5;
            const double positionX = filteredLocationValid_ ? filteredLocationX_ : location_.x;
            RCLCPP_INFO(node_->get_logger(),
                "%s kick foot=%s lane_offset=%.1f attack_kick=%d goal_distance=%.2f",
                robot_.c_str(), leftFoot_ ? "left" : "right", shotYawOffset_,
                attackKickCount_, std::abs(goalX - positionX));
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
        const cupcup::BoundaryDecision boundary = boundaryGuard_.update(
            teamColor, role, location_.x, margin);
        bool unsafeStep = false;
        if (boundary.forwardBoundary) {
            // Both teams use a positive local step toward the opponent. At
            // our own penalty boundary only a negative retreat is unsafe.
            unsafeStep = body.step < -0.001;
        } else if (boundary.defenderBoundary) {
            // A defender must not advance toward the opponent at midfield;
            // retreating to its home lane remains allowed.
            unsafeStep = body.step > 0.001;
        }
        if (unsafeStep && body.type == common::msg::BodyTask::TASK_WALK) {
            // Stop only the forbidden direction. Stopping both directions can
            // deadlock a forward at kickoff when it starts near its boundary.
            body.step = 0.0;
            body.lateral = 0.0;
            body.count = 0;
        }
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

    void updateTargetYaw(bool locationFresh)
    {
        if (!locationFresh) return;
        const double goalX = color_ == Color::Red ? -4.5 : 4.5;
        const bool useFilteredFinalAim = id_ == 1 && attackKickCount_ >= 2 &&
            filteredLocationValid_;
        const double positionX = useFilteredFinalAim ? filteredLocationX_ : location_.x;
        const double positionZ = useFilteredFinalAim ? filteredLocationZ_ : location_.z;
        if (std::abs(goalX - positionX) <= 0.7) return;
        const double geometric = std::atan2(-positionZ, std::abs(goalX - positionX)) *
            180.0 / M_PI;
        const double desired = wrapDegrees(
            attackYaw_ + (goalX < 0.0 ? geometric : -geometric) + shotYawOffset_);
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

    void updateTeammateStatus(double time)
    {
        teammateStatus_ = cupcup::parseTeamStatus(teammateTalk_);
        teammateDecision_ = cupcup::arbitrate(teammateStatus_, time - teammateTalkAt_);
    }

    double ballDistanceEstimate() const
    {
        if (!ball_.valid || ball_.radius <= 0.001) return 99.0;
        // The calibrated camera projection makes the normalized ball radius a
        // useful short-range distance proxy.  It is only used to choose which
        // robot claims the ball, never as an absolute navigation measurement.
        return clampValue(0.050 / ball_.radius, 0.12, 8.0);
    }

    bool worldBall(double &x, double &z) const
    {
        if (!visible(nowSeconds()) || age(locationAt_) > 2.0) return false;
        const double distance = ballDistanceEstimate();
        const double globalYaw = (yawSign_ * imu_.yaw + yawOffset_ + bearingDegrees()) * M_PI / 180.0;
        x = location_.x + distance * std::cos(globalYaw);
        z = location_.z - distance * std::sin(globalYaw);
        return std::isfinite(x) && std::isfinite(z);
    }

    void updateTacticalDecision(double time, bool locationFresh)
    {
        double ballX = 0.0;
        double ballZ = 0.0;
        const bool hasWorldBall = locationFresh && worldBall(ballX, ballZ);
        cupcup::TacticalInput input;
        input.color = color_ == Color::Red ? cupcup::TeamColor::Red : cupcup::TeamColor::Blue;
        input.id = id_;
        input.selfHealthy = healthy_;
        input.selfBall = visible(time) && hasWorldBall;
        input.selfLocationFresh = locationFresh;
        input.selfBallScore = ball_.score;
        input.selfBallDistance = ballDistanceEstimate();
        input.selfBallAge = std::max(0.0, time - ballSeenAt_);
        input.selfBallX = ballX;
        input.selfBallZ = ballZ;
        input.teammate = teammateStatus_;
        input.teammateMessageAge = time - teammateTalkAt_;
        tacticalDecision_ = cupcup::decideTactics(input, claimStaleAfter_, takeoverAfter_);
        tacticalAction_ = tacticalDecision_.action;
        claim_ = tacticalDecision_.claim;
        clearMode_ = tacticalAction_ == cupcup::TacticalAction::Clear;
    }

    void navigateTo(common::msg::BodyTask &body, double targetX, double targetZ,
                    double maxForward = 0.032)
    {
        const double dx = targetX - location_.x;
        const double dz = targetZ - location_.z;
        const double distance = std::hypot(dx, dz);
        const double currentYaw = (yawSign_ * imu_.yaw + yawOffset_) * M_PI / 180.0;
        const double localForward = dx * std::cos(currentYaw) - dz * std::sin(currentYaw);
        const double localLeft = -dx * std::sin(currentYaw) + dz * std::cos(currentYaw);
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
        const double ownAttackSide = color_ == Color::Red ? -1.0 : 1.0;
        const double targetX = ownAttackSide * supportX_;
        const double targetZ = teammateStatus_.valid ?
            clampValue(teammateStatus_.ballZ * 0.35, -1.4, 1.4) : 0.0;
        if (locationFresh) navigateTo(body, targetX, targetZ, 0.020);
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
        if (std::abs(ball_.x - 0.5) > 0.10) {
            commandedHeadYaw_ = clampValue(head_.yaw +
                clampValue(correction / 3.5, -2.5, 2.5), -60.0, 60.0);
        }
        const bool handoffView = state_ == ForwardState::Orbit && ball_.radius > 0.045 &&
            std::abs(ball_.x - 0.5) < 0.15 && ball_.y > 0.32;
        if (handoffView && commandedHeadPitch_ < kickPitch_ - 0.5) {
            // Move into the calibrated downward view while ORBIT still has
            // enough range to follow the ball. A direct jump loses near balls.
            commandedHeadPitch_ = std::min(kickPitch_, commandedHeadPitch_ +
                clampValue((kickPitch_ - commandedHeadPitch_) * 0.22, 0.7, 2.2));
        } else if (ball_.y < 0.30 || ball_.y > 0.68) {
            commandedHeadPitch_ = clampValue(head_.pitch +
                clampValue((ball_.y - 0.49) * 4.0, -1.8, 1.8), 8.0, kickPitch_);
        }
    }

    void selectShotLane(bool locationFresh)
    {
        if (shotLaneSelected_) return;
        if (id_ == 1 && !clearMode_ && attackKickCount_ >= 2) {
            // At final-shot range the geometric target already points through
            // the middle of the goal. A large keeper-avoidance offset can
            // turn an otherwise certain short shot into a post-side miss.
            shotYawOffset_ = 0.0;
            shotLaneSelected_ = true;
            return;
        }
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
            if (hasBall && std::abs(lastBearing_) <= 45.0)
                transition(ForwardState::Approach, time);
            // Keep the body planted during the visual scan.  In this Webots
            // model short isolated turn tasks can destabilize a standing
            // robot; the head sweep already covers the validated kickoff
            // view, and body motion starts only after confirmation.
        } else if (state_ == ForwardState::Approach) {
            if (!hasBall) {
                if (time - ballSeenAt_ > 1.0) transition(ForwardState::Search, time);
            } else {
                if (std::abs(lastBearing_) > 45.0) {
                    // An edge candidate is not reliable enough to justify a
                    // body turn. Re-enter the head scan and reacquire it in a
                    // centered pose; this is safer than turning on a false
                    // YOEO cell and losing balance.
                    transition(ForwardState::Search, time);
                } else {
                const double speed = ball_.radius < 0.025 ? 0.05 :
                    (ball_.radius < 0.045 ? 0.04 :
                    clampValue((0.075 - ball_.radius) * 0.75, 0.018, 0.032));
                // A pure turn task is not reliable in this Webots gait: in
                // some initial poses the commanded yaw is accepted but the
                // robot does not change heading.  Use a small crawl while
                // turning so the approach cannot deadlock at a persistent
                // 20--40 degree bearing.  Once centered, use the calibrated
                // forward speed.
                const double forward = std::abs(lastBearing_) > 24.0 ? 0.010 :
                    speed * clampValue(1.0 - std::abs(lastBearing_) / 70.0, 0.65, 1.0);
                // Recover the four-player kickoff view quickly when the ball
                // starts near the edge or behind the camera.
                double turn = clampValue(lastBearing_ * 0.18, -4.0, 4.0);
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
                if (std::abs(headingError()) < 15.0 && std::abs(lastBearing_) < 22.0 &&
                    std::abs(head_.yaw) < 12.0 && head_.pitch > kickPitch_ - 12.0 &&
                    ball_.y > 0.30 && ball_.radius > 0.050 && ball_.radius < 0.12) {
                    leftFoot_ = lastBearing_ >= 0.0;
                    transition(ForwardState::Align, time);
                }
            }
        } else if (state_ == ForwardState::Align || state_ == ForwardState::Settle) {
            commandedHeadYaw_ = 0.0;
            commandedHeadPitch_ = kickPitch_;
            const bool fixedView = std::abs(head_.yaw) < 6.0 &&
                std::abs(head_.pitch - kickPitch_) < 5.0;
            if (state_ == ForwardState::Align && hasBall && fixedView && !footLocked_) {
                leftFoot_ = std::abs(ball_.x - leftKickX_) <=
                    std::abs(ball_.x - rightKickX_);
                footLocked_ = true;
            }
            const double desiredX = leftFoot_ ? leftKickX_ : rightKickX_;
            const bool linedUp = hasBall && fixedView && std::abs(headingError()) < 14.0 &&
                std::abs(ball_.x - desiredX) < 0.070 && std::abs(ball_.y - kickY_) < 0.080 &&
                ball_.radius > 0.048;
            const bool holdPose = hasBall && fixedView && std::abs(headingError()) < 19.0 &&
                std::abs(ball_.x - desiredX) < 0.095 && std::abs(ball_.y - kickY_) < 0.105 &&
                ball_.radius > 0.042;
            if (freshImage_) stableFrames_ = linedUp ? std::min(stableFrames_ + 1, 20) :
                (holdPose ? stableFrames_ : 0);
            if (state_ == ForwardState::Settle) {
                if (!holdPose) transition(ForwardState::Align, time);
                else if (stateAge > settleSeconds_ && linedUp &&
                         stableFrames_ >= alignStableFrames_)
                    transition(ForwardState::Kick, time);
            } else if (!hasBall) {
                if (time - ballSeenAt_ > 0.8) transition(ForwardState::Recover, time);
            } else if (fixedView) {
                if (std::abs(headingError()) > 27.0) transition(ForwardState::Orbit, time);
                else if (stableFrames_ >= alignStableFrames_) {
                    const double goalX = color_ == Color::Red ? -4.5 : 4.5;
                    const double positionX = filteredLocationValid_ ?
                        filteredLocationX_ : location_.x;
                    const double goalDistance = std::abs(goalX - positionX);
                    const bool carryIntoFinalRange = id_ == 1 && !clearMode_ &&
                        attackKickCount_ >= 2 &&
                        (!locationFresh || goalDistance > finalShotDistance_);
                    // The alignment counter already represents consecutive
                    // fresh camera frames and `linedUp` is true for the final
                    // frame.  Enter KICK directly instead of spending another
                    // 10 Hz control cycle in SETTLE; this removes about 0.1 s
                    // of visible wind-up without weakening the consecutive-frame
                    // confirmation or the final ball-position check.
                    if (carryIntoFinalRange) {
                        // After two advancing kicks, do not waste the third
                        // action just short of the goal line.  Use the same
                        // proven top speed as APPROACH while the goal is far,
                        // then taper near the shooting threshold so the ball
                        // remains between the feet.  The previous fixed
                        // 0.022 m/s crawl could look stationary for more than
                        // a minute after an ineffective second kick.
                        const double distanceOutsideRange =
                            std::max(0.0, goalDistance - finalShotDistance_);
                        const double carrySpeed = clampValue(
                            0.030 + 0.020 * distanceOutsideRange,
                            0.030, finalCarrySpeed_);
                        walk(body, carrySpeed,
                            clampValue((desiredX - ball_.x) * 0.22, -0.018, 0.018),
                            clampValue(headingError() * 0.18, -4.0, 4.0));
                    } else if (id_ == 1 && !clearMode_ && attackKickCount_ >= 2) {
                        // The final shot follows a dribble. Give the gait one
                        // bounded settle cycle so the action engine receives
                        // the kick from a planted support foot. Advancing
                        // kicks keep the direct fast path.
                        transition(ForwardState::Settle, time);
                    } else {
                        transition(ForwardState::Kick, time);
                    }
                }
                else walk(body, clampValue((kickY_ - ball_.y) * 0.24, -0.022, 0.030),
                          clampValue((desiredX - ball_.x) * 0.30, -0.028, 0.028),
                          clampValue(headingError() * 0.22, -6.0, 6.0));
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

        if ((state_ == ForwardState::Approach && time - enteredAt_ > approachTimeout_) ||
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
        updateTeammateStatus(time);
        double ballBearing = 0.0;

        if (hasBall) {
            ballBearing = bearingDegrees();
            commandedHeadYaw_ = clampValue(head_.yaw + ballBearing / 3.0, -55.0, 55.0);
        } else {
            const int index = static_cast<int>(ageInState) % 6;
            commandedHeadYaw_ = scanYaw[index];
            commandedHeadPitch_ = scanPitch[index];
        }

        if (!defenderClearEnabled_ || time < defenderClearCooldownUntil_) {
            defenderClearStableFramesSeen_ = 0;
        } else {
            cupcup::DefenderClearInput clearInput;
            clearInput.color = color_ == Color::Red ? cupcup::TeamColor::Red : cupcup::TeamColor::Blue;
            clearInput.ballVisible = hasBall;
            clearInput.locationFresh = locationFresh;
            clearInput.forwardBusy = teammateDecision_.forwardBusy;
            clearInput.locationX = location_.x;
            clearInput.ballRadius = ball_.radius;
            clearInput.bearing = ballBearing;
            clearInput.headingError = headingError();
            const bool clearCandidate = cupcup::shouldDefenderClear(clearInput,
                defenderClearRadius_, defenderClearBearing_, defenderClearHeading_,
                defenderBoundaryMargin_);
            if (clearCandidate) {
                defenderClearStableFramesSeen_ = std::min(
                    defenderClearStableFramesSeen_ + 1, defenderClearStableFrames_);
            } else if (!defenderClearIssued_) {
                defenderClearStableFramesSeen_ = 0;
            }
        }

        // Track a conservative defensive anchor.  It follows the ball only
        // laterally, keeping the robot between its goal and the play while
        // respecting the own-half boundary.
        const double ownHomeX = color_ == Color::Red ? defenderHomeX_ : -defenderHomeX_;
        const double observedZ = hasBall ? [&]() {
            double ignoredX = 0.0;
            double worldZ = defenderHomeZ_;
            if (worldBall(ignoredX, worldZ)) return worldZ;
            return defenderHomeZ_;
        }() : (teammateStatus_.valid ? teammateStatus_.ballZ : defenderHomeZ_);
        const double targetZ = clampValue(
            defenderHomeZ_ + defenderAnchorGain_ * (observedZ - defenderHomeZ_), -1.45, 1.45);
        if (locationFresh) navigateTo(body, ownHomeX, targetZ, 0.018);
        else stop(body);

        // A defender may issue one single-shot clearance only after a close
        // ball has stayed aligned for several frames, the striker is not
        // claiming it, and the robot remains safely in its own half.
        if (defenderClearEnabled_ && !defenderClearIssued_ &&
            defenderClearStableFramesSeen_ >= defenderClearStableFrames_ &&
            time >= defenderClearCooldownUntil_) {
            leftFoot_ = ballBearing >= 0.0;
            body.type = common::msg::BodyTask::TASK_ACT;
            body.count = 1;
            body.actname = leftFoot_ ? "left_kick" : "right_kick";
            defenderClearIssued_ = true;
            defenderClearIssuedAt_ = time;
            defenderClearCooldownUntil_ = time + defenderClearCooldown_;
            defenderClearStableFramesSeen_ = 0;
            RCLCPP_INFO(node_->get_logger(), "%s defender clear foot=%s bearing=%.1f",
                robot_.c_str(), leftFoot_ ? "left" : "right", ballBearing);
        }
        if (defenderClearIssued_ && time - defenderClearIssuedAt_ > 0.45) {
            defenderClearIssued_ = false;
        }
        enforceSafety(body);
        headTask.yaw = commandedHeadYaw_;
        headTask.pitch = commandedHeadPitch_;
    }

    Ball detect(const cv::Mat &rgb)
    {
        if (rgb.empty()) return Ball();
        if (!ballNet_.empty()) return detectModel(rgb);
        return detectTraditional(rgb);
    }

    Ball detectModel(const cv::Mat &rgb)
    {
        Ball best;
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
        auto sigmoid = [](double value) { return 1.0 / (1.0 + std::exp(-value)); };
        double bestRank = 0.0;
        double bestRobotRank = 0.0;
        const double currentTime = nowSeconds();
        for (const auto &output : outputs) {
            if (output.dims != 4 || output.size[1] != 7) continue;
            const int height = output.size[2];
            const int width = output.size[3];
            const float *data = output.ptr<float>();
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    auto value = [&](int channel) {
                        return data[channel * height * width + y * width + x];
                    };
                    const double objectness = sigmoid(value(4));
                    const double confidence = objectness * sigmoid(value(5));
                    const double robotConfidence = objectness * sigmoid(value(6));
                    if (std::max(confidence, robotConfidence) < 0.30) continue;
                    const double cx = (sigmoid(value(0)) + x) * side / width - left;
                    const double cy = (sigmoid(value(1)) + y) * side / height - top;
                    const double w = std::exp(value(2)) * 99.99983 * side / 416.0;
                    const double h = std::exp(value(3)) * 99.99983 * side / 416.0;
                    if (!std::isfinite(w + h) || cx < 0 || cy < 0 || cx >= rgb.cols || cy >= rgb.rows) continue;
                    const double nx = cx / rgb.cols;
                    const double ny = cy / rgb.rows;
                    const double nw = w / rgb.cols;
                    const double nh = h / rgb.rows;
                    const double ballRadius = (w + h) / (4.0 * rgb.cols);
                    if (robotConfidence > 0.45 && value(6) > value(5) && ny < 0.65 &&
                        nw > 0.025 && nh > 0.06 && robotConfidence > bestRobotRank) {
                        bestRobotRank = robotConfidence;
                        keeperCandidate_ = {true, nx, ny, nw, nh, robotConfidence};
                    }
                    if (confidence < minScore_ || value(5) < value(6)) continue;
                    // Very large decoded boxes are usually a robot/body or a
                    // field edge, not the ball. Rejecting them keeps a close
                    // false positive from driving the striker into a turn.
                    if (ballRadius > 0.11) continue;
                    double rank = confidence;
                    // The initial validated implementation used temporal
                    // candidate ranking before the tracker. This prevents a
                    // high-scoring edge/robot candidate from teleporting the
                    // ball estimate when several YOEO cells fire.
                    if (state_ != ForwardState::Search &&
                        ball_.valid && currentTime - ballSeenAt_ < 0.5) {
                        const double dx = nx - ball_.x;
                        const double dy = ny - ball_.y;
                        rank *= 0.65 + 0.35 * std::exp(-20.0 * (dx * dx + dy * dy));
                    }
                    if (rank > bestRank) {
                        bestRank = rank;
                        best.valid = true;
                        best.x = nx;
                        best.y = ny;
                        best.radius = ballRadius;
                        best.score = confidence;
                    }
                }
            }
        }
        if (keeperCandidate_.valid) {
            const bool consistent = keeper_.valid &&
                std::abs(keeperCandidate_.x - keeper_.x) < 0.15;
            keeper_ = keeperCandidate_;
            keeperAt_ = currentTime;
            keeperHits_ = consistent ? std::min(keeperHits_ + 1, 20) : 1;
        } else if (currentTime - keeperAt_ > 0.5) {
            keeper_ = RobotDetection();
            keeperHits_ = 0;
        }
        keeperCandidate_ = RobotDetection();
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
        std::ostringstream message;
        double worldX = 0.0;
        double worldZ = 0.0;
        const bool hasWorldBall = worldBall(worldX, worldZ);
        const bool active = claim_ || defenderClearIssued_ ||
            tacticalAction_ == cupcup::TacticalAction::Chase ||
            tacticalAction_ == cupcup::TacticalAction::Clear;
        message << "cupcup|id=" << id_ << "|role=" << (id_ == 1 ? "forward" : "defender")
                << "|state=" << (id_ == 1 ? stateName(state_) :
                    (tacticalAction_ == cupcup::TacticalAction::Clear ? stateName(state_) : "DEFEND"))
                << "|ball=" << (visible(time) ? 1 : 0) << "|active=" << (active ? 1 : 0)
                << "|kick=" << ((state_ == ForwardState::Kick || defenderClearIssued_) ? 1 : 0)
                << "|claim=" << (claim_ ? 1 : 0)
                << "|healthy=" << (healthy_ ? 1 : 0)
                << "|conf=" << (visible(time) ? ball_.score : 0.0)
                << "|bdist=" << (visible(time) ? ballDistanceEstimate() : 99.0)
                << "|age=" << clampValue(time - ballSeenAt_, 0.0, 9.9);
        if (hasWorldBall) message << "|bx=" << worldX << "|bz=" << worldZ;
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
    cupcup::BallTracker ballTracker_;
    cv::Mat frame_;
    common::msg::ImuData imu_;
    common::msg::HeadAngles head_;
    common::msg::GameData gameData_;
    common::msg::Location location_;
    std::string teammateTalk_;
    cupcup::TeamStatus teammateStatus_;
    cupcup::TeammateDecision teammateDecision_;
    cupcup::MatchLifecycle lifecycle_;
    cupcup::BoundaryGuard boundaryGuard_;
    int gameState_ = -1;
    double imageAt_ = -100.0;
    double imuAt_ = -100.0;
    double headAt_ = -100.0;
    double gameAt_ = -100.0;
    double locationAt_ = -100.0;
    double teammateTalkAt_ = -100.0;
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
    double leftKickX_ = 0.399;
    double rightKickX_ = 0.575;
    double kickY_ = 0.78;
    double kickPitch_ = 60.0;
    double forwardBoundaryMargin_ = 0.75;
    double defenderBoundaryMargin_ = 0.75;
    double ballTrackGate_ = 0.30;
    double ballTrackTimeout_ = 0.60;
    double defenderHomeX_ = 1.55;
    double defenderHomeZ_ = 0.0;
    double defenderAnchorGain_ = 0.45;
    double supportX_ = 0.40;
    double claimStaleAfter_ = 1.20;
    double takeoverAfter_ = 2.50;
    bool defenderClearEnabled_ = true;
    double defenderClearRadius_ = 0.075;
    double defenderClearBearing_ = 28.0;
    double defenderClearHeading_ = 18.0;
    double defenderClearCooldown_ = 4.0;
    double actionPulseSeconds_ = 1.0;
    int defenderClearStableFrames_ = 3;
    int defenderClearStableFramesSeen_ = 0;
    bool defenderClearIssued_ = false;
    double defenderClearIssuedAt_ = -100.0;
    double defenderClearCooldownUntil_ = 0.0;
    double gameTimeout_ = 2.50;
    double approachTimeout_ = 60.0;
    double orbitTimeout_ = 25.0;
    double settleSeconds_ = 0.08;
    double finalShotDistance_ = 0.82;
    double finalCarrySpeed_ = 0.050;
    double filteredLocationX_ = 0.0;
    double filteredLocationZ_ = 0.0;
    int alignStableFrames_ = 3;
    int ballConfirmHits_ = 2;
    double commandedHeadYaw_ = 0.0;
    double commandedHeadPitch_ = 20.0;
    double lastBearing_ = 0.0;
    double shotYawOffset_ = 0.0;
    double targetYaw_ = 0.0;
    std::size_t imageSequence_ = 0;
    std::size_t processedImageSequence_ = 0;
    int visibleHits_ = 0;
    int stableFrames_ = 0;
    int attackKickCount_ = 0;
    bool filteredLocationValid_ = false;
    int recoveryCount_ = 0;
    bool freshImage_ = false;
    bool searchLowFirst_ = false;
    bool shotLaneSelected_ = false;
    bool actionIssued_ = false;
    bool leftFoot_ = true;
    bool footLocked_ = false;
    Ball ball_;
    RobotDetection keeper_;
    RobotDetection keeperCandidate_;
    int keeperHits_ = 0;
    cupcup::TacticalDecision tacticalDecision_;
    cupcup::TacticalAction tacticalAction_ = cupcup::TacticalAction::Hold;
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

    rclcpp::WallRate loopRate(10.0);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        strategy.tick();
        loopRate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
