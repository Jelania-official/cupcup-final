// Test-only instrumentation. The released judge is compiled below unchanged;
// only its Supervisor type gains a read-only hook after each real step().
// No truth publisher, world edits, random-seed override or extra physics steps.
#include <webots/Supervisor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <common/msg/body_task.hpp>
#include <common/msg/game_data.hpp>
#include <common/msg/location.hpp>
#include <common/msg/talk.hpp>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace cupcup_trace {
std::string quote(const std::string &value) {
  std::string result = "\"";
  for (char c : value) { if (c == '"') result += '"'; result += c; }
  return result + '"';
}

// Copy immediately: Webots owns these buffers. IDs identify the queried
// solid/descendant, NOT the other body. Shared world-space contact points are
// used to identify a ball/robot contact, retaining raw points for auditing.
std::vector<webots::ContactPoint> contacts(webots::Node *node) {
  int count = 0;
  const auto *points = node->getContactPoints(true, &count);
  return points && count > 0 ? std::vector<webots::ContactPoint>(points, points + count)
                            : std::vector<webots::ContactPoint>{};
}

bool samePoint(const webots::ContactPoint &a, const webots::ContactPoint &b) {
  double distance2 = 0;
  for (int i = 0; i < 3; ++i) distance2 += std::pow(a.point[i] - b.point[i], 2);
  return distance2 <= 1e-10;  // 10 micrometres, not a proximity-to-ball test.
}

std::string pointsText(const std::vector<webots::ContactPoint> &points) {
  std::ostringstream out; out << std::setprecision(10);
  for (const auto &p : points)
    out << p.node_id << ':' << p.point[0] << ':' << p.point[1] << ':' << p.point[2] << ';';
  return out.str();
}

struct RobotSample {
  webots::Node *node = nullptr;
  double obsX = std::numeric_limits<double>::quiet_NaN(), obsZ = obsX;
  unsigned long talkSeq = 0, bodySeq = 0, kickSeq = 0;
  double kickTime = -1;
  std::string talk, kickName;
  common::msg::BodyTask body;
  rclcpp::Subscription<common::msg::Talk>::SharedPtr talkSub;
  rclcpp::Subscription<common::msg::Location>::SharedPtr locationSub;
  rclcpp::Subscription<common::msg::BodyTask>::SharedPtr bodySub;
};
}  // namespace cupcup_trace

namespace webots {
class TracedSupervisor : public Supervisor {
 public:
  TracedSupervisor() {
    const char *path = std::getenv("CUPCUP_TRACE_PATH");
    if (!path || !*path) throw std::runtime_error("traced judge requires CUPCUP_TRACE_PATH");
    output_.open(path);
    if (!output_) throw std::runtime_error("cannot open requested trace");
    output_ << std::setprecision(10);
    std::ofstream metadata(std::string(path) + ".meta.json");
    metadata << "{\"mode\":\"released-read-only-step-wrapper\","
      "\"released_source_sha256\":\"" RELEASED_SUPERVISOR_SHA "\","
      "\"wrapper_sha256\":\"" TRACE_WRAPPER_SHA "\","
      "\"sampling_seconds\":0.02,\"truth_published\":false,"
      "\"localization_seed_applied\":false,"
      "\"contact_method\":\"shared-world-contact-point-1e-5m\"}\n";
    ball_ = getFromDef("Ball");
    if (!ball_) throw std::runtime_error("released Ball DEF missing");
    ball_->enableContactPointsTracking(20, true);
    observer_ = std::make_shared<rclcpp::Node>("cupcup_read_only_contact_trace");
    gameSub_ = observer_->create_subscription<common::msg::GameData>(
      "/sensor/game", 5, [this](common::msg::GameData::ConstSharedPtr m) { game_ = *m; });
    output_ << "time,state,remain_time,red_score,blue_score,ball_x,ball_z,ball_vx,ball_vz,ball_y,ball_contacts";
    for (size_t i = 0; i < names_.size(); ++i) {
      auto &r = robots_[i]; const auto &name = names_[i];
      r.node = getFromDef(name);
      if (!r.node) throw std::runtime_error("released robot DEF missing: " + name);
      r.node->enableContactPointsTracking(20, true);
      r.talkSub = observer_->create_subscription<common::msg::Talk>(
        "/" + name + "/talk/talk_str", 5, [this, i](common::msg::Talk::ConstSharedPtr m) {
          robots_[i].talk = m->talk_str; ++robots_[i].talkSeq;
        });
      r.locationSub = observer_->create_subscription<common::msg::Location>(
        "/sensor/" + name + "_location", 5, [this, i](common::msg::Location::ConstSharedPtr m) {
          robots_[i].obsX = m->x; robots_[i].obsZ = m->z;
        });
      r.bodySub = observer_->create_subscription<common::msg::BodyTask>(
        "/" + name + "/task/body", 10, [this, i](common::msg::BodyTask::ConstSharedPtr m) {
          auto &r = robots_[i];
          const bool kick = m->type == m->TASK_ACT &&
            (m->actname == "left_kick" || m->actname == "right_kick");
          if (kick && (r.body.type != m->type || r.body.actname != m->actname)) {
            ++r.kickSeq; r.kickTime = getTime(); r.kickName = m->actname;
          }
          r.body = *m; ++r.bodySeq;
        });
      for (const auto *suffix : {"true_x", "true_z", "vx", "vz", "omega_y", "yaw",
          "obs_x", "obs_z", "talk_seq", "talk", "body_seq", "body_type", "actname",
          "step", "lateral", "turn", "kick_seq", "kick_time", "kick_name",
          "ball_contact_count", "ball_contact_points"}) output_ << ',' << name << '_' << suffix;
    }
    output_ << '\n'; output_.flush();
  }

  int step(int milliseconds) override {
    const int result = Supervisor::step(milliseconds);
    if (result == -1 || !rclcpp::ok()) return result;
    // Record before the original judge handles this step's rules/resets.
    // Messages are tagged by receipt simulation time, not exposure time.
    rclcpp::spin_some(observer_);
    const auto ballContacts = cupcup_trace::contacts(ball_);
    const auto *bp = ball_->getPosition(); const auto *bv = ball_->getVelocity();
    output_ << getTime() << ',' << game_.state << ',' << game_.remain_time << ','
      << game_.red_score << ',' << game_.blue_score << ',' << bp[0] << ',' << bp[2]
      << ',' << bv[0] << ',' << bv[2] << ',' << bp[1] << ','
      << cupcup_trace::quote(cupcup_trace::pointsText(ballContacts));
    for (const auto &r : robots_) {
      const auto *p = r.node->getPosition(); const auto *v = r.node->getVelocity();
      const auto *o = r.node->getOrientation();
      std::vector<ContactPoint> shared;
      for (const auto &rp : cupcup_trace::contacts(r.node)) {
        for (const auto &bc : ballContacts) if (cupcup_trace::samePoint(rp, bc)) {
          shared.push_back(rp); break;
        }
      }
      output_ << ',' << p[0] << ',' << p[2] << ',' << v[0] << ',' << v[2] << ',' << v[4]
        << ',' << std::atan2(-o[6], o[0]) << ',' << r.obsX << ',' << r.obsZ << ',' << r.talkSeq
        << ',' << cupcup_trace::quote(r.talk) << ',' << r.bodySeq << ',' << r.body.type
        << ',' << cupcup_trace::quote(r.body.actname) << ',' << r.body.step << ',' << r.body.lateral
        << ',' << r.body.turn << ',' << r.kickSeq << ',' << r.kickTime
        << ',' << cupcup_trace::quote(r.kickName) << ',' << shared.size()
        << ',' << cupcup_trace::quote(cupcup_trace::pointsText(shared));
    }
    output_ << '\n';
    if (++rows_ % 50 == 0) output_.flush();
    return result;
  }

 private:
  std::array<std::string, 4> names_{{"red_1", "red_2", "blue_1", "blue_2"}};
  std::array<cupcup_trace::RobotSample, 4> robots_;
  webots::Node *ball_ = nullptr;
  rclcpp::Node::SharedPtr observer_;
  common::msg::GameData game_;
  rclcpp::Subscription<common::msg::GameData>::SharedPtr gameSub_;
  std::ofstream output_;
  unsigned long rows_ = 0;
};
}  // namespace webots

// Supervisor.hpp was already included before this substitution; original main
// remains the executable entry point, with unchanged rules and random engine.
#define Supervisor TracedSupervisor
#include RELEASED_SUPERVISOR_FILE
#undef Supervisor
