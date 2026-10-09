#include "tactical_sim.hpp"
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
using namespace cupcup::sim;
void point(std::ostream &out, Point p) { out << '[' << p.x << ',' << p.z << ']'; }
void agent(std::ostream &out, const Agent &p, double now) {
    out << "{\"id\":" << p.id << ",\"x\":" << p.position.x << ",\"z\":" << p.position.z
        << ",\"yaw\":" << p.yaw << ",\"speed\":" << p.speed << ",\"action\":\""
        << actionName(p.action) << "\",\"reason\":\"" << p.reason << "\",\"target\":";
    point(out, p.target);
    out << ",\"kick_target\":"; point(out, p.kickTarget);
    out << ",\"preparing_s\":" << p.preparing << ",\"penalty_s\":" << p.penaltyRemaining
        << ",\"head_pan_deg\":" << p.headPan
        << ",\"observation_age_s\":" << std::max(0.0, now - p.belief.capturedAt)
        << ",\"sees_ball\":" << (p.belief.seesBall ? "true" : "false")
        << ",\"belief_confidence\":" << p.belief.confidence << ",\"observed_ball\":";
    if (p.belief.seesBall) point(out, p.belief.ball); else out << "null";
    out << ",\"observed_ball_relative\":";
    if (p.belief.seesBall) point(out, p.belief.relativeBall); else out << "null";
    out << ",\"mapped_ball\":";
    const auto &ball = p.map.ball();
    if (ball.fresh(now, 0.65)) point(out, {ball.x, ball.z}); else out << "null";
    out << ",\"mapped_ball_age\":";
    if (ball.fresh(now, .65)) out << now - ball.observedAt; else out << "null";
    out << ",\"mapped_ball_source\":\"" << (ball.fresh(now, .65) ?
        cupcup::ballPositionSourceName(p.map.latestBallSource()) : "unknown") << "\"";
    out << ",\"mapped_self\":";
    const auto &self = p.map.self();
    if (self.fresh(now, 2.0)) point(out, {self.x, self.z}); else out << "null";
    out << ",\"peer_obstacle_tracks\":[";
    for (std::size_t i = 0; i < p.sharedObstacles.size(); ++i) {
        const auto &o = p.sharedObstacles[i];
        if (i) out << ',';
        out << "{\"x\":" << o.position.x << ",\"z\":" << o.position.z
            << ",\"confidence\":" << o.position.confidence << ",\"uncertainty\":" << o.uncertainty
            << ",\"age_s\":" << now - o.position.observedAt
            << ",\"team\":\"" << cupcup::robotTeamName(o.team)
            << "\",\"source\":\"teammate_direct_number_square\"}";
    }
    out << ']';
    out << '}';
}
void team(std::ostream &out, const Team &t, double now) {
    out << '['; agent(out, t.players[0], now); out << ','; agent(out, t.players[1], now); out << ']';
}
void frame(std::ostream &out, const World &w) {
    out << "{\"t\":" << w.time - 0.001 << ",\"play_time\":" << w.playTime
        << ",\"phase\":\"" << (w.pauseRemaining > 0.0 ? "PAUSE" : "PLAY")
        << "\",\"event\":\"" << w.event << "\",\"ball\":";
    point(out, w.ball); out << ",\"ball_velocity\":"; point(out, w.velocity);
    out << ",\"red_score\":" << w.teams[0].score << ",\"blue_score\":" << w.teams[1].score
        << ",\"red\":"; team(out, w.teams[0], w.time); out << ",\"blue\":"; team(out, w.teams[1], w.time); out << '}';
}
bool valid(double v, double low, double high) { return std::isfinite(v) && v >= low && v <= high; }
void capability(std::ostream &out, const Capabilities &cap) {
    out << "{\"speed_mps\":" << cap.speed << ",\"turn_degps\":" << cap.turn
        << ",\"kick_speed_mps\":" << cap.kickSpeed << ",\"setup_s\":" << cap.setup
        << ",\"kick_range_m\":" << cap.kickRange << ",\"cooldown_s\":" << cap.cooldown << '}';
}
void metrics(std::ostream &out, const Team &team, double playTime, const char *color) {
    out << ",\"" << color << "_mean_ball_attack_m\":" << team.ballPositionIntegral / std::max(1e-9, playTime)
        << ",\"" << color << "_opponent_half_s\":" << team.territorySeconds
        << ",\"" << color << "_own_box_s\":" << team.threatSeconds
        << ",\"" << color << "_uncontested_ball_access_s\":" << team.accessSeconds
        << ",\"" << color << "_contested_ball_s\":" << team.contestedSeconds
        << ",\"" << color << "_robot_blocked_s\":" << team.blockedSeconds
        << ",\"" << color << "_kick_reach_s\":" << team.kickReachSeconds
        << ",\"" << color << "_kick_facing_s\":" << team.kickFacingSeconds
        << ",\"" << color << "_kick_aim_s\":" << team.kickAimSeconds
        << ",\"" << color << "_kick_ready_s\":" << team.kickReadySeconds
        << ",\"" << color << "_kick_setup_resets\":" << team.setupResets
        << ",\"" << color << "_max_kick_preparing_s\":" << team.maxPreparingSeconds;
}
}

int main(int argc, char **argv) {
    Config config;
    bool fullFrames = true;
    if (argc % 2 == 0) { std::cerr << "options require a value\n"; return 2; }
    try {
        for (int i = 1; i < argc; i += 2) {
            const std::string key = argv[i], value = argv[i + 1];
            auto numeric = [&value]() {
                std::size_t used;
                const double n = std::stod(value, &used);
                if (used != value.size()) throw std::invalid_argument("number");
                return n;
            };
            if (key == "--seed") {
                const double seed = numeric();
                if (!valid(seed, 0.0, 4294967295.0) || seed != std::floor(seed)) throw std::invalid_argument("seed");
                config.seed = static_cast<unsigned>(seed);
            } else if (key == "--duration") config.duration = numeric();
            else if (key == "--frames" && (value == "full" || value == "final"))
                fullFrames = value == "full";
            else if (key == "--noise") config.noise = numeric();
            else if (key == "--self-noise") config.selfNoise = numeric();
            else if (key == "--delay") config.delay = numeric();
            else if (key == "--dropout") config.dropout = numeric();
            else if (key == "--ball-decay") config.ballDecay = numeric();
            else if (key == "--kick-hysteresis") config.kickDirectionHysteresis = numeric();
            else if (key == "--defense-mode" && (value == "line" || value == "fixed"))
                config.defenderLineAnchor = value == "line";
            else if (key == "--vision-model" && (value == "pan" || value == "legacy"))
                config.headPanVision = value == "pan";
            else if (key == "--kick-speed") config.own.kickSpeed = config.other.kickSpeed = numeric();
            else if (key == "--speed") config.own.speed = numeric();
            else if (key == "--turn") config.own.turn = numeric();
            else if (key == "--setup") config.own.setup = numeric();
            else if (key == "--opponent-speed") config.other.speed = numeric();
            else if (key == "--opponent-turn") config.other.turn = numeric();
            else if (key == "--opponent-setup") config.other.setup = numeric();
            else if (key == "--opponent-kick-speed") config.other.kickSpeed = numeric();
            else if (key == "--opponent") config.opponent = value;
            else if (key == "--scenario") config.scenario = value;
            else if (key == "--initial-state") {
                std::istringstream input(value);
                for (auto &n : config.initialState) {
                    if (!(input >> n)) throw std::invalid_argument("initial state requires 16 numbers");
                }
                input >> std::ws;
                if (!input.eof()) throw std::invalid_argument("extra initial state data");
                config.customInitialState = true;
            }
            else if (key == "--cupcup-color" && (value == "red" || value == "blue"))
                config.cupcupColor = value == "red" ? cupcup::TeamColor::Red : cupcup::TeamColor::Blue;
            else if (key == "--mode" && (value == "oracle" || value == "realistic")) config.oracle = value == "oracle";
            else throw std::invalid_argument("unknown option or value: " + key);
        }
        if (!valid(config.duration, 1, 900) || !valid(config.noise, 0, 1) || !valid(config.selfNoise, 0, 1) ||
            !valid(config.delay, 0, 2) || !valid(config.dropout, 0, 1) || !valid(config.ballDecay, 0.05, 5) ||
            !valid(config.kickDirectionHysteresis, 0, 2) ||
            (config.opponent != "press" && config.opponent != "block" && config.opponent != "keeper" && config.opponent != "shared") ||
            (config.scenario != "kickoff" && config.scenario != "own-half" && config.scenario != "incoming" &&
             config.scenario != "sideline" && config.scenario != "shot")) throw std::invalid_argument("parameter out of range");
        for (auto cap : {config.own, config.other})
            if (!valid(cap.speed, 0, 1.5) || !valid(cap.turn, 1, 360) || !valid(cap.kickSpeed, 0, 8) ||
                !valid(cap.setup, 0.05, 10)) throw std::invalid_argument("capability out of range");
        if (config.customInitialState) {
            const auto &s = config.initialState;
            for (int i = 0; i < 4; ++i) {
                if (!valid(s[3*i], -4.3, 4.3) || !valid(s[3*i+1], -2.8, 2.8) ||
                    !valid(s[3*i+2], -180, 180)) throw std::invalid_argument("initial robot out of range");
                for (int j = 0; j < i; ++j)
                    if (distance({s[3*i], s[3*i+1]}, {s[3*j], s[3*j+1]}) < .36)
                        throw std::invalid_argument("initial robots overlap");
            }
            if (!valid(s[12], -4.5, 4.5) || !valid(s[13], -3, 3) ||
                !valid(s[14], -8, 8) || !valid(s[15], -8, 8))
                throw std::invalid_argument("initial ball out of range");
        }
    } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 2; }

    Simulation sim(config);
    std::ostringstream frames; frames << std::setprecision(8);
    if (fullFrames) frame(frames, sim.world);
    const int ticks = static_cast<int>(std::ceil(config.duration / stepSeconds));
    for (int tick = 1; tick <= ticks; ++tick) {
        sim.tick();
        if (fullFrames && (tick % 4 == 0 || tick == ticks)) { frames << ','; frame(frames, sim.world); }
    }
    if (!fullFrames) frame(frames, sim.world);
    const auto &w = sim.world;
    std::cout << std::setprecision(8) << "{\"model\":\"bounded-2d-v3\",\"calibrated\":false,\"seed\":" << config.seed
        << ",\"abstract_opponent_revision\":2"
        << ",\"robot_contact_revision\":2"
        << ",\"ball_contact_revision\":2"
        << ",\"observation_execution_revision\":2"
        << ",\"horizontal_vision_revision\":" << (config.headPanVision ? 2 : 1)
        << ",\"recording_mode\":\"" << (fullFrames ? "full" : "final") << "\""
        << ",\"duration_s\":" << config.duration << ",\"opponent\":\"" << config.opponent
        << "\",\"cupcup_color\":\"" << (config.cupcupColor == cupcup::TeamColor::Red ? "red" : "blue")
        << "\",\"mode\":\"" << (config.oracle ? "oracle" : "realistic") << "\",\"scenario\":\"" << config.scenario
        << "\",\"parameters\":{\"max_speed_mps\":" << config.own.speed << ",\"max_turn_degps\":" << config.own.turn
        << ",\"observation_sigma_m\":" << config.noise << ",\"self_sigma_m\":" << config.selfNoise
        << ",\"delay_s\":" << config.delay << ",\"dropout\":" << config.dropout << ",\"ball_decay_per_s\":" << config.ballDecay
        << ",\"near_ball_translation_fraction\":0.25,\"robot_candidate_confidence\":"
        << (config.oracle ? 1.0 : 0.25)
        << ",\"vision_model\":\"" << (config.headPanVision ? "pan" : "legacy") << "\""
        << ",\"horizontal_fov_deg\":" << (config.headPanVision ? 2 * cameraHalfFov : 190.0)
        << ",\"head_pan_speed_degps_provisional\":" << (config.headPanVision ? headPanSpeed : 0.0)
        << ",\"vision_limits\":\"oracle bypasses visibility; no pitch, occlusion or panel geometry\""
        << ",\"kick_direction_hysteresis\":" << config.kickDirectionHysteresis
        << ",\"defense_mode\":\"" << (config.defenderLineAnchor ? "line" : "fixed") << "\""
        << ",\"kick_speed_mps\":" << config.own.kickSpeed << ",\"own\":";
    capability(std::cout, config.own); std::cout << ",\"opponent\":"; capability(std::cout, config.other);
    std::cout << ",\"initial_state\":";
    if (config.customInitialState) {
        std::cout << '[';
        for (unsigned i = 0; i < config.initialState.size(); ++i)
            std::cout << (i ? "," : "") << config.initialState[i];
        std::cout << ']';
    } else std::cout << "null";
    std::cout << "},\"metrics\":{\"red_kicks\":" << w.teams[0].kicks << ",\"blue_kicks\":" << w.teams[1].kicks
        << ",\"red_penalties\":" << w.teams[0].penalties << ",\"blue_penalties\":" << w.teams[1].penalties
        << ",\"restarts\":" << w.restarts << ",\"collision_steps\":" << w.collisions
        << ",\"play_seconds\":" << w.playTime << ",\"stationary_ball_s\":" << w.stationaryBallSeconds;
    metrics(std::cout, w.teams[0], w.playTime, "red"); metrics(std::cout, w.teams[1], w.playTime, "blue");
    std::cout << "},\"frames\":[" << frames.str() << "]}\n";
}
