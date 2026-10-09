// Generate reference joint TARGETS from the installed original action engine.
// This does not connect to Webots, ROS services, or competition robot control.
#include <seurobot/action_engine.hpp>
#include <seurobot/seu_robot.hpp>
#include "target_trace.hpp"
#include <fstream>
#include <iomanip>

int main(int argc, char **argv) {
    if (argc != 5) return 2;  // robot.conf offset.conf action.conf output.csv
    auto robot = std::make_shared<seurobot::SeuRobot>(argv[1], argv[2]);
    seurobot::ActionEngine engine(argv[3], robot);
    std::ofstream output(argv[4]);
    if (!output) return 3;
    output << std::setprecision(10) << "action,index,";
    cupcup_measure::header(output);
    for (const std::string action : {"reset", "ready", "left_kick", "right_kick"}) {
        const auto frames = engine.runAction(action);
        if (frames.empty()) return 4;
        for (std::size_t i = 0; i < frames.size(); ++i) {
            output << action << ',' << i; cupcup_measure::angles(output, frames[i]);
        }
    }
    return output ? 0 : 3;
}
