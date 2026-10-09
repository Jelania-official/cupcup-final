// Test-only operator. Link the untouched released CtrlWindow implementation;
// never publish GameData ourselves or accelerate/replace its 20 ms timer.
#include "ctrlwindow.hpp"
#include <QElapsedTimer>
#include <QJsonDocument>
#include <QJsonObject>
#include <fstream>
#include <iostream>

int main(int argc, char **argv)
{
    if (argc != 5) {
        std::cerr << "usage: released_referee red|blue opponent play-seconds result.json\n";
        return 2;
    }
    const std::string color = argv[1], opponent = argv[2], resultPath = argv[4];
    int target = 0;
    try {
        std::size_t parsed = 0;
        target = std::stoi(argv[3], &parsed);
        if (parsed != std::string(argv[3]).size()) target = 0;
    } catch (const std::exception &) {}
    if ((color != "red" && color != "blue") || target < 1 || target > 900 ||
        opponent.empty() || opponent == "cupcup") return 2;
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    CtrlWindow window(QString::fromStdString(color == "red" ? "cupcup" : opponent),
                      QString::fromStdString(color == "blue" ? "cupcup" : opponent));
    window.show();
    auto observer = rclcpp::Node::make_shared("cupcup_released_referee_operator");
    common::msg::GameData latest;
    bool received = false, finishRequested = false;
    int redPenalties = 0, bluePenalties = 0;
    auto subscription = observer->create_subscription<common::msg::GameData>(
        "/sensor/game", 10, [&](common::msg::GameData::SharedPtr msg) {
            for (int i = 0; i < 2; ++i) {
                if (msg->red_players[i].state == common::msg::Player::PLAYER_WAIT &&
                    (!received || latest.red_players[i].state != common::msg::Player::PLAYER_WAIT)) {
                    ++redPenalties;
                    std::cout << "player_out team=red robot=" << i + 1 << std::endl;
                }
                if (msg->blue_players[i].state == common::msg::Player::PLAYER_WAIT &&
                    (!received || latest.blue_players[i].state != common::msg::Player::PLAYER_WAIT)) {
                    ++bluePenalties;
                    std::cout << "player_out team=blue robot=" << i + 1 << std::endl;
                }
            }
            latest = *msg; received = true;
        });
    int previousBall = common::msg::FieldData::BALL_NORMAL;
    auto fieldSubscription = observer->create_subscription<common::msg::FieldData>(
        "/sensor/field", 10, [&](common::msg::FieldData::SharedPtr msg) {
            if (msg->ball_state != common::msg::FieldData::BALL_NORMAL &&
                msg->ball_state != previousBall)
                std::cout << "field event=" << msg->ball_state << " score=" <<
                    msg->red_score << ':' << msg->blue_score << std::endl;
            previousBall = msg->ball_state;
        });
    QElapsedTimer wall, phase;
    wall.start(); phase.start();
    int previousState = -1, previousRemaining = -1, pauses = 0;
    QTimer operatorTimer;
    QObject::connect(&operatorTimer, &QTimer::timeout, [&]() {
        rclcpp::spin_some(observer);
        if (!received) return;
        if (latest.state != previousState) {
            phase.restart();
            if (latest.state == latest.STATE_PAUSE) ++pauses;
        }
        if (latest.state != previousState || latest.remain_time != previousRemaining) {
            QJsonObject status{{"state", latest.state}, {"remain_time", latest.remain_time},
                {"red_score", latest.red_score}, {"blue_score", latest.blue_score},
                {"wall_seconds", wall.elapsed() / 1000.0}};
            std::cout << "released_referee " << QJsonDocument(status).toJson(
                QJsonDocument::Compact).toStdString() << " score=" <<
                latest.red_score << ':' << latest.blue_score << std::endl;
        }
        previousState = latest.state; previousRemaining = latest.remain_time;
        const int played = 900 - latest.remain_time;
        if (latest.state == latest.STATE_END) {
            // END and remaining time are observed from the actual publisher,
            // not inferred from the harness's wall deadline.
            QJsonObject result{{"schema", "cupcup-released-referee-v1"},
                {"requested_play_seconds", target}, {"referee_play_seconds", played},
                {"remaining_seconds", latest.remain_time}, {"end_observed", true},
                {"target_completed", played >= target},
                {"formal_900_completed", target == 900 && latest.remain_time == 0},
                {"wall_seconds", wall.elapsed() / 1000.0}, {"pauses", pauses},
                {"red_score", latest.red_score}, {"blue_score", latest.blue_score},
                {"red_penalties", redPenalties}, {"blue_penalties", bluePenalties},
                {"simulation_seconds", QJsonValue::Null},
                {"referee_cpp_sha256", REFEREE_CPP_SHA256},
                {"referee_hpp_sha256", REFEREE_HPP_SHA256},
                {"caveat", "Original referee timer and judge; test operator automates buttons. "
                           "No simulation-clock or actual-touch evidence from this result."}};
            std::ofstream output(resultPath);
            output << QJsonDocument(result).toJson(QJsonDocument::Indented).toStdString();
            output.flush();
            app.exit(output.good() && played >= target ? 0 : 4);
            return;
        }
        if (played >= target && !finishRequested) {
            // For 900 s let the original timer finish naturally. A shorter run
            // presses Finish and is explicitly NOT a formal full match.
            if (target < 900) window.OnBtnFinishClicked();
            finishRequested = true;
        } else if (!finishRequested) {
            if (latest.state == latest.STATE_INIT && phase.elapsed() >= 2000)
                window.OnBtnReadyClicked();
            else if (latest.state == latest.STATE_READY && phase.elapsed() >= 3000)
                window.OnBtnPlayClicked();
            else if (latest.state == latest.STATE_PAUSE && phase.elapsed() >= 2000)
                window.OnBtnInitClicked();
        }
    });
    operatorTimer.start(100);
    const int result = app.exec();
    rclcpp::shutdown();
    return result;
}
