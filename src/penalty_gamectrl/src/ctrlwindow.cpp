#include "ctrlwindow.hpp"
#include <map>
#include <set>
#include <fstream>

using namespace std;

map<int, string> states = {
    {common::msg::GameData::STATE_INIT, "Init"},
    {common::msg::GameData::STATE_READY, "Ready"},
    {common::msg::GameData::STATE_PLAY, "Play"},
    {common::msg::GameData::STATE_PAUSE, "Pause"},
    {common::msg::GameData::STATE_END, "End"},
};

TeamLabel::TeamLabel(QString name, TeamColor color)
{
    colorLabel = new QLabel();
    colorLabel->setFixedWidth(16);
    QString cstr = color == TEAM_RED ? "background-color:red;" : "background-color:blue";
    colorLabel->setStyleSheet(cstr);
    nameLabel = new QLabel(name);
    nameLabel->setStyleSheet("font-size:20px;");
    scoreLabel = new QLabel("0");
    scoreLabel->setFixedWidth(30);
    scoreLabel->setAlignment(Qt::AlignCenter);
    scoreLabel->setStyleSheet("font-size:20px;");

    QHBoxLayout *lay = new QHBoxLayout();
    lay->addWidget(colorLabel);
    lay->addWidget(nameLabel);
    lay->addWidget(scoreLabel);
    this->setLayout(lay);
}

StartDlg::StartDlg(std::string cfg)
{
    std::set<std::string> teamSet;
    ifstream ifs(cfg);
    if(ifs) {
        string line;
        while(getline(ifs, line)) {
            if(!line.empty() && line.back() == '\n') {
                line.pop_back();
            }
            if(!line.empty()) {
                teamSet.insert(line);
            }
            line.clear();
        }
        ifs.close();
    }
    QStringList teams;
    for (auto team : teamSet) {
        teams << QString::fromStdString(team);
    }
    QVBoxLayout *leftLayout, *rightLayout;
    leftLayout = new QVBoxLayout;
    rightLayout = new QVBoxLayout;
    QLabel *redLabel = new QLabel();
    redLabel->setStyleSheet("background-color: red;");
    QLabel *blueLabel = new QLabel();
    blueLabel->setStyleSheet("background-color: blue;");
    redTeamBox = new QComboBox();
    blueTeamBox = new QComboBox();
    redTeamBox->addItems(teams);
    blueTeamBox->addItems(teams);
    leftLayout->addWidget(redLabel);
    leftLayout->addWidget(redTeamBox);
    rightLayout->addWidget(blueLabel);
    rightLayout->addWidget(blueTeamBox);

    startBtn = new QPushButton("Start");
    connect(startBtn, &QPushButton::clicked, this, &StartDlg::OnStart);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *upLayout = new QHBoxLayout;
    upLayout->addLayout(leftLayout);
    upLayout->addLayout(rightLayout);
    mainLayout->addLayout(upLayout);
    mainLayout->addWidget(startBtn);
    setLayout(mainLayout);
}

void StartDlg::OnStart()
{
    redName = redTeamBox->currentText();
    blueName = blueTeamBox->currentText();

    if(redName.size() == 0 || blueName.size() == 0) {
        return;
    }
    if(redName == blueName) {
        QMessageBox::warning(this, "Error", "Two teams is the same!");
        return;
    }
    this->close();
}

std::string g_redName, g_blueName;

void GetColorService(const std::shared_ptr<common::srv::GetColor::Request> req,
    std::shared_ptr<common::srv::GetColor::Response> res)
{
    if (req->team == g_redName) {
        res->color = "red";
    } else if (req->team == g_blueName) {
        res->color = "blue";
    } else {
        res->color = "invalid";
    }
    std::cout << "request: " << req->team << ' ' << res->color << std::endl;
}

CtrlWindow::CtrlWindow(QString red, QString blue)
{
    redName = red;
    blueName = blue;
    g_redName = redName.toStdString();
    g_blueName = blueName.toStdString();
    std::cout << g_redName << ' ' << g_blueName << std::endl;
    totalTime = 60 * 5; // 点球仅有 5 个，每个 1 分钟

    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout, *rightLayout;
    leftLayout = new QVBoxLayout();
    rightLayout = new QVBoxLayout();

    timeLabel = new QLabel(QString::number(totalTime));
    timeLabel->setAlignment(Qt::AlignCenter);
    timeLabel->setStyleSheet("font-size:36px;");
    stateLabel = new QLabel("Init");
    stateLabel->setAlignment(Qt::AlignCenter);
    stateLabel->setStyleSheet("font-size:36px;");
    infoLabel = new QLabel();

    redTeam = new TeamLabel(red, TEAM_RED);
    blueTeam = new TeamLabel(blue, TEAM_BLUE);


    leftLayout->addWidget(timeLabel);
    leftLayout->addWidget(stateLabel);
    leftLayout->addWidget(infoLabel);
    leftLayout->addWidget(redTeam);
    leftLayout->addWidget(blueTeam);

    stateInitBtn = new QPushButton("Init");
    stateInitBtn->setMinimumHeight(40);
    stateInitBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(stateInitBtn);
    stateReadyBtn = new QPushButton("Ready");
    stateReadyBtn->setMinimumHeight(40);
    stateReadyBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(stateReadyBtn);
    statePlayBtn = new QPushButton("Play");
    statePlayBtn->setMinimumHeight(40);
    statePlayBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(statePlayBtn);
    statePauseBtn = new QPushButton("Pause");
    statePauseBtn->setMinimumHeight(40);
    statePauseBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(statePauseBtn);
    stateFinishBtn = new QPushButton("Finish");
    stateFinishBtn->setMinimumHeight(40);
    stateFinishBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(stateFinishBtn);

    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    QWidget *mainWidget = new QWidget();
    mainWidget->setLayout(mainLayout);
    setCentralWidget(mainWidget);

    fTimer = new QTimer();
    connect(fTimer, &QTimer::timeout, this, &CtrlWindow::OnFTimer);
    connect(stateInitBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnInitClicked);
    connect(stateReadyBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnReadyClicked);
    connect(statePlayBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnPlayClicked);
    connect(statePauseBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnPauseClicked);
    connect(stateFinishBtn, &QPushButton::clicked, this, &CtrlWindow::OnBtnFinishClicked);

    fTimer->start(mBasicTime);
    remainTime = kickstartTime = totalTime;
    runTime = 0;
    mGdata.mode = mGdata.MODE_KICK;

    mGdata.state = common::msg::GameData::STATE_INIT;
    mGdata.red_players[0].name = "red_1";
    mGdata.blue_players[0].name = "blue_1";
    mGdata.remain_time = totalTime;
    for (size_t i = 0; i < 1; i++) {
        mGdata.red_players[i].state = common::msg::Player::PLAYER_NORMAL;
        mGdata.blue_players[i].state = common::msg::Player::PLAYER_NORMAL;
    }
    gameDataPublisher = std::make_shared<GameDataPublisher>();
    fieldDataSubscriber = std::make_shared<FieldDataSubscriber>();
    teamColorServiceNode = rclcpp::Node::make_shared("gamectrl_GetColorSrv");
    getColorSrv = teamColorServiceNode->create_service<common::srv::GetColor>("gamectrl/get_color", &GetColorService);
}

void CtrlWindow::OnFTimer()
{
    std::map<int, QString> infos = {
        {mFdata.BALL_OUT, "ball out of field"},
        {mFdata.BALL_NOMOVE, "ball does not move"},
        {mFdata.BALL_GOAL, "goal"},
    };
    rclcpp::spin_some(gameDataPublisher);
    rclcpp::spin_some(fieldDataSubscriber);
    rclcpp::spin_some(teamColorServiceNode);
    mFdata = fieldDataSubscriber->GetData();
    if (mGdata.state == mGdata.STATE_PLAY) {
        if (mFdata.ball_state != mFdata.BALL_NORMAL) {
            mGdata.red_score = mFdata.red_score;
            mGdata.blue_score = mFdata.blue_score;
            mGdata.state = mGdata.STATE_PAUSE;
            OnBtnPauseClicked();
            infoLabel->setText(infos[mFdata.ball_state]);
            blueTeam->SetScore(mGdata.blue_score);
            redTeam->SetScore(mGdata.red_score);
        }
        runTime += mBasicTime;
        if(runTime % 1000 == 0) {
            remainTime--;
            for (size_t i = 0; i < 1; i++) {
                if (mGdata.red_players[i].state == common::msg::Player::PLAYER_WAIT) {
                    if (mGdata.red_players[i].wait_time - remainTime > 30) {
                        mGdata.red_players[i].state = common::msg::Player::PLAYER_NORMAL;
                    }
                }
                if (mGdata.blue_players[i].state == common::msg::Player::PLAYER_WAIT) {
                    if (mGdata.blue_players[i].wait_time - remainTime > 30) {
                        mGdata.blue_players[i].state = common::msg::Player::PLAYER_NORMAL;
                    }
                }
            }
        }
    }
    timeLabel->setText(QString::number(remainTime));
    if(kickstartTime - remainTime >= 60) {
        OnBtnPauseClicked();
    }
    if(remainTime <= 0) {
        OnBtnFinishClicked();
    }
    mGdata.remain_time = remainTime;
    if (mGdata.state != common::msg::GameData::STATE_INIT) {
        for (size_t i = 0; i < 1; i++) {
            if (mFdata.red_players[i].state == common::msg::Player::PALYER_OUT) {
                mGdata.red_players[i].state = common::msg::Player::PLAYER_WAIT;
                mGdata.red_players[i].wait_time = remainTime - (kickstartTime - 60);
            }
            if (mFdata.blue_players[i].state == common::msg::Player::PALYER_OUT) {
                mGdata.blue_players[i].state = common::msg::Player::PLAYER_WAIT;
                mGdata.blue_players[i].wait_time = remainTime - (kickstartTime - 60);
            }
        }
    }
    gameDataPublisher->Publish(mGdata);
}

void CtrlWindow::OnBtnInitClicked()
{
    mGdata.state = common::msg::GameData::STATE_INIT;
    for (size_t i = 0; i < 1; i++) {
        mGdata.red_players[i].state = common::msg::Player::PLAYER_NORMAL;
        mGdata.red_players[i].wait_time = 0;
        mGdata.blue_players[i].state = common::msg::Player::PLAYER_NORMAL;
        mGdata.blue_players[i].wait_time = 0;
    }
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
    infoLabel->setText("");
    remainTime = remainTime / 60 * 60; // Init 后把上一个点球的剩余时间扣除
    kickstartTime = remainTime;
}

void CtrlWindow::OnBtnReadyClicked()
{
    mGdata.state = common::msg::GameData::STATE_READY;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void CtrlWindow::OnBtnPlayClicked()
{
    mGdata.state = common::msg::GameData::STATE_PLAY;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void CtrlWindow::OnBtnPauseClicked()
{
    mGdata.state = common::msg::GameData::STATE_PAUSE;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void CtrlWindow::OnBtnFinishClicked()
{
    mGdata.state = common::msg::GameData::STATE_END;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
    QString endTime = QTime::currentTime().toString("HH:mm:ss");
    ofstream ofs("results.txt", ios::out | ios::app);
    if(!ofs) {
        return;
    }
    ofs << startTime.toStdString() << " --- " << endTime.toStdString() << endl;
    ofs << redName.toStdString() << " : " << blueName.toStdString() << "\t" << mFdata.red_score << " : " << mFdata.blue_score << endl;
    ofs << endl;
    ofs.close();
}