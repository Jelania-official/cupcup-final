# SEURoboCup KidSize 2026 复赛：cupcup

这是 `cupcup` 队的 KidSize 复赛独立提交包。复赛策略位于
[`src/cupcup`](src/cupcup)，不依赖初赛工作区；当前工程的 `common`、`motion`、
`gamectrl`、Webots 启动和裁判基础设施仍作为运行环境使用。

## 快速开始

先准备 ROS 2 Humble、Webots 以及本工程的基础依赖，然后在 ASCII 临时目录构建，
避免中文路径影响 `colcon`：

```bash
source /opt/ros/humble/setup.bash
colcon build \
  --build-base /tmp/cupcup-build \
  --install-base /tmp/cupcup-install
source /tmp/cupcup-install/setup.bash
```

启动仿真和裁判基础设施后，启动两台 `cupcup` 机器人：

```bash
ros2 launch start start_launch.py
ros2 run gamectrl gamectrl
ros2 launch cupcup player_launch.py
```

`player_launch.py` 默认启动 `cupcup_1` 和 `cupcup_2`。如需让同一份代码作为另一
支测试队伍运行，可使用 `team_name:=cupcup_b`，仅用于自对抗回归。

## 代码结构

- `src/cupcup/src/player.cpp`：ROS/Webots 适配、视觉、运动状态机和比赛安全逻辑。
- `src/cupcup/src/strategy_logic.hpp`：无 ROS 依赖的球权、角色和边界决策。
- `src/cupcup/config/strategy.yaml`：阈值、脚法、角色边界和超时参数。
- `src/cupcup/tests/`：离线单元测试、包契约检查和 Webots 回归夹具。
- `docs/CONTRIBUTING.md`：分支、提交、测试和协作约定。
- `src/cupcup/README.md`：策略说明和已完成回归记录。

## 测试

离线包检查：

```bash
python3 src/cupcup/tests/test_package.py
python3 -m py_compile src/cupcup/launch/player_launch.py \
  src/cupcup/tests/run_match.py src/cupcup/tests/mock_gamectrl.py
```

构建后运行 CTest：

```bash
ctest --test-dir /tmp/cupcup-build/cupcup --output-on-failure
```

Webots 回归示例：

```bash
python3 src/cupcup/tests/run_match.py \
  --color red --duration 180 --opponent unirobot
python3 src/cupcup/tests/run_match.py \
  --color blue --duration 180 --opponent unirobot
python3 src/cupcup/tests/run_match.py \
  --color red --duration 120 --opponent cupcup
```

`unirobot` 是工程内的回归基线，不是正式比赛指定对手。回归结果和已知限制记录在
[`src/cupcup/tests/TEST_REPORT.md`](src/cupcup/tests/TEST_REPORT.md)。比赛规则以
[`docs/东南大学第二十三届RoboCup竞赛规则 KidSize组(复赛细则） .pdf`](<docs/东南大学第二十三届RoboCup竞赛规则 KidSize组(复赛细则） .pdf>)
为准。

## 当前提交边界

第一版针对正常 2v2，不实现点球专用策略；重点是视觉闭环、动态球权、后卫接管、
红蓝对称、边界保护和故障恢复。模型来源与许可证复核状态见
`src/cupcup/models/bitbots-2026/provenance.json`，正式对外发布前必须完成模型许可证
确认。
