# Cupcup 复赛独立包

独立提交的正常 2v2 策略包；初赛项目仅作算法与标定记录参考，不是运行依赖。
当前状态：基础闭环、地图记录与二维研究平台已接通，强对抗表现和正式长局仍未达标。
历史成功场次不代表当前版本性能，完整证据与失败记录见 [测试报告](tests/TEST_REPORT.md)。

## 启动

先启动组委会场地、运动与裁判基础设施，以及另一支队伍，再启动：

```bash
source /opt/ros/humble/setup.bash
source <当前安装目录>/setup.bash
ros2 launch cupcup player_launch.py
```

启动两名队员 `cupcup_1` / `cupcup_2`，通过裁判接口确定红蓝映射。
比赛包兼容原发布消息；本地新增时间戳/实测头角等实验扩展不是提交依赖。
原发布环境的本机 Webots 库适配见 [基线说明](../../docs/UPSTREAM_BASELINE.md)。

使用 ASCII 构建/安装目录，避免中文路径的构建兼容问题。参数只维护在
[strategy.yaml](config/strategy.yaml)，模型使用安装包路径，不访问初赛工作区。

## 系统与行为

观测 → `WorldModel` → `MatchState / MatchIntent` → 真实动作或二维有界执行。

- 号码权限固定：1 号不得进入己方大禁区；2 号不得越过中线。动态分工不会交换权限。
- 按球观测、健康状态、距离和时效仲裁球权；从合法站位够不到的球不得保留球权。
  另一人支援或保护己方半场。目前不是经过验证的时间最优抢球/传球系统。
- 前锋进攻和后卫解围共用
  `SEARCH → APPROACH → ORBIT → ALIGN → SETTLE → KICK → VERIFY / RECOVER`。
  让出球权时取消旧准备状态；DEFEND 内不再独立下发踢球。左右脚参数独立，
  踢球请求为有界接受窗口，不持续重复触发。
  SETTLE 默认要求 16 个独立稳定图像帧（仍保留墙钟下限），避免墙钟等待过快而原
  停步队列尚未排空；`kick_settle_frames:=3` 仅用于复现旧门槛对照。它不是动作回执。
- 暂停、重开、摔倒和传感器超时停止或重置；安全位姿使用带缓冲/滞回的粗地图。
  起身由原运动层负责，策略等待站稳。
- 模型检测配合连续帧确认；无模型有传统 OpenCV 回退。通用机器人框可确认红/蓝/未知，
  但局部编号不是球衣号码。估距未充分验证，位置置信度不超过 0.25，暂不输入 ROS
  地图对抗；近距离图像让行仍保留。

世界模型与限制详见 [WORLD_MODEL](../../docs/WORLD_MODEL.md)；
比赛规则以 [规则审计](../../docs/COMPETITION_RULES_AUDIT.md) 和原发布文档/代码为准。

## 二维平台与测试

```bash
ros2 run cupcup cupcup_visualizer
# 仅使用独立沙盒：
ros2 run cupcup cupcup_visualizer --no-ros
```

默认访问 http://127.0.0.1:8765 。比赛观察只读记录估计与意图，支持导出/回放；
独立沙盒使用同一决策入口与有界抽象对手，显示原始观测、决策地图及模拟真值。
真值不进入实际比赛策略。二维运动/感知/裁判仍是部分标定的研究模型，不是完整 Webots
副本，不能将其成绩当作正式比赛胜率。详见 [平台说明](../../docs/TACTICAL_PLATFORM.md)。

`tests/run_match.py` 支持原 unirobot、同代码自对抗及脚本压力对手。`--duration` 是
墙钟秒，不是正式 PLAY 时长；KICK 转移也不是成功触球证据。原 unirobot 是弱模板，
真实比赛与其他队伍对战。受控速度/脚法/滚动测量与后续验证路线见
[TACTICAL_SIMULATION](../../docs/TACTICAL_SIMULATION.md)。

合作遵循 [CONTRIBUTING](../../docs/CONTRIBUTING.md)：简单有界、实测优先、失败可复现；
未经用户要求不提交/推送，仅在当前明确要求时安排自动关机。
## 可选球候选否决器（验证中）

`player_launch.py` 提供 `ball_verifier_model`，默认空字符串、不启用。
它只否决负例分数≥.9且球分数≤.1的候选，模糊项保留；不改变球尺寸或地图信任。
模型必须为兼容的32×32 NHWC灰度输入、6值输出的外部ONNX模型。
加载/推理异常会明确警告并关闭组件，恢复原检测；传统视觉回退未接入该组件。
研究权重未随包提供，具体提交授权尚未确认，不要把研究缓存复制进提交包。
本组件已做同图回放验证，尚不能据此宣称完整比赛感知或竞技表现改善。
