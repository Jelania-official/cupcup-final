# 二维策略测试与信息验证路线

## 2026-10-09 被动阻挡实验

`run_measurement.py --profile kick-blocking --color red --repeats 1` 使用现有独立
测量安装运行8个左右脚/被动对手场景；红侧进程结束后可换blue运行，禁止两场并行。
原控制器/运动未改；监督器只在ACT开始放球/对手，此后自然物理，不运行生产策略。
用 `analyze_measurement.py <红目录> <蓝目录> --output <报告.json>` 输出独立
cupcup-kick-blocking-v1，不能拿该报告替代名义速度/滚动标定。
已完成两侧共16场景：正面截断与侧向擦碰显著不同；完整数据和局限见TEST_REPORT。
尚未因此更新二维半径/反弹参数，不把被动对手实验当成未知策略对抗成绩。

## 2026-10-09 当前决策依据

原裁判900秒PLAY自对抗已完成，0:0；九次请求均采样触球，但八个窗口随后有其他
机器人触球。应区分“局部踢得到”和“对抗持续推进”，不要把严格无混合接触标签失败
直接称为踢空。新头目标切换保护及实际地面向量诊断仍需真实短局确认，未用旧长局
冒充新版本耐久验证。原启动链仍有服务无限等待风险，异步env未彻底解决。

二维 `evaluate_tactics.py --observation-noise` 支持显式0..1m相对观测sigma敏感性，
保持默认.2、不宣称已校准。同能力/setup2.6/开发种子/场景的288场对照中，有限信息
sigma=.02/.05/.2比分44:4、24:10、0:38；oracle均0:0。零噪声有限信息也0:0。
这既暴露准备过程对噪声敏感，也提示完全信息/无扰动条件的僵持不能仅靠感知改进。
其中shared自对抗需与固定对手分开，换色是旋转副本；不作正式胜率或现实收益结论。
零噪声换边复现纠正中线正负浮点零的半场统计，未改变运动/策略/判罚。

后续研究应固定能力与真实执行约束，先复现近球准备打断及对称争抢场景，再比较
针对执行可行性的策略；不能降低sigma、缩短准备时间或冻结对手来刷成绩。
完整分层指标、二进制身份及失败记录见 `src/cupcup/tests/TEST_REPORT.md`。

## 原输入动态球几何验证（2026-10-08）

`run_measurement.py --profile ball-dynamic --repeats 1 --controller-adapter` 提供
8 个预定视角：慢/快步行、左右转弯、近/远头目标切换后的早期和稳定样本。
使用原始 RGB/IMU/头目标；真值只作本地离线标签，测量适配器不进入比赛启动链。
用同一生产解码器回放，然后由 `score_ball_views.py` 调用生产相机投影评分。
动态报告标记 `comparison_clock=receipt-only`：是接收时刻误差代理，不是曝光同步精度。

红方 `measurements/20261008-160117-red`，蓝方 `20261008-160232-blue`，各完整 8 个，
检出分别 5/8、6/8，无测量跌倒。有效子集地面相对二维误差 median/max 分别
.02254/.03988 m、.03357/.04356 m；半径距离误差 median 为 .20740/.16627 m。
双方早期头目标切换均未检出，红方左转也未检出；没有将缺失计作零误差。
报告在 `perception/20261008-dynamic-ball-{red,blue}-{replay,score}.json`。
未据此重新拟合二维感知 sigma，也不证明全局定位、全部球观测或对手感知可靠。

生产使用接收新鲜度、头目标稳定窗口和跌倒保护条件启用球投影，其余回退半径代理。
短局分析可用 `analyze_world_trace.py <match>/world_trace.csv` 查看距离来源分层。
分层样本按 Talk 序号去重，不是独立曝光；不同来源的出现条件不同，不作配对因果比较。

### 下一处战术改进的取舍

对照 [B-Human 2024 Zweikampf 官方实现说明](https://docs.b-human.de/coderelease2024/behavior/zweikampf/)，
近球对抗不是只比较球的飞行走廊：还检查踢球站位、转身约束、边线风险，并给已选方向
连续性奖励。该队也明确列出计算开销和方向抖动问题；因此不移植整套势场/行为框架。
这属于 SPL/NAO 的参考，不能把它的比赛效果直接归到本平台。

当前 `match_policy.hpp` 已有少量候选方向和滞回，却主要对球路打分，尚未完整评估
抵达踢球站位的代价、站位被占和动作准备时间。后续优先在同一候选评分入口补足这些
执行可行性，而非新建“抢球状态机”；先在二维阻挡场景、不同准备时间和留出对手中
比较，保留旧策略基线与完整失败分母。对手感知仍低可信时不可升级硬障碍，也不可用
不可用的敌方速度预测制造优势。900秒耐久回归期间冻结在跑二进制，不热替换策略。

本轮已试验踢球站位占用软惩罚，共用 `kickStagingPoint`，无新增行为状态。
初版对所有候选计分，在288场开发模型中失球38→42，因此不采用该版本。
修正为沿用导航的 .45 可信门槛：地图当前 .25 低可信候选不参与新惩罚；
开发完整信息子集踢球240→260、被阻挡5779.8→5081.2秒，仍0:0；
带噪声逐场指标完全等同旧版，不能据此声称真实比赛增强。
另选930001–930002的留出种子，完整信息踢球240→256、被阻挡5779.8→5083.4秒，
带噪声仍逐场等同旧版，合计比分0:40。每组288场、180模型秒、setup暂定2.6秒；
颜色是旋转副本，完整信息副本高度相关，既不等于288个独立样本，也不是正式胜率。
这只验证修正解决了部分理想几何占位，未解决得分、估计质量或动作到达时间。
候选通过独立ASCII路径编译测试，未替换正在900秒回归的ROS安装版本。

局部距离仲裁试验未采用：保持其他条件，直接用未平滑本机相对观测代替全局图差分
在带噪声开发子集踢球50→30、失球38→42，已撤回接口/行为，不保留死分支。
这说明“输入形式更一致”不自动等于更鲁棒；当前二维相对噪声未完整标定，仍需区分
原平台稳定近脚几何与二维未测噪声，不据此放弃局部几何用于执行的价值。

## 原输入近球静止校准（2026-10-08）

`run_measurement.py --profile ball-views` 沿用现有测量监督器：固定左右脚前、
中间及偏轴球位，距离约 .16/.18/.3/.6/1/2 m，按视角选择头部目标角。
每个视角等待 5.5 仿真秒后保存原 RGB PPM、IMU、头部目标角与接收年龄，
真值位置只写 `ball_views.csv` 作为离线标签，不发送给比赛策略。
测试适配器只保障测量运行，不能当作原控制器默认启动可靠性证明。

```bash
CUPCUP_INSTALL=/home/j/.cache/cupcup/upstream-validation/install \
ROS_DOMAIN_ID=42 \
FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/cupcup/tests/measurement/fastdds-udp.xml" \
python3 src/cupcup/tests/run_measurement.py --color red \
  --profile ball-views --repeats 1 --timeout 240 --controller-adapter
```

`ball_perception.hpp` 是从生产 player 抽出的同一球解码/纹理验证/候选排序函数，
不引入第二套检测实现。`robot_perception_test --replay` 同时输出球观测；
`replay_robot_perception.py --image-pattern '*.ppm'` 保存模型、执行文件及原图哈希。
`score_ball_views.py <测量目录> <回放JSON> --camera-binary <camera_geometry_test> --output <报告>`
直接调用生产相机投影，只评分完整且新鲜、未跌倒、球已静止的固定视角。

红蓝各 8 个视角都检出球；地面投影相对误差中位 .0086/.0090 m，最大 .0208/.0143 m。
旧 `0.05/radius` 距离代理误差中位 .216/.221 m，脚前两个视角约 .34–.35 m。
这说明原图可支持有用的静止几何，旧半径代理尤其不能代表脚前实际距离。
但它不证明运动中头部目标角等于真实相机姿态，也不是完整检测 PR 或全局地图精度。
原图仍无有效曝光时间戳，不通过假造时间戳拼成动态标定；未拟合二维噪声 sigma，
未把静止厘米级误差直接设置成比赛模型精度。下一步需要运动/转头时的受控验证。

## 原输入静止机器人视角验证（2026-10-08）

测量程序增加 `robot-views`：前面距离 0.6/1.2/2/3 m，背面 1.2/2 m，
侧面及偏轴正面，共 8 个预定视角，红蓝分别运行。
保存原控制器 RGB PPM、原 IMU、头部目标角和接收年龄；目标根节点位置
只写离线评分文件，不发布给比赛代码。测量监督器是测试适配器，不替换提交基础设施。

```bash
CUPCUP_INSTALL=/home/j/.cache/cupcup/upstream-validation/install \
ROS_DOMAIN_ID=41 \
FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/cupcup/tests/measurement/fastdds-udp.xml" \
python3 src/cupcup/tests/run_measurement.py --color red \
  --profile robot-views --repeats 1 --timeout 240 --controller-adapter
```

PPM 回放使用 `replay_robot_perception.py --image-pattern '*.ppm'`，
再用 `score_robot_replay.py --static-views <测量目录> --camera-binary <camera_geometry_test>`
评分，几何直接调用生产相机变换。此静止测试隔离相对几何误差，不含全局自机误差，
不证明运动时头部目标角等于实际姿态。红蓝旋转副本不是独立随机样本。

`robot_map_smoke.py` 是可选 ROS 故障回归，仅允许隔离域 142：使用记录的原输入
和合成零点自机位置，验证生产节点在头部数据中断、恢复和空白图像时更新/过期地图。
它不加载检测网络，不驱动运动，不是比赛成绩或定位准确度验收。

## 2026-10-07 当前推进状态

比赛策略已在原发布 common 与基础设施上编译并运行，原环境测试仅使用启动库路径适配。
实验扩展与原包区分见 `UPSTREAM_BASELINE.md`。二维与 ROS 共用 `match_policy.hpp` 的
状态/意图边界；沙盒模型和真实近脚执行仍不同，详见 `TACTICAL_PLATFORM.md`。

原包测试可加 `run_match.py --official-launch-compat --observe`：将实时地图与意图记录到
比赛目录的 `belief.jsonl`，不要求原控制器提供额外真值/时间字段。图像模型现在支持多框
去重、红蓝/未知分类和短时关联，框高估距仍是低可信诊断信息，未上线地图对抗。
离线回放使用同一生产解码器：

```bash
python3 src/cupcup/tests/replay_robot_perception.py <历史比赛目录>/images \
  --binary /home/j/.cache/cupcup/upstream-validation/build/cupcup/robot_perception_test \
  --model src/cupcup/models/bitbots-2026/opencv.onnx --samples 80 \
  --output /home/j/.local/state/cupcup/perception/robot-replay.json
```

原输入的图像 header 时间为零，不能沿用旧录图脚本的“零时间直接丢弃”。现在
`record_images.py` 对这种输入按接收单调时钟限频，并明确以 `*_receipt_*.jpg` 命名；
`capture.jsonl` 记录真实 header 缺失、接收时间、RGB/BGR 和尺寸，绝不把接收时间充作
仿真拍摄时间。可以做视觉标注/离线检测回放，不能据文件名配对 Supervisor 真值。
有有效 header 的历史文件名保持兼容；机器人代理打分器只接受原 capture 文件名。
RGB、行 padding、两种时钟限频和非法缓冲由独立回调单测覆盖。

当前尚未完成：独立复核与充分场景覆盖的标签评估、机器人全局位置误差验证、原包头部目标角
滞后模型，以及运动/脚法/球物理的完整校准。首轮受控运动、左右脚和自由滚动实验已完成，
见下节；转向上限、加速度、碰撞和观测延迟仍未标定。不要把候选数、无罚站、踢球转移
当成上述工作完成。下一步按这些信息与模型门槛推进，然后研究推进/拦截，而非先刷比分。

种子口径纠正（2026-10-08）：`--seed` 只在修改后的实验监督器生效，原发布监督器
`time(NULL)` 不读取它。下文原环境命令保留历史请求，但不能作同噪声的配对证明。
新清单明确标记种子是否生效；二维固定种子与 Webots 物理随机性也不是一回事。

## 原环境触球旁路（2026-10-08）

2026-10-08 最新进展：40 张 Codex 视觉审阅标签与生产解码回放已用于初步漏检/
错色对照，不再只计输出框数；小样本不替代独立标注复核。二维批测新增 `--setup`
敏感性参数，对手 slow/equal/fast 的准备时间按己方的 1.8/1/.6 倍设置，命令和来源
写入报告。使用实测速率、同种子 971001..971002、相同生产沙盒的两个 288 场对照，
从 1 到 2.6 模型秒后，有限信息固定对手子集由 6:0 变为 0:12，说明旧假设会改变
结论；2.6 来自三例接收时钟近似，只作暂定敏感性值，仍非完整校准。

原环境启动时两次出现 GetAngles 响应发送超时、物理时钟未推进。可用测试专用
`tests/measurement/fastdds-udp.xml` 隔离默认传输路径，既不删除共享内存文件，也不
改原控制器、运动、消息或裁判；运行清单记录环境及 XML 哈希。该条件下短回归已
进入比赛，不能因此断言共享内存就是根因或默认部署稳定性已经通过。使用方式：

```bash
FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/cupcup/tests/measurement/fastdds-udp.xml" \
CUPCUP_INSTALL=/home/j/.cache/cupcup/upstream-validation/install ROS_DOMAIN_ID=41 \
python3 src/cupcup/tests/run_match.py --color red --opponent cupcup \
  --official-launch-compat --released-referee /home/j/.cache/cupcup/released-referee-build/released_referee \
  --referee-play-seconds 90 --duration 220 --observe --images
```

传输配置参考与本机 2.6.12 一致的
[Fast DDS UDP 官方示例](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/udp/udp.html)。

不能只统计 KICK。测试包装器复用未改发布监督器的 main/规则，每次真实 step 后只读
记录球和四机位置/速度、既有定位、Talk、BodyTask 和物理接触点；策略只用原传感器，
没有真值发布话题。原环境默认不注入。先编译独立测量工程，再显式选择：

```bash
cmake -S src/cupcup/tests/measurement -B /home/j/.cache/cupcup/measurement-build \
  -DRELEASED_CONTROLLER_SOURCE=/home/j/.cache/cupcup/upstream-validation/seurobocup-kidsize2026Fin/src/simulation/controller/src
cmake --build /home/j/.cache/cupcup/measurement-build --target traced_supervisor -j2
CUPCUP_INSTALL=/home/j/.cache/cupcup/upstream-validation/install \
python3 src/cupcup/tests/run_match.py --color blue --opponent unirobot \
  --official-launch-compat --trace \
  --trace-judge /home/j/.cache/cupcup/measurement-build/traced_supervisor --observe \
  --released-referee /home/j/.cache/cupcup/released-referee-build/released_referee \
  --referee-play-seconds 150 --duration 400
```

应先 source ROS 和原包隔离安装。包装器保留额外采样的调度开销，并非原二进制完全
相同；它不增加物理步或修改种子。`world_trace.csv.meta.json` 给出原源码和包装器哈希。
20 ms 接触点采样匹配球与机器人同一世界坐标（容差 10 微米），不拿“离球很近”代替
接触。Webots 的 node_id 是本体/子体，不是碰撞对方，依据
[R2023b 接触点提取源码](https://github.com/cyberbotics/webots/blob/R2023b/src/webots/nodes/WbSolid.cpp#L2182)。
采样可能漏掉短触碰，接触也不能单独证明是哪只脚或动作的因果效果。

`analyze_kick_contacts.py` 为每次接收到的新踢球请求保留 4 仿真秒窗口，报告初始相对
球位、首次采样触碰延迟、球速和前向/侧向位移。换阶段、球跳变、采样缺口、新请求或
尾部不足会截断并标记，不能因重开瞬移算成功。仅允许丢弃退出时最后一个不完整 CSV
记录，内部坏数据必须失败；原始文件保持不变。对手同时触球和初始滚动单独排除干净
推进标签。内部脚部节点获取探针没有得到有效数据，已经撤回；当前不声称获得了
实际脚部轨迹或运动服务接受动作的回执。

红 `20261007-153422-red-vs-unirobot` 和蓝 `20261007-153809-blue-vs-unirobot` 各完成
150 原裁判 PLAY 秒，两局 0:0、本队无罚站/跌倒。红两请求都观察到接触和前向移动
1.370/1.229 m；蓝三请求仅一次观察到接触和前向移动 1.121 m。第一脚球没有移动，
第三脚窗口里对手触球，不能把 3 个 KICK 算成 3 次成功。它们是弱基线的诊断局，
不代表胜率；原环境噪声没有固定。下一步根据动作时序和脚位证据修正，不能盲目调阈值。
分析器另记录 `command_pulse_sim_s`：蓝三请求实际接收到的保持跨度为
0.46/0.78/0.78 仿真秒，不能把策略墙钟 1 秒窗口当作固定仿真秒。第一脚未在健康或
球权层被提前取消；较短窗口仍只是相关线索，需要受控隔离等待、脉冲和脚位验证。

## 停步到踢球的受控时序（2026-10-08）

先隔离执行，不把踢空直接归因于视觉。测量工具新增 `--profile kick-timing`，默认
`calibration` 的八项测量保持不变。时序配置同样每轮八项：静止右脚的两种球位、走路
后等待 0/0.8/1.6 仿真秒、0.12/0.8 仿真秒命令，以及走路后左脚对照。走路前史为
3 仿真秒、`step=0.05,count=2`；停步始终 `count=0`。它比线上 `count=1` 的走路队列
更长，结果不能直接当作线上成功概率。

球在走路阶段放远，到 ACT 开始才按当时真实身体朝向放在预先声明的位置，隔离步态
前史与球位变化。这个摆球是**测量夹具**，不进入比赛代码，也没有策略真值输入。
首次 ACT 发布在下一个 20 ms 步，报告区分声明球位、自然关节目标与采样触碰。

有界控制器适配器只记录它原本收到的 GetAngles 响应，**不额外调用消耗队列的服务**。
用安装的原 ActionEngine 离线生成参考，匹配 12 个腿部关节目标；排除 ready/reset 和
左右脚共有姿势，至少三个不同的动作专属目标才记为目标序列证据。它不是关节传感器
反馈、动作完成回执，更不等于触球。触碰仍由球与机器人共有接触点独立核对。

```bash
CUPCUP_INSTALL=/home/j/.cache/cupcup/upstream-validation/install ROS_DOMAIN_ID=41 \
  python3 src/cupcup/tests/run_measurement.py --profile kick-timing \
  --color blue --repeats 2 --controller-adapter --timeout 600
python3 src/cupcup/tests/analyze_measurement.py <已完成测量目录> \
  --output <已完成测量目录>/timing.json
```

先待本场结束再换色，不能同时启动两套 Webots。运行器校验测量程序、运动程序、配置、
参考和适配器哈希在测试期间不变；分析器要求完整观察窗口、时序连续、接触汇总与原始
轨迹一致。需要读 `result.json` 和分析器结果，不能拿“trial 完成”当动作成功。

工程接口参考：[Bit-Bots 动态踢球报告 §4.1](https://bit-bots.de/wp-content/uploads/2021/10/Praktikumsbericht.pdf)
使用 Action Server 状态/完成反馈区分请求与执行；不是本机效果数据。本轮只借鉴证据
分层，没有复制其踢球引擎或给组委会原运动接口增加回执。

已完成红蓝各两轮共 32 项，同一测量/适配器二进制；报告
`measurements/20261008-kick-timing-paired.json`。走路后等 0.8 秒、发 0.12 秒的右脚
4/4 无动作专属目标且无触球；等 1.6 秒 4/4 有动作目标、触球和前向移动。走路后不等
待的右脚有 3/4 球前移，但 4/4 无动作目标，说明接触+球移动也可能只是步态推球。
不能借这些测量宣称一般成功率或修改名义踢球距离。

比赛执行候选只改既有 SETTLE 门槛：默认 `kick_settle_frames=16`，沿用独立图像帧
计数、持球像素容差、0.8 墙钟秒下限、失效退出和有界 ACT；没有新增状态机或改变
原 motion。原图像约 10 Hz，但收帧数**不是严格仿真计时/动作回执**，丢帧只会延后。
`run_match.py --kick-settle-frames 3` 可用同一二进制复现旧门槛作独立对照。原环境噪声
不能固定；必须换色/重复，不能把两次自主比赛称为相同定位噪声的配对测试。

原控制器自主短回归：旧 3 帧蓝方两个完整窗口中 1 次触球前移，另一个请求观察窗口
被 END 截断，不计失败；16 帧蓝方 1/1、红方 2/2 观察到触球及前移，三次前向
1.300/1.325/1.383 m。每局 150 原裁判 PLAY 秒、0:0，本队无罚站/跌倒。这里只
支持执行链的小改动，不能推出胜率或总体成功率。记录见 TEST_REPORT 后续章节。

新分析字段 `observed_settle_to_request_s / observed_settle_to_contact_s` 从连续接收到的
SETTLE 状态估算准备时长，回 ALIGN、比赛换阶段会截断旧准备；缺记录保持未知，不
把动作命令到触球的延迟误当全部准备耗时。蓝/红这三次约 2.400/2.480/2.740 仿真秒。
状态消息有接收滞后，且仅三例；二维默认 `setup=1.0` 没有因此被证明有效。下一轮
需在这一时间区间及压力下检验模型，不能用旧的一秒模型刷出“策略更强”。

## 原始裁判计时验收（2026-10-07）

新增测试专用 `tests/official_referee/`：直接编译未改动发布包中的 `ctrlwindow.cpp/hpp`，
只替换入口，跳过选队对话框并自动调用其公开按钮槽。原 20 ms Qt PLAY 计时、判罚、
比分和自然 END 保持不变；不自行发布 GameData，不加速计时，不进入提交运行依赖。
INIT 等待 2 s、READY 等待 3 s；PAUSE 等待 2 s 后按 INIT/READY/PLAY 重开，这是测试
操作员约定，不是 PDF 对重开耗时的规定。原 INIT 清罚站的行为也没有被修正。

在已 source ROS 与原包隔离安装环境的终端编译：

```bash
cmake -S src/cupcup/tests/official_referee \
  -B /home/j/.cache/cupcup/released-referee-build \
  -DRELEASED_GAMECTRL_SOURCE=/home/j/.cache/cupcup/upstream-validation/seurobocup-kidsize2026Fin/src/gamectrl/src
cmake --build /home/j/.cache/cupcup/released-referee-build -j2
```

先做短验收，再做自然结束的完整原裁判比赛：

```bash
CUPCUP_INSTALL=/home/j/.cache/cupcup/upstream-validation/install \
python3 src/cupcup/tests/run_match.py --color red --opponent cupcup \
  --official-launch-compat --observe --seed 7601 \
  --released-referee /home/j/.cache/cupcup/released-referee-build/released_referee \
  --referee-play-seconds 900 --duration 1800
```

`--duration` 在此模式只是墙钟看门狗，超时返回失败。`released_referee.json` 必须实际
观察到 END、原裁判剩余时间为 0，才记录 `formal_900_completed=true`；不足 900 秒的
短测自动按 Finish，明确标为 false。报告包含原裁判两文件哈希，运行清单另存适配器
及安装态策略二进制哈希；原裁判写的 `results.txt` 隔离在本次日志目录。
原输入没有有效仿真时间时保留 `simulation_seconds=null`，不能由墙钟或 Qt 时间反推。
这是原发布裁判流程/计时验证，不自动证明触球、真实地图精度、全轨迹合规或竞技水平。

短验收 `20261007-150331-red-vs-unirobot` 已完成 20 个裁判 PLAY 秒，观察到 END、剩余
880 s，`formal_900_completed=false`。前一次 `20261007-150117-red-vs-unirobot` 启动未收到
球场心跳，已保留失败记录；原裁判未启动，不能算计时或比赛验证。

完整自对抗 `20261007-150454-red-vs-cupcup` 随后完成：900 原裁判 PLAY 秒、自然 END、
剩余 0；操作器 913.219 墙钟秒、含启停 925.0 秒，期间一次 PAUSE/自动按钮重开。
双方 0:0、0 罚站/跌倒；红/蓝 11/4 个去重 KICK 请求。仿真时间仍未知，触球未由真值
验证；此结果证明原裁判时长与运行链稳定，不证明对抗策略优秀。见完整测试记录。

## 局部图像与全局地图分开测

`compare_ball_geometry.py --output <报告.json>` 复用已有 capture-time 匹配与相机投影，
新增近球/远球及行为状态分组的像素中心误差。图像子集不依赖 FK 姿态字段有效；FK 与
像素的样本群体不同。真值仅离线使用，匹配误差不超过 40 ms；相机为历史实验扩展，
不能当作原发布头部目标角已验证。缺失、过期、视野外目标不补零误差。

历史 `20261007-004124-red-vs-unirobot` 前锋实际球距 <1 m 的 575 个接受观测：像素中心
误差中位 1.817 px、P95 5.262 px、最大 33.431 px；横向/纵向绝对误差 P95 为
3.378/4.694 px（640×480）。这不是检测精确率/召回率，也未统计漏检，不能将该数字
直接换成全局地图噪声。支持继续用地图选择方向、用近脚图像执行；二维近脚执行与实际
图像伺服差异仍需单独标定，不为得到比分而降低全局噪声。

## 受控运动与脚法复测（2026-10-07）

`tests/run_measurement.py` 在原发布安装环境运行独立测量监督器，不启动比赛策略。
球和机器人真值只写本地 CSV；不改变原运动、动作、球模型或正式裁判。
每方 3 次重复，每次 8 个条件：两种前进指令、turn=10、左脚、两种右脚站位、两种自由
滚动初速度。INIT 1.5 s、READY 1.5 s、观察 1 s，运动 8 s 或踢球/滚动 1.5 s，再观察 5 s。
这些都是 Webots 仿真秒。踢球指令只发最初 0.12 s。

原控制器的无界角度服务等待曾导致整个场景停滞，失败实验
`20261007-044219-red` 已保留。测试适配只将等待改为有界重试，仍编译未改的原
`SimRobot.cpp`；这会改变控制调度，不能声称与原控制器逐帧完全相同。新运行器在
90 墙钟秒内没有 CSV 进展时退出并保存原因，不再空等到总超时。

```bash
CUPCUP_INSTALL=/home/j/.cache/cupcup/upstream-validation/install \
  python3 src/cupcup/tests/run_measurement.py --color red --repeats 3 \
  --controller-adapter --timeout 700
# red 完成并退出后，将 color 改为 blue；不要同时启动两套 Webots。
python3 src/cupcup/tests/analyze_measurement.py <红方目录> <蓝方目录> --output calibration.json
```

红 `20261007-131810-red`、蓝 `20261007-132523-blue` 共 48 条完整条件，日志在
`/home/j/.local/state/cupcup/measurements`。报告副本
`src/cupcup/tests/data/measurement-20261007.json` 含原始 CSV/安装产物哈希和限制。
绝对路径仅作实验来源，不是比赛运行依赖。

| 测量 | 6 次重复的结果 | 解释边界 |
| --- | --- | --- |
| step=0.025 / 0.05 前进 | 稳定净速度中位 0.064 / 0.121 m/s | 每次取 ACT 后 2–8 s 的净位移，不是摆动瞬时峰值 |
| turn=10 | 净转速中位 15.77°/s | 不是最大转速，也未测左右转向对称性 |
| 左脚 fwd=0.18、left=+0.08 | 位移中位 1.663 m，严格成功 6/6 | 固定 READY/头角，不等同自主接近成功率 |
| 右脚 fwd=0.18、left=-0.04 | 1.425 m，接触 6/6、严格成功 0/6 | 准备阶段推球超过 3 cm |
| 右脚 fwd=0.22、left=-0.04 | 1.237 m，严格成功 6/6 | 左右脚不能强行镜像 |

严格成功要求位移≥0.25 m、前向、方向偏差≤20°、无跌倒、准备推球≤0.03 m。
这份历史标定的 `contact/contacts` 由位移≥0.25 m 代理，不是接触点证据；不能与
2026-10-08 新时序报告的 `sampled_contact` 混称。本轮发现走路推球反例后已明确区分。
左右参数是**设置位姿**，READY 后实际相对位置也单独记录；重复样本高度相关。

自由滚动只用红方前两次重复拟合，红第三次和全部蓝方作留出验证：指数衰减
λ=0.696/s，8 个留出条件的位置 RMSE 中位 0.0093 m、最大 0.0133 m；恒定减速度
候选中位 0.2132 m。指数模型优于该候选，仅证明所测自由滚动段，碰撞/弹跳不在范围内；
同一留出集用于选择模型，仍需新条件作最终独立检验。

报告提供 `nominal_sandbox_parameters`：前进净速度、所测转速、左脚峰值球速、滚动衰减。
批测可传 `--calibration src/cupcup/tests/data/measurement-20261007.json`；网页可导入同一
报告。默认或导入后的二维结果都保持 `calibrated=false`，不掩盖其他未标定因素。

## 当前测量链：先量信息和运动，不先假设模型

`tests/run_match.py --trace` 会启用监督器本地 CSV 记录。每行包含 Webots 真值（球和
四台机器人的平面位置/速度）、既有定位话题实际发布的位置，以及 `Talk` 中 cupcup
已经交给策略的自机/球世界坐标估计。记录器只订阅/读取并写文件，不发布真值 ROS 话题，
策略进程无法从这条评估旁路读取真值。

运行示例：

```bash
python3 src/cupcup/tests/run_match.py --color red --opponent unirobot \
  --duration 180 --seed 1701 --trace
```

回归脚本会将轨迹放在本场比赛的日志目录 `world_trace.csv`，并自动运行
`analyze_world_trace.py`。可对已有文件再次分析：

```bash
python3 src/cupcup/tests/analyze_world_trace.py \
  /home/j/.local/state/cupcup/matches/<本场目录>/world_trace.csv
```

结果分别报告定位话题误差、策略 Talk 位姿误差、策略可见时的球地图误差、机器人平移/转向
速度和球速分布；新增按 Webots 真值球距分桶的图像半径测距代理误差和地图误差。位姿使用
`pose_age`、球地图使用独立的 `bmap_age` 筛选新鲜样本，距离代理还要求新鲜图像及唯一 Talk
序号；同时报告从图像回调到 Talk 发布的时间、以及图像回调时最近头部/IMU消息的年龄。
更新后的 Webots 控制器为图像、头部、IMU 和粗定位消息写入同一仿真毫秒时钟，报告也会
给出图像相对这些传感器的仿真时钟年龄，并对每帧采集，而非仅统计看见球时刻。
旧的回调年龄仍是本机单调时钟间隔。
由于真值和 Talk 在不同进程中采样，
当前按最近仿真帧对照；消息时延仍是误差的一部分，不能将结果解释成严格同步的传感器标定
精度。一次自由比赛中的速度峰值也不是物理最大速度。

`bdist` 当前只是 `0.050 / 图像归一化球半径` 的距离代理，用于球权优先级和粗球地图，
地图又会影响支援/防守锚点；它不是精确导航距离，近脚控制仍使用图像闭环。
分桶误差用于检验它在哪些视角/距离失效，不会自动校准或改变比赛行为。
进行逐帧球几何对照时必须用 `bstamp_ms`（该球观测被接受时的图像戳）与球框像素配对，
不能用 `istamp_ms`（可能是不含球的新图像戳）；先前按最新图像戳的动态几何统计已作废。

启用 `--trace` 时，策略 Talk 额外附带图像球心/半径、图像尺寸、头部和 IMU 姿态以及
图像/姿态数据年龄。2026-10-07 起，正常赛和点球控制器在 `step()` 完成后读取图像、
实际头角和 IMU，不再用新时间戳标记前一步的图像。策略以有界的 5 ms 回调窗口消化
高频传感器消息；诊断模式保存 32 个姿态样本，按被接受球帧的 `bstamp_ms` 选最近
头角/IMU（误差不大于 40 ms，不做外推）。从 PROTO 的完整机体—颈部—相机链计算
正运动学，并输出 `bgvalid/bgx/bgz` 等影子估计。它只用于日志比较，尚不改变在线地图
或控制；普通比赛不增加这些诊断字段，也不读取真值。仿真采样时间不等同于实机经过
验证的曝光时间，目前未实现姿态插值或跨时刻自机运动补偿。
`Location.msg` 因增加仿真时间字段而改变 ROS 接口；混合部署时必须先统一更新并重建 `common`
及所有消息消费者。非 Webots 发布者若不填该字段，Cupcup 会将它标记为无效并保留回调时序诊断。
正常赛和点球 Webots 控制器的 `/sensor/joint/head` 现已改为读取 `NeckS/Neck2S`
位置传感器的实测角度，过去发布的是电机目标角度；这会直接影响转头期间的视觉几何判断。

受控静态标定场景（需已构建完整工作区）：

```bash
python3 src/cupcup/tests/run_match.py \
  --calibration --color red --opponent unirobot --duration 100 --seed 4421
```

它让裁判停在 READY，机器人身体停止；Webots 将球依次放在距当前队伍 1 号机器人
1.5/2.5/3.5 米处，并让头部依次朝 -20/0/20°、俯仰 20/40°。每阶段 2 秒，
分析器只保留后 0.8 秒中真值位置稳定、实际头部到位、图像戳有效的独立帧，并逐组报告
看见球的比例和旧半径测距误差。球真值只写离线 trace，机器人策略没有真值输入。
将 `--color` 改为 `blue` 可运行镜像站位；分析器支持传入红方拟合的 `--height` 与
`--offset`，用于蓝方留出验证。

可对普通比赛 trace 比较当前球框半径距离代理、静止标定的简化射线法，以及仅供离线
评估的 Webots 真实相机位姿射线—球心高度交点：

```bash
python3 src/cupcup/tests/compare_ball_geometry.py \
  /home/j/.local/state/cupcup/matches/<本场目录>/world_trace.csv --color red
```

真实相机位姿只在监督器的本地 trace 中，比赛策略不会读取。静止受控场景中，用正前方
样本拟合的有效高度约 0.57 m、俯仰偏置约 10.5°，对蓝方镜像场景有收益；但普通走动
比赛中，简化射线法的误差反而明显大于半径法。问题不是只要按图像时间重新取一次头角/IMU
就能解决：运动时它与真实相机俯仰角有数度偏差。真实相机位姿评估显示完整几何仍有潜力，
但它是**不可上线的真值上限**。更新后独立实现的传感器正运动学影子方案，在已测红蓝
动态场景中优于半径法（具体中位/P95 和样本口径见测试报告），但尚未证明在线战术收益。
当前在线比赛继续使用半径法；
只有传感器可复现的相机外参/正运动学模型在红蓝走动比赛、不同角色及尾部误差上都通过
留出验证，才考虑替换或选择性融合。

## 当前运行、复现和失败检查

默认安装目录为 `/home/j/.cache/cupcup/install`，日志保存在
`/home/j/.local/state/cupcup/matches`；可用 `CUPCUP_INSTALL` 和 `CUPCUP_MATCH_LOG`
覆盖。构建缓存放在 ASCII 路径，避免中文源目录的构建兼容问题，也避免重启后 `/tmp`
清理导致原始实验和二进制丢失。每场 `manifest.json` 记录参数及主要二进制/配置 SHA256。

`--duration` **是墙钟秒数，不是比赛仿真秒数**。`summary.txt` 分别记录墙钟总耗时
（含启动/退出）和 CSV 首末时间之差 `simulation_seconds`。本机仿真慢于实时，180 秒
测试可能只有约 90–115 秒仿真；它不能被称为正式三分钟或十五分钟完整比赛。当前简化
裁判仍以墙钟倒计时，正式时长验收必须另行统一裁判与仿真时钟，不能仅将参数写成 900。

```bash
# 真实相机图像旁路记录；仅本队两机，2 Hz，便于检查自身肢体/遮挡误检
python3 src/cupcup/tests/run_match.py --color red --duration 180 --seed 4103 --images
# 蓝方与同代码自对抗：两队都运行 cupcup，四台机器人都使用真实传感器
python3 src/cupcup/tests/run_match.py --color blue --opponent cupcup \
  --duration 180 --seed 4104 --trace
# 有界服务故障：只暂停本场前锋运动服务进程三秒，然后恢复
python3 src/cupcup/tests/run_match.py --color blue --duration 90 --seed 4102 \
  --motion-outage 3 --trace
```

模型曾把自身灰色膝盖及白色号码误认为球。当前 `ball_pattern_filter` 默认启用很小的
黑白纹理后检查，`--no-ball-pattern-filter` 可做本队 A/B；这依赖本平台的球贴图，
不是通用足球视觉或完整自遮挡掩码。`replay_ball_pattern.py` 使用同一 C++ 判断回放已
保存图像，并按真值像素投影标注近球/离群候选；标签是投影代理，不是人工标注精确率。

启动参数只在 `config/strategy.yaml` 维护一份；ROS 通配节点键 `/**` 让动态命名的
两台队员实际读取它。启动文件仅覆盖安装模型路径及明确的开关，不再复制全部参数。

## 为什么先做测量

现有定位噪声在仿真监督器中按半径 `[0, 1] m` 均匀抽样；二维模型若直接假设一个“速度、
射程、传感器精度”，可能从第一天起就与平台不符。先将现有估计与仿真真值离线比较，
再用受控站位/动作测量速度、转弯、球滚动与踢球位移。真值只能用于评估，不得流入比赛
策略或成为二维策略基准的输入。

## 后续阶段

1. 用固定站位/指令采集多个种子的定位与视觉地图误差，确认误差分布及随距离、视角、运动
   的变化。当前报告的字段不够时再扩展日志，不先扩展地图滤波器。
2. 通过短时受控 Webots 动作测量实际平移/转向上限、球速衰减和左右脚踢球的方向/距离
   分布；初赛标定仅作先验，不作为确定参数。
3. 将实测参数接入已建立的独立二维战术环境（使用方法见 `TACTICAL_PLATFORM.md`），
   保持 oracle 真值观测与受噪声/延迟的 `BeliefState` 分离。真实比赛只用估计状态；
   oracle 只用于性能上限和离线评分，不因二维画面流畅就认定比赛策略可靠。
4. 用多种限速对手、多种开局、多随机种子评估拦截、角色站位和推进，不针对单一脚本对手
   调参；用 Webots 做最终的感知/步态迁移检查。

## 初步双机观测结果

一次 seed 2811 的红蓝 cupcup 自对抗中，四台机器人都提供了新鲜球位。按相近观测时间配对，
简单取两机球位中点只将误差中位数改善约 1–2 cm，P95 改善约 0.007–0.012 m；连续帧不独立，
且仍只有一个 seed，因此这是“没有明显证据支持无条件平均”的信号，不是融合收益的统计证明。
同一批测量还显示蓝方后卫的球位误差高于前锋；后续若做融合，应先按距离/视角估计各自不确定度，
必要时剔除低质量观测，而不是默认两台传感器同精度。当前策略未启用双机球位平均。

## 2026-10-09：动态机器人测量入口

`run_measurement.py --profile robot-dynamic --color red --repeats 1` 复用原有测量
监督器和 `robot_views.csv`，八个固定场景为慢走、快走、左右转向，以及 0.6/2.0m
目标在头部目标俯仰由40切到20度后约0.12/1.5秒取样。目标正面朝向观察者，
不是任意对手朝向覆盖。观察者实际速度、角速度和阶段时间写入同一CSV。
使用 `score_robot_replay.py --static-views <测量目录>` 兼容入口评分；输出明确标记
`profile=robot-dynamic`，保留八个场景分母、缺测、运动字段和相机CLI默认高度.365m。
该高度不是生产近球投影的.345m；真值仅用于离线关联与误差，不进入生产策略。
原图无曝光戳、头部消息为目标角，因此这些结果仍是接收时刻代理，不是曝光同步标定。

红方原始控制器测量 `20261009-013126-red` 完成8/8，号码板6/8可关联，
相对位置误差中位.02304m、最大.06765m；两次早期转头取样未获得号码板。
YOEO输出6个框但目标中心代理关联为0，不能把框数当成目标召回率。
结果不提高全局定位或机器人地图置信度，也不据此开启地图硬避障。
蓝方首次 `20261009-013242-blue` 原运动服务回复超时，90.4秒无进展退出、
未完成场景；这是启动失败而非感知失败。显式异步DDS配置尚未解决启动鲁棒性。
原控制器重试 `20261009-013433-blue` 同样失败；有界等待测试适配器
`20261009-013616-blue` 完成8/8，号码板5/8可关联，中位.02422m、最大.06764m。
等待/重试时序不同，不能把这组数据称为原环境启动通过。完整记录见TEST_REPORT末尾。

## 2026-10-09 服务发现等待实验

本机 `rmw_fastrtps_shared_cpp` 6.2.10、Fast DDS 2.6.12。核对
[同版本发送回复源码](https://github.com/ros2/rmw_fastrtps/blob/6.2.10/rmw_fastrtps_shared_cpp/src/rmw_response.cpp)
后，日志 `client will not receive response` 对应等待客户端回复订阅匹配超时，
该等待使用回复writer的 `reliability.max_blocking_time`，不是图像检测或步态失败。
原控制器无界等待未收到的回复，因此一次发现超时即可阻塞全世界同步步进。
这解释了阻塞链，不解释网络为何未及时发现，也不证明全部超时均同一原因。

独立测试配置 `tests/measurement/fastdds-udp-service.xml` 保持UDP、异步/DYNAMIC默认，
只增加名为 `service` 的可靠回复writer配置，将等待上限设10秒。
[Fast DDS 2.6文档](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/ros2/ros2_configure.html)
说明服务回复writer如何选择该配置。运行时须 `RMW_FASTRTPS_USE_QOS_FROM_XML=1`，
并将 `FASTRTPS_DEFAULT_PROFILES_FILE` 指向该文件；没有改变默认提交启动或组委会源码。
测量清单新增DDS环境和配置文件哈希，且检查运行中配置未变。
延长等待不重发请求、不恢复后续丢失回复，存在最长10秒阻塞代价；不能称为通用容错。

新配置蓝方 `20261009-013919-blue` 与红方 `20261009-014027-red` 均在原控制器下
完成8/8，未使用适配器。此前蓝方两次异步配置失败均保留，未覆盖旧结果。
再次蓝方 `20261009-014149-blue` 同样完成8/8；三次测量各约58.1墙钟秒。
新的成功仍是小样本，不替代负载下比赛与长期启动验收。
随后 `20261009-014312-blue-vs-unirobot` 在原控制器与正式裁判下完成120秒PLAY，
图像/地图/trace同时记录，无启动阻塞；139.2墙钟秒、69.98物理仿真秒，0:0且无踢球请求。
这是负载下运行回归，不是完整十五分钟或战术提升验收；测试过程与限制见TEST_REPORT末尾。

## 参考的优秀球队做法

- Hamburg Bit-Bots 的 2025 调查称其在笛卡尔规划中用 IPM，并结合 IMU 姿态与正运动学；
  其 2025 扩展报告还专门为视觉延迟补偿 IPM 与粒子滤波更新，并指出近/远目标检测仍有边缘
  误差。[Bit-Bots 2025 调查](https://humanoid.robocup.org/wp-content/uploads/HamburgBit-Bots_1.pdf)、
  [Bit-Bots 2025 扩展报告](https://humanoid.robocup.org/wp-content/uploads/Hamburg_Bit_Bots-tdp-679d10f9170cf.pdf)。
  这支持记录图像捕获时附近的头部/IMU 姿态和观测年龄，而不是把当前姿态套到旧图像上。
- [ROS-Sports `soccer_ipm`](https://github.com/ros-sports/soccer_ipm) 是 Bit-Bots 提到的
  通用 IPM 软件包，Apache-2.0；其 ROS TF 接口可作为坐标变换参考。我们先校验本机相机几何，
  暂不引入整套 TF 依赖或复制球队行为代码。
- [NUbots 视觉模块](https://nubook.nubots.net/system/modules/vision/) 对球候选使用
  地平线、圆形拟合、角尺寸与投影距离一致性等后检查；借鉴的是“检测后验证再入地图”。
  本队当前黑白纹理检查是针对已观测失败的本平台实现，没有复制 VisualMesh 代码，
  也尚未具备 NUbots 的全部几何检查。
- [RobôCIn 2023 Soccer Simulation 2D 技术报告](https://tdp.robocup.org/wp-content/uploads/tdp/robocup/2023/robocupsoccer-simulation-2d/robocin-429/robocup-2023-robocupsoccer-simulation-2d-robocinIb0KopVX9H.pdf)：报告中的防守拦截会考虑球路、球门方向和对手间线路；值得借鉴的是“预测可达/拦截点并分配角色”，而不是将其 2D 仿真参数照搬到人形机器人。
- [UT Austin Villa 2011 3D 团队报告](https://www.cs.utexas.edu/~AustinVilla/details/AI1110-macalpine.html)：报告了动态角色和阵型定位系统。可作为“角色应随局面调整、而非固定站点”的参考，硬件与运动模型仍不同。
- [CMDragons 选择性响应式协同论文](https://ojs.aaai.org/index.php/AAAI/article/view/10415)：其重点是协调层先形成团队计划，并只在重要变化时响应；有助于避免每个机器人独立追逐每个瞬时噪声。本项目第一步仍是获取可信测量，暂不复制其协调架构。

以上资料提供算法设计依据，不表示这些系统的完整代码、数据或参数可直接用于本队。每个可迁移主张都要在本项目平台上通过对照测试。
