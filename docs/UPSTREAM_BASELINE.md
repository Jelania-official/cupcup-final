# 复赛发布源码基线追溯

日期：2026-10-07。此记录用于区分发布原包、首次 Git 提交和当前修改，不能以本队
Git 根提交代替组委会源码。

## 找到的改动前压缩包

- 当前路径：`/home/j/.local/share/Trash/files/seurobocup-kidsize2026Fin.zip`。
- 回收站记录的原路径：`/home/j/桌面/seurobocup-kidsize2026Fin.zip`。
- 文件修改时间：2026-09-21 13:57:56 +0800；删除时间：2026-09-21 13:58:38。
- 大小：54,983,530 字节；ZIP CRC 完整性检查通过。
- 包内 README 标题 `SEURoboCup2026`，介绍为“东南大学 Robocup Kidsize 2026年校赛复赛代码”。
- 包内没有 cupcup 包、Git 历史或规则 PDF；规则 PDF 是当前项目中另行提供的文档。
- 原包保留了完整裁判、模型、模板、控制器、motion、启动链和 teams.cfg。

SHA-256：

```text
ea5c238234521d9458642a609ebbc652860b1f324e9d36477c0ded6549faaa16
```

这是找到的改动前发布包候选，时间、目录和 README 均支持其为原始复赛工程。
但本地 ZIP 不含签名或发送者证明，仍请用户确认它就是当时组委会提供的压缩包。
本次直接读取 ZIP，未覆盖工作区，未移动/恢复回收站文件。不要清空回收站丢失这份原包。

## 与本队 Git、工作区的区别

对照提交：`03cf5e66cd88083211457713f1f5c6b8797eb40f`（首次纳入运行基础设施）。
本队根提交 `a8ae968` 主要是球队包，不是官方源码导入历史。

| 文件/模块 | 与 03cf5e6 比较 | 与当前工作区比较 |
| --- | --- | --- |
| template README、player、topics | 字节一致 | 字节一致 |
| gamectrl ctrlwindow.cpp/.hpp | 字节一致 | 字节一致 |
| 正常 2v2 supervisor.cpp | 字节一致 | 已加日志、标定场景、种子与 Location 时间字段 |
| sim-robot.wbt、场地/球门 PROTO | 字节一致 | 字节一致 |
| SEURobot.proto | 字节一致 | 相机增加 DEF 标签；没有改变相机位姿、机器人颜色或物理参数 |
| SimRobot.cpp/.hpp | 字节一致 | 已改变采样顺序、头部反馈和时间戳 |
| motion.cpp、services.cpp/.hpp | **已有修改** | 仍非原包实现 |
| controller.cpp | **已有修改** | 仍非原包实现 |
| start_launch.py | **已有修改** | 仍非原包实现 |
| HeadAngles / ImuData 消息 | 字节一致 | 字段未改变，仅注释/末尾换行变化；发布的时间语义改变了 |
| Location 消息 | 字节一致 | **新增 stamp 字段**，原包只有 x/z |
| 根 README、teams.cfg | 本队配置/说明不能当发布依据 | 与原包不同 |

逐文件扫描 ZIP 中所有文件，与当前工作区对比，发现 17 个文件内容不同，没有原包文件缺失。
这个数字不包括工作区新加的 cupcup、测试和文档；不表示所有差异均是规则更改。

首次 Git 纳入前已有的修改包括：动作只消费一次、动作锁和清空角度队列；控制器从
阻塞等待改为短等待/推进 Webots；WEBOTS_HOME 改为本机 Webots 安装路径。
这些有工程目的，但历史测试是在改动环境下运行，不能冒充原包运行验证。

## 原包中的关键接口事实

- 原 RGB 图像发布器完成 BGRA→RGB，图像每 5 个 20 ms 仿真步发布一次，约 10 Hz。
- 原 Image header.stamp 使用默认 `rclcpp::Time()`；IMU stamp 也由默认时间写入，
  不是有效采样时间。HeadAngles 的 time 没有被赋有效采样时间，Location 没有时间字段。
- 原 SimRobot 的 `mHAngles` 来自运动控制下发的目标，并直接发布；没有启用/读取颈部
  PositionSensor。它不是本队后来增加的真实关节角反馈。
- 因而原包地图应考虑头部动作的跟随滞后、图像延迟和接收时间不等于采样时间，
  不能依赖我们的新增 stamp 或传感器修改才具备基本运行能力。
- 原 motion 已根据球员罚站状态停止运动；这确认罚站停止机制是原有基础设施，
  但不自动扩大模板文档对策略可读字段的许可范围。
- 原监督器的定位噪声种子为 `time(NULL)`，没有本队实验环境的 `CUPCUP_SIM_SEED`
  支持。原环境测试清单里的请求 seed 不证明配对噪声；2026-10-08 已补生效标记。

## 原环境兼容性验证补充（2026-10-07）

将原 ZIP 解压到 ASCII 缓存目录，仅额外加入当前 cupcup 包。16 个包完整构建通过；
cupcup 用原 common 编译成功，Location 不再要求新增 stamp，原零时间戳保留无效标记
并回退到接收时间。实验扩展尚未撤回，不要把工作区整体构建当成原包验证。

原启动第一次因 R2023b 头文件与 ROS 驱动附带 R2025a libController 混用而退出。
测试专用 `official_start_launch.py` 加载原安装 launch，只替换 Webots 库根目录为本机
安装路径；未修改原包控制器、motion、传感器、裁判、消息和模型。使用
`run_match.py --official-launch-compat` 时必须设置 `CUPCUP_INSTALL` 指向原包隔离安装。
这属于启动环境适配，不能写成“原启动文件不加适配即可工作”。

2026-10-08 新增显式 `--trace --trace-judge <测试二进制>`：该二进制编译上表同哈希的
原 `supervisor.cpp`，仅将 Supervisor 类型替换为带只读 `step()` 记录钩子的子类。
原 main、随机引擎、判罚、重置、传感器与运动不改；记录球/四机位置、原定位话题、
Talk、动作请求和接触点到本地 CSV，没有真值 ROS 发布器。它不是原二进制字节一致的
测试：额外订阅与采样有调度开销，清单记录包装器二进制哈希，trace 元数据记录两份
源码哈希。默认原环境启动不注入包装器；原环境 `--trace` 不提供记录器则提前报错，
不能静默成功却没有真值证据。测试工具不进入提交运行依赖。

第一次适配尝试 35 秒等不到 field；90 秒等待复测成功。红方 25 墙钟秒 smoke、蓝方
180 墙钟秒回归已运行；蓝方两机持续工作，0 本队罚站、0 跌倒、5 次踢球状态切换、0:0。
日志分别为 `/home/j/.local/state/cupcup/matches/20261007-041025-red-vs-unirobot` 和
`20261007-041804-blue-vs-unirobot`。这证明原消息/执行链可运行，不证明正式十五分钟、
有效触球次数或感知精度已经验收。

## 原文件指纹

下面路径均相对于 ZIP 中 `seurobocup-kidsize2026Fin/`，不是修改后的工作区指纹。

| 文件 | SHA-256 |
| --- | --- |
| src/simulation/controller/src/supervisor.cpp | `8cd3126979146ed37dd922c428c3e6f0f8e493628322b9d1fe1ad52d2eeb70e8` |
| src/gamectrl/src/ctrlwindow.cpp | `05b653136499e8571d1f4267c591a431749dd78e41433016e7a8f1869caa85bb` |
| src/template/src/README.md | `ec3ab2a4c1e65f83c52afe63be302306cc33776ed95ec3e1a044aa794fda47a3` |
| src/simulation/webots/models/worlds/sim-robot.wbt | `994356a545c60b1942dcc4fcf04ac84823763db0a5b6957604b59ea02e28759a` |
| src/simulation/webots/models/protos/RobocupSoccerField.proto | `13144791218d8a8f0b179e3463256f187d2574af2d45559529aa5b8182bbbc77` |
| src/simulation/webots/models/protos/SEURobot.proto | `c6f7cc3f29f41240a5454be1ecda58182eecbab2a1a8bdd9ead211e8e40c0fbf` |
| src/motion/src/motion.cpp | `82486d1358f3503481dd1e4ab289fb9e869c1ed5025ddaf0ada6ada07544f83a` |

规则与具体判定的复核见 [COMPETITION_RULES_AUDIT.md](COMPETITION_RULES_AUDIT.md)。
