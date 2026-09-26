# cupcup 协作开发约定

本文档是本仓库的最低协作规则。目标是让不同成员可以独立修改、测试和合并，避免
把本地构建产物、未验证参数或初赛路径带入复赛提交包。

## 分支和提交

默认分支为 `main`。每项工作从 `main` 创建短分支，例如：

```text
feat/tactical-handoff
fix/blue-kick-alignment
docs/test-report
```

提交必须遵守 Conventional Commits：

```text
<type>(<scope>): <简短中文标题>

正文使用中文，说明为什么改、改了什么、验证了什么。
必要时补充已知限制或兼容性影响。
```

常用类型：

- `feat`：新增比赛能力；
- `fix`：修复行为、编译或运行问题；
- `test`：新增或调整测试；
- `docs`：文档、规则和实验记录；
- `refactor`：不改变外部行为的代码整理；
- `chore`：构建、工具和仓库维护。

示例：

```text
feat(strategy): 增加后卫失效接管

在队友通信超时且球位于己方半场时允许后卫临时接管。
保留中线安全边界，并通过 strategy_logic_test 和 120 秒 Webots 回归。
```

不要使用没有语义的提交信息，例如 `update`、`修改一下` 或 `final`。一个提交应
尽量只包含一个逻辑主题；如果同时修改策略和文档，正文必须说明二者的关系。

## 合并前检查

提交前至少运行：

```bash
python3 src/cupcup/tests/test_package.py
source /opt/ros/humble/setup.bash
colcon build \
  --build-base /tmp/cupcup-build \
  --install-base /tmp/cupcup-install
ctest --test-dir /tmp/cupcup-build/cupcup --output-on-failure
```

涉及运动、视觉、球权或边界的改动，还应运行至少一场 180 秒 Webots 回归，并把
颜色、对手、时长、比分、踢球次数、跌倒次数和日志目录写入
`src/cupcup/tests/TEST_REPORT.md`。失败实验也要记录，不能只保留成功结果。

## 修改边界

1. 不修改初赛目录，也不把初赛目录的绝对路径写入运行代码。
2. 不把 `build/`、`install/`、`log/`、`__pycache__/` 或本地临时文件提交。
3. 新增 ROS 接口前先讨论；当前策略优先使用已有 `BodyTask`、`HeadTask` 和 `Talk`。
4. 改动规则相关行为时，先对照 `docs/` 中的复赛细则，再更新测试。
5. 模型、外部代码和外部权重必须记录来源、许可证和是否允许随包发布。
6. 参数调优应写入 `config/strategy.yaml`，不要在 `player.cpp` 中散落同一参数的
   多个魔数。

## Pull Request 内容

PR 描述至少包含：

- 问题和预期行为；
- 主要改动文件；
- 执行过的离线测试和 Webots 场景；
- 结果与已知限制；
- 是否改变规则边界、接口或模型来源。

评审重点是安全边界、状态恢复、红蓝对称性、动作是否可能持续触发，以及是否会把
一个局部修复扩展成难以维护的复杂状态机。
