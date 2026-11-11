# 手套闭环机构：MuJoCo 控制预览修正版

## 启动

在仓库根目录中，激活已经安装 `mujoco` 和 `numpy` 的环境（例如项目的 `myglove` 环境）：

```bash
conda activate myglove
python3 simulation/glove/view_glove.py
```

- 启动后四根 EF 杆的端点连线与手掌平面平行，初始视角自动居中并拉近；空格切换暂停/运行。
- Backspace 恢复 EF 水平参考姿态及其控制目标；Shift+Tab 显示右侧面板，展开 Control。
- 使用 Control 控制目标角度，单位为弧度。Joint 面板直接改状态，可能暂时破坏闭环，不建议用于检查联动。
- 每根手指只有两个滑块：MCP 外展／内收（名称见下表）和舵盘关节（食指 `index_motor`、中指 `middle_motor`、无名指 `ring_motor`、小指 `little_motor`）。
  面板从上到下按食指、中指、无名指、小指排列，每指 MCP 和舵盘控制成对放置。
  四个舵盘滑块统一为左拉展开到水平、右拉扣紧，不能继续向上扬起。
  其余连杆关节没有执行器，是被动关节，通过闭环约束随舵盘关节运动。
- 食指 `index_motor` 和无名指 `ring_motor` 的执行器 gear 为 -1，显示控制角度与模型关节角度相反；
  中指和小指保持 gear 为 +1。桥接脚本会在两个方向自动换算，标定零点、方向、偏移均保持关节角度含义，
  无需因为调整滑条方向而重新标定。`ef_horizontal` 的控制目标已按执行器顺序和 gear 同步设置。
- 目标角度不一定都能实现，仍受闭环几何和机构奇异位置约束。
- 四指弯曲控制的下界为 EF 水平参考姿态，MCP 外展范围不受这次限制影响。
  这是控制目标范围，不会删除关节或修改连杆几何；直接编辑 Joint 状态仍可能破坏闭环。

也可以使用标准查看器：

```bash
python -m mujoco.viewer --mjcf=simulation/glove/scene.xml
```

## 与实物舵机同步

4 个 XL330 弯曲舵机分别驱动各指连在舵盘上的连杆关节：ID 2 食指 `index_motor`、ID 3 中指 `middle_motor`、
ID 4 无名指 `ring_motor`、ID 5 小指 `little_motor`。外展／内收关节没有实物传感器，始终用 sidebar 控制。
实际映射以 `config/servos.json` 为准；重新标定会保留其中的 ID、关节对应关系和方向。

舵盘关节与执行器使用同一名称，Control 面板中的新旧名称对应如下：

| 原名称 | 当前名称 | 手指 / 实物 ID |
| --- | --- | --- |
| j2 | index_motor | 食指 / 2 |
| j10 | middle_motor | 中指 / 3 |
| j26 | ring_motor | 无名指 / 4 |
| j18 | little_motor | 小指 / 5 |

此次仅同步名称，当前配置中的零点、方向、偏移和位置限位均保留，不需要重新标定。重启 viewer 后生效。

标定时扭矩会被关闭。把四根 EF 杆摆到与手掌平面平行的位置，按 Enter 记录零点，
再把每根手指活动到可用范围并按 Enter 记录限位。显示 `Saved` 后结果写入 `config/servos.json`。
`offset` 自动从模型的 `ef_horizontal` 参考姿态取得，不再沿用旧零姿态留下的偏移。
旧配置不会被预览程序自动重写；使用新参考姿态前需要完成一次这种水平姿态标定：

```bash
python3 scripts/calibration.py
```

然后选择方向启动：

```bash
python3 simulation/glove/servo_bridge.py real2sim   # 扭矩关闭，动实物手指，MuJoCo 跟随
python3 simulation/glove/servo_bridge.py sim2real   # 扭矩开启，拖 sidebar 滑块，舵机跟随
```

- sim2real 默认把四个已配置电机的 Goal Current 统一设为 **70 mA**，上下运动使用相同上限，
  不根据方向或手掌朝向切换。开始前确认全部为模式 5，关闭扭矩后按需提高各自 Current Limit，
  读回核对设置，再按正常流程回位并接受滑块控制。Kp/Ki/Kd、标定和运动范围保持原值。
  这是运行期间的设置；正常关窗、异常退出或 Ctrl+C 时先关闭扭矩，再恢复启动前的电流设置。
  如需原来的 20 mA 对照，显式添加 `--current-ma 20`。real2sim 和 `--settings-only` 不应用这项默认值。
- sim2real 启动时仿真保持 `ef_horizontal` 姿态，实物先保持当前位置，再以默认 0.3 rad/s 的目标变化速度
  缓慢回到 EF 水平参考。以编码器位置确认四指均到位（误差不超过 13 tick，约 0.020 rad / 1.14°），
  且连续保持 0.2 秒后，终端显示 `Sliders ready`，
  才接受 sidebar 控制。回位期间拖动滑块不会改变实物目标。关闭窗口或 Ctrl+C 时释放扭矩。
- 回位要求配置的 `reference_pose` 为 `ef_horizontal`、标定零点与模型参考匹配，统一限流的默认启动方式要求模式 5，
  且起始位置位于已标定的原始位置限位内。实物在水平边界另一侧时，会从当前位置逐步返回，
  不会一步跳到滑块边界；整个回位路径仍受原始标定限位约束。15 秒内实物未到位则退出并释放扭矩。
- 回位完成后，舵机目标直接取 sidebar 滑块值（不等仿真动力学），双向使用相同的默认 6 rad/s 目标限速。
  限速按真实经过的时间计算，单次计算最多按 0.05 秒推进，避免窗口卡顿后补发大跨度目标。
  这是目标变化速度限制；实物能否跟上仍取决于电流、负载和电机内部控制。
- 桥接窗口的弯曲滑条范围是模型允许范围与实物标定范围的交集；回位后的实际指令使用相同限制。
  real2sim 仍是扭矩关闭、仿真跟随实物；超出该区间的实物读数仍会读取，但仿真控制目标止于水平边界。

可调整正常目标限速、回位速度和回位超时，并每秒查看实物反馈：

```bash
python3 simulation/glove/servo_bridge.py sim2real --diagnostics
# 可选：--max-speed 1.0 --home-speed 0.2 --home-timeout 20
# 到位判定容差：--home-tolerance-ticks 13
```

启动时会读取并显示每个 ID 的 Operating Mode、Goal Current、Current Limit、Drive Mode 和 Profile 参数，
其中电流参数显示的是已应用的统一运行上限；PID、Drive Mode 和 Profile 不自动修改。
`--diagnostics` 额外显示 HOMING/READY 阶段、目标角度、实际角度、电流、电压和扭矩状态。
回位期间每秒显示 `HOMING: sliders locked` 和各 ID 的实物位置误差，滑块此时尚不能改变实物目标。
先前 2 tick 的回位容差容易被低电流下的静态跟随误差卡住；现在把物理到位容差与标定一致性检查分开。
回位目标仍是原来的精确水平零点，标定配置、运动限位、电流和 PID 不因容差变化而改写。
若抬起时目标已变化但实际位置明显滞后，且电流接近 Mode 5 的 Goal Current 上限，
应检查限流是否导致驱动力不足；电流未达到上限时，还需检查机构阻力、供电和控制参数。
单凭上下速度差不能确定原因；软件限速相同也不保证负载不同的双向实物速度相同。

### 分步排查电流与 Kp/Kd

首先关闭占用串口的 viewer 或 Wizard，读取四个已配置 ID 的基线参数：

```bash
python3 simulation/glove/servo_bridge.py sim2real --settings-only
```

该命令只读取并打印设置后退出，不打开 viewer、不写寄存器、不改变 Torque Enable。
表中还包括 Kp/Ki/Kd（原始寄存器值）及当前扭矩开关状态。若电机原本已经开启扭矩，此命令不会将其关闭。

然后用 `sim2real --diagnostics --current-ma 20` 做原 20 mA 的基线运动：只动一根手指，以同一小幅度、同一滑块速度
重复屈曲和回到水平三次，比较上抬与下放时的目标/实际位置差、电流和电压。
后续按以下顺序对照，每轮仍使用相同的运动区间和速度：

1. 保持 Kp、Ki、Kd 不变，先检查 Mode 5 的 Goal Current 和 Current Limit；具体调整幅度依据读数确定。
   Goal Current 不能超过 Current Limit；两者都较低时，单独增大 Goal Current 会被硬件限制。
2. 电流条件确认后，保持电流设置、Ki、Kd 不变，仅小幅调整 Kp，比较跟随误差和上抬速度。
3. Kp 确认后，再单独评估 Kd；已有过冲或振荡时比较阻尼效果，不把提高 Kd 作为提高抬起速度的首选。

只读报告不修改参数；基线指定 20 mA，默认运行指定 70 mA，均不自动调节 PID 或根据方向切换电流。
XL330 的反馈电流是输入端电流，适合结合位置误差观察趋势，
不能仅凭一次读数接近上限，就断定电流饱和或精确推算关节扭矩。
寄存器和模式说明见 [ROBOTIS XL330-M288 e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/#position-pid-gain80-82-84)。

单指电流对照测试可用以下命令（示例为 ID 2 从已读取的 20 mA 改为 25 mA）：

```bash
# 基线：保持原参数，记录相同小幅度的下放和抬起
python3 simulation/glove/servo_bridge.py sim2real --diagnostics --current-ma 20
# 关闭基线窗口后，只调整 ID 2 的电流，PID、速度和其他 ID 的参数保持不变
python3 simulation/glove/servo_bridge.py sim2real --diagnostics --test-id 2 --test-current-ma 25
```

显式指定这两个测试选项时，只临时调整该测试 ID，跳过四指默认 70 mA 设置。
测试 ID 必须在配置中、运行在模式 5，且开始时扭矩关闭；不能与 `--current-ma` 同时使用。
若原 Current Limit 不足以允许指定 Goal Current，脚本先临时提高该 ID 的 Current Limit，再写 Goal Current，
并核对写入值；测试完成、窗口异常或 Ctrl+C 后先关闭扭矩，再恢复原 Goal Current 和 Current Limit。
其他 ID 仍按正常桥接逻辑回到水平并响应滑块，但其电流和 PID 设置不改变。
基线和测试轮都只操作同一根手指，以相同运动幅度比较。

Current Limit 属于 EEPROM；强制杀进程、断电或串口中断时不能保证恢复。
测试退出后用 `--settings-only` 核对原值。若恢复失败，程序会报告错误，不会假定参数已恢复。

- 若某根手指在仿真和实物中方向相反，把 `config/servos.json` 中该舵机的 `direction` 改为 `-1`；
  重新标定会保留已设置的方向。
- 闭环几何会限制可达角度（例如食指 `index_motor` 从初始姿态正向约 0.13 rad 即卡住），
  超出部分仿真无法跟随；sim2real 下舵机仍按滑块值运动，只受标定范围限制。
- 串口适配器默认 16 ms 延迟会把同步频率限制在约 30 Hz，可设为 1 ms：
  `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`

## 外展／内收关节命名

XML 中的关节名称、驱动器名称及其 joint 引用使用相同名称：

| 原编号 | 手指 | 当前名称 |
| --- | --- | --- |
| j1 | 第2指：食指 | index_mcp_abduction |
| j9 | 第3指：中指 | middle_mcp_abduction |
| j25 | 第4指：无名指 | ring_mcp_abduction |
| j17 | 第5指：小指 | little_mcp_abduction |

这是直接修改 MuJoCo XML，尚未修改 Onshape 的配合名称。
`config.json` 和验证报告保留导出时的原始编号。重新导出会覆盖 XML 中的命名；
若在 Onshape 中使用新名称，需要给配合名称加 `dof_`，并在导出配置中
为这些新名称设置 `actuated: true`（原来的 `j*` 规则不匹配新名称）。
舵机映射在 `utils/glove_servos.py` 的 `DEFAULT_SERVOS` 和 `config/servos.json` 中，后续改控制器名称时也应同步更新。

## 修正内容

原 XML 包含 41 个物理 body 元素、40 个 hinge 元素、40 个 actuator 元素。
同一物理连杆沿不同闭环路径重复导出；重复件的网格在初始世界坐标下基本完全重合。

- 保留 25 个实际刚体（不包含 world）；移除 16 个重复 body 元素。
- 保留 24 个唯一树形 hinge 关节；只给 4 个外展关节和 4 个舵盘关节配位置执行器（共 8 个），其余 16 个为被动关节。
- 恢复 8 处闭合旋转连接，每处用两个 connect equality 约束表示，共 16 个 connect。
- 保留原装配的初始网格位姿，不修改 STL 外形。
- 用网格体积及几何恢复近似质量和惯性，替代全部 `1e-9` 占位值。
- 金属螺丝使用 7850 kg/m³；其余网格使用 1250 kg/m³ 作为预览近似。两个非封闭螺丝网格的惯性采用凸包近似。
- 关闭重力和接触，使用隐式积分。这是机构/控制预览，不能用于验证真实重量、驱动力、接触或触觉反馈。
- 配置文件保留原 Assembly 链接。重新运行 onshape-to-robot 会重写手工修正的 XML，因此建议先在 CAD 中完成下述命名修正。

## Onshape 中的对应闭合 Mate

原旋转类型保留，只更改配合名称：

| 当前名称 | 闭合约束名称 |
| --- | --- |
| dof_j4 | closing_j4 |
| dof_j8 | closing_j8 |
| dof_j11 | closing_j11 |
| dof_j16 | closing_j16 |
| dof_j19 | closing_j19 |
| dof_j23 | closing_j23 |
| dof_j27 | closing_j27 |
| dof_j32 | closing_j32 |

其余配合仍使用 dof_，固定配合和 Group 保留。上述选择基于当前 XML 的重复连杆、初始网格位姿及原铰链轴恢复。

闭合连接作为 equality，而非独立 hinge/actuator，因此没有对应的独立 Control 滑块；机构实物连接仍然保留。保留下来的控制器包括：

index_mcp_abduction, index_motor,
middle_mcp_abduction, middle_motor,
little_mcp_abduction, little_motor,
ring_mcp_abduction, ring_motor。

各指被动关节：食指 j3, j5, j6, j7；中指 j12, j13, j14, j15；小指 j20, j21, j22, j24；无名指 j28, j29, j30, j31。

## 验证

使用 MuJoCo 3.15.0：模型编译通过；10 秒零输入运行保持初始装配；24 个控制器分别进行 0.3 rad 目标输入测试，没有数值警告（此验证在改为 8 个执行器之前完成；改后仅驱动舵盘关节时，被动连杆跟随的 connect 残差低于 0.05 mm）。基于近似参数的最大 connect 残差低于 0.2 mm。具体结果见 validation.json 和 validation_follow.json。

这组验证不代表任意滑块组合或完整角度范围都已验证；真实动力学仍需材料、质量、碰撞及实际主动/被动关节配置。

## 装配预览

![手套装配预览](preview.png)

## 食指力反馈 demo（实验）

`haptic_demo.py` 默认仅预览，`--hardware` 只连接 index（当前 ID 2）。原 real2sim / sim2real
仍使用 `servo_bridge.py`。绿色球是食指末端连杆上的简化接触代理，橙色方块是固定物体；
蓝色连杆等其他网格不参与碰撞，其他三指保持参考姿态。

```bash
# 纯预览：按空格开启电流计算，拖动 index_motor
/home/jianai/miniconda3/envs/myglove/bin/python /home/jianai/projects/modular_teleoperation_glove/simulation/glove/haptic_demo.py

# 实物：关闭 Wizard 等串口使用者，启动后用手活动食指
/home/jianai/miniconda3/envs/myglove/bin/python /home/jianai/projects/modular_teleoperation_glove/simulation/glove/haptic_demo.py --hardware
```

启动时 Torque OFF。点击窗口后按空格启用反馈，再次按空格释放；Q / Esc、关闭窗口或 Ctrl+C
退出。只有实物模式会写电机；实物模式的 sidebar 不控制电机。可在水平附近或其伸展侧启用，
但必须在机械标定范围内，不能已经明显弯曲进入接触区。

### 固定几何边界

旧版本仍把完整位置目标交给软接触求解器，造成碰撞球穿透约 10 mm。现在在连接实物之前，
求解四个被动关节的闭链运动学，沿弯曲路径扫描球与方块的首次表面相交，并细化接触边界。
运行时显示姿态始终限制在该边界以内，保留完整实物输入用于电流计算；即使输入突然跳到
最大值，也不会跳过碰撞。回撤后模型立即跟随。

这是单指、固定物体的几何虚拟墙：显示采用闭链运动学解，不再由动态位置伺服推动模型穿过软接触。
不是 MuJoCo 接触力的精确回传，也不表示已完成真实手指力标定。物体移动、多指或任意碰撞面需另行扩展。

当前默认几何与标定下，接触边界约为 slider 0.3823 rad / 编码器 1611；显示停在这个姿态，
`input` 与 sidebar 仍可以继续变化。日志另显示 `sim`、`boundary`、`penetration`、目标/实测电流和 raw。
这里 penetration 是电机输入超出虚拟角度边界的量，不是显示球穿入方块的距离。

### 电流与保护

实物暂时采用 Current 模式 0：

- 未压过边界或反馈关闭：0 mA。
- 默认刚度 **500 mA/rad**，默认电流幅值上限 **70 mA**。
- 当前食指的阻力方向为负电流；按 `-gear * direction` 换算。
- 接触后约 0.14 rad（8° 电机轴角度、约 91 个编码器刻度）达到 70 mA，约在 raw=1702，
  早于机械上限 1802。达到上限也不保证实物完全无法继续移动。
- 电流增加按实际时间限速 **200 mA/s**，减小或释放立即执行；时间间隔最多按 50 ms 计。

可用 `--max-current-ma 20` 降低上限，或 `--stiffness 120` 恢复较软的弹簧。旧刚度下当前行程内
最多约 35 mA，所以仅把上限设为 70 并不会获得 70 mA。`--wall-fraction 0.65`（范围 0.3–0.9）
控制方块沿自由弯曲路径的位置，改变后启动时重新求解边界。

启动位置越界也可打开窗口，反馈保持 OFF，等待手动回到范围内。运行中位置超出实物标定范围时保持 Torque ON、
立即把反馈电流降为零并保留窗口；回到有效接触区后按限速自动恢复反馈，不再要求重新按空格。
范围外不会继续施加阻力，机械标定范围没有放宽。仿真水平显示边界以外但仍在实物标定
范围内的伸展，保持零电流和水平显示，不触发退出。硬件错误、通信错误或反馈循环超过 150 ms 仍中止。
另设 200 ms Bus Watchdog；它监控所有通信包，不能代替应用检查或断电措施。

正常退出、Python 异常、Ctrl+C 时恢复原模式、电流上限/目标电流、PID、速度增益、前馈、PWM 和运动参数，
最终保持 Torque OFF。不保存或重放旧 Goal Position（116）；随后用位置控制时必须先初始化新位置目标。
现有 `servo_bridge.py` 驱动已在启用 Torque 前设置当前位置保持目标。
恢复某项失败会继续尝试后续设置，并报告期望值、读回值；Torque OFF 无法确认时不继续写 EEPROM。
强制杀进程、断电或断开串口时不能保证参数恢复。启动打印的 `/tmp/glove-haptic-crash-<进程号>.log`
记录未处理 Python 异常与原生崩溃堆栈。

离线测试覆盖几何不穿透、闭链约束、慢速/突跳/随机输入、回撤释放、时间限速、越界零电流、回到范围后自动恢复与参数恢复。
这不等于实物力度验证；应在远离物理限位的位置确认阻力方向和感受。

```bash
/home/jianai/miniconda3/envs/myglove/bin/python -B -m pytest -q -p no:cacheprovider tests/test_haptic_demo.py
```

寄存器行为参考 [ROBOTIS XL330 手册](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/)，
模型接口参考 [MuJoCo 文档](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。
