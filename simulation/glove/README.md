# 手套闭环机构：MuJoCo 控制预览修正版

## 启动

在仓库根目录中，激活已经安装 `mujoco` 和 `numpy` 的环境（例如项目的 `myglove` 环境）：

```bash
conda activate myglove
python3 simulation/glove/view_glove.py
```

- 启动后直接运行，初始视角自动居中并拉近；空格切换暂停/运行。
- Backspace 复位；Shift+Tab 显示右侧面板；展开 Control。
- 使用 Control 控制目标角度，单位为弧度。Joint 面板直接改状态，可能暂时破坏闭环，不建议用于检查联动。
- 每套机构有一个MCP 外展／内收滑块（名称见下表）和多个连杆滑块。
- 在同一套机构中，最近操作的连杆滑块作为驱动输入；其他连杆的控制目标自动跟随其当前角度，避免多个位置控制器互相阻碍闭环运动。
- 目标角度不一定都能实现，仍受闭环几何和机构奇异位置约束。

也可以使用标准查看器查看全部 24 个位置控制器：

```bash
python -m mujoco.viewer --mjcf=simulation/glove/scene.xml
```

标准查看器同时保持所有控制目标；它不会自动执行上述跟随逻辑。

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
控制脚本的 `FINGER_CONTROLS` 显式列出每根手指的控制器，后续改连杆名称时也应同步更新。

## 修正内容

原 XML 包含 41 个物理 body 元素、40 个 hinge 元素、40 个 actuator 元素。
同一物理连杆沿不同闭环路径重复导出；重复件的网格在初始世界坐标下基本完全重合。

- 保留 25 个实际刚体（不包含 world）；移除 16 个重复 body 元素。
- 保留 24 个唯一树形 hinge 关节和 24 个位置控制滑块。
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

index_mcp_abduction, j2, j3, j5, j6, j7,
middle_mcp_abduction, j10, j12, j13, j14, j15,
little_mcp_abduction, j18, j20, j21, j22, j24,
ring_mcp_abduction, j26, j28, j29, j30, j31。

## 验证

使用 MuJoCo 3.15.0：模型编译通过；10 秒零输入运行保持初始装配；24 个控制器分别进行 0.3 rad 目标输入测试，没有数值警告。基于近似参数的最大 connect 残差低于 0.2 mm。具体结果见 validation.json 和 validation_follow.json。

这组验证不代表任意滑块组合或完整角度范围都已验证；真实动力学仍需材料、质量、碰撞及实际主动/被动关节配置。

## 装配预览

![手套装配预览](preview.png)
