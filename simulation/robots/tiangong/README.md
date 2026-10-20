# 天宫 TG2 / InspireHand 模型

从本机 `/home/bobo/DOGlove/assets/tg2_inspirehand/` 导入
（不是 `/home/bobo/DOGlove_origin`）。这些文件用于 TeleGlove 后续机器人端仿真。

## 文件

| 文件 | 用途 |
| --- | --- |
| `tg2_with_hands.urdf` | 含腿部及手部定义的 TG2 模型 |
| `tg2_with_hands_no_legs.urdf` | 不含腿部关节的版本 |
| `tg2_inspirehand.xml` | 本地项目已有的 MuJoCo 目标模型，19 个位置变量、19 个执行器 |
| `meshes/` 和本目录下的 STL | 模型实际引用的网格；保留原始相对路径布局 |
| `LICENSE.DOGlove` | 来源 DOGlove 仓库根目录附带的 MIT 许可证 |
| `provenance.json` | 来源版本与导入文件的 SHA-256 |

三个模型文件及网格均按原样复制，没有修改关节、惯性、碰撞或控制参数。
URDF 和 MJCF 是不同的模型表示，不能假定关节集合、碰撞行为及控制方式完全相同。

## 在 MuJoCo 中打开

在仓库根目录运行：

```bash
conda activate myglove
python -m mujoco.viewer --mjcf=simulation/robots/tiangong/tg2_inspirehand.xml
```

Python 中加载：

```python
from pathlib import Path
import mujoco

model_path = Path("simulation/robots/tiangong/tg2_inspirehand.xml").resolve()
model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)
```

两个 URDF 也可以通过 `MjModel.from_xml_path` 导入 MuJoCo，但 URDF
没有 actuator 定义，导入后 `nu=0`，不能直接通过 `data.ctrl` 驱动。
MuJoCo 导入时会合并固定关节，因此其 body 数不等于 URDF link 数。

这次只导入机器人模型及预览入口；尚未加入 TeleGlove 手套到机器人手的
关节映射、标定、同步控制或真实硬件连接。

## 来源

来源仓库：<https://github.com/zjn6060/DOGlove>，本地分支 `dev_chi`，
导入时 HEAD：`d0cf30d11335c3571e8b9877354c05854531f7b1`。
对应目录：`assets/tg2_inspirehand/`。

保留了来源根目录中的许可证。模型目录内没有发现独立的许可证或模型作者说明；
这里记录的是本地文件来源，不代表重新认定模型作者或授予额外权利。

## 验证

使用 MuJoCo 3.15.0：

- 全部 XML/URDF 可解析，所有显式引用的网格路径存在。
- 三个模型均可通过 `MjModel.from_xml_path` 编译加载。
- `tg2_inspirehand.xml`：`nq=19`、`nv=19`、`nu=19`。
- 两个 URDF：`nq=19`、`nv=19`、`nu=0`。

这些检查验证文件完整性和加载能力，不验证真实机器人动力学、完整关节范围
或 sim2sim 控制效果。
