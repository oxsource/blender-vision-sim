# blender-vision-sim

Blender 视觉算法仿真插件集合：用 Blender/Cycles 生成**与真实相机、真实传感器一致**的合成数据，用于感知算法的开发、标定与回归验证。

仓库以「插件 + 纯 Python 核心」的方式组织：算法数学与文件格式放在不依赖 `bpy` 的 `core/`，Blender 集成（属性、着色器、算子、UI）放在 `bl/`，因此核心逻辑可以在 Blender 之外单测，也能被导出工具、标定工具复用。

## 插件列表

| 插件 | 状态 | 说明 |
| --- | --- | --- |
| [`opencv_camera`](addons/opencv_camera) | 0.1.0 可用 | Cycles 自定义相机，支持 OpenCV 内参（fx/fy/cx/cy）与畸变（**fisheye / Brown-Conrady / rational**），可导入导出标定文件、设置 OpenCV 外参、一键生成验证场景、渲染自检 |
| `camera_rig` | 规划中 | 多相机刚体（外参）、同步渲染、标定数据集导出 |
| `sensor_sim` | 规划中 | IMU / GNSS / LiDAR 轨迹与噪声仿真 |
| `dataset_export` | 规划中 | 渲染 + 真值导出（位姿/内参/深度/分割），KITTI / COLMAP / EuRoC 布局 |
| `calibration_tools` | 规划中 | 场景内标定板、角点/ArUco 反标定回环验证 |

路线图见 [`docs/roadmap.md`](docs/roadmap.md)。

## 目录结构

```text
blender-vision-sim/
├── addons/<addon_id>/           # 每个插件一个 4.2+ Extension 包
│   ├── blender_manifest.toml    # Extensions 规范清单（唯一的插件元数据来源）
│   ├── __init__.py              # 只做 register / unregister 编排
│   ├── core/                    # 纯 Python：模型、变换、IO（禁止 import bpy）
│   ├── bl/                      # Blender 集成：properties / shader / apply / selftest / ui / operators
│   └── shaders/*.osl            # 随插件分发的 OSL 源文件（权威副本）
├── docs/                        # 架构、相机模型、路线图
├── scripts/                     # dev_install / dev_uninstall / package / run_tests
└── tests/                       # 纯 Python 单测 + 无头 Blender 集成测试
```

## 快速开始

```bash
# 1. 链接到 Blender 的 User Default 扩展仓库（默认 4.5，可用 -v 指定版本）
scripts/dev_install.sh

# 2. 启动 Blender → Preferences ▸ Add-ons，启用 "OpenCV Camera"
#    或使用 Python：
#    bpy.ops.preferences.addon_enable(module='bl_ext.user_default.opencv_camera')

# 3. 打包成可分发/可安装的 zip
scripts/package.sh

# 4. 跑全部测试
scripts/run_tests.sh
```

Blender 中使用：

1. 选中相机对象 ▸ `Object Data Properties ▸ Lens ▸ OpenCV Camera`；
2. 选畸变模型（默认 `Fisheye (equidistant)`）、填 fx/fy 与系数（或 `Import Calibration` 导入标定文件，
   仓库自带参考标定 `addons/opencv_camera/presets/avm_minibus_front.yaml`）；
3. 点 `Apply to Camera` —— 插件会写入对应 OSL 着色器、编译、把参数送进 Cycles；
4. 点 `Add Test Scene` —— 生成棋盘方块 + 棋盘地面 + 灯光并设好 Cycles，直接 F12 看畸变效果；
5. 点 `Run Self Test` —— 渲染目标并与 OpenCV 模型比对，报出像素误差（参考相机实测 0.03–0.08 px）。

> 自定义相机只在 **Cycles** 下生效，且只能使用 **CPU 或 OptiX** 后端（macOS 无 OptiX ⇒ 只能 CPU）。
> 默认参数取真实 AVM 前相机（1280×960 鱼眼），直接可用；换成自己的标定即可。

### 可视化验证

`Add Test Scene` 生成的场景（同一套 K，仅切换畸变开关）：

| 鱼眼（`fisheye`, k1..k4） | 理想针孔（`enable_distortion = 0`） |
| --- | --- |
| ![鱼眼](docs/images/fisheye_on.png) | ![针孔](docs/images/pinhole_off.png) |

地面网格的弯曲程度就是鱼眼畸变的直接体现；两图均可用 `docs/camera-model.md` 中的公式复算。

## 开发约定

- 插件采用 Blender 4.2+ **Extension** 规范（`blender_manifest.toml`），不使用 `bl_info`；
- 插件内部**只用相对导入**（扩展模式下包名是 `bl_ext.<repo>.<id>`，绝对导入会失效）；
- `core/` 不得 `import bpy`；`bl/` 才可以访问 `bpy` / `mathutils`；
- 插件自有 PropertyGroup 是参数的唯一真源，`camera.cycles_custom[...]` 只作为写入目标；
- 任何写 Cycles 参数的路径都必须校验 `custom_bytecode` 非空（编译失败会**静默沿用旧着色器**）；
- 新增/修改行为都要有测试：核心逻辑进 `tests/test_core.py`，需要渲染的进 `tests/run_blender_tests.py`。

细节见 [`docs/architecture.md`](docs/architecture.md)，相机模型推导与实测数据见 [`docs/camera-model.md`](docs/camera-model.md)。

## 实测结论（Blender 4.5.3 LTS, Cycles CPU）

| 验证 | 结果 |
| --- | --- |
| 零畸变自定义相机 vs 自带透视相机（128×128，50mm/36mm） | 16384/16384 像素完全一致（max\|Δ\| = 0） |
| 主点偏移等价性（shift_x=0.1, shift_y=0.15） | 完全一致（cx=51.2, cy=83.2 px） |
| 多项式畸变目标成像位置 vs OpenCV 前向模型 | 误差 0.018 px |
| 鱼眼（AVM 前相机，θ≈72°） | 误差 0.048 px |
| 鱼眼贴近 90° 边界（θ≈87°，射线用 sin/cos） | 误差 0.026 px |
| 分辨率换算（同比 1920×1080 → 960×540） | fx 按比例缩放，FOV 保持；宽高比不一致时自动改为原始像素尺度的中心裁剪 |

## 许可

GPL-3.0-or-later（与 Blender 插件生态一致）。