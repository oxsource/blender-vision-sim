# blender-vision-sim

Blender 视觉算法仿真插件集合：用 Blender/Cycles 生成**与真实相机、真实传感器一致**的合成数据，用于感知算法的开发、标定与回归验证。

仓库以「插件 + 纯 Python 核心」的方式组织：算法数学与文件格式放在不依赖 `bpy` 的 `core/`，Blender 集成（属性、着色器、算子、UI）放在 `bl/`，因此核心逻辑可以在 Blender 之外单测，也能被导出工具、标定工具复用。

## 插件列表

| 插件 | 状态 | 说明 |
| --- | --- | --- |
| [`opencv_camera`](addons/opencv_camera) | 0.18.0 可用 | Cycles 自定义相机，支持 OpenCV 内参（fx/fy/cx/cy）与畸变（**fisheye / Brown-Conrady / rational**），可导入导出标定文件、设置相机外参（Location + Euler）、渲染自检；**算法场景**（`Camera Scene`、`AVM Scene`、`Drive Scene`）统一在 `bl/scenes/` 注册，见 [`docs/avm-scene.md`](docs/avm-scene.md) |
| `camera_rig` | 规划中 | 多相机刚体（外参）、同步渲染、标定数据集导出 |
| `sensor_sim` | 规划中 | IMU / GNSS / LiDAR 轨迹与噪声仿真 |
| `dataset_export` | 规划中 | 渲染 + 真值导出（位姿/内参/深度/分割），KITTI / COLMAP / EuRoC 布局 |
| `calibration_tools` | 规划中 | 场景内标定板、角点/ArUco 反标定回环验证 |

路线图见 [`docs/roadmap.md`](docs/roadmap.md)。

### 入口与图标

菜单图标是**单色线条风格的一套图标**（`scripts/make_icon.py` 纯 Python 生成，无 Pillow 依赖），
每个菜单项各有其形，16 px 下也能分辨：

| 图标 | 用于 | 形状 |
| --- | --- | --- |
| `visionsim` | `Add ▸ VisionSim` | 眼睛 + 实心瞳孔 |
| `camera` | `VisionSim ▸ Camera` | 相机机身 + 镜头 |
| `fisheye` | Fisheye 条目 | 圆 + 外弓十字（广角） |
| `brown_conrady` | Brown-Conrady 条目 | 方 + 外弓十字 |
| `rational` | Rational 条目 | 方 + 外弓十字 + 中心点（高阶项） |
| `pinhole` | Pinhole 条目 | 方 + 笔直十字 |
| `camera_scene` | Camera Scene | 立方体轮廓 |
| `avm_scene` | AVM Scene | 俯视场地 + 四角实心标定块 + 车体轮廓 |
| `drive_scene` | Drive Scene | 俯视停车场车道 + 车位线 + 车辆轮廓 |

这些都是**原创标识**（不是 OpenCV 商标本身），想改图案改脚本里的形状定义后重跑：

```bash
python3 scripts/make_icon.py     # 重新生成 addons/opencv_camera/icons/visionsim.png
```

图标通过 `bpy.utils.previews` 载入并用 `icon_value` 使用；无 UI 的会话（后台/测试）拿不到有效
icon id，此时自动回退到 Blender 内置图标，不会出现空白按钮。

## 目录结构

```text
blender-vision-sim/
├── addons/<addon_id>/           # 每个插件一个 4.2+ Extension 包
│   ├── blender_manifest.toml    # Extensions 规范清单（唯一的插件元数据来源）
│   ├── __init__.py              # 只做 register / unregister 编排
│   ├── core/                    # 纯 Python：模型、变换、IO（禁止 import bpy）
│   │   └── scenes/              # 每个算法场景的几何/模型/IO（avm_layout / avm_coverage / drive_lot / drive_path）
│   ├── bl/                      # Blender 集成：properties / shader / apply / selftest / ui / operators
│   │   └── scenes/              # 场景框架 + 每个场景一个模块/包
│   │       ├── base.py / view.py / debounce.py  # SceneDefinition、默认 3/4 视角、通用去抖
│   │       ├── camera_scene.py          # 原 bl/scene_builder.py
│   │       ├── avm_scene/               # AVM Scene（properties/builder/controller/io/coverage/corners/falcon/operators/ui）
│   │       └── drive_scene/             # Drive Scene（properties/builder/controller/recording/operators/ui）
│   ├── presets/avm_scene/default.json   # AVM 内置默认参数（离线反算产物）
│   ├── models/unlit_round_bowls.glb     # AVM 真实碗形地面（app 投影面，AVM_Ground 默认用它）
│   └── shaders/*.osl            # 随插件分发的 OSL 源文件（权威副本）
├── docs/                        # 架构、相机模型、路线图
├── .github/workflows/ci.yml     # 打 v* tag 自动测试+出包+发 Release（不依赖 Blender）
├── scripts/                     # dev_install / dev_uninstall / package(.sh|.py) /
│                                # version.sh（npm version 风格）/ run_tests / make_icon
└── tests/                       # 核心单测 + 无头 Blender 集成测试 + 发布工具测试
```

## 快速开始

```bash
# 1. 链接到 Blender 的 User Default 扩展仓库（默认 4.5，可用 -v 指定版本）
scripts/dev_install.sh

# 2. 启动 Blender → Preferences ▸ Add-ons，启用 "VisionSim: OpenCV Camera"
#    或使用 Python：
#    bpy.ops.preferences.addon_enable(module='bl_ext.user_default.opencv_camera')

# 3. 打包成可分发/可安装的 zip
scripts/package.sh

# 4. 跑全部测试（核心 + 无头 Blender + 发布工具）
scripts/run_tests.sh

# 5. 打包（无需 Blender；--blender 走官方 builder 做校验）
scripts/package.sh              # -> dist/opencv_camera-<version>.zip (+ .sha256)
scripts/package.sh --blender    # 可选：用官方 builder 再校验一次（需要本机 Blender）-> dist-official/

# 6. 发版：bump 版本 + 提交 + 打 tag（npm version 风格）
scripts/version.sh patch --push   # 也可以 minor / major / 1.2.3
```

> 构建/发版只需 **Python 3.11+**，不依赖 Blender；只有本地跑集成测试（`scripts/run_tests.sh` 的第三段）才需要 Blender。

### 发版流程

```text
scripts/version.sh <patch|minor|major|X.Y.Z> [--push] [--dry-run]
        │  读/写 addons/opencv_camera/blender_manifest.toml 的 version（唯一真源）
        │  commit: chore(release): vX.Y.Z
        └─ tag:    vX.Y.Z（带注释）
                │
                └─ push tag ──► GitHub Actions（单一工作流 .github/workflows/ci.yml）
                                 1. 校验 tag 与 manifest 版本一致
                                 2. 快速检查（编译 + 核心单测 + 发布工具测试，纯 Python 数秒）
                                 3. 打包（scripts/package.py，可复现 zip + sha256）
                                 4. 创建/更新 GitHub Release，附带本版本的 zip 与 sha256
                                 （CI 不下载 Blender；无头集成测试只在本地跑）
```

- `--dry-run` 只打印将要发生的版本变化；`--show` 打印当前版本；工作区不干净时拒绝执行（和 npm 一致）。
- **只有一个工作流文件** `.github/workflows/ci.yml`，只有一个 job `release`（**仅 `v*` tag 触发**，普通 commit / 分支推送完全不跑）：校验 tag ↔ manifest → 快速检查（编译/核心单测/发布工具测试）→ `package.py` 出包 → 发布 Release（zip + sha256）+ artifact。**CI 不需要 Blender**（打包是纯 Python）。
- 手动运行（`workflow_dispatch`）：`tag` 留空 = 只打包不发布；填 `tag=vX.Y.Z` = 补发该 tag 的 Release。
- **重跑已有 tag 不会触发**：tag 已存在时 `git push origin vX.Y.Z` 是空操作。要重新触发：
  `git push origin :refs/tags/vX.Y.Z && git push origin vX.Y.Z`，或 `gh workflow run ci.yml -f tag=vX.Y.Z`（网页 Actions ▸ release ▸ Run workflow 同样可以）。
- tag 必须推到 **origin（GitHub）**才会触发 Actions；推到 `gitlab` 不会（`scripts/version.sh --push` 推的是当前分支的远端，main → origin）。
- 若想恢复"提交/PR 也先跑检查"，在 `on.push` 下加 `branches: ["**"]` 并加回 `pull_request:` 即可（release 仍然只对 tag 生效）。

### 目录结构

Blender 中使用：

1. 选中相机对象 ▸ `Object Data Properties ▸ CV Intrinsics`；
2. 选畸变模型（默认 `Fisheye (equidistant)`）、填 fx/fy 与系数（或 `Import Calibration` 导入标定文件，
   仓库自带参考标定 `addons/opencv_camera/presets/default_camera.yaml`）；
3. 点 `Apply` —— 插件会写入对应 OSL 着色器、编译、把参数送进 Cycles；
4. `Add ▸ VisionSim  Camera Scene` —— 生成棋盘方块 + 棋盘地面 + 灯光并设好 Cycles，直接 F12 看畸变效果
   （场景里没有相机时会自动先建一台鱼眼相机）。地面是水平面（和 AVM Scene 同一套约定），
   方块落在相机视线的着地点上；视口会自动摆到标准 **3/4 视角**（方位 135°、仰角 30°），
   转飞了用面板里的 `Frame View` 复位；
5. F12 渲染（**输出尺寸/格式用 Blender 自带的 `Render ▸ Output`**，插件会按当前渲染分辨率自动换算内参）。

> 自定义相机只在 **Cycles** 下生效，且只能使用 **CPU 或 OptiX** 后端（macOS 无 OptiX ⇒ 只能 CPU）。
> 默认参数取真实 AVM 前相机（1280×960 鱼眼），直接可用；换成自己的标定即可。

### 三种使用方式

| 方式 | 操作 |
| --- | --- |
| **新建相机**（推荐） | `Add ▸ VisionSim ▸ Camera ▸ Fisheye / Brown-Conrady / Rational / Pinhole`，创建出来即为 Custom 相机、已挂载着色器与参数，可选 `At 3D Cursor` / `Add Rig Empty`（父级空物体，便于多相机/外参）。同一菜单下还有 `Camera Scene`（棋盘方块/地面/灯光） |
| **改造现有相机** | 选中相机 ▸ `CV Intrinsics ▸ Apply` |
| **批量/脚本** | `bpy.ops.opencv_cam.add_camera(model="fisheye", preset="default_camera", use_rig=True)` |

> Blender 不允许插件扩展 `Camera.type` 枚举（该枚举定义在 C 侧 RNA），所以"添加自定义相机"以
> `Add ▸ Camera` 菜单算子的形式提供，这是 Blender 插件生态里的标准做法；相机数据块本身仍是
> `Lens Type = Custom` + 我们的 OSL 着色器。

### 输出图像尺寸

插件**没有**单独的输出尺寸面板：**渲染分辨率、格式、保存路径都用 Blender 自带的
`Render ▸ Output`**。内参属于**标定分辨率**（`intrinsics.image_width/height`），渲染时按当前
`scene.render.resolution_*` 换算：

- 宽高比一致 → **等比缩放**（FOV 不变）；
- 宽高比不一致 → **保持像素尺度的中心裁剪**，避免把图像拉伸。

`Scale Intrinsics to Render`（默认开）控制是否换算；关掉则内参按标定值原样使用。
预览也按当前渲染分辨率的宽高比走。

### 参数面板与预览

- **Live Apply**（默认开）：面板上改任意内参/畸变/外参即时写入 Cycles，不需要再点 Apply；
  外参面板改 Rotation（欧拉角）会即时转动相机物体；`Model` 切换会自动换着色器并重编译。
- **Preview**：按**渲染分辨率的宽高比**快速渲染并显示在 Blender 的 Image Editor 里（与 F12 同一位置），
  **渲染设置用完即还原**，Preview 不会改动你的场景设置；其中
  `Preview Size`（256/384/512/720）是**预览图的分辨率长边**（短边按渲染宽高比推导，面板会显示实际
  预览尺寸），`Preview Samples`/`Denoise Preview` 控制预览的噪声与速度；F12 的尺寸用
  Blender 的 `Render ▸ Output`、采样数用场景自己的设置。`Preview On Change` 打开后停止拖动约 0.6 s 自动重渲染。
- 面板同时显示：当前生效的着色器/字节码长度、分辨率与宽高比提示。
- 预览图示例：`docs/images/preview_example.png`（384×288，16 采样）。
- 注意：Blender 4.x 已移除 `UILayout.template_preview` / `Image.preview`，插件无法在面板里内嵌图片，
  因此预览走 Image Editor；3D 视口是否支持自定义相机**尚未实测确认**（视口渲染需要交互式刷新），
  所以暂不作为预览方案（见 `docs/roadmap.md` 待办）。

### 面板结构

所有模块都是 camera 数据属性里的**顶层面板**（与 Blender 自带的 Lens 等平级，不嵌套），统一用
`CV ` 前缀，并用**负的 `bl_order` 排在最前面**：

```
Object Data Properties
├── CV Intrinsics   模型、fx fy cx cy、畸变（模型 + 系数 + 迭代 + Discard Invalid Rays）、
│                   标定分辨率 + Scale Intrinsics to Render、生效值只读框，
│                   然后才是 [Apply] [Preview] [Live Apply] [Recompile] 与状态框
├── CV Extrinsics   Location + Rotation（XYZ 欧拉，度）；改角度即时转动相机物体
├── CV Presets      标定文件 [Import] / [Export]、Load Preset、Reset Defaults（默认折叠）
├── CV Preview      预览尺寸/采样/去噪/自动预览、Preview 按钮（默认折叠）
── Lens / Camera / Depth of Field ...      （Blender 自带面板排在后面）
```

- 参数在前、动作在后：`Apply` / `Preview` / `Live Apply` / `Recompile` 都在 `CV Intrinsics` 的参数之后。
- `enable_distortion` 这类开关在**本插件面板里是真复选框**（`distortion.enabled`、`discard_invalid_rays`）。
- Cycles 依据 OSL 形参自动生成的那串裸参数列表默认**被隐藏**（值以数字显示、且 Cycles 对自定义相机
  参数不支持复选画法），需要时打开 `Show Cycles Raw Parameters` 即可恢复显示。
- `Discard Invalid Rays`：反畸变迭代发散时（极强畸变、或像素远超标定域）**丢弃该射线**（画面变黑），
  而不是保留一个勉强算出来的方向；默认关闭（保留 best effort），需要严格边缘行为时打开。

### 可视化验证

`Add ▸ VisionSim ▸ Camera Scene` 生成的场景（同一套 K，仅切换畸变开关）：

| 鱼眼（`fisheye`, k1..k4） | 理想针孔（`enable_distortion = 0`） |
| --- | --- |
| ![鱼眼](docs/images/fisheye_on.png) | ![针孔](docs/images/pinhole_off.png) |

地面网格的弯曲程度就是鱼眼畸变的直接体现；两图均可用 `docs/camera-model.md` 中的公式复算。

### AVM Scene（环视标定场景）

`Add ▸ VisionSim ▸ AVM Scene` 一键搭出「真实场景的仿真版」：1 个地面、1 个立方体车模、
4 个规格相同的实心标定块（放在车四周四对角）与 4 台 OpenCV 鱼眼相机，
默认参数来自 `filament_avm` 的 minibus 配置（离线反算，见 `scripts/solve_avm_defaults.py`）。

```text
输入：各部件的位置姿态（场地尺寸 / 车辆 / 相机安装位姿）+ 相机内参
输出：4 路相机预览图像 + 各元素大小/位置/姿态与相机内参的导出
```

- **面板**：`AVM Scene` 在 Scene Properties（全部参数 + 覆盖评估 + 导入导出）；
  3D 视口 N 侧栏 `VisionSim ▸ AVM Scene` 只放**布局**（尺寸/位姿/图层）。
  相机内参不在这里，继续用既有的 `CV Intrinsics` / `CV Presets`。
- **真实地面**：`AVM_Ground` 默认不是平面，而是内置的 **Falcon 真实碗形 mesh**
  （`models/unlit_round_bowls.glb`：中心平坦、圆角方形围壁，轴向半径 15 m、碗边离地 5 m，
  正是 app 的投影面），让仿真的 4 路相机看到与真机一致的边界/遮挡；
  `Bowl Radius` / `Rim Height` 可分别调**半径**与**碗边高度**（底面仍平放在 z=0）；
  `Real Ground Mesh` 可关掉切回平面，`Model` 可换自定义 `.glb/.gltf/.fbx/.obj`
  （缺失/导入失败自动回退平面）；N 面板的 `[Export Bowl]` 把当前碗形 mesh 单独导出成 `.glb`。
- **默认视角**：建场景时视口自动摆到标准 **3/4 视角**（方位 135°、仰角 30°，看车头 + 右侧 +
  四块 + 四台相机）并自动取景，两个面板里的 `Frame View` 可随时复位（拖滑杆重建时不抢镜头）。
- **本场景不做位姿解算**：只摆放、渲染、导出；PnP 只在离线脚本里跑一次用来产出默认预设。
- **覆盖评估**：`[Analyze Coverage]` 给出每台相机的贴地覆盖曲线、并集/重叠/盲区、
  场地覆盖率，以及「哪个标定块被哪几台相机看到」的可见性矩阵。
- **角点检测**：每台相机的条目有 `[Detect Corners]`（单视图，numpy 亚像素，无 cv2 依赖），
  生成标注图（绿点=角点+序号，红点=投影）可用 `[Show Corners]` 在图像编辑器查看；
  输入不变时自动命中缓存不重渲染。
- **参数导出 / 导入**：`[Export Parameters]` / `[Import Parameters]` 把整套场景参数写成 JSON
  再还原（完整 `avm_scene` 或精简 `plane_scene` 两种格式）。
- **Export Falcon**：`[Export Falcon]`（N 面板 / Scene 面板）导出最终标定配置——把 4 张原始图
  （`front/back/left/right.png`）、4 张角点标注图（`*_annotated.png`）和 `vehicle_avm.json`
  打包成一个 **zip**，字段/顺序对齐 `mediapipe_avm_calib` 的 `JsonGenerator`（数组尽量一行）；
  已检测过的相机会复用缓存渲染图，不重复渲染。每台相机的 `[X]` 可单独清除检测与缓存。
- 完整设计见 [`docs/avm-scene.md`](docs/avm-scene.md)（含与 HTML 工具 / `mediapipe_avm_calib`
  的字段对齐、场景目录结构与新增场景的方法）。

### Drive Scene（停车场行驶素材）

`Add ▸ VisionSim ▸ Drive Scene` 搭出「室内停车场 + 车辆按计划行驶」，用来给**透明底盘**算法造素材：
一键把整段行驶**逐帧渲染**出来，并同步导出每帧的车速与真值位姿。

```text
输入：停车场地参数 + 车辆尺寸 + 行驶参数（里程 / 速度曲线 / 帧率）
输出：frame_%04d.png 序列 + clip.mp4 + frames.csv（逐帧时间/里程/车速/车体与相机世界位姿）
      + clip.json（整段规格，含相机 K/D/安装位姿）；[Export Clip…] 打包成一个 zip
```

- **停车场**：地坪可切 **水泥 / 柏油 / 环氧**（`ground_texture`，程序化低对比度斑驳——宽污渍 + 细颗粒
  两种尺度叠加，看起来就是普通地坪，不是纯色也不是单一自相似噪声：透明底盘靠纹理对齐验证，需要一点
  非重复信号，但不能花）、
  车道边线 /
  车位分隔线 / **中线虚线**（车正下方那条线就是算法要重建出来的）、黄色禁停框、两侧立柱与围墙；
  照明是**一整片与地块同大的柔和顶光**（不是沿车道一排小灯——那会在地面烧出一圈圈光斑，
  同一车位在不同位置的亮度对不上，逐帧就没法比），`show_bays` 关标线、`ground_texture` 还有
  `checker` / `plain` 两个对照项（最强纹理 / 无纹理）。
- **车位与停放车**：每侧车位都刷**编号**（`A01…` / `B01…`，平铺在车道上正对该车位，行驶时相机
  一直看得见——对着编号就能核对那块区域重建得对不对）；`parked_cars` 按排停放若干辆车
  （车头朝墙、车型与车漆循环、紧邻立柱的车位留空），`show_parked` 可整组隐藏。
- **相机**：`DRIVE_Cam_Front` 就是插件的 **OpenCV Camera 组件**（fisheye），内参/安装位姿取内置
  minibus 标定预设，画面与真机可比；内参照旧在 `CV Intrinsics` / `CV Presets` 里改。
- **与 AVM Scene 一致**：车漆与 AVM Scene 共用同一份 minibus 调色板（`avm_layout.MINIBUS_MATERIALS`），
  前摄的 **K/D/输出与安装位姿**也与 AVM 默认逐项相等（`test_drive_matches_avm_defaults` 断言），
  两场景的实验参数不会漂移。
- **运动**：`DRIVE_Vehicle` 空物体承载世界位姿（车与相机挂在它下面，相机永远保持车体系安装位姿），
  逐帧关键帧由纯 Python 的运动模型给出：`constant` 匀速或 `trapezoid` 加速-巡航-刹停
  （里程不够跑满会退化成三角形，仍停在精确里程上）。
- **录制 / 导出**：`[Render Clip…]` 逐帧渲染前摄，写出 PNG 序列 + `frames.csv` + `clip.json`；
  `[Export Clip…]` 在同一份内容上加 **H.264 mp4**（Blender 内置 FFmpeg，不重渲染 3D）并**打包成一个 zip**。
  `frames.csv` 的 `cam_*` 列是每帧相机的世界位姿（车体位姿 × 固定安装位姿，纯 Python 算出）。
  分辨率用 Blender 自带的 `Render ▸ Output`，渲染设置会自动还原，场景里其它灯光录制时临时屏蔽。
- 完整设计见 [`docs/drive-scene.md`](docs/drive-scene.md)。

## 开发约定

- 插件采用 Blender 4.2+ **Extension** 规范（`blender_manifest.toml`），不使用 `bl_info`；
- 插件内部**只用相对导入**（扩展模式下包名是 `bl_ext.<repo>.<id>`，绝对导入会失效）；
- `core/` 不得 `import bpy`；`bl/` 才可以访问 `bpy` / `mathutils`；
- **版本适配是全局机制**：同一套代码要跑 Blender 4.5 LTS 与 5.x，所有跨版本 API 差异只写在
  `bl/compat.py`（**特性探测优先**，禁止 `bpy.app.version` 分支、禁止写死改名后的枚举/算子/节点名），
  由 `tests/test_version_policy.py` 静态强制；新增差异只动 `compat.py` 与它的用例（见 §2.3）；
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