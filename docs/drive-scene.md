# Drive Scene 设计说明（室内停车场行驶素材）

> 状态：**P0 + 四路已实施**（直线路径 + 匀速/梯形速度 + **四摄逐帧录制** + 逐帧真值）。
> 用途：为**透明底盘**与 **AVM 渲染通路**提供带真值的仿真素材 —— 四路相机画面 +
> 每帧车速与位姿 + 每台相机的世界位姿 + 车体几何。
> 实现：场景 `addons/opencv_camera/bl/scenes/drive_scene/`，运动模型 `core/scenes/drive_path.py`，
> 相机与车体的单一真源 `core/scenes/avm_cameras.py` / `core/scenes/vehicle.py`。
> **四路改造的设计与逐条证据**见 [`drive-scene-multicam.md`](drive-scene-multicam.md)（已实施）。
> 下游消费契约见 `filament_avm/docs/sim_clip_render_pipeline.md`。

## 1. 目标与契约

| | 内容 |
| --- | --- |
| 输入 | 停车场地参数（车道/车位/立柱/灯光）+ 车辆尺寸 + 行驶参数（里程 / 速度 / 加速度 / 帧率）+ **录哪几路相机** |
| 输出 | 每路一条 `<camera>.mp4`（`front/back/left/right`）+ `frames.csv`（逐帧时间/里程/车速/**车体世界位姿 + 每台相机的世界位姿**）+ `clip.json`（整段规格：`cameras[]` 的 K/D/安装位姿、**车体几何**、mp4 编码处方）+ `frame_%04d_<camera>.png` 序列（`clip_keep_frames`，默认开）；`[Export Clip…]` 打包成一个 zip |
| 相机 | AVM Scene 同款四摄（`front/back/left/right`），插件自带的 **OpenCV Camera 组件**（fisheye），内参/安装位姿取**同一份**内置 minibus 标定预设；**Drive Scene 不持有任何相机参数** |
| 非目标 | 卷帘快门、悬架俯仰侧倾、逐帧位姿回放到渲染器（消费侧预留接口，本场景不实现）|

## 2. 场景内容与取舍

| 元素 | 做法 |
| --- | --- |
| 地面 | **室内停车场**：地坪可切 **concrete / asphalt / epoxy**（程序化**低对比度多尺度斑驳**：宽污渍 + 细颗粒，见 §10）+ 车道/车位标线（真实几何，见 §3） |
| 车位 | 两侧车位**编号**（`A01…` / `B01…`，文字平铺在车道上、正对该车位）与**停好的车**（`parked_cars`/排，车型与颜色循环）|
| 结构 | 两侧立柱（`pillar_count`/侧）、四面围墙（`show_walls`）、一整片柔和顶光（见 §3） |
| 车辆 | 程序化 minibus，**尺寸取自 `core/scenes/vehicle.py`**（与 AVM Scene 同一份数字），几何仍由本场景自带实现；车漆取自共享调色板 |
| 相机 | `DRIVE_Cam_{Front,Back,Left,Right}`，`Add ▸ VisionSim ▸ Drive Scene` 时按预设创建（K/D/输出 + 车体系安装位姿）；`active_camera` 决定视口/F12 用哪台，`cameras[].enable` 决定录哪几台（**默认四路全开**）|
| 运动 | `DRIVE_Vehicle` 空物体承载世界位姿，车与**四台相机**都挂在它下面（相机因此永远保持车体系位姿）|

**相机的单一真源（本场景的核心约束）**：Drive Scene **不负责相机参数**——"有四台、叫什么、安装在哪、内参是多少"全部来自 `core/scenes/avm_cameras.py`（角色/顺序/对象命名）与内置标定预设 `presets/avm_scene/default.json`，经 `bl/camera_factory.configure_from_record()` 写入相机对象。所以**改 AVM 的相机（预设）两个场景同时跟着变**；若在 AVM 面板里手工调过，可用 `[Sync Cameras from AVM Scene]` 把这台 .blend 里**活的** AVM 相机单向抄到 Drive 四台上（非默认路径；可复现的真源仍是预设）。`tests/run_blender_tests.py::test_drive_matches_avm_defaults` 对**四台逐项**断言 K/D/输出/挂载矩阵，`::test_drive_sync_from_avm` 覆盖单向同步。

**与 AVM Scene 的关系**：同一个 OpenCV Camera 组件、同一份 minibus 标定预设、同一份共享调色板（`avm_layout.MINIBUS_MATERIALS`）、同一份车体几何（`core/scenes/vehicle.py`）——相机内外参、车色与车体因此**不可能漂移**（不再靠"两处各写一遍再用测试对齐"）；**其余组件不复用**，Drive Scene 自带地面/立柱/车模实现。两者可同时存在于一个 .blend（对象前缀分别是 `DRIVE_` / `AVM_`，互不干扰）。

**三点本质差别**（也是这个场景存在的理由）：

1. **地面有纹理** —— 透明底盘靠"纹理对齐"验证，纯色地面等于没有验证信号（AVM Scene 的地面是刻意去纹理的）；
2. **车辆在动** —— 世界坐标下的真值位姿逐帧变化；
3. **产出序列与逐帧元数据**，不是单张标定图。

## 3. 停车场布局

世界约定与 AVM 一致：米、Z 向上、**车头 +Y**、车道沿 +Y；地块以原点为中心。

```text
        x = -lot_width/2                                x = +lot_width/2
        ┌───────────────┬──────────────────────┬───────────────┐
        │   车位一排     │       车 道           │   车位一排     │  y = +lot_length/2
        │  (bay_depth)  │   (aisle_width)      │  (bay_depth)  │
        │   ┊ 车 ┊ 车 ┊  │ A11  ┈┈┈  B11        │  ┊ 车 ┊ 车 ┊  │
        │   ┊  ┊  ┊  ┊  │  ┈┈┈┈ 中线虚线 ┈┈┈┈   │  ┊  ┊  ┊  ┊   │
        │   ┊  ┊ 车 ┊  │ A12   ▲ 行驶方向 +Y  B12 │  ┊ 车 ┊  ┊  │
        └───────────────┴──────────────────────┴───────────────┘  y = -lot_length/2
```

- **地块尺寸**：`lot_size() = max(aisle_length, drive_distance + 2·bay_depth) × (aisle_width + 2·bay_depth)` —— 行驶里程会自动把地块撑长，车绝不会开出地坪。
- **统一布局**（`core/scenes/drive_lot.py`，纯 Python）：车位、分隔线、立柱、停放车、编号都出自同一份布局 —— 所以编号永远正对它的车位，停放车也永远不会杵进立柱里。
- **标线**（`show_bays` 总开关）：
  - 车道边线：沿 Y 的两条通长线；
  - 车位分隔线：每侧每 `bay_width` 一条，自车道边线伸到墙（端头内缩半个线宽，不越出地坪）；
  - **中线虚线**：车道中心 3 m 画、3 m 空 —— **车正下方那条线就是透明底盘要重建出来的东西**；
  - 黄色禁停框：车道尽头（`keep_clear_y`）—— 全场景唯一的黄色，方便肉眼定位。
- **车位编号**（`bay_numbers`，默认开）：左排 `A01…`／右排 `B01…`，**平铺在车道上、正对该车位**（用文字对象，数字在内置字体里，不需要找中文字体）。放在车道上而不是车位里，是因为停好的车不会盖住它，而且行驶过程中相机一直看得见 —— 这正是"后面好区分效果"要的：**每一帧都能对着画面里的编号核对该区域重建得对不对**。关掉 `show_bays` 时编号一并去掉。
- **停放的车**（`parked_cars`/排，默认 4）：车头朝墙、按车位均匀铺开（不是挤在一起），车型与车漆按固定表循环 —— 每次重建都长得一样。**紧邻立柱的车位保持空着**（真实车库就是这样，也避免车身穿模）。
- **标线是真实几何**（与 AVM Scene 的黑块同一决策）：贴在地面名义平面 `z = 0`（车轮正好压在上面），地坪下沉 `SLAB_DROP = 2 mm`，两者永不共面、不会 z-fighting；编号再抬高到 `z = 4 mm`（±2 mm 挤出）压在标线之上。
- **立柱**：`aisle_width/2 + 0.45 m` 处左右各一列，沿 Y 等距 —— 既像真实车库，也在前摄画面里提供遮挡。
- **顶光**：**一整片**贴在顶棚高度（墙顶 + 20 cm）、和地块一样大的面光，而不是沿车道一排小灯 —— 一排小灯会在地面烧出一圈圈高光斑，同一个车位在不同 y 上亮度都不一样，一段素材的逐帧就没法比；大面光把整片地坪照匀，默认 2000 W（实测地面平均约 0.4、无截断）。`shadows` 控制它是否投影（AVM Scene 关阴影是为了黑块检测器，这里要的是真实车库的光照观感）。隐藏 `show_walls` 时顶灯**不跟着关**——那是照明，不是结构。

## 4. 运动模型（纯 Python）

`core/scenes/drive_path.py` —— 不依赖 `bpy`，可单测：

- **速度曲线**：`constant`（整段匀速，**默认**）/ `trapezoid`（加速 → 巡航 → 刹停）；
  里程不够跑满两条斜坡时自动退化为**三角形**（峰值降到刚好停在该里程上，绝不冲过头）；
- **采样**：`t_n = n / fps`，帧数 `ceil(T·fps) + 1`，末帧落在**精确的 T**（`T·fps` 不是整数时最后一步略短，CSV 里的 `time_s` 才是权威）；
- **每帧真值**：`Frame(index, time, distance, speed, x, y, yaw)`，位姿 = 起点 + 前向 × 里程，前向 = `(-sin(yaw), cos(yaw))`（与 Blender 的 Z 欧拉一致）。

Blender 侧把**同一份 plan** 写进关键帧（每帧一个 key，LINEAR），并写进 CSV —— 两边同源，画面与 CSV 不可能对不上。

## 5. 录制与导出

`[Export Clip…]` 把整段内容**打包成一个 zip**（每路一条 mp4 + csv + json，默认还带 PNG 序列）：

```
<输出目录>/ 或 <clip.zip>/
├── frame_0000_front.png … frame_NNNN_right.png   # 每台被录相机每帧一张（`frame_%04d_<camera>.png`）
│                                      #   —— `clip_keep_frames` 关掉时不出（见下）
├── front.mp4  back.mp4  left.mp4  right.mp4      # 每路一条 H.264（Export Clip 才有）
├── frames.csv                         # frame,time_s,distance_m,speed_mps,x_m,y_m,yaw_deg,
│                                      #   cam_<camera>_{x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg} × N
└── clip.json                          # 路径/速度参数 + cameras[]（K/D/车体系安装位姿）+ vehicle
                                       #   （车体几何）+ 文件名契约 + 渲染尺寸与采样
                                       #   + 时间口径 + video 名 + video_encode 编码处方
```

**产物契约（v2，破坏性变更）**：`clip.json` 的 `version` 是 `2`，单数 `"camera"` 块换成 `"cameras"` 数组。
下游要能判别版本，所以版本号是唯一的判别位。

- **录哪几路是设置**（`cameras[].enable`，**默认四路全开**），**顺序即 `core/scenes/avm_cameras.CAMERAS`**——
  这个顺序同时是 `frames.csv` 列组的顺序、`clip.json` 的 `cameras` 数组顺序、以及文件名里的相机名；
  至少有一路开着，否则录制直接报错（不会静默出一个空 clip）。
- **`frames.csv` 是 `7 + 6 × N` 列**（N = 实际录制路数）：车体 7 列 + 每台相机一组 6 列，列名带相机名
  （`cam_front_x_m`…），"哪列是哪台"不必猜。那三列 `roll/pitch/yaw_deg` 是 **Blender XYZ 欧拉角**，
  不是相机 pitch/yaw——这是渲染侧 pose 的真实形态，写错会让人套错轴。
- **`clip.json` 的 `cameras[]`** 每条给 `name`（对象名 `DRIVE_Cam_Front`）、`camera`（键 `front`）、
  `model` / `output` / `K` / `D` / `mount{frame, location, rotation_deg}`。`K` 是**按渲染分辨率换算后**的值，
  所以下游可以直接投影，不必自己重算缩放规则。`frame_pattern` / `video_pattern` 把文件名约定写进文件本身。
- **`video` 只在单路时是文件名**；多路时为空字符串，契约是**每路一条 `<camera>.mp4`**（消费侧的
  `FrameSource` 正是按这个规则取路）。
- **`vehicle` 块只有一份**（车体不随相机走）：`frame`（车体系，显式写出）+ `body{length_m,width_m,height_m}`
  + `ground_clearance_m` + `axles{wheel_base_m,rear_track_m,rear_center_offset_m}`，供下游按几何解算自车掩码。
  数字来自 `core/scenes/vehicle.py`（AVM/Drive 两个 mesh 与 Falcon 的转向线配置同源）；`body` 是**名义车身盒**，
  车轮与灯箱略微凸出（每侧 +0.11 m / 每端 +0.02 m），所以对象 `dimensions` 比它大——这一点在测试里
  用**车体材质面的实际顶点**量出来验证，而不是把设置再抄一遍。
- **分辨率用 Blender 自带的 `Render ▸ Output`**（插件不另设输出尺寸），`clip.json` 记录实际尺寸。
- **mp4 的画布 = 静帧的实际像素尺寸**：编码用的是新建的临时序列场景，新建场景默认 1920×1080，而
  序列条在画布里是**按原尺寸绘制**（不会自动缩放）——如果不动它，96×72 的素材会被渲成
  1920×1080 的黑底中间贴一张小图（实测）。所以 `encode_video()` 读第一张静帧的尺寸来设定画布。
- **默认文件名** `drive_scene.zip`（固定值，与当前 .blend 的文件名无关）。
- **质量档位** `clip_quality`：`draft`（默认）/ `balanced` / `high`。只改**渲染成本**（采样数、降噪、光追反弹上限、自适应采样阈值、焦散、跨帧持久化缓存），**输出尺寸与每台相机的 K/D 完全不变**；`high` 等于历史行为（64 采样、不改降噪/反弹）。当前实际采样数写进 `clip.json` 的 `render.samples`，档位名写进 `render.quality`。
- **渲染设备** `clip_device`：`cpu`（默认）/ `gpu`（仅 NVIDIA OptiX）。OpenCV 相机是 **OSL 着色器**，Cycles 只在 **CPU 与 OptiX** 上求值；macOS 的 **Metal**、以及 CUDA/HIP/oneAPI 都不支持，选 `gpu` 会被**拒绝并回退 CPU**（状态里给出原因），保证相机正确。实际使用的设备写进 `clip.json` 的 `render.device`。
- **`clip_keep_frames`**（默认开）：关掉则导出只留每路 mp4 + `frames.csv` + `clip.json`。PNG 序列只是编码器的输入（单路 68 帧 1920×1080 是 138 MB，mp4 只有 1.7 MB），**吃视频的下游不需要它**；关掉后仍可从 `clip.json` 的场景参数重渲。渲染-only 的 `render_clip()` 不受此开关影响——没有视频时 PNG 就是产物本身。
- `frames.csv` 的 `cam_<camera>_*` 列是**每帧该相机的世界位姿**（车体位姿 × 该台的固定安装位姿，由 `core/scenes/drive_path.camera_world_pose` 纯 Python 算出），下游不必自己组合。
- **视频**用 Blender 内置 FFmpeg（H.264/mp4）：把已渲染的 PNG 序列喂给一个临时序列编辑器场景编码，**不重渲染** 3D 场景，编完即删；**每路一个临时场景**（同一个场景里几条序列条就分不开了）。临时场景的**视图变换与 look 镜像渲染场景**（`Standard`）——新建场景默认是 `AgX`，会把已经显影完的像素再调一次（实测 −14 dB、高光 −33 LSB），这是必须镜像的原因。
- **`clip.json` 的 `video_encode`** 记录编码处方：`container / codec / constant_rate_factor / view_transform / look / fps / blender`。下游要靠它区分"算法漂移"与"编码漂移"；`blender` 版本是唯一的编码器版本线索（Blender 4.5 已移除 `bpy.app.ffmpeg_version`）。
- 录制时**临时屏蔽场景里不属于本场景的灯**（新建 Blender 场景自带点光，否则每段素材的光照都不一样），并对**每一台被录相机**应用一次着色器与 Cycles 参数；渲染设置（引擎 / 设备 / 格式 / 采样 / 降噪 / 自适应 / 焦散 / 反弹 / 线程 / 持久化 / 时间轴 / 相机 / 当前帧）整体保存并恢复。
- **进度与中断**：`[Export Clip…]` 在 GUI 里是**模态逐帧渲染**（`ClipJob` 每 tick 渲**一帧 × 全部被录相机**），面板与状态栏实时显示 `frame k/N · xx% · ETA m:ss`，**按 ESC 中断**；中断时不编码、不打包，`clip_status` 记为 `cancelled`。无头 / 脚本路径走同一个 `ClipJob` 的同步驱动。进度单位仍是"帧"，相机在帧内循环，所以 ETA 不会被相机数放大。
- **可复现性**：同一段 clip 渲两遍，**像素完全一致**（实测 Δ = 0 LSB），文件字节的差异只在 PNG 的 `tEXt` 墙钟字段（`Date` / `RenderTime` / `cycles.*_time`）。所以"无损参考"可以**按需再生**而不必存盘——但比对时要**比解码像素，不要哈希文件**。用 `scripts/check_clip_reproducibility.py` 复验（该探针只录前摄，跑得快；四路由测试套件覆盖）。

## 6. 参数与面板

两个面板（Scene Properties + 3D 视口 N 侧栏 `Drive Scene`），都只在场景已建立时出现：

- **概览**：地块尺寸 + `N frames @ fps`、时长、里程、峰值速度 + 「巡航速度下每帧位移 = v / fps」（选速度与帧率时最实用的一个数）；
- **Car park**：`aisle_length` / `aisle_width` / `bay_depth` / `bay_width` / `ground_texture`(concrete/asphalt/epoxy/checker/plain) / `show_bays` / `bay_numbers` / `parked_cars` / `pillar_count` / `light_energy` / `shadows`；
- **Vehicle**：长 / 宽 / 高 / 离地间隙（默认值来自 `core/scenes/vehicle.py`，与 AVM 同一份）；
- **Drive**：`drive_distance` / `drive_speed` / `drive_profile`（默认 `constant` 匀速）/ `drive_accel`（仅 `trapezoid` 显示）/ `drive_fps` / `drive_heading`；
- **Cameras**：`active_camera`（视口 / F12 用哪台，改完立即切 `scene.camera`）+ 四个 `Record` 开关 + 一行 `k of 4 recorded` + `[Sync Cameras from AVM Scene]`。**这里没有 K/D/位姿**——那是标定，编辑入口是 `CV Intrinsics` / `CV Presets`；
- **Show / Hide**：地面（含编号）/ 围墙（含立柱）/ 停放的车 / 自车（只切显隐、不重建；顶灯不属于任何图层，隐藏围墙不会把画面弄黑）；
- **Record**：`clip_quality`（draft / balanced / high，默认 draft）、`clip_device`（cpu / gpu，默认 cpu）与 `clip_keep_frames` —— 只影响渲染成本与产物体积，输出尺寸与相机参数不变；导出中面板显示进度与 ETA，**ESC 取消**；
- **动作**：`[Rebuild]` `[Reset Defaults]` `[Frame View]` `[Sync Cameras from AVM Scene]` `[Export Clip…]`（每路 mp4 + PNG + csv + json 打包成 zip）`[Remove Drive Scene]`。

## 7. 模块划分

```text
core/scenes/avm_cameras.py           # 四摄的角色/顺序/对象命名/预设记录→安装位姿（纯 Python，AVM 与 Drive 共用）
core/scenes/vehicle.py               # 车体几何（车身盒 + 轴距/轮距/后轴偏移）与 clip.json 的 vehicle 块（纯 Python）
core/scenes/drive_lot.py             # 车位/分隔线/立柱/停放车/编号的确定性布局（纯 Python）
core/scenes/drive_path.py            # 路径 + 速度曲线 → 逐帧真值 + CSV（`csv_header(cameras)` / `csv_text(plan, mounts)`）
bl/scenes/drive_scene/
├── __init__.py                      # DEFINITION + register 编排（注册表里加一行即可）
├── properties.py                    # scene.drive_scene（地块 / 车辆 / 行驶 / 相机开关 / 图层）
├── builder.py                       # 地坪+标线、立柱、围墙、顶灯、车模、四台相机、关键帧
├── controller.py                    # 去抖重建
├── recording.py                     # ClipJob（逐帧 × 逐相机渲染/进度/ETA/设备）→ PNG + frames.csv + clip.json；每路一条 mp4；打包 zip
├── operators.py                     # opencv_cam.drive_*（含模态 Export Clip + Sync Cameras from AVM Scene）
└── ui.py                            # 两个面板
```

## 8. 测试

- `tests/test_core.py::test_avm_cameras`：四摄角色/顺序（`avm_layout.CAMERAS` 与 `avm_cameras.CAMERAS` 同一对象）、对象命名、`enum_items()`、`key_of()`（与消费侧 `FrameSource::NormalizeCameraKey()` 同一约定）、预设记录 → 安装位姿；
- `tests/test_core.py::test_vehicle`：车身盒与轴距的数值、`avm_falcon.STEERING` 读同一份真源、`vehicle.block()` 的形状（含 `frame` 与"只有一份、不含任何按相机的尺寸"）；
- `tests/test_core.py::test_drive_path`：速度曲线（匀速/梯形/退化三角形）、采样（`v = ds/dt`、单调性、末帧落在精确 T）、CSV 契约（表头 = `csv_header(cameras)`、`7 + 6 × N` 列、每台相机的世界位姿列跟随**各自的**安装位姿、按实际录制集合出列、零里程）；
- `tests/test_core.py::test_drive_lot`：车位数量/编号（A01…/B01…）/位置、分隔线等距且铺满地块、立柱落在车位排内、**停放车不杵进立柱、不超车位、车头朝墙、均匀铺开、可复现**；
- `tests/run_blender_tests.py::test_drive_scene`：对象命名与集合、**四台相机**（都是 Custom + bytecode + 同一份 minibus K + 四个不同的车体系位姿 + 都挂在 `DRIVE_Vehicle` 上）、**相机开关**（默认四路全开、`active_camera` 即时切 `scene.camera`、`view_targets` 含四台）、地坪材质槽与程序化噪声、中线标线、地块覆盖里程、停放车、编号、**关键帧与纯 Python 位姿逐帧一致**、小尺寸录 9 帧（**每帧 4 张、命名符合 `frame_name(index, camera)`、总数 = 帧数 × 路数**、CSV 表头/列数/每组位姿、`clip.json` v2 的 `cameras[]` 与**生效 K** 逐条对齐对象、`camera_names`/`frame_pattern`/`video_pattern`/`vehicle` 块、**车体材质面的实测尺寸 = 导出的车身盒**、改车宽导出跟着变、`vehicle` 块只有一份）、**质量档位/设备解析/进度文本**、**Export Clip 的 zip 含每路 mp4（无 `clip.mp4`）+ PNG + csv + json**、**每路 mp4 的解码帧数 = CSV 行数、画布 = 渲染分辨率、fps = plan.fps**、**关掉一路 → 3 路 / 25 列 / `cameras` 3 条、`vehicle` 块仍是一份**、**一路都不开 → 明确报错**、`[Reset Defaults]` 把四路都放回录制、取消时报告 `cancelled` 且不留下 zip、渲染设置与外部灯光的恢复、移除后四台相机都清掉；
- `tests/run_blender_tests.py::test_drive_matches_avm_defaults`：车漆、车身尺寸（都读 `vehicle.py`）与**四台相机逐项**（K/D/输出/挂载矩阵）+ 四台确实各自用了预设里对应那条记录；
- `tests/run_blender_tests.py::test_drive_sync_from_avm`：单向同步把活的 AVM 位姿/内参抄到 Drive 四台、父级仍是 `DRIVE_Vehicle`、重建后保留、没有 AVM 场景时拒绝。

## 9. 分期

| 期 | 内容 |
| --- | --- |
| **P0** | 直线 + 匀速/梯形 + 前摄逐帧 + PNG/CSV/clip.json + **mp4 与 zip 导出** + 测试 + 本文档 |
| **P1** | **四路相机**（[`drive-scene-multicam.md`](drive-scene-multicam.md) 的 M1–M4 **已实施**：相机与车体定义上提 `core/`、Drive 侧四台、录制契约 v2、可选的单向同步算子）、曲线路径与航向、模态化可中断录制与进度（后两项已实施） |
| P2 | 真值扩展（深度 / 实例分割 / 逐帧相机位姿文件），布局向路线图 M7 `dataset_export` 靠拢 |
| P3 | 地面附着（z 跟随坡面）、悬架俯仰 / 侧倾、滚动快门 / 运动模糊 |

## 10. 风险与注意点

- **渲染成本随路数线性增长**：一次录制 = 帧数 × 路数 张图。默认四路全开是为 AVM 素材通路准备的；只想看前摄时把另外三台的 `Record` 关掉（`[Reset Defaults]` 会把它们放回来）。1280×960 × 上百帧 × 4 路的 Cycles CPU 不便宜 —— `clip_quality` 默认 `draft`（8 采样 + 降噪 + 2 次反弹 + 自适应采样 + 关焦散 + 跨帧持久化缓存），出片要更干净可升到 `balanced` / `high`；先小尺寸试跑，`clip_keep_frames` 关掉可把导出体积压回视频量级；
- **编码画布必须跟着静帧走**（见 §5）：`encode_video()` 新建的序列场景默认 1920×1080，而序列条按原尺寸绘制，不设画布就会把小尺寸素材渲成黑底中间贴小图。测试里钉了"mp4 画布 = 渲染分辨率"这条断言；
- **GPU 基本用不上（macOS）**：相机是 OSL 自定义相机，Cycles 的 OSL 只在 **CPU 与 NVIDIA OptiX** 上求值，macOS 的 Metal 不支持 —— 所以 `clip_device=gpu` 在本机只会回退 CPU；想用 GPU 需要带 NVIDIA 显卡（OptiX）的机器。导出会强制/校验设备，绝不让 GPU 破坏相机；
- **纹理要像真实停车场、但要有信号**：纯棋盘等于"送分"（对齐误差天然小），单一尺度的噪声又**自相似**
  （很多平移都拟合得差不多）；而裂缝网格、高对比斑点又太花、不像真地面。`ground_texture` 提供三种真实
  地坪——`concrete`（水泥，中灰哑光）、`asphalt`（柏油，近黑哑光 + 骨料亮点）、`epoxy`（环氧，浅灰半光）
  ——都用两种**低对比度**尺度叠加（宽污渍 ~0.9 m + 细颗粒 ~3 cm），每块地面各有轻微差异、逐帧不同，
  纹理锚在**地面物体**坐标系上（车动它不动）；`checker` / `plain` 是对照实验（最强纹理 / 无纹理）；
- **四路一起录时不要只喂一路**（消费侧口径）：只喂一路时渲染侧的碗模型其余扇区是纯黑，统计指标必须
  限定在有纹理的连通区域内，否则大量黑像素会把误差摊平（见 `filament_avm/docs/sim_clip_render_pipeline.md` §10-F3）；
- **时间口径**：仿真时间 `t = frame / fps`，没有绝对时钟，与真实采集的差异写进了 `clip.json`；
- **外部灯光/相机内参**：录制时会屏蔽外部灯；相机内参由 minibus 预设决定，要改请在 `CV Intrinsics` 改（与其它场景一致），改完两个场景同时生效。
