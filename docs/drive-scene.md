# Drive Scene 设计说明（室内停车场行驶素材）

> 状态：**P0 已实施**（直线路径 + 匀速/梯形速度 + 前摄逐帧录制 + 逐帧真值）。
> 用途：为**透明底盘**算法提供带真值的仿真素材 —— 前摄画面 + 每帧车速与位姿。
> 实现：场景 `addons/opencv_camera/bl/scenes/drive_scene/`，运动模型 `core/scenes/drive_path.py`。

## 1. 目标与契约

| | 内容 |
| --- | --- |
| 输入 | 停车场地参数（车道/车位/立柱/灯光）+ 车辆尺寸 + 行驶参数（里程 / 速度 / 加速度 / 帧率）|
| 输出 | `frame_%04d.png` 序列 + `frames.csv`（逐帧时间/里程/车速/**真值位姿**）+ `clip.json`（整段规格）|
| 相机 | 插件自带的 **OpenCV Camera 组件**（fisheye），内参/安装位姿取内置 minibus 标定预设 |
| 非目标 | 多相机同步、深度/分割真值、悬架俯仰侧倾、滚动快门、mp4（P1 起）|

## 2. 场景内容与取舍

| 元素 | 做法 |
| --- | --- |
| 地面 | **室内停车场**：混凝土地坪（程序化噪声）+ 车道/车位标线（真实几何，见 §3） |
| 车位 | 两侧车位**编号**（`A01…` / `B01…`，文字平铺在车道上、正对该车位）与**停好的车**（`parked_cars`/排，车型与颜色循环）|
| 结构 | 两侧立柱（`pillar_count`/侧）、四面围墙（`show_walls`）、一整片柔和顶光（见 §3） |
| 车辆 | 程序化 minibus（长/宽/高/离地间隙可调），几何与 AVM Scene 同构但**本场景自带实现** |
| 相机 | `DRIVE_Cam_Front`，`Add ▸ VisionSim ▸ Drive Scene` 时按预设创建（K/D/输出 + 车体系安装位姿）|
| 运动 | `DRIVE_Vehicle` 空物体承载世界位姿，车与相机都挂在它下面（相机因此永远保持车体系位姿）|

**与 AVM Scene 的关系**：相机用的是同一个 OpenCV Camera 组件与同一份 minibus 标定预设（这样仿真画面才和真机可比）；**其余组件不复用**，Drive Scene 自带地面/立柱/车模实现，只参考 AVM 的做法。两者可同时存在于一个 .blend（对象前缀分别是 `DRIVE_` / `AVM_`，互不干扰）。

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

- **速度曲线**：`constant`（整段匀速）/ `trapezoid`（加速 → 巡航 → 刹停）；
  里程不够跑满两条斜坡时自动退化为**三角形**（峰值降到刚好停在该里程上，绝不冲过头）；
- **采样**：`t_n = n / fps`，帧数 `ceil(T·fps) + 1`，末帧落在**精确的 T**（`T·fps` 不是整数时最后一步略短，CSV 里的 `time_s` 才是权威）；
- **每帧真值**：`Frame(index, time, distance, speed, x, y, yaw)`，位姿 = 起点 + 前向 × 里程，前向 = `(-sin(yaw), cos(yaw))`（与 Blender 的 Z 欧拉一致）。

Blender 侧把**同一份 plan** 写进关键帧（每帧一个 key，LINEAR），并写进 CSV —— 两边同源，画面与 CSV 不可能对不上。

## 5. 录制与导出

```
<输出目录>/
├── frame_0000.png … frame_NNNN.png    # 前摄画面（鱼眼，标定分辨率换算后）
├── frames.csv                         # frame,time_s,distance_m,speed_mps,x_m,y_m,yaw_deg
└── clip.json                          # 路径/速度参数 + 相机 K/D/车体系安装位姿 + 渲染尺寸与采样 + 时间口径
```

- **分辨率用 Blender 自带的 `Render ▸ Output`**（插件不另设输出尺寸），`clip.json` 记录实际尺寸；内参按渲染分辨率换算后写入 `K`。
- 录制时**临时屏蔽场景里不属于本场景的灯**（新建 Blender 场景自带点光，否则每段素材的光照都不一样），渲染设置（filepath / 格式 / 采样 / 相机 / 当前帧）整体保存并恢复。
- P0 是**阻塞式**整段渲染（与 `Export Falcon` 一致）；长片段建议先小尺寸试跑，可中断/进度条见 P1。

## 6. 参数与面板

两个面板（Scene Properties + 3D 视口 N 侧栏 `Drive Scene`），都只在场景已建立时出现：

- **概览**：地块尺寸 + `N frames @ fps`、时长、里程、峰值速度 + 「巡航速度下每帧位移 = v / fps」（选速度与帧率时最实用的一个数）；
- **Car park**：`aisle_length` / `aisle_width` / `bay_depth` / `bay_width` / `ground_texture`(concrete/checker/plain) / `show_bays` / `bay_numbers` / `parked_cars` / `pillar_count` / `light_energy` / `shadows`；
- **Vehicle**：长 / 宽 / 高 / 离地间隙；
- **Drive**：`drive_distance` / `drive_speed` / `drive_profile` / `drive_accel` / `drive_fps` / `drive_heading`；
- **Show / Hide**：地面（含编号）/ 围墙（含立柱）/ 停放的车 / 自车（只切显隐、不重建；顶灯不属于任何图层，隐藏围墙不会把画面弄黑）；
- **动作**：`[Rebuild]` `[Reset Defaults]` `[Frame View]` `[Render Clip…]` `[Remove Drive Scene]`。

## 7. 模块划分

```text
core/scenes/drive_lot.py             # 车位/分隔线/立柱/停放车/编号的确定性布局（纯 Python）
core/scenes/drive_path.py            # 路径 + 速度曲线 → 逐帧真值 + CSV（纯 Python）
bl/scenes/drive_scene/
├── __init__.py                      # DEFINITION + register 编排（注册表里加一行即可）
├── properties.py                    # scene.drive_scene（地块 / 车辆 / 行驶 / 图层）
├── builder.py                       # 地坪+标线、立柱、围墙、顶灯、车模、相机、关键帧
├── controller.py                    # 去抖重建
├── recording.py                     # 逐帧渲染 → PNG + frames.csv + clip.json
├── operators.py                     # opencv_cam.drive_*
└── ui.py                            # 两个面板
```

## 8. 测试

- `tests/test_core.py::test_drive_path`：速度曲线（匀速/梯形/退化三角形）、采样（`v = ds/dt`、单调性、末帧落在精确 T）、CSV 契约（表头、行数、精度、零里程）；
- `tests/test_core.py::test_drive_lot`：车位数量/编号（A01…/B01…）/位置、分隔线等距且铺满地块、立柱落在车位排内、**停放车不杵进立柱、不超车位、车头朝墙、均匀铺开、可复现**；
- `tests/run_blender_tests.py::test_drive_scene`：对象命名与集合、地坪材质槽与**程序化噪声**、中线标线存在、地块覆盖里程、停放车（数量/朝向/车漆表/图层开关）、编号（数量、FONT 平铺、在车道内、开关会增删对象）、相机（Custom + bytecode + minibus K + 车体系位姿）、**关键帧与纯 Python 位姿逐帧一致**、时间轴、小尺寸录 9 帧（PNG 数、CSV 行与速度列、clip.json 的 K/尺寸/挂载系）、渲染设置与外部灯光的恢复、移除后对象清理。

## 9. 分期

| 期 | 内容 |
| --- | --- |
| **P0（本批）** | 直线 + 匀速/梯形 + 前摄逐帧 + PNG/CSV/clip.json + 测试 + 本文档 |
| P1 | 曲线路径与航向、四路相机、mp4（Blender 内置 FFmpeg）、模态化可中断录制与进度 |
| P2 | 真值扩展（深度 / 实例分割 / 逐帧相机位姿文件），布局向路线图 M7 `dataset_export` 靠拢 |
| P3 | 地面附着（z 跟随坡面）、悬架俯仰 / 侧倾、滚动快门 / 运动模糊 |

## 10. 风险与注意点

- **渲染耗时**：1280×960 × 上百帧的 Cycles CPU 不便宜 —— 默认只录前摄，先小尺寸试跑；
- **纹理不能太规整**：纯棋盘等于"送分"（对齐误差天然小）—— 默认混凝土噪声，`ground_texture` 可切棋盘/纯色做对照实验；
- **前摄的盲区**：车正下方与两侧本就看不到，透明底盘必然出现空洞 —— 这是素材要暴露的现象（多相机在 P1）；
- **时间口径**：仿真时间 `t = frame / fps`，没有绝对时钟，与真实采集的差异写进了 `clip.json`；
- **外部灯光/相机内参**：录制时会屏蔽外部灯；相机内参由 minibus 预设决定，要改请在 `CV Intrinsics` 改（与其它场景一致）。
