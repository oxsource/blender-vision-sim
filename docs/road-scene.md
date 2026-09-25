# Road Scene 设计说明（闭合测试道路素材）

> 状态：**已实施**（闭合环线 + 直道/弯道/上下坡 + 可选曲线侧倾 + 路侧道具 + 四摄逐帧录制 + 三维真值 + 按段导出）。
> 用途：把 Drive Scene 只承诺过的「平地、匀速直行、单一光照」扩展到**可证**的
> 弯道 / 坡道 / 变速 / 倒车工况，为 `filament_avm` 透明底盘的 **M5 扩展工况验证**
> （`docs/specs/004_trans_chassis/`，CH-018）提供「道路类型 × 速度 × 方向」矩阵素材。
> 实现：场景 `addons/opencv_camera/bl/scenes/road_scene/`，道路模型 `core/scenes/road_track.py`，
> 运动模型 `core/scenes/road_path.py`，共享录制引擎 `bl/scenes/clip_core.py`。

## 1. 目标与契约

| | 内容 |
| --- | --- |
| 输入 | 环线尺寸（直道 / 弯道半径 / 曲线侧倾 / 坡高与坡长 / 路宽 / 路肩）+ 路侧道具数量 + 光照 + 车辆尺寸 + 行驶参数（速度 / 曲线 / 帧率 / 方向 / 整圈或某一段）|
| 输出 | 每路一条 `<camera>.mp4`（`front/back/left/right`）+ `frames.csv`（逐帧时间/里程/车速/**三维车体位姿** + 每台相机世界位姿 + 工况标签）+ `clip.json`（v3：`motion` + `segments[]` + `cameras[]` + `vehicle`）+ `frame_%04d_<camera>.png` 序列；`[Export Clip…]` 打包成 zip |
| 相机 | 与 AVM / Drive Scene **同一份** minibus 标定预设的四摄（`front/back/left/right`），本场景不持有任何相机参数 |
| 非目标 | 悬架俯仰/侧倾、滚动快门、逐帧位姿回放；动态交通流的完整建模（行人可动，其余道具静态）|

## 2. 与 Drive Scene 的关系

同一个场景框架、同一份相机真源（`core/scenes/avm_cameras.py`）、同一份车体几何
（`core/scenes/vehicle.py`）、同一个**共享录制引擎**（`bl/scenes/clip_core.py`）。差别只在：

1. **道路**从「室内停车场平地」变成「闭合环线，含真实上下坡」；
2. **真值**从七列（`x/y/yaw`）扩成十三列（追加 `z_m/pitch_deg/roll_deg` 与
   `segment/road_type/direction`）；
3. `clip.json` 版本升到 **3**：保留 v2 的 `cameras[]`（v2 读取器仍可解析），新增
   `motion` 与 `segments[]` 块。

Drive Scene 的产物契约与 `blsim 1.0.3` 基线**不受影响**：`clip_core` 只是把引擎参数化，
`drive_scene/recording.py` 仍是原来的 profile 与文件名（`drive_scene.zip`、`version 2`）。

## 3. 环线模型（纯 Python，`core/scenes/road_track.py`）

世界约定与其余场景一致：米、Z 向上、车头 `+Y`（`yaw = 0`）、前向 `(-sin yaw, cos yaw)`。

环线由有序的 `SegmentSpec` 组成，构建时逐段求出世界位姿，并**强制闭合校验**
（位置、高度、航向三者在毫米/毫弧度内回到起点，否则报错）：

| kind | 几何 | 注明 |
| --- | --- | --- |
| `straight` | 平直段 | 基线 |
| `curve` | 定半径圆弧（`radius` 带符号，正=左转；`angle_deg` 同号） | 平面曲率 `1/R`；可设 `bank_deg` 峰值侧倾，使用 sin² 包络在段边界回到水平 |
| `ramp` | 竖向曲线：平面上是直线，坡度按半正弦变化 | 起止坡度为 0，与直道**无折角**地衔接；`rise` 为总高差 |
| `s_curve` | 变道：航向 `A·sin(2πn s/L)` | 整数周期时净航向与净横移均为 0（自定义模板用，不在默认环线中） |

模板（`default_track_specs`）是一个圆角矩形环线：两段 `straight` + 四个 90° 弯；另一对直道被
拆成「平段 + 上坡 + 平段」与「平段 + 下坡 + 平段」，两对直道的总长相等，因此环线在任意
半径/直道长度下都闭合；长度恒为 `L = 4·直道 + 2π·半径`。环线以原点为中心（x/y）。

**两套命名尺寸**（`TRACK_PRESETS`，面板一个下拉切换，同一时刻只建一条环线）：

| preset | 直道 | 半径 | 坡长/坡高 | L | 25 km/h 一圈（scenario） |
| --- | --- | --- | --- | --- | --- |
| `compact`（默认） | 20 m | 7 m | 10 m / 1.0 m | ≈124 m | ≈28 s（含斑马线；带停车入库 ≈38–41 s）|
| `full` | 40 m | 14 m | 12 m / 1.6 m | ≈248 m | ≈43 s |

两套都含全部 M5 道路要素（直道/弯道/上坡/下坡/斑马线），`custom` 时用面板滑杆微调。
`compact` 是默认，因为一段素材的渲染量 ∝ 时长 × fps × 相机数：248 m 一圈约 1700 张静帧，
124 m 一圈约 1000 张（`compact` 的 `scenario` 无停车时）。

`curve` 段的 `bank_deg` 使用 `sin²(πt/L)` 平滑包络，在段连接处回到零、段中达到设定峰值；默认值为 0°。
`RoadTrack` 提供 `pose_at(s)`（任意弧长处返回 `x/y/z/yaw/pitch/roll/curvature`）、`segment_at(s)`、
`samples()`、`bounds()` 与 `describe()`（逐段的起止里程/长度/高差/半径/road_type）。

## 4. 运动模型（纯 Python，`core/scenes/road_path.py`）

- **方向**：`forward`（车头朝前）或 `reverse`（倒车）。倒车时车辆朝向与切向相反，
  `yaw = wrap(tangent + 180°)`、`pitch = -tangent_pitch`（上坡倒车变成"车头朝下"），
  导出的仍是普通 Blender XYZ 欧拉三元组。
- **速度曲线**：
  - `scenario`（**默认**）：按弧长解一条几何感知的速度场（`speed_field`）——从静止起步、
    在**斑马线**前减速到 `slow_speed`、其余按 `drive_speed`（默认 7 m/s）巡航，
    最后按 `drive_decel` **精确刹停在终点**（一圈起点）。速度场做**加速度受限双遍扫描**
    （前向限加速、后向限制动），`v(0)=v(end)=0`；帧的 `(距离, 速度)` 用分段常加速度
    **精确积分**得到，保证 `ds/dt == speed_mps`。**只慢斑马线，弯道/坡道不额外减速。**
  - `constant` / `trapezoid`：Drive Scene 的两条对照曲线（匀速 / 加速-巡航-刹停）。
- **停车入库**（`parking`，默认开）：起点侧有若干垂直车位。整圈 clip 因此是三段拼接：
  **出库**（从车位沿 90° 圆弧前进上主路）→ **一圈 scenario** → **倒车入库**
  （沿同一圆弧倒回，`direction=reverse`，车头仍朝外）。圆弧几何由
  `road_track.parking_arc()` 给出：半径 = 路宽/2 + 路肩 + 车长/2，圆弧终点落在中心线上
  且航向与中心线一致。这样一条 clip 里就同时包含**倒车**工况（M5 的方向维度）。
  关掉 `parking` 时退回"中心线起步/停"的纯一圈。
- **按段**：`start_distance` 指定起点弧长，`loops` 指定圈数（分数即一段）。
  `drive_segment` 选某段时，`distance = 该段长度`；倒车时从该段**末端**启程，
  保证覆盖的是同一段。
- **逐帧真值** `Frame(index, time, distance, speed, x, y, z, yaw, pitch, roll,
  segment, road_type, direction, steering_deg, gear)`；`csv_text()` 写 CSV。
- **车辆信号**（供 AVM 算法按整车信息驱动）：`speed_mps` 车速；`steering_deg` **前轮转角**
  （左正，`tan(δ)=轴距·曲率`，倒车翻号）；`gear ∈ {P, R, D}`（静止为 `P`）。
- **相机世界位姿** `camera_world_pose(frame, mount)` 是 Drive 平路公式的完整三维推广
  （车体旋转 × 安装位姿，再取 XYZ 欧拉角）；平路前向时与 `drive_path.camera_world_pose`
  逐位相同（测试断言）。

### CSV 契约（`frames.csv`）

```text
frame,time_s,distance_m,speed_mps,x_m,y_m,yaw_deg,        # 前 7 列 == Drive Scene
z_m,pitch_deg,roll_deg,                                    # 追加：三维姿态
segment,road_type,direction,                               # 追加：工况标签
steering_deg,gear,                                         # 追加：车辆信号
cam_<camera>_{x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg} × N  # 每台被录相机一组 6 列
```

`15 + 6 × N` 列。**车辆信号**（喂给 AVM 算法模拟整车信息）：
- `speed_mps` 车速；`steering_deg` **前轮转角**（左正；自行车模型
  `tan(δ)=轴距·曲率`，倒车时符号翻转），轴距取 `core/scenes/vehicle.py` 的 `WHEEL_BASE_M`；
- `gear ∈ {P, R, D}`：速度为 0 为 `P`，否则前进 `D` / 倒车 `R`。
`road_type ∈ {straight, curve, slope_up, slope_down, parking}`，
`segment` 是段名（`straight_a` / `curve_0` / `ramp_up`… / `park_exit` / `park_entry`），
`direction ∈ {forward, reverse}`。**停车场景里 `direction` 是逐帧的**：一圈前向，最后一段
倒车入库为 `reverse`。
消费侧 `TrajectoryFileSource` 按列名取 `time_s/x_m/y_m/yaw_deg`，因此新列是**向后兼容的追加**；
坡道工况需要的 `z/pitch` 也随文件交付，供下游扩展。

### `clip.json`（v3）

保留 v2 的 `format/version/created/fps/frames/duration_s/render/cameras[]/camera_names/
frame_pattern/video_pattern/vehicle/frames_csv/video/video_encode/time_base`，并新增：

- `motion`：`profile` / `cruise_speed_mps` / `slow_speed_mps` / `accel_mps2` /
  `decel_mps2` / `direction` / `loops` / 起始里程 / 环线长度 / 路宽/路肩 /
  `parking` + `parking_speed_mps`；
- `road`：`preset` / `surface` / `markings` / `crosswalk` / 坡与半径参数；
- `signals`：逐帧车辆信号的列名与语义（`speed_mps` / `steering_deg` / `gear`，
  `gear_values=[P,R,D]`，转向为前轮角、左正、倒车翻号）；
- `segments[]`：逐段的 `name / road_type / kind / start_m / end_m / length_m /
  start_z_m / end_z_m / radius_m / rise_m`。

`version` 升到 `3` 是唯一判别位：`cameras[]` 结构不变，新增的是**字段与 CSV 列**，不是改名。

## 5. 场景内容（`bl/scenes/road_scene/builder.py`）

- **道路带**：沿中心线每 1 m 采样，按路宽生成沥青带 + 两侧路肩 + 放坡到基准地面的草地
  （放坡比 1:1.5），并铺一整块基准草地。坡道因此是**真实几何**：路面 `z` 与放坡一起抬高。
  沥青用两种尺度的程序化噪声，且**骨料颗粒感较强**（细颗粒权重高于宽污渍，色带同时向深色
  胶结料和浅色骨料两端拉开）——透明底盘靠纹理对齐验证，路面不能是近乎纯黑。
- **标线**（常开，不再提供开关）：两侧白色实线（路缘内缩）+ 中线黄虚线（3 m 画 / 3 m 空，
  抬高 4 mm，不与路面 z-fighting）。标线是真实几何，任何相机分辨率下都锐利。
  **斑马线**（常开）：在上坡起步前的一条白色横向条带（条带沿行车方向、横贯车道，
  再抬高 2 mm 避免与中线虚线的 z-fighting），两端各留一名行人在路肩等待
  （`crosswalk_distance()` 给出它的弧长）。有纹理的斑马线是很好的重建对齐目标，
  也让"坡前减速/让行"这一常见工况有画面依据。
- **路侧道具**（`bl/scenes/prop_mesh.py`，纯程序化、确定性）：
  - 行人：路肩两侧交错站立，`animate_pedestrians` 打开时沿路肩**行走**（逐帧关键帧，
    该模式关闭跨帧持久化缓存，是动态内容用例）；
  - 树、路灯（灯臂转向路面上方）、路牌：按弧长均匀分布；**落在停车位上的路灯会自动挪到
    两车之间的空隙**，不会杵进车头。
  - 停放的车只在垂直**停车位**里（见 §停车入库），不再另设路肩停车排。
- **车辆与相机**：车体 mesh 取自**共享**的 `bl/scenes/vehicle_mesh.py`——与 Drive Scene
  是**同一个 minibus**（同尺寸/造型/车轮位置），停车位里的车也用它，两场景不会画出不同的车；
  `ROAD_Vehicle` 空物体承载三维世界位姿（车与四台相机挂在其下，相机永远保持车体系安装位姿）；
  `ROAD_Cam_{Front,Back,Left,Right}` 由预设创建，内参/安装位姿与 AVM/Drive 同源；
  `[Sync Cameras from AVM Scene]` 支持单向抄取。
- **光照**（与 Drive Scene 同一套"平光"）：**暗世界背景 + 一盏方向光（太阳）**——
  太阳辐照度在整条环线上处处相同，不会像大面积面板那样在环内烧出热斑；默认 `light_energy=4`，
  **不投影（无阴影）**。录制时临时屏蔽非本场景灯光（与 Drive Scene 相同的还原机制）。

## 6. 参数与面板

两个面板（Scene Properties + 3D 视口 N 侧栏 `Road Scene`），只在场景已建立时出现：

- **概览**：环线总长 / 段数 / 逐帧位移 / 计划摘要；
- **Track**：`track_preset`（compact / full / custom）/ `straight_length` / `curve_radius` / `curve_bank_deg` /
  `ramp_rise` / `ramp_length` / `road_width` / `shoulder_width` /
  `ground_texture`（asphalt / concrete / epoxy / checker / plain）/ `parking_bays` /
  `parking_speed`。**标线、斑马线、停车入库都是常开的，面板不再给开关**；
- **Roadside**：`pedestrians` / `trees` / `lamps` / `signs` /
  `animate_pedestrians` / `pedestrian_speed`；
- **Lighting**：`light_energy`（太阳强度，默认 4）；世界背景固定为暗环境，太阳**不投影**
  （无阴影），与 Drive Scene 的平光一致；
- **Vehicle**：长 / 宽 / 高 / 离地间隙（默认值来自 `core/scenes/vehicle.py`）；
- **Drive**：`drive_speed`（巡航，默认 7 m/s ≈25 km/h）/ `drive_profile`（默认 scenario）/
  `slow_speed`（斑马线速度，默认 3.5 m/s ≈13 km/h）/ `drive_accel`（默认 2.5）/
  `drive_decel`（默认 2.5）/
  `drive_fps` / `drive_direction`（forward / reverse）/ `drive_segment`（整圈或某段）/
  `drive_loops`；
- **Record**：与 Drive Scene 完全相同的 `clip_quality` / `clip_device` / `clip_keep_frames`
  与逐帧模态导出（进度 + ETA + ESC 取消）；
- **动作**：`[Rebuild]` `[Reset Defaults]` `[Frame View]` `[Sync Cameras from AVM Scene]`
  `[Export Clip…]` `[Remove Road Scene]`。

## 7. 模块划分

```text
core/scenes/road_track.py            # 闭合环线：段/位姿/闭合校验/default_track（纯 Python）
core/scenes/road_path.py             # 环线运动 + 三维真值 + 扩展 CSV + 通用相机位姿（纯 Python）
bl/scenes/clip_core.py               # 共享录制引擎（ClipProfile 参数化；Drive/Road 共用）
bl/scenes/vehicle_mesh.py            # 与 Drive Scene 共享的车体 mesh（同一 minibus）
bl/scenes/prop_mesh.py               # 路侧道具 mesh（行人/树/路灯/路牌）
bl/scenes/road_scene/
├── __init__.py                      # DEFINITION + register 编排（注册表加一行即可）
├── properties.py                    # scene.road_scene（环线 / 道具 / 光照 / 车辆 / 行驶 / 相机）
├── builder.py                       # 道路带+标线、道具、车模、四台相机、三维关键帧
├── controller.py                    # 去抖重建
├── recording.py                     # Road Scene 的 ClipProfile（v3 + motion/segments）
├── operators.py                     # opencv_cam.road_*（含模态 Export Clip）
└── ui.py                            # 两个面板
```

## 8. 测试

- `tests/test_core.py::test_road_track`：闭合、四种 road_type、居中、航向连续、
  坡道坡度变号、高度回零、左弯正曲率、`describe()` 覆盖全长、不闭合被拒、
  `s_curve` 净航向/净横移为零且双向弯曲；
- `tests/test_core.py::test_road_path`：整圈里程=环长、速度恒定、方向标签、
  上坡正坡度、倒车航向 +180°且坡度变号、按段驱动只落在该段、三维相机位姿在平路时
  退化为 `drive_path` 公式、CSV 表头/列数/标签/相机位姿、零里程、摘要；
- `tests/run_blender_tests.py::test_road_scene`：操作符与注册、对象命名、四台相机
  （Custom + bytecode + 同一 K + 不同安装位姿 + 挂在 `ROAD_Vehicle`）、跟踪闭合、
  关键帧与纯 Python 位姿逐帧一致、材质槽与程序化噪声、道具数量与父级、太阳+补光、
  **坡前斑马线**（位置在坡脚前、开关会增减白色面片、两端各一名行人）、
  **车辆信号**（转向随曲率且倒车翻号、档位 P/R/D、停车计划 P→R→P）、
  按段/倒车标签、静态场景持久化开关、`render_clip` 出 PNG + `frames.csv`（十五列契约）
  + `clip.json` v3（motion/segments/cameras/vehicle）、行人行走时关键帧且关闭持久化、
  `Reset Defaults` 与移除。

## 9. 分期

| 期 | 内容 |
| --- | --- |
| **P0（本文件）** | 闭合环线 + 直道/弯道/上下坡 + 路侧道具 + 四摄逐帧 + 三维真值与按段导出 + 共享录制引擎 + 测试/文档 |
| P1 | 动态交通流（对向车辆、交叉口） |
| P2 | 真值扩展（深度 / 实例分割 / 逐帧相机位姿文件），与 M7 `dataset_export` 合流 |
| P3 | 悬架俯仰/侧倾、滚动快门/运动模糊、非平面路面（路缘/起伏）|

## 10. 风险与注意点

- **坡道与固定平面假设**：本场景刻意让路面沿中心线抬高、弯道保持水平（`roll = 0`），
  这样 M5 才能把「固定平面 + IPM」的误差量级归因到坡度本身，而不是被横坡掩盖。
  坡度越陡、弯道越急，2D 世界画布的投影误差越大——这正是矩阵要测出来的「在界内/越界」。
- **闭合校验**：改坏模板会**立即报错**，不会静默产出一条断头路；自定义模板务必保证
  两对直道总长相等、圆弧成对。
- **动态内容与缓存**：`render.use_persistent_data` 会缓存静态场景；打开
  `animate_pedestrians` 时 profile 自动关掉它，否则行人会冻结在首帧。
- **渲染成本**：默认环线约 248 m，整圈 5 m/s、10 fps 约 500 帧 × 4 相机。
  先小尺寸试跑、`clip_quality=draft`、`clip_keep_frames` 关掉可把体积压回视频量级；
  按段导出（`drive_segment`）是取得短素材的推荐方式。
- **切片脚本**：`scripts/check_clip_reproducibility.py` 目前针对 Drive Scene 单路探针；
  Road Scene 的可复现性由 `test_road_scene` 渲染路径覆盖。
