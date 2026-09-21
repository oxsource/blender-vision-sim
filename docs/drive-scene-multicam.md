# Drive Scene 多相机改造方案（相机定义统一到 AVM Scene）

> 状态：**已实施（M1–M4，2026-09-21）**。本文保留为**设计与证据记录**，落地后的行为说明看
> [`drive-scene.md`](drive-scene.md)；两者冲突时以 `drive-scene.md` 为准。
> 动机（**2026-09-21 重述**）：不再挂在"透明底盘 540"名下 —— 540 方案已**挂起**
> （`filament_avm/docs/transparent_chassis_540_design.md` 头部说明），改为服务于**独立需求**
> 「仿真素材渲染通路（clip → AVM 渲染）」（`filament_avm/docs/sim_clip_render_pipeline.md`）。
> 四路 `drive_clip` 仍然是需要的交付物，只是不再是 540 的前置；`filament_avm` 侧要能
> 逐帧吃下这套产物并渲出正确的图，这件事本身先独立验证。
> 依据：`docs/drive-scene.md` §9 P1「四路相机」、`filament_avm/docs/sim_clip_render_pipeline.md` §8-D3。
>
> **落地结果（与本文的差异，逐条）**：
> 1. **四摄**：`DRIVE_Cam_{Front,Back,Left,Right}`，与 AVM 的 `AVM_Cam_*` 同构；相机与顺序来自
>    `core/scenes/avm_cameras.py`，AVM/Drive 两侧都读它。
> 2. **产物契约 v2 已落地**：`clip.json.version = 2`、`cameras[]`（含 `name` 对象名 / `camera` 键 /
>    `K`/`D`/`output`/`mount`）、`camera_names`、`frame_pattern`、`video_pattern`、`vehicle` 块；
>    `frames.csv` 为 `7 + 6 × N` 列且列名带相机名；每路一条 `<camera>.mp4`，多路时 `video` 为空。
> 3. **E5 改判**：默认**四路全开**（本文当时建议只开 `front`）。因为本次需求的目标就是产出 AVM 测试素材，
>    默认不能直接喂下游等于每次都要手动开；代价是渲染 ×4，用 `cameras[].enable` 关掉即可回到单摄。
> 4. **M4 已实施**：`[Sync Cameras from AVM Scene]`（单向、显式、非默认路径），把活的 AVM 相机
>    内参与位姿抄到 Drive 四台；可复现的真源仍是预设。
> 5. **§5.6.4 第 1 条改法**：`clip.json["vehicle"]["body"]` 是**名义车身盒**，不取 `car.dimensions` ——
>    生成的 mesh 里车轮外侧面比车身每侧凸出 0.11 m、灯箱比两端凸出 0.02 m，所以 `dimensions` 比车身盒
>    大（实测 2.62 × 4.84 × 2.885）。"证明取自真源"改为：从**车体材质面**的顶点量出车身板尺寸，
>    与导出的 `body` 比对（`tests/run_blender_tests.py::test_drive_scene`），这比读 `dimensions` 更准。
> 6. **顺带修掉一个既有缺陷（与四路无关）**：`encode_video()` 的新建序列场景默认 1920×1080，
>    而序列条按**原尺寸**绘制 ⇒ 非 1080p 的素材被渲成"黑底中间贴一张小图"（实测 96×72 的静帧
>    → 1920×1080 的 mp4，内容占 96×72）。已改为**读第一张静帧的尺寸来设定画布**，并加断言
>    "mp4 画布 = 渲染分辨率"。默认 1920×1080 时这个缺陷不可见，但消费侧按 `clip.json` 的 `K/D`
>    投影，画布错了就等于几何与像素不同尺度。
>
> 本文以下内容为**原始设计**（保留当时的取舍与证据），其中 §3 的方案 A/B、§4/§5 的契约、§6/§7 的
> 断言清单都已按上面的落地结果实现。

## 1. 目标与非目标

| | 内容 |
| --- | --- |
| 目标 | Drive Scene 携带 **AVM Scene 同款四摄**（`front/back/left/right`），并逐帧录出四路 PNG + 每路的世界位姿真值 |
| 目标 | **相机的内外参与安装位姿只有一个真源**：Drive Scene 不再持有/编辑任何相机参数（读预设，或从 AVM Scene 单向同步） |
| 目标 | 产物契约（PNG 命名 / `frames.csv` / `clip.json`）一次改到位，下游（透明底盘）不再有"单相机"特例 |
| 非目标 | 四路**严格同帧同步**的硬件时序语义（Blender 是逐帧串行渲染，天然同帧）；卷帘快门、悬架俯仰侧倾仍不做 |
| 非目标 | 不改 AVM Scene 的任何行为（AVM 侧只允许"把共享的相机命名/顺序上提到 `core/`"这类纯搬迁） |

## 2. 现状（代码事实）

### 2.1 相机只有一路，且是"半个真源"

| 位置 | 事实 |
| --- | --- |
| `bl/scenes/drive_scene/builder.py:33` | `CAMERA_NAME = "DRIVE_Cam_Front"` —— 只有前摄一个常量 |
| `builder.py:687-698` | `preset_camera(name)` 从 `avm_layout.load_preset()` 取记录 —— **内参已经来自 AVM 预设**，方向是对的 |
| `builder.py:709-727` | `_ensure_camera()` 只 `preset_camera("front")`，创建一台 |
| `builder.py:912-915` | `remove()` 只删这一台 |
| `bl/scenes/drive_scene/properties.py:1-10` | 文档已声明"内参不在这里，留在 `camera.data.opencv_cam`" —— 但**挂载位姿也没有 Drive 侧的开关**，等于靠"创建时抄一次" |
| `bl/camera_factory.py:43-51` | `configure_from_record()` 的 docstring 明确写着"AVM Scene 与 Drive Scene 通过这**一个**函数配置相机，所以内参不可能漂移" —— 已有的正确接缝 |

**结论**：Drive Scene 在**内参**上已经"不负责"了；缺的是 (a) 四路，(b) 挂载位姿同样显式地不负责，(c) 录制契约。

### 2.2 录制契约是单相机的（三处硬编码）

| 位置 | 事实 |
| --- | --- |
| `bl/scenes/drive_scene/recording.py:64-65` | `frame_name(index) -> "frame_%04d.png"` —— 无相机维度 |
| `recording.py:153-161` | `camera_mount()` 读**唯一**那台相机的局部位姿 |
| `recording.py:164-223` | `clip_meta()` 写 `"camera": {…}`（**单数**），只有一组 `K/D/mount` |
| `recording.py:266-288 / 397-409` | `ClipJob.start()` 抓一台相机、`step()` 每帧渲一张 |
| `recording.py:477-505` | `encode_video()` 用 `frame_name(plan.frames[0].index)` 建序列 —— 单序列 |
| `core/scenes/drive_path.py:27-29` | `CSV_HEADER` 里只有一组 `cam_x_m…cam_yaw_deg`（**13 列**） |
| `core/scenes/drive_path.py:199-213` | `csv_text(plan, mount)` 只接受一个 `Mount` |
| `tests/test_core.py:545-546` | 断言"每行 13 列" —— 契约被测试钉死 |

### 2.3 `"camera"` 单数形式的下游影响

`filament_avm` 侧的方案文档目前按单相机写了三件错事，改造后要一并修正：
`frames.csv` 的 `cam_roll/pitch/yaw_deg` 是 **Blender XYZ 欧拉角**（不是相机 pitch/yaw）；
它们只属于**一路**相机；且没有任何字段声明"这是哪台相机"。

## 3. 设计约束（本次给定）

1. 相机的**参数与位置保持与 AVM Scene 一致**；
2. 因此 **Drive Scene 不负责相机参数设置**。

第 2 条的含义要落到"单一真源"上，而不是"两处各写一遍然后靠测试对齐"。可选的两个层次：

| | 方案 A：**预设为真源**（现状的推广） | 方案 B：**活的 AVM Scene 为真源** |
| --- | --- | --- |
| 数据来自 | `avm_layout.load_preset()`（`presets/avm_scene/default.json`，由 `scripts/solve_avm_defaults.py` 离线反算） | 当前 .blend 里实际存在的 `AVM_Cam_*` 物体（`location` + `rotation_euler` + `opencv_cam`） |
| 优点 | 零顺序依赖；Drive Scene 可独立存在，可复现；与 `test_drive_matches_avm_defaults` 的现有断言一致 | 用户在 AVM 面板手调过的挂载位姿能"顺过来"，语义上最"一致" |
| 缺点 | 用户若在 AVM 里手调了相机，Drive 不会跟 | **先建 AVM 才能建 Drive**；两个场景互相耦合；`.blend` 换机器后结果可能不同 |
| 建议 | **采用 A**，并把方案 B 降级成一个**显式单向算子** `[Sync Cameras from AVM Scene]`（可选、非默认路径） | |

> 理由：Drive Scene 的定位是"可复现的算法素材生成器"（`docs/drive-scene.md` §1）。真源一旦变成"当前 .blend 的偶然状态"，同一组参数在别人机器上就录不出同一段素材，这比"少一个便利按钮"的代价大。
> 图省事的正确形态是**预设**：要改相机就改预设（或改完 AVM 后回写预设），两个场景同时跟着变。

## 4. 架构：把相机定义上提到 `core/`

现在"四摄有哪些、叫什么名"这件事**散在三处**：`avm_layout.CAMERAS`（名字顺序）、
`avm_scene/builder.py:66 CAMERA_SUFFIX`（名字→对象后缀）、`avm_scene/properties.py:32 CAMERA_ITEMS`（UI 枚举）。
要让两个场景共用一套相机，先把这件事收进 `core/`（纯 Python，可单测）：

```python
# core/scenes/avm_cameras.py （新增；或就近并入 avm_layout.py）

CAMERAS = ("front", "back", "left", "right")      # 顺序即录制顺序

def object_suffix(name: str) -> str: ...          # "front" -> "Front"
def object_name(prefix: str, name: str) -> str:   # ("AVM_Cam_", "front") -> "AVM_Cam_Front"
def enum_items() -> list: ...                     # [("front", "Front", "Front camera"), ...]
def mount_of(record: dict) -> Mount: ...          # 记录 -> 车辆系 Mount（位置 + XYZ 欧拉度）
```

改造后：

- `avm_scene/builder.py` 与 `drive_scene/builder.py` 都用 `core` 里的定义 → **不可能再漂移**；
- `avm_scene/properties.py` 的 `CAMERA_ITEMS` 改为由 `enum_items()` 生成（Blender 要求 items 是模块级列表，
  所以写 `CAMERA_ITEMS = avm_cameras.enum_items()`，符合 `docs/architecture.md` §2.0）；
- `drive_scene` 的相机对象名为 `DRIVE_Cam_{Front,Back,Left,Right}`，与 `AVM_Cam_*` 同构，
  两个场景仍可共存（`docs/drive-scene.md` §2 的既有约定）。

新增一个 Drive 侧的**相机选择**设置（记录用，不是参数）：

```python
# drive_scene/properties.py
active_camera: EnumProperty(items=CAMERA_ITEMS, default="front")   # 视口/单帧渲染用哪台
cameras: CollectionProperty(type=DriveCameraSettings)             # 每台一个 enable 开关
# DriveCameraSettings: name / enable（只有"录不录"，没有任何 K/D/位姿字段）
```

**边界**：`enable` 是**录制**设置（录哪几路），属于 Drive Scene；`K/D/output/mount` 是**相机标定**，一个字都不放在这里。
这条边界要写进 `properties.py` 的模块 docstring，和现在那句话并列。

### 4.1 与路线图 M6 `camera_rig` 的关系（重要）

`docs/roadmap.md` 的 **M6** 写的是「`camera_rig`：多相机刚体、同步渲染、多相机标定导出（含双目/HFOV 组合）」。
本方案的 M1/M2 **与它重叠**：都是"把相机定义收成一份、支持多台"。

两条路走法不同，必须现在选一条，否则会写出两套：

| | 走法 A：**按本方案直接做** | 走法 B：**先把 M6 的 `camera_rig` 抽出来再做** |
| --- | --- | --- |
| 做法 | `core/scenes/avm_cameras.py` 收敛相机定义；Drive Scene 直接用 | 在 `core/` 里建一个通用的"相机刚体"抽象（一组 `(name, mount, record)` + 顺序 + 遍历），`avm_cameras` 只是它的一个实例 |
| 代价 | 快，但"四摄"仍是 AVM/Drive 两个场景的**局部约定**；将来 M6 的双目/HFOV 组合要再抽一层 | 慢一步，但 M6 直接复用；双目/HFOV 组合只是同一抽象的另一个实例 |
| 风险 | 将来返工（很可能） | 现在多做一层设计（有过度设计的可能，因为 M6 的"同步渲染"需求尚未明确） |

**本文倾向 A，但限于一个约束**：`avm_cameras.py` 的职责只定义**"名字 + 顺序 + 一条记录 → 安装位姿"**这四件事，
**不碰渲染、不碰目录结构、不碰导出**。满足这个约束时，A 与 B 的差异只剩"要不要多一个类"，
将来 M6 抽 `camera_rig` 时把它当数据源包一层即可，不会返工。
反过来，如果 M2 顺手把"给每台相机各建一套 collection / 各渲染一份"这类渲染侧约定写进去，A 就会变成 B 的债。

> 这一条建议在实施 M1 之前先和该仓库的 M6 计划对齐一次；本文只把耦合面标出来。

## 5. 产物契约（本节是本方案最需要评审的部分）

四路一起录，产物必须自描述。三个候选：

| | 方案 i（推荐） | 方案 ii | 方案 iii |
| --- | --- | --- | --- |
| PNG | `frame_%04d_<name>.png`<br>（如 `frame_0000_front.png`） | `<name>/frame_%04d.png` 子目录 | `frame_%04d.png` + 相机子目录各一份 |
| 优点 | 平铺，`glob("frame_*_front.png")` 直接可用；`frames.csv.frame` 一列对齐；mp4 每路一条 | 每路一个文件夹，数量多时不挤 | —— |
| 缺点 | 文件名变长；下游要解析后缀 | 单目录浏览要进子目录；`zip` 解出来是树 | 与 i 等价但多一层，不选 |

**推荐 i**。理由：透明底盘要的是"同一帧的 4 张图"，平铺命名让"第 N 帧"这个维度保持是**文件名的主键**（排序、对拍、切片都最省事）。

### 5.1 `frames.csv` 列布局（推荐）

不再有单一的 `cam_*`，改成**每台被录相机一组 6 列**，顺序 = `avm_cameras.CAMERAS` 中 `enable` 的顺序：

```text
frame,time_s,distance_m,speed_mps,x_m,y_m,yaw_deg,
cam_front_x_m,cam_front_y_m,cam_front_z_m,cam_front_roll_deg,cam_front_pitch_deg,cam_front_yaw_deg,
cam_back_x_m, ... cam_back_yaw_deg,
cam_left_x_m, ... cam_left_yaw_deg,
cam_right_x_m, ... cam_right_yaw_deg
```

= `7 + 6 × N` 列。四路全开是 31 列。

- 列名里带相机名，**"这列是哪台相机"不再靠猜**；
- 显式 `roll/pitch/yaw` 之前加一句注释/文档说明它们是 **Blender XYZ 欧拉角**（`docs/drive-scene.md` §5 与
  `drive_path.CSV_HEADER` 的 docstring 都要写），这是 `filament_avm` 侧 R8 风险的正面解决；
- 每台相机的世界位姿由 `drive_path.camera_world_pose(frame, mount)` 逐个算出 —— 纯 Python，可单测。

签名改造：

```python
def csv_header(cameras: Sequence[str]) -> str          # 取代模块级 CSV_HEADER
def csv_text(plan_: Plan, mounts: Mapping[str, Mount]) -> str
```

> 兼容性：这是**破坏性变更**（原来 13 列）。但 `filament_avm` 侧目前只有方案文档、没有读取代码，
> 一次改到位成本最低。真要保兼容，退路是保留 `cam_*` = 前摄，另加 `cam_<name>_*` —— **不建议**：
> 两套列名并存，正是 R8 那类"靠猜"问题的温床。

### 5.2 `clip.json`（版本升到 2）

```diff
-  "camera": { "name": "DRIVE_Cam_Front", "model": "fisheye", "output": […], "K": […], "D": […], "mount": {…} },
+  "cameras": [
+    { "name": "DRIVE_Cam_Front", "model": "fisheye", "output": […], "K": […], "D": […],
+      "mount": { "frame": "vehicle", "location": […], "rotation_deg": […] } },
+    … 四路
+  ],
+  "camera_names": ["front", "back", "left", "right"],
+  "frame_pattern": "frame_%04d_<camera>.png",
```

`format` 保持 `drive_clip`，`version` 1 → 2（下游要能判别）。

### 5.3 视频

**契约变更（2026-09-21 定）**：视频从"附赠产物"升级为**主产物**。
消费侧（`filament_avm`）在运行时侧解码抽帧，所以本仓库**可以只输出视频**——
这样仿真输出与量产车机的输入形态（压缩码流）同构，消费侧的采集层从"读 PNG 序列"
退化成"读一路码流"。方案与实测见该仓库 `docs/transparent_chassis_540_design.md` §4.6。

每路一条 `<name>.mp4`（`front.mp4` …）。`encode_video()` 已经是"喂 PNG 序列给临时序列场景"，
扩成循环调用即可；`clip.mp4` 这个名字取消（单路时它就是前摄，四路时语义不清）。

**编码档位必须一并定**（否则消费侧建不了回归基准）。实测（68 帧 1920×1080，
RGB 域 PSNR 对无损 PNG）：

| 档位 | 体积 | PSNR（均值 / 最差帧） |
| --- | --- | --- |
| x264 CRF20 yuv420 帧间 | 1.03 MB | 46.71 / 45.63 |
| x264 CRF20 yuv420 **全 I 帧** | 3.35 MB | 46.78 / 45.90 |
| x265 CRF20 yuv420 | 0.61 MB | 47.03 / 45.89 |
| FFV1 rgb24（**真·字节级无损**） | 67.84 MB | lossless |
| *对照：现状 `clip.mp4`* | *1.73 MB* | ***32.61 / 32.28*** |

三条必须记住的规律：

1. **`yuv420p` 的天花板 ≈ 50 dB，是色度下采样决定的，换编码器无解**
   （连 YUV420 域无损也只有 49.62 dB）。要更高必须 `yuv444p`；
   要**真正字节级无损**只能用 FFV1/rgb24。
2. **帧间 vs 全 I 帧的保真度几乎相同**（46.71 vs 46.78 dB，差 0.07 dB），代价是 3.3× 体积。
   → **就用 Blender 自己的档位**（`constant_rate_factor = HIGH`，实测 46.72 dB / 1.57 MB）：
   消费侧是**顺序解码**，用不到 seek；Blender 的 FFmpeg 面板本来也不暴露 all-I 开关。
3. **逐帧最差帧只比均值低约 1 dB**，没有异常劣化帧。

**回归基准：不存，改为按需再生（2026-09-21 定）**。视频确实有损，所以"将来要能区分
算法退化与编码变化"这个需求是真的；但**满足它不需要存无损集**——
`scripts/check_clip_reproducibility.py` 实测同一段 clip 渲两遍**像素完全一致（Δ = 0 LSB）**，
只有 PNG 的 `tEXt` 墙钟字段不同（详见 §5.5）。所以 `clip.json` 里记全场景 + 编码处方，
需要时重渲一遍即可。**代价是比对必须用解码后的像素，而不是文件哈希。**

**`clip.json` 要记录编码元数据**：已落地为 **`video_encode`** 块
（`container / codec / constant_rate_factor / view_transform / look / fps / blender`）。
`blender` 字段是必需的那个——Blender 4.5 已移除 `bpy.app.ffmpeg_version`，
**编码器版本只能靠 Blender 版本号定位**（它绑定同捆的 FFmpeg）。

### 5.4 缺陷（**已修复**）：`encode_video()` 的视图变换不一致

**这是本次调查发现的既有缺陷，与多相机改造无关，但必须在"只输出视频"落地前修掉。**

- `drive_scene/builder.py:787` 把渲染场景的 `view_transform` 强制为 `Standard`，
  所以 **PNG 序列用的是 Standard 色调曲线**；
- 但 `recording.py::encode_video()` 为编码新建了**一个全新的临时场景**
  `bpy.data.scenes.new("__drive_encode__")`，而**新建场景的默认视图变换是 `AgX`**
  （本机 Blender 4.5.3 实测：`view_transform = "AgX"`，`look = None`）。

于是同一批 PNG 先被 AgX 转了一道再编码，产生**内容相关**的色调偏移：

| 原图亮度区间 | 像素占比 | 解码后平均偏移 |
| --- | --- | --- |
| 32–64（暗部） | 40.6% | −6.83 LSB |
| 96–128（中间调） | 11.2% | −1.54 LSB |
| 160–192 | 10.0% | −10.31 LSB |
| 192–256（高光） | 1.0% | **−33.39 LSB** |

复现与修正对比（同为 H.264 CRF20，唯一差别是临时场景的视图变换）：

| 路径 | 体积 | PSNR | 最差帧 |
| --- | --- | --- | --- |
| 现状（临时场景默认 AgX） | 1,812,384 B | 32.61 dB | 32.28 |
| **修正（临时场景设 `Standard`）** | **1,567,076 B** | **46.72 dB** | **45.62** |

**一行修复：体积 −14%，保真度 +14.1 dB。**

反证也成立：用 ffmpeg 直接从同一批 PNG 编码（绕开 Blender 临时场景），CRF20 得
**46.71 dB / 1.03 MB**——保真度与"修后的 Blender"一致。所以**问题在临时场景，不在编码器**。

**修复落地（2026-09-21）**：不硬编码 `"Standard"`，而是让 `encode_video()` **镜像渲染场景的
`(view_transform, look)`**（`ClipJob.start()` 时读取，作为参数传入）。镜像比再写一个常量更难漂移：
"编码场景与写盘时不一致"在构造上不再可能发生。常量 `OUTPUT_VIEW_TRANSFORM` / `OUTPUT_LOOK`
只作为无场景可镜像时的兜底（探针、直接调用）。

配套改动：

| 位置 | 改动 |
| --- | --- |
| `clip.json` | 新增 `video_encode` 块：`container / codec / constant_rate_factor / view_transform / look / fps / blender`。这是**编码处方**——消费侧要靠它区分"算法漂移"与"编码漂移"（见 §5.3 的修订） |
| `DriveSceneSettings.clip_keep_frames` | 新增开关（默认 **on**）。关掉后导出只留 `clip.mp4 + frames.csv + clip.json`，不再把 138 MB 的 PNG 装进 zip |
| `recording.ClipJob` | `keep_frames=None` 表示"取场景设置"，所以操作符 / 脚本 API / 测试三者自动一致；`_drop_frames()` 在编码之后、打包之前删除 PNG，并同步修剪 `report()["files"]` |
| 测试 | 新增 **9 条**：`clip.json` 记录编码处方与 `view_transform`、渲染-only 路径无处方、默认保留帧、lean zip 不含 png 且更小、lean 导出仍带 `video` 与处方、渲染-only 路径即使关掉开关也不丢帧、关帧模式下导出操作符可跑通 |

**算法层面的代价（在 `filament_avm` 侧实测）**：这个偏移不只是"帧看起来不对"。
把同一段素材按透明底盘的历史填充重建 BEV，AgX 那一路的 BEV PSNR 是 **29.33 dB**，
修复后是 **39.14 dB**（最差 8×8 块 31.74 → 10.05 LSB）。
详见 `filament_avm` 的 `docs/transparent_chassis_540_design.md` §4.6.6。

复现工具（在 `filament_avm` 仓库）：
`tools/chassis_poc/blender_encode_probe.py --vt Standard`，再用 `video_fidelity.py` 测 PSNR。

### 5.5 无损参考要不要存（**已定：不存**）

原判断是"必须保留一个小而无损的回归集"。`scripts/check_clip_reproducibility.py` 实测后
**改为不存**：同一段 clip 渲三遍（两次背靠背、一次中间做完整场景重建），

- **像素完全一致**：9 帧全部 Δ = 0 LSB（max 与 mean 都是 0）；
- **文件字节不一致**，差异只在 PNG 的 `tEXt` 元数据：`Date`、`RenderTime`、
  `cycles.ViewLayer.render_time`、`cycles.ViewLayer.total_time`（全是墙钟与耗时）；
  像素数据（IDAT）、`frames.csv`、`clip.json`（除 `created`）逐字节相同。

→ 无损参考**按需再生**即可（`clip.json` 已记全场景与编码处方）。两条必须写进流程的约束：

1. **比对解码后的像素，不要哈希 PNG 文件**——否则每次重渲都会因为 `Date` 报假阳性
   （这个探针第一次跑就是 1/9 的"不一致"，很容易被当成真问题）；
2. **再生的前提是场景参数完整落在 `clip.json` 里**：将来若引入随机种子 / 随机纹理 / 随机车型，
   必须一并记录，否则"可再生"这个前提就没了。

> **未验证**：只在 192×144 / 4 samples / 9 帧上测过。生产尺寸（1920×1080 / 24+ samples / 68 帧）
> 与换机器（不同 CPU/GPU、不同 Blender 小版本）都**没有测**。要写进 CI 需先补这两项。

### 5.6 车体几何进契约（**已定 2026-09-21**）

**问题**：透明底盘要把被自车遮住的地面补出来，第一步必须知道"哪些地面确实被车身挡住了"。
PoC 的做法是从渲染图里按**颜色**切出"贴底边的深青色连通域"（`chassis_analysis.py:111`），
设计文档自己把它标成"**作弊**"（`transparent_chassis_540_design.md:271`）。

**为什么不能就这么算了**：颜色阈值依赖**渲染出来的外观**——

- 换车身颜色 / 换材质 / 变光照 / 变曝光，阈值就得重新标；
- 量产车拿不到"自车是渲染物体"这个前提，只有 ISP 出来的图；
- 后果是**验证用的掩码与量产用的掩码不是同一套**，而设计文档把"掩码错一点，车底就会出现
  一条自己的车漆"列为**本方案最容易被低估的工作量**（其 §5.5）。

**决定**：仿真侧**导出车体几何**，消费侧按几何解算掩码。理由是与量产 v2 的路线
（"静态掩码 + 车辆 3D 模型实时投影"）**同源**，而不是绕开它。

#### 5.6.1 现状：车体几何散在三处，且导出侧一份都没有（代码事实）

| 位置 | 内容 | 用途 |
| --- | --- | --- |
| `avm_scene/properties.py:111,115` | `car_length=4.8` / `car_width=2.4` / `car_height` | AVM 车体 mesh（`builder.py:1051-1055`） |
| `drive_scene/properties.py:167-174` | `car_length=4.8` / `car_width=2.4` / `car_height=2.88` / `car_clearance=0.0` | Drive 车体 mesh（`builder.py:870`） |
| `core/scenes/avm_falcon.py:63-81` `STEERING` | `wheel_base=3.2` / `rear_track=1.8` / `body_width=2.4` / `rear_center_offset=2.8` / `length=[3.6,0,0]` | 转向线几何（发给 Falcon 的 `config.json`） |
| `clip.json` | **无** | —— |

两点结论：

1. **数字没有打架**：`body_width 2.4` 与 `car_width 2.4` 一致；相机安装高度 `2.69 m` 与车高
   `2.88 m` 也自洽（车顶下 0.19 m）。问题不是"谁算错了"，而是**没有单一真源、且出口没有**。
2. **`mask_overlay` 不能复用**：`filament_avm` 里的 `MaskOverlay`（`falcon/core/scene/mask_overlay.cc`）
   造的是**碗模型四路融合**的角度渐变遮罩（`refer` = 中心点 + 碗半径），不是自车掩码。
   名字像、用途不同——这条要写下来，因为它是最容易被顺手误用的地方。

#### 5.6.2 契约（拟）

`clip.json` 增加一个 `vehicle` 块，**车体系、米**，与既有 `camera.mount.frame = "vehicle"` 同系：

```json
"vehicle": {
  "frame": "vehicle",
  "body": { "length_m": 4.8, "width_m": 2.4, "height_m": 2.88 },
  "ground_clearance_m": 0.0,
  "axles": { "wheel_base_m": 3.2, "rear_track_m": 1.8, "rear_center_offset_m": 2.8 }
}
```

- `body` + `ground_clearance_m` 足够建 v1 的**静态轮廓掩码**（车体在鱼眼图中的固定轮廓）；
- `axles` 取自 `avm_falcon.STEERING`，让 v2 的 3D 投影有轮位可锚；
- **`frame: "vehicle"` 必须写出来**——与 §4.7「坐标系不钉死就一定出错」同一类风险；
- 块只有**一份**，不随相机复制：车体不随相机走。

#### 5.6.3 单一真源怎么落（本节真正的设计决定）

| | 做法 A：**`core/` 里定义车体几何（推荐）** | 做法 B：两个 properties 各留一份，导出时读 Drive 那份 |
| --- | --- | --- |
| 数据来自 | `core/scenes/vehicle.py` 一份常量（4.8 / 2.4 / 2.88 + 轴距 / 轮距 / 后轴偏移） | `drive_scene/properties.py` 的 `car_*` |
| 优点 | AVM mesh、Drive mesh、`clip.json`、将来 M6 的相机刚体都读同一份；"渲染的车"与"导出的几何"在构造上不可能不一致 | 改动最小 |
| 缺点 | 要动 AVM 侧（但只是纯搬迁，符合 §1"AVM 行为零变化"的约束） | 导出描述的是 **Drive 设置里的车**，而 `STEERING` 的轴距/轮距仍是另一份——**mask 要用的 `rear_track` 没有真源** |
| 建议 | **采 A**，并把 `avm_falcon.STEERING` 的 `wheel_base / rear_track / body_width` 改为读它 | |

> 边界同 §4.1：`vehicle.py` 只放**数字与坐标系**，不碰 mesh 生成、不碰渲染、不碰导出格式。
> AVM / Drive 的 `builder.py` 仍各自建 mesh，只是尺寸从同一处取。

#### 5.6.4 落地后必须能证明

1. `clip.json["vehicle"]["body"]["width_m"]` == Drive 车体 mesh 的**实际** X 向尺寸
   （从 `car.dimensions` 读，而不是把设置再抄一遍——抄一遍就等于没验证真源）；
2. `body.width_m` == `avm_falcon.STEERING["body_width"]`——同一真源的可测证据；
3. 改 Drive 车宽设置后 `clip.json["vehicle"]` 跟着变（证明读的是真源而非硬编码）；
4. 四路导出里 `vehicle` 块**只有一份**。

## 6. 逐文件改造清单

| 批次 | 文件（行号为 2026-09-21 实测） | 改动 |
| --- | --- | --- |
| M1 | `core/scenes/avm_cameras.py` | **新增**：`CAMERAS` / `object_suffix()` / `object_name()` / `enum_items()` / `mount_of()` |
| M1 | `core/scenes/avm_layout.py:52` | `CAMERAS` 改为从 `avm_cameras` 导入再 re-export（既有 import 不破） |
| M1 | `core/scenes/vehicle.py` | **新增**（§5.6.3）：车体尺寸 + 轴距/轮距/后轴偏移的唯一真源 |
| M1 | `bl/scenes/avm_scene/builder.py:35,66,908-935,1051-1055` | 相机与车体尺寸改为引用 `core`（**纯搬迁，行为零变化**） |
| M1 | `bl/scenes/avm_scene/properties.py:32,111-120,263` | `CAMERA_ITEMS = avm_cameras.enum_items()`；`car_size()` 读 `vehicle.py` |
| M2 | `bl/scenes/drive_scene/builder.py:33,709,800,870,904` | `CAMERA_NAME` → `CAMERA_PREFIX = "DRIVE_Cam_"` + 四台；`_ensure_camera` → `_ensure_cameras`；`_car_mesh` 尺寸读 `vehicle.py`；`remove()` 清四台；`view_targets()` 取全部相机（**C5：取景会变**） |
| M2 | `bl/scenes/drive_scene/properties.py:103` | `active_camera` + `cameras` 集合 + `recorded_cameras()`；docstring 写明"参数属相机标定，一个字都不放这里" |
| M2 | `bl/scenes/drive_scene/operators.py:45` | `_RESET_KEYS` 增补相机开关；报错文案 → "no cameras enabled" |
| M2 | `bl/scenes/drive_scene/ui.py:85` | Cameras 框：`active_camera` + 逐台 `enable` + 提示行 |
| M3 | `core/scenes/drive_path.py:27-29,199` | `CSV_HEADER` → `csv_header(cameras)`；`csv_text(plan, mounts)`；docstring 注明那三列是 **Blender XYZ 欧拉角** |
| M3 | `bl/scenes/drive_scene/recording.py:49-54` | `VERSION` 1 → 2；`VIDEO_NAME` 取消（改为每路 `<name>.mp4`）；`FORMAT` 不变 |
| M3 | `bl/scenes/drive_scene/recording.py:84,173-181,184-245` | `frame_name(index, camera)`；`camera_mount()` → `camera_mounts() -> Mapping`；`clip_meta()` 出 `cameras` 数组 + 新增 `vehicle` 块（§5.6.2） |
| M3 | `bl/scenes/drive_scene/recording.py:432-444,505-525,568-606` | `step()` 每帧渲 N 张（进度仍以"帧"为单位，相机在帧内循环）；`finish()` / `encode_video()` 每路一条 mp4 |
| M3 | `docs/drive-scene.md` | §1 目标表、§2 相机行、§5 产物三件套、§7 模块、§8 测试、§9 P1 状态 |
| M3 | `tests/test_core.py:545-546` | CSV 断言从"每行 13 列"改为 `7 + 6×N`；新增 `test_avm_cameras` |
| M3 | `tests/run_blender_tests.py` | `test_drive_scene` 录制断言改造（每帧 4 张 / CSV 表头 / `clip.json` v2 / `vehicle` 块）；`test_drive_matches_avm_defaults` 扩到四台逐项 |

总量约 **600 行**（含测试与文档，比加入 §5.6 前多约 50 行）。

### 6.1 断言清单（实施时逐条落，不打折）

**纯 Python（`tests/test_core.py`）**

1. `avm_cameras.CAMERAS == avm_layout.CAMERAS`，且 `object_name("AVM_Cam_", "front") == "AVM_Cam_Front"`；
2. `enum_items()` 与 `CAMERA_ITEMS` 逐项一致（值 / 显示名 / 描述）；
3. `csv_header(["front", "back", "left", "right"])` 的列名与列数 == `7 + 6×4`；
4. 每台相机的 6 个世界位姿列 == `camera_world_pose(frame, mounts[name])`（逐列、逐相机）；
5. 只录两路 → 列数 == `7 + 6×2`（证明"按实际录制集合出列"）；
6. `vehicle.py` 的体宽默认值 == `STEERING["body_width"]`（§5.6.4 第 2 条）。

**Blender 集成（`tests/run_blender_tests.py`）**

7. 四台 `DRIVE_Cam_*` 存在、`Camera.type == "CUSTOM"`、`custom_bytecode` 非空；
8. 小尺寸录 9 帧 → 每帧 **4** 张 PNG，文件名逐个符合 `frame_name()`；
9. `frames.csv` 表头 == `csv_header(recorded)`，行数 == 帧数 + 1；
10. `clip.json` 有 4 条 `cameras`、`version == 2`、每条的 `K/D/mount` 与该相机对象一致；
11. `clip.json` 有 `vehicle` 块、`frame == "vehicle"`，且 `body.width_m` == 车体 mesh 的 `dimensions.x`（§5.6.4 第 1 条）；
12. 关掉 `back.enable` → 只出 3 路、CSV 只 3 组、`cameras` 只 3 条；
13. 录两路时 `vehicle` 块仍**只有一份**（§5.6.4 第 4 条）；
14. **解码帧数 == `frames.csv` 行数**，四路**逐路**成立（把消费侧的 §4.6.4 契约在生产侧也钉一次）。

### 6.2 回滚点

| 批次 | 回滚方式 | 成立前提 |
| --- | --- | --- |
| M1 | 单提交 revert；`avm_cameras.py` / `vehicle.py` 是新增文件，删掉即回到现状 | AVM 侧**行为零变化**——现有 `test_drive_matches_avm_defaults` 全绿即证据 |
| M2 | 单提交 revert；四台相机是新增对象，`DRIVE_Cam_Front` 这个名字保留，旧素材仍可读 | 录制契约未动，M2 单独 revert 不留半成品 |
| M3 | **必须单独提交**：v1→v2 是破坏性变更，与 M2 混在一起就无法二分定位 | `clip.json` 的 `version` 是唯一判别位；下游尚无读取代码（C1），不需要并存两套列名 |

## 7. 测试改造（必须做到"能证明没漂移"）

1. `test_avm_cameras`（纯 Python）：四台的名字/顺序/`enum_items()`/`object_name()` 与 `avm_layout.CAMERAS` 一致。
2. `test_drive_path`：
   - `csv_header(["front","back","left","right"])` 的列名与列数 = `7 + 6×4`；
   - 每台相机的世界位姿列 == `camera_world_pose(frame, mount)`（逐列断言，逐相机）；
   - 只录两路时列数 = `7 + 6×2`（证明"按实际录制集合出列"）。
3. `test_drive_scene`：
   - 四台 `DRIVE_Cam_*` 存在、`Camera.type == "CUSTOM"`、`custom_bytecode` 非空；
   - 小尺寸录 9 帧 → 每帧 **4** 张 PNG、文件名逐一张符合 `frame_name()`；
   - `frames.csv` 表头 = `csv_header(recorded)`，行数 = 帧数 + 1；
   - `clip.json` 有 4 条 `cameras`、`version == 2`、每条的 `K/D/mount` 与该相机对象一致；
   - 关掉 `back` 的 `enable` → 只出 3 路、CSV 只出 3 组。
4. `test_drive_matches_avm_defaults`：**从"前摄一台"扩成四台逐项**（`K/D/output/model/挂载矩阵`），
   断言方式不变（AVM 世界位姿 ↔ Drive 车体系局部位姿）。这条是本方案的**核心回归**。

## 8. 分期

| 期 | 内容 | 可独立验证 |
| --- | --- | --- |
| **M1** | `core/scenes/avm_cameras.py` + 两场景引用 + `test_avm_cameras` + AVM 侧零行为搬迁 | ✅ 纯 Python + 现有 Blender 套件全绿（AVM 行为不变） |
| **M2** | Drive 侧四台相机（builder/properties/ui/operators）+ 四台对齐断言 | ✅ 不碰录制契约 |
| **M3** | 录制契约（`drive_path` 列布局 / `recording` / `clip.json` v2 / 每路 mp4）+ 测试 + 文档 | ✅ 录一段小尺寸素材端到端跑通 |
| **M4**（可选） | `[Sync Cameras from AVM Scene]` 单向算子 | ✅ 仅在需要时做 |

M1 与 M2 无下游影响，先落；M3 是破坏性契约变更，建议**单独一个提交**便于回滚。

## 9. 风险

| # | 风险 | 应对 |
| --- | --- | --- |
| C1 | M3 是破坏性契约变更，`frames.csv` 老列名消失 | 单独提交；`clip.json` 版本号升 2；下游（`filament_avm`）尚无读取代码，一次改到位 |
| C2 | 四路渲染成本 ×4（Cycles CPU） | 默认只开 `front`，`enable` 逐台打开；`clip_quality=draft` 已存在；先小尺寸试跑（`docs/drive-scene.md` §10 的老建议仍然适用） |
| C3 | `encode_video` 每路一条 mp4，导出时间增加 | **已定（E4）**：视频是**主产物**——消费侧在运行时解码抽帧，它才是算法吃的东西。四路时"每路一条"仍成立，但可用 `clip_keep_frames` 关掉 PNG 序列把导出体积压回视频量级 |
| C4 | 进度语义从"帧"变成"帧 × 相机" | `progress_text` 以"帧"为单位推进、相机在帧内循环，ETA 仍然可比；`step()` 内失败要能整体取消 |
| C5 | `view_targets()` 取 4 台相机会改变默认取景 | 保持取 `DRIVE_Car` + 全部相机（包围盒只是略变）；如需固定，退路是只取 `active_camera` |
| C6 | macOS 上 `clip_device=gpu` 仍回退 CPU（OSL 相机限制） | 与本次改造无关，但四路会把耗时放大 —— 素材生成建议放到有 NVIDIA 的机器（`docs/drive-scene.md` §10） |
| ~~C7~~ | ~~`encode_video()` 临时场景的视图变换（AgX）与渲染场景（Standard）不一致~~ | **已修复 2026-09-21**：`encode_video()` 改为**镜像渲染场景的 `(view_transform, look)`**，并补了 9 条断言（§5.4）。算法层面的代价也测了：BEV PSNR 29.3 → 39.1 dB |
| C8 | 车体几何散在三处（两个 properties + `avm_falcon.STEERING`），导出侧**一份都没有**（§5.6.1） | 按 §5.6.3 采做法 A 收敛到 `core/scenes/vehicle.py`。断言用**mesh 实际尺寸**而不是再抄一遍设置值 —— 抄一遍就等于把"抄写"当成验证 |
| C9 | `MaskOverlay` 名字像自车掩码，实际是**碗模型四路融合**的角度渐变遮罩，容易被顺手误用 | §5.6.1 第 2 条写明了用途差异；实施时若有人想复用它，先解释为什么角度渐变遮罩等价于车体轮廓 |

## 10. 待决策

| # | 议题 | 选项 | 本文建议 |
| --- | --- | --- | --- |
| E1 | 真源层次 | 预设 / 活的 AVM Scene | 预设（§3），AVM 同步降级为可选的显式算子 |
| E2 | PNG 命名 | 平铺后缀 / 子目录 | 平铺 `frame_%04d_<name>.png`（§5） |
| E3 | `frames.csv` 列 | 每相机一组 / 保留 `cam_*`+新增 | 每相机一组（破坏性，一次到位） |
| ~~E4~~ | ~~mp4 覆盖范围~~ | — | **已定 2026-09-21：视频为主产物**，每路一条 `<name>.mp4`；PNG 序列降级为可选调试输出（§5.3） |
| E5 | 默认录制路数 | 只 `front` / 四路 | 只 `front`（不改变现有单摄工作流与耗时，用户显式开） |
| E6 | 与路线图 M6 `camera_rig` 的关系 | 直接做 / 先抽 `camera_rig` | 直接做，但把 `avm_cameras.py` 的职责**限制在数据层**（§4.1），避免将来 M6 抽层时返工 |
| ~~E7~~ | ~~视频编码档位~~ | — | **已定 2026-09-21：用 Blender 自己的 `HIGH`**（实测 46.72 dB / 1.57 MB）。消费侧顺序解码用不到 seek，全 I 帧要多花 3.3× 体积（§5.3） |
| ~~E8~~ | ~~是否保留无损回归集~~ | — | **已定 2026-09-21：不留，按需再生**（实测像素完全可复现，只有 `tEXt` 墙钟字段不同）。约束：比解码像素而非文件哈希；场景参数必须记全（§5.5） |
| ~~E9~~ | ~~视图变换修复的落点~~ | — | **已定 2026-09-21：不硬编码，镜像渲染场景的 `(view_transform, look)`**。镜像比常量更抗漂移，且对"用户把场景改成别的曲线"也成立（§5.4） |
| **E10** | `clip_keep_frames` 的默认值 | 默认 on（现状） / 默认 off（只出视频） | **默认 on**，不改变现有导出契约；验证流程显式关掉。若确认下游只吃视频，再单独一次提交把默认翻过来 |

---

## 附：落地后需要回填的文档

- `docs/drive-scene.md`：§1 目标表（相机行）、§2（相机一行 + 与 AVM 的关系）、§5（产物三件套的新契约）、
  §7（模块划分）、§8（测试）、§9（P1 状态打勾）；
- `docs/roadmap.md`：Drive Scene 后续项（视频为主产物、AgX 修复）；
- `filament_avm/docs/transparent_chassis_540_design.md`：§3.1（`frames.csv` 列名说明改为四路显式列）、
  §10.1 R8（结案）、§10.3（4 路 clip 事项由"申请"变为"已具备生成路径"）。

## 附：本次调查用到的复现工具

在 `filament_avm` 仓库（消费侧）：

```bash
V=/Users/moks/.workbuddy/binaries/python/envs/avm_poc/bin/python
# 编码矩阵 + 逐帧 PSNR（§5.3 的表）
$V tools/chassis_poc/video_fidelity.py
# 视频 vs 单帧的算法级 A/B（§5.4 末尾的 BEV PSNR 数字）
$V tools/chassis_poc/video_vs_frames.py --video bazel-datasets/drive_scene/clip.mp4
# Blender 侧编码探针：复现 AgX 缺陷 / 验证修正（§5.4）
/Applications/Blender.app/Contents/MacOS/Blender -b \
  --python tools/chassis_poc/blender_encode_probe.py -- --vt Standard
```

在本仓库（仿真侧，§5.5 的依据）：

```bash
/Applications/Blender.app/Contents/MacOS/Blender -b --factory-startup \
  --python scripts/check_clip_reproducibility.py
```

`video_fidelity.py` 测的是**RGB 域** PSNR（先 `format=rgb24` 再比），
所以色度下采样的损失不会被 ffmpeg 的自动格式协商藏掉——这一点很关键，
否则 YUV420 域的"无损"会给出误导性的 52 dB。
