# AVM Scene 设计方案（评审稿 v3）

> 状态：**设计已按评审意见收敛，待最终确认后实现**。
> 参考布局：`filament_avm/tools/plane_scene_calib.html`（标定布场地 · 尺寸标定，BEV 俯视）。
> 权威实现：`mediapipe_avm_calib`（场地方程 / 标定流程 / 配置生成，见 §15）。
> 默认相机参数来源（**离线反算一次**，§6）：`filament_avm/configs/vehicle_avm_minibus.json`
> + `falcon/core/camera/camera_pose.cc`。

## 0. 评审结论（已确认）

| # | 决策 | 结论 |
| --- | --- | --- |
| A | 标定布 = 标定块 | **4 个规格相同的纯黑方块**（不用棋盘格），放在车四周、四对角；**布面即黑块本身**，不另设布面 |
| B | 参数存储 | **Scene 级** `scene.avm_scene`，面板无需选中对象；面板在 AVM Scene 建立后才显示（§5.2） |
| C | 3D 视口 | **提供 N 面板侧栏** |
| D | 黑块实现 | **几何面片**（真实四边形 + 独立材质，非程序化贴图） |
| E | 车模 | **立方体**，长宽 = `core`（可独立覆盖） |
| F | 相机内外参 | `K`/`D` 从 minibus 配置取；`location`/`rotation` 由**离线** `points_3d` + PnP 反算一次，固化成内置预设（§6.6） |
| G | 相机参数 | **每台独立** |
| H | `core` / 车长来源 | **面板滑杆设置**（不从 GLB 反推） |

---

## 1. 目标与范围

在 VisionSim 中新增 **AVM Scene**：一键搭出「真实场景的仿真版」——车辆、4 台鱼眼相机、
4 块标定布摆在真实的位置姿态上，并给出与 HTML 同款的**尺寸/位姿控制器**。

### 1.1 输入 / 输出契约（本设计的第一原则）

**输入**

| # | 输入 | 说明 |
| --- | --- | --- |
| 1 | **各部件的摆放位置姿态** | 场地尺寸（border / corner / inner / core）、车辆尺寸、**4 台相机的 `location` / `rotation`** |
| 2 | **相机内参** | 每台相机的 `K`（fx/fy/cx/cy）、`D`（k1..k4）、输出尺寸 |

**输出**

| # | 输出 | 说明 |
| --- | --- | --- |
| 1 | **各摄像头的预览图像** | 4 路鱼眼渲染（`Preview` / F12 / `avm_render_cameras` 落盘 PNG） |
| 2 | **场景描述导出** | 各元素的大小 / 位置 / 姿态 + 相机内参（§8 参数文件、§16 素材包） |

**明确不涉及**（这些属于外部工具，本场景不做）：

- **位姿解算（PnP）**、外参标定；
- 角点检测、AVM 拼接 / 鸟瞰变换等 AVM 算法；
- 相机安装是否合理的最优性求解。

> `filament_avm` 的 `points_3d + PnP` 反算**只在离线做一次**，产物是一套**内置默认参数**
> （§6），不是场景的运行时功能。

### 1.2 用途

1. **变尺寸仿真评估**：任意调节车辆/场地/相机位姿，实时看 **4 台鱼眼相机的视野与地面覆盖**，
   评估真实场地是否够用（覆盖是否完整、有没有盲区、标定块是否都看得见）。
2. **无实车时的素材生成**：输出真实场景所需的**场地参数**与**4 路摄像头照片**，
   为 AVM（标定/拼接）提供原始素材，供外部程序做角点检测与反标定。

→ 这两条决定了必须补上 **§16 覆盖评估与素材导出**（覆盖足迹 / 盲区 / 可见性矩阵 / 一键出图）。

元素（4 类 / 共 10 个对象）：

| # | 元素 | 数量 | 说明 |
| --- | --- | --- | --- |
| 1 | 地面 ground | 1 | 大平面，承载车模与标定块 |
| 2 | 车模 car | 1 | **立方体**占位（长宽 = `core`，高可调） |
| 3 | 鱼眼相机 fisheye camera | 4 | front / back / left / right，复用 `opencv_camera` fisheye 模型，**每台独立内外参** |
| 4 | 标定布 / 标定块 cloth=block | 4 | **规格相同的纯黑方块**（边长 `corner`），放在车四周四对角，位置由场地方程决定 |

> **标定布就是标定块**：不再有单独的布面平面，4 个黑块即 4 张「布」。
> `border` 因此不产生几何，只用于**场地范围**（地面网格 / BEV 边界 / `sceneW×sceneH`），与标定无关。

**非目标**：真实车模 GLB、位姿解算/标定算法、角点检测、AVM 拼接、多车/多场地实例。

---

## 2. 参考布局（HTML）语义

HTML 用一个嵌套环带方程描述整块场地（单位 cm，原点在场地中心）：

```text
sceneW = coreW + 2 * (borderW + corner + innerW)
sceneH = coreH + 2 * (borderH + corner + innerH)
```

从外到内四层：`border`（外边界留白）→ `corner`（四角标定块）→ `inner`（内边界间隔）→ `core`（车位核心区）。
HTML 明确写出 **`car ≡ core`**：车辆俯视占位 = 车位核心区。导出 JSON 与 `PlaneScenePerfs.Store` 一一对应：

```json
{ "border": "10x10", "corner": 50, "inner": "0x0", "car": "262x474" }
```

**本方案的差异**：`corner` 由「棋盘格」改为「纯黑方块」，其余语义不变。

### 2.1 用 minibus 配置验证布局（已跑通）

`vehicle_avm_minibus.json` 每台相机给了 8 组 `points_3d`（全部 z=0，即地面）。四台合并后：

```text
x ∈ {-2.4, -1.4, +1.4, +2.4}      y ∈ {-4.2, -3.2, +3.2, +4.2}
```

即 **4 个 1.0 m × 1.0 m 的方块落在场地的 4 个对角**（左前 / 右前 / 左后 / 右后），与 HTML 的 `corner`
语义完全一致；front 相机看到左前 + 右前，left 相机看到左前 + 左后……
参考项目的 `PlaneScenePerfs.Model.points(camera)` 就是这 8 个点的生成函数（§15.2），
**4 个对角方块即 4 张标定布**（评审决策 A）。

---

## 3. 几何模型

世界坐标沿用 Blender 约定（Z 向上，车辆朝 **+Y**），原点在车辆中心的地面投影处；
HTML 的 `(x, y)` 直接映射到 Blender 的 `(X, Y)`，`Z = 0` 为地面。

### 3.1 场地方程（`core/scenes/avm_layout.py`，纯 Python）

```text
coreHx = coreW / 200          # 车辆半宽 [m]
coreHy = coreH / 200          # 车辆半长 [m]
cInX   = coreHx + innerW/100  # 黑块内沿
cOutX  = cInX   + corner/100  # 黑块外沿
cInY   = coreHy + innerH/100
cOutY  = cInY   + corner/100
hx     = cOutX  + borderW/100 # 场地半宽
hy     = cOutY  + borderH/100 # 场地半长
```

### 3.2 4 个标定块（= 4 张标定布）的位置

标定块是 4 个边长为 `corner` 的正方形，位于场地的 4 个对角：

```text
                 x = -hx   -cOutX  -cInX   cInX   cOutX    hx
        ┌──────────────────────────────────────────────────────┐  y = hy
        │            ┌────┐                      ┌────┐         │  y = cOutY
        │            │ 左前│                      │ 右前│         │  y = cInY
        │            └────┘                      └────┘         │
        │                                                      │
        │                    core / car（车辆）                 │  y = 0
        │                                                      │
        │            ┌────┐                      ┌────┐         │  y = -cInY
        │            │ 左后│                      │ 右后│         │  y = -cOutY
        │            └────┘                      └────┘         │
        └──────────────────────────────────────────────────────┘  y = -hy
```

| 块 | x 范围 | y 范围 | 边长 |
| --- | --- | --- | --- |
| 左前 FL | [-cOutX, -cInX] | [cInY, cOutY] | `corner` |
| 右前 FR | [cInX, cOutX] | [cInY, cOutY] | `corner` |
| 左后 RL | [-cOutX, -cInX] | [-cOutY, -cInY] | `corner` |
| 右后 RR | [cInX, cOutX] | [-cOutY, -cInY] | `corner` |

- 块的外沿（`cOutX` / `cOutY`）与场地方程一致；`border` 只决定**场地范围** `hx` / `hy`，不改变块位；
- 4 个块边长都是 `corner`，保证「规格相同」；
- 与 `vehicle_avm_minibus.json` 的实测块位一致（|x| ∈ [1.4, 2.4]，|y| ∈ [3.2, 4.2]）。

### 3.3 高度方向

| 对象 | Z |
| --- | --- |
| 地面 | 0 |
| 标定块（= 布） | `block_lift`（默认 1 mm，避免与地面 z-fighting） |
| 车模 | `[0, carH]`，`carH` 默认 1.6 m |
| 相机 | 由内置预设给出（minibus 前相机 z ≈ 2.69 m） |

---

## 4. 参数模型（PropertyGroup，Scene 级）

存储：`bpy.types.Scene.avm_scene`（PointerProperty）。理由：HTML 控制器是「场景级面板」，
无需先选中对象；几何对象统一收进 `AVM Scene` 集合，可选挂到 `AVM_Root` 空物体便于整体移动。

> 这个选择同时决定了 **N 面板不需要绑定实体对象**（见 §5.2 B）：面板只要读 `context.scene.avm_scene`
> 即可，与当前选中/激活对象无关。`AVM_Root` 只是几何容器，**不承载参数**。

「是否已建立」由 `root: PointerProperty(type=bpy.types.Object)` 判定：建场景时写入指向 `AVM_Root`
的指针，对象被删除时 Blender 自动清空该指针 → 两个面板随之隐藏（§5.2）。

### 4.1 场地 / 车辆 / 地面 / 标定块

| 组 | 字段 | 默认 | 范围 | 对应 HTML |
| --- | --- | --- | --- | --- |
| 场地 | `border_w` / `border_h` | 0 / 0 cm | 0–200 | `border`（**仅场地范围，不产生几何**） |
| | `corner` | 100 cm | 10–200 | `corner`（= 标定块边长） |
| | `inner_w` / `inner_h` | 20 / 80 cm | 0–200 | `inner`（车缘到块内沿） |
| | `core_w` / `core_h` | 240 / 480 cm | 100–500 / 100–800 | `car`（= core，**滑杆设置**，决策 H） |
| 车辆 | `car_length` / `car_width` | 跟随 `core_h` / `core_w` | 可覆盖 | HTML 的 `car ≡ core` |
| | `car_height` | 1.60 m | 0.2–4.0 | —（HTML 无高度） |
| | `car_clearance` | 0.00 m | 0–0.5 | — |
| 地面 | `ground_w` / `ground_d` | 30 / 30 m | 2–200 | — |
| 标定块 | `block_lift` | 0.001 m | 0–0.05 | —（离地抬升） |
| 显示 | `show_ground/car/blocks/cameras/coverage` | 全 on | | 图层开关 |
| 只读 | `scene_w` / `scene_h` / `block_area` | 派生 | | 顶部 chip / stats |

默认值由 §6 的 minibus 求解结果填入（border 取 0，core 由滑杆给定，默认 2.4 m × 4.8 m）。

### 4.2 相机（每台独立，G）

**内参完全复用既有的自定义相机配置**：`K`（fx/fy/cx/cy）、畸变 `D`、输出尺寸都住在
`camera.data.opencv_cam` 上，仍由既有的 `CV Intrinsics` / `CV Presets` / `CV Output` 面板管理。
AVM PropertyGroup **不新增任何 K/D 字段、不覆盖那些面板**，只保存**安装位置与姿态**（Blender 原生表达）
与调度信息，用 `CollectionProperty` 存 4 条固定记录：

| 字段 | 说明 |
| --- | --- |
| `name` | front / back / left / right |
| `enable` | 是否参与渲染 |
| `location` (3) | 相机在车辆系（= Blender 世界系）中的安装位置 [m] |
| `rotation` (3) | 安装姿态，XYZ 欧拉角 [度]（Blender 原生，UI 可直接拖） |

> **参数只存位置 + 姿态，不存 OpenCV 的 R / t**。R/t 只是 `opencv_cam.pose` 内部用于
> 「射线 ↔ 像素」换算的中间量，由既有插件的 `euler ↔ R/t` 同步机制自动维护（§5.1），
> 不进入本场景的参数模型、也不进导入导出文件（§8）。
> **场景不做位姿解算**（§1.1）：改内参不会移动相机；要恢复默认位姿用 `[恢复默认位姿]`
> （重读内置预设 §6.4）。

另有一个 `active_camera` 枚举（默认 front）：指定**哪台驱动渲染分辨率**（因为四台的 `output` 会互相争抢
`scene.render.resolution_*`），其余相机只应用自身内参。

---

## 5. 控制器

### 5.1 交互模型：属性 → 去抖 → 幂等重建

与 HTML「拖滑块 → 立刻重绘 canvas」等价，在 Blender 里是：

```text
PropertyGroup.update 回调
        │  （拖滑块会触发几十次，必须去抖）
        ▼
avm_controller.schedule_rebuild(scene)      ← bpy.app.timers，0.2 s 去抖
        ▼
avm_builder.rebuild(scene, settings)        ← 幂等：只改 mesh/transform，不重建对象
        ├── 地面：改 plane 尺寸
        ├── 车模：改 cube 尺寸与高度
        ├── 标定块 ×4：重算 mesh（单个四边形面片）
        └── 相机 ×4：按 `location` / `rotation` 写物体变换（R/t 由插件自动同步）
```

- **去抖**复用 `bl/preview.py` 已验证的 `bpy.app.timers` 模式（`schedule_preview` / `_preview_timer`），
  后台模式 no-op。
- **幂等重建**：对象只创建一次（`AVM_Ground` / `AVM_Car` / `AVM_Block_FL…` / `AVM_Cam_Front…`），
  改参数只改尺寸与变换；只有标定块因 `corner` 变化而重算 mesh（4 个小 mesh，开销可忽略）。
  不破坏用户的选中状态、材质与父子关系。
- **单一真源**：相机位姿只由 `avm_scene.cameras[i].location / .rotation` 推导；
  写完 `camera.location` / `camera.rotation_euler` 后，R/t 由既有插件的
  `_update_euler` 回调自动同步进 `opencv_cam.pose`（无需本模块自己维护 R/t），
  并套用既有 `_SYNCING` 重入守卫，避免 update 回调成环。
- **用户手拖相机**（在 3D 视口里移动/旋转）时反向回填 `location` / `rotation`
  （纯参数编辑，不做任何解算）。

### 5.2 面板

两个面板都**只在 AVM Scene 已建立时出现**（与现有 `CV *` 面板 poll `context.camera` 的做法一致）。
入口只有 `Add ▸ VisionSim ▸ AVM Scene`，不靠面板当入口。

**判定「已建立」**：`avm_scene.root`（指向 `AVM_Root` 空物体的 `PointerProperty`）非空即存在。
Blender 在对象被删除时会自动把该指针清空，所以不会出现「面板残留但物体没了」；
poll 再兜一层名字检查：

```python
def _has_scene(context):
    settings = getattr(context.scene, "avm_scene", None)
    root = settings.root if settings else None
    return root is not None and root.name in bpy.data.objects
```

**A. Scene Properties（`avm_scene.root` 存在时显示）**

```text
Scene Properties
├── AVM Scene
│   ├── 概览：场地 480×840 cm | 标定块 x.xx m²      （只读）
│   ├── 场地：border / corner / inner / core 的数值 + 滑块（对齐 HTML 的 SPEC 分组）
│   ├── 车辆：长 / 宽 / 高 / 离地间隙
│   ├── 地面：尺寸
│   ├── 标定块：抬升
│   ├── 相机（仅布局）：active_camera + 四台条目（enable / location / rotation / [选中该相机]）
│   ├── 图层：地面 / 标定块 / 车辆 / 相机 / 覆盖 开关
│   └── 动作：[重建] [还原默认] [快速预设 ▾] [覆盖评估] [导入参数…] [导出参数…]
│            [生成 JSON] [应用 JSON] [移除 AVM Scene]
```

> **相机内参不在本面板**：`K`/`D`/输出尺寸完全复用既有的**自定义相机配置**
> （`CV Intrinsics` / `CV Presets` / `CV Output`，挂在 `camera.data.opencv_cam` 上，§4.2）。
> AVM 面板只负责**安装位置与姿态**；`[选中该相机]` 只是把该相机设为激活对象，
> 让用户顺手去改 `CV Intrinsics`，**不复制、不覆盖**那些面板。

**B. 3D Viewport N 面板侧栏（C）**

定位：**只控制各「大件」的布局属性**（尺寸 / 位置 / 姿态 / 图层），
**不含相机内参、不含参数导入导出**——那些留在 Scene 面板与既有 `CV *` 面板。

结论：**不需要绑定任何实体对象**（`AVM_Root` 只是几何容器、不承载参数），
但**只在 AVM Scene 已建立时显示**。参数存在 **Scene 级**（决策 B）：

```text
数据源            context.scene.avm_scene      ← 与选中/激活对象无关
面板声明          bl_space_type = 'VIEW_3D'
                  bl_region_type = 'UI'          （N 侧栏）
                  bl_category    = 'VisionSim'   （侧栏标签页）
                  bl_context     = 不用（那是 PROPERTIES 专用）
poll(context)     _has_scene(context)            ← 未建场景时整个侧栏标签页不出现
```

- Blender 的 `VIEW_3D / UI` 面板只要求一个 `poll`，**不要求 active_object**；只要
  `context.scene` 可读就能画，属性读写走 `context.scene.avm_scene`。
- 什么时候会「必须绑定对象」：只有把参数存到**根空物体**（`AVM_Root.avm_scene`）时才需要
  `context.active_object == AVM_Root` 才能找到数据。我们**没有**选这条路线（决策 B），所以不存在该约束。
- 若将来要支持**一个场景里多个 AVM 场地实例**，那时才需要改成「根空物体 + 面板 poll 认 active object」，
  或给面板加一个 `avm_scene.instances` 枚举来切换实例（届时再议，本期不做）。

布局（与 Scene 面板共用同一套 draw 辅助函数，只放**布局子集**）：

```text
N ▸ VisionSim ▸ AVM Scene            （仅在 AVM Scene 存在时出现）
├── 概览（只读一行：场地 480×840 cm）
├── 场地：border / corner / inner / core 滑杆
├── 车辆：长 / 宽 / 高
├── 地面：尺寸
├── 标定块：抬升
├── 相机布局：active_camera + 4 台的 location / rotation / enable
├── 图层开关：地面 / 标定块 / 车辆 / 相机 / 覆盖
└── 动作：[重建] [还原默认]
```

> - **相机内参**（K/D/输出尺寸）与**参数导入导出 / JSON 文本框 / 快速预设**都**只在 Scene 面板**，
>   N 侧栏不放（侧栏窄，且这些不属于「布局」）；
> - N 侧栏的 `[选中该相机]`（如有）同样只是切激活对象，引导到 `CV Intrinsics`。

### 5.3 快速预设（与 HTML 相同，尺寸参数）

| 预设 | border | corner | core | inner |
| --- | --- | --- | --- | --- |
| 默认（跨车） | 10×10 | 50 | 262×474 | 0×0 |
| 无外边界 | 0×0 | 50 | 262×474 | 0×0 |
| 大标定块 + 间隙 | 10×10 | 100 | 262×474 | 20×20 |
| SUV 宽体 | 20×20 | 60 | 300×520 | 10×10 |
| **minibus（默认，来自配置）** | 0×0 | 100 | 240×480 | 20×80 |

---

## 6. 默认参数的来源（离线反算一次，产物是内置预设）

> **本节不是场景的运行时功能**（§1.1）：场景只接受 `location`/`rotation`/`K`/`D` 作为输入。
> 这里描述的是**一次性、离线**地把 `filament_avm` 的 minibus 配置反算成一套**内置默认参数**，
> 供 `Add ▸ VisionSim ▸ AVM Scene` 直接使用。

### 6.1 反算流水线（离线脚本，已逐行对照 C++ 源码）

权威实现：`falcon/core/camera/camera_pose.cc: SolvePnP()` + `config.cc: ImagePoints2D()` + `ba_optimization.cc`。
本方案用 `scripts/solve_avm_defaults.py`（**开发期脚本，可用 cv2**）复刻它：

```text
输入：K (3x3)、D (k1..k4)、points_2d (8x2)、points_3d (8x3, z=0)、ba_opt

1) 2D 去畸变（fisheye）：
     pts2 = cv2.fisheye.undistortPoints(points_2d, K, D, P=K)
     —— 与 config.cc:130 完全一致（P=K，输出仍是该 K 下的像素坐标）

2) 可选 cy 精化（仅 ba_opt=True，即 left / right）：
     K' = K; K'[1,2] += cy
     目标：min_cy || projectPoints(points_3d, solvePnP(points_3d, pts2, K')) - pts2 ||
     用单参数 Levenberg-Marquardt（初值 cy=0，λ0=1e-3，h=1e-6，最多 100 次）
     —— 与 ba_optimization.cc: OptimizeCameraCy 一致

3) PnP（零畸变、迭代法）：
     rvec, tvec = cv2.solvePnP(points_3d, pts2, K', None, flags=SOLVEPNP_ITERATIVE)
     R = Rodrigues(rvec),  t = tvec
     —— 与 camera_pose.cc:117 一致

输出（**只用于生成默认参数**，见 §6.6）：
     相机内参 = 原始 K（未经 cy 精化）+ 畸变 D
     相机位姿 = 由 (R, t) 转成 Blender 的 location + rotation（XYZ 欧拉）
     坐标系：world = Blender 车辆系（X 右, Y 前, Z 上），
             camera = OpenCV 相机系（X 右, Y 下, Z 前）
```

> **关键修正（易错）**：`camera_pose.cc:30-37` 先把 **原始 K** 抄进 `value.K`（注释写明
> *"must be original K values before any optimization"*），随后才把 K 以**引用**传给 `SolvePnP`，
> 由 `ba_optimization.cc` 就地修改 `K[1,2] += cy`。因此：
> - **Blender 相机内参 = 原始 K**（minibus 四台 `cy=477.820` 相同）；
> - `ba_opt` 的 `cy` 偏移（left +45.71 / right +44.78）**只用于 PnP 求位姿**，绝不能写进相机内参。


### 6.2 校验：与 C++ 源码中记录的相机中心逐位吻合

相机中心 `C = -Rᵀ t`（车辆系）与 `camera_pose.cc: SolveOrbit` 注释里记录的实测值对比：

| 相机 | 本方案解出 C | C++ 注释实测 | 误差 |
| --- | --- | --- | --- |
| front | [-0.031, +2.467, +2.691] | [-0.030997, 2.466795, 2.690680] | < 1 mm |
| back | [-0.071, -2.437, +2.834] | [-0.071000, -2.436533, 2.833544] | < 1 mm |
| left | [-1.280, -0.025, +2.340] | [-1.279710, -0.025104, 2.340202] | < 1 mm |
| right | [+1.185, +0.044, +2.321] | [1.185362, 0.044024, 2.321480] | < 1 mm |

→ 流水线复刻正确。`ba_opt` 解出的 cy 偏移：left `+45.71`（cy=523.53）、right `+44.78`（cy=522.60）。

### 6.3 场地尺寸推导（points_3d → 场地方程）

| 量 | 来源 | minibus 取值 |
| --- | --- | --- |
| `corner` | 块边长 = x/y 相邻点距 | 1.00 m = 100 cm |
| `cInX` / `cOutX` | 块内/外沿 | 1.4 / 2.4 m |
| `cInY` / `cOutY` | 块内/外沿 | 3.2 / 4.2 m |
| `border_w/h` | points 不可推（块即场地最外沿） | 默认 0，可调 |
| `core_w` | **面板滑杆**（决策 H），默认参考 `steering_line.body_width` | 2.4 m |
| `core_h` | **面板滑杆**（决策 H） | 4.8 m |
| `inner_w` | `cInX − coreHx` | 1.4 − 1.2 = 0.20 m |
| `inner_h` | `cInY − coreHy` | 3.2 − 2.4 = 0.80 m |

> **不可分离性（重要）**：`points_3d` 只依赖 `core/2 + inner`（即 `cInX` / `cInY`）与 `corner`，
> **`core` 与 `inner` 的拆分对 `points_3d` 没有任何影响**。也就是说：
> - **标定相关几何**（标定块位置、PnP 默认位姿）只由 `(core/2+inner, corner)` 决定；
> - `core` 与 `inner` 如何拆分，只影响**车模尺寸**（与 `border` 一起决定场地范围），纯视觉；
> - 因此 `core` 必须由外部给定（车体尺寸），而不是从 `points_3d` 反推。

### 6.4 实现方式与依赖

| 环节 | 位置 | 方案 |
| --- | --- | --- |
| 反算脚本 | `scripts/solve_avm_defaults.py`（**开发期，可用 cv2/numpy**） | 解析 `vehicle_avm_*.json` → 去畸变 → cy 精化 → `cv2.solvePnP` → 转成 `location`/`rotation` |
| 产物 | `addons/opencv_camera/presets/avm_scene/default.json` | 场地尺寸 + 4 台相机的 `location` / `rotation` / `K` / `D`（**内置默认值**） |
| 场景运行时 | `core/scenes/avm_layout.py`（纯 Python） | 只做场地方程、`points(camera)`、读预设；**不含 PnP** |
| 可选：从配置读场地尺寸 | `core/scenes/avm_layout.py` | `points_3d` → `corner` / `cInX` / `cInY` 的纯算术反推（无 PnP） |

**为什么 PnP 不进 `core` / 不进场景**（§1.1）：

1. **职责**：场景只做「摆放 + 渲染 + 导出」，位姿解算属于外部 AVM 工具；
2. **依赖**：CI 只跑 `actions/setup-python`（无 numpy/cv2），Blender 也不带 cv2；
   离线脚本用 cv2 最省事，且反算只需跑一次；
3. **可复现**：产物是随包分发的 `presets/avm_scene/default.json`，场景行为与 cv2 版本无关。

**离线脚本的对照验证**：脚本自带校验（与 `camera_pose.cc` 注释里的相机中心比对 < 1 mm，
见 §6.2），并写成 `tests/test_release_scripts.py` 风格的用例（**无 cv2 则 SKIP**）。
本机（cv2 4.14.0）已验证流水线可行。

### 6.5 保真度说明（重要）

用解出的 `K, D, R, t` 反投影 `points_3d`，与原始 `points_2d` 的鱼眼重投影残差为：

```text
front 7.2 px | back 6.4 px | left 32.5 px | right 33.1 px
```

这是**源数据本身的残差**（2D 点为实拍检测值，8 点共面无法同时约束 6 自由度外参 + cy），
`filament_avm` 生产路径同样如此。结论：
- 相机**位置/朝向**与 filament_avm 一致（§6.2 已验证），可用于渲染与算法回归；
- 标定块在图像中的落点会有**数十像素**偏差，不能直接当作角点真值；
- 若将来需要像素级回环，需引入多视图/多位置 bundle adjustment（列为 P7，不在本期）。

### 6.6 反算的定位：离线一次，产物是内置预设

**场景里没有位姿解算**（§1.1）。反算只在开发期跑一次，把结果固化成随包分发的默认预设：

| 时机 | 行为 |
| --- | --- |
| 开发期（离线） | `scripts/solve_avm_defaults.py` 跑 §6.1 流水线 → 写 `presets/avm_scene/default.json` |
| `Add ▸ VisionSim ▸ AVM Scene` | 读内置预设里的 `location` / `rotation` / `K` / `D` 建场景，**不做任何解算** |
| 用户拖动相机 / 改角度 | 直接改 `location` / `rotation`（纯参数编辑） |
| 用户改内参 | **相机不动**（场景不解算；要新默认位姿就重跑离线脚本或导入预设） |
| 导入/导出参数文件（§8） | 只有 `location` / `rotation` / `K` / `D`，**没有 R / t** |

> R/t 是**离线脚本内部**的中间量，`location` + `rotation` 才是场景的参数真源。
> 原决议 #10 的「改内参自动重解」**因本节的收窄而作废**（场景不做解算）。

---

## 7. 材质与网格

| 对象 | 网格 | 材质 |
| --- | --- | --- |
| 地面 | 大 plane（1 面） | 深灰 + 可选程序化网格（对照 HTML 的 `地面网格` 图层） |
| 车模 | cube | 浅灰车身 |
| 标定块 ×4 | **几何面片**（独立四边形，抬高 `block_lift`） | 纯黑（对照 `--cloth-ink`） |

标定块按决策 D 用**真实四边形面片**（非程序化贴图）：位置/尺寸精确、可导出 GLB/FBX、便于后续检测。
每个块是独立对象 `AVM_Block_FL/FR/RL/RR`（**独立对象**，便于单独选中/替换/导出），
共 4 个，边长由同一个 `corner` 驱动，保证「规格相同」。
因为不再有单独布面，**浅色布面**由地面材质承担（保证黑块有足够对比度，利于后续黑色区域检测）。

---

## 8. 参数导入 / 导出

控制器**全部参数**（场地 / 车辆 / 地面 / 标定块 / 每台相机内外参）都必须能
**导出成文件、再导入还原**，并兼容 HTML 工具。

### 8.1 两种格式

| 格式 | `"format"` | 内容 | 用途 |
| --- | --- | --- | --- |
| 完整格式 | `"avm_scene"` | 场地/车辆/地面/标定块 + 4 台相机的 `location`/`rotation`/`K`/`D` + active_camera | Blender 端保存 / 恢复 / 回归 / CI |
| 精简格式 | `"plane_scene"` | 仅 `border` / `corner` / `inner` / `car` | 与 HTML `plane_scene_calib.html` 双向互通 |

完整格式把精简键**原样保留在顶层**，HTML 侧 `fromStore()` 直接可读；AVM 额外参数放 `avm` 段：

```json
{
  "format": "avm_scene",
  "version": 1,
  "border": "0x0",
  "corner": 100,
  "inner": "20x80",
  "car": "240x480",
  "avm": {
    "units": "m",
    "car_height": 1.6,
    "car_clearance": 0.0,
    "ground": "30x30",
    "block_lift": 0.001,
    "active_camera": "front",
    "cameras": [
      { "name": "front", "enable": true,
        "location": [-0.031, 2.467, 2.691],
        "rotation": [20.4, -1.1, 1.7],
        "K": [317.77563818112867, 318.0250964604786, 636.2327868307656, 477.8201435641188],
        "D": [0.08476733270570755, 0.043184113434448945, -0.037989564107367736, 0.009428162434166068] }
    ]
  },
  "meta": { "source": "vehicle_avm_minibus.json",
            "solver": "scripts/solve_avm_defaults.py (offline cv2 PnP)",
            "solved_at": "2026-09-17T00:00:00Z" }
}
```

- **相机只有 `location` + `rotation`（XYZ 欧拉，度），没有 R / t**（§4.2、§6.6）；
- 单位：顶层 HTML 键一律 **cm**（与 HTML 一致）；`avm` 段一律 **m**（与 Blender 内部一致），
  由 `units` 显式声明，导入时按声明换算；`rotation` 单位固定为**度**；
- `version` 用于向后兼容（未来字段增删按版本迁移）；
- `meta` **只写不读**，便于追溯求解来源；
- 精简格式可省略 `format`（缺省按 `plane_scene` 处理）。

### 8.2 两个入口

1. **面板内文本框**（对齐 HTML 的 `ioText`）：`[生成 JSON]` `[应用 JSON]`，适合快速复制/粘贴与调试，不落盘。
   Blender 面板没有多行文本控件（`StringProperty` 只能画单行），所以完整参数主要走文件对话框；
   文本框更适合精简格式与快速查看。
2. **文件对话框**（`ImportHelper` / `ExportHelper`，与 `CV Presets` 的 Import / Export 风格一致）：
   - 导出算子带 `format` 枚举 {完整 `avm_scene` / 精简 `plane_scene`}，**写 JSON**（精确往返）；
   - 导入算子自动识别完整/精简，支持 `.json` / `.yaml` / `.yml`（YAML 走内置的无依赖子集解析器）。

### 8.3 语义

- **精简导入**：只覆盖场地尺寸（border / corner / inner / car），车辆 / 地面 / 相机保持不变；
- **完整导入**：**先校验、再整体应用**，校验失败**不半套用**（避免场景处于半旧半新状态）；
  成功后回填全部面板控件并触发一次重建；
- **解析容错**：沿用 HTML 的 `sizeFrom()`（`"wxh"` / 数字 / `[w,h]` / `{width,height}`）；
- **往返稳定**：`导出 → 导入 → 再导出` 结果逐字节稳定（单测覆盖）；
- **相机**：完整格式的 `K/D` 从各相机的 `opencv_cam` **读取**（导出）/ **写入**（导入），
  不经过 AVM 自己的属性（§4.2）；`location` / `rotation` 写入 `avm_scene.cameras[i]`
  并驱动物体变换（R/t 由插件自动同步，不进文件，§5.1）；
- **默认值**：一键建场景时默认套用 minibus 的完整参数（即内置一份默认 `avm_scene` 参数）。

### 8.4 算子

| 算子 | 说明 |
| --- | --- |
| `opencv_cam.avm_export_params` | 文件对话框导出，`format` 枚举选完整 / 精简 |
| `opencv_cam.avm_import_params` | 文件对话框导入，自动识别格式 |
| `opencv_cam.avm_export_json` / `avm_apply_json` | 面板文本框的生成 / 应用（无对话框） |
| `opencv_cam.avm_reset_defaults` | 还原内置默认参数（重读 `presets/avm_scene/default.json`，含相机默认位姿） |
| `opencv_cam.avm_remove_scene` | 移除 AVM Scene（删对象 + 清空 `root` 指针，面板随之隐藏） |
| `opencv_cam.avm_render_cameras` | **导出 4 路渲染图**（决议 #13）：按各相机自身的 K/D 与输出尺寸渲染到 PNG 目录，供外部程序检测角点 |
| `opencv_cam.avm_analyze_coverage` | **覆盖评估**（§16）：算 4 台相机的地面足迹、并集/重叠/盲区、标定块可见性矩阵；生成贴地覆盖曲线 |
| `opencv_cam.avm_export_materials` | **一键出素材**（§16）：4 路 PNG + `plane_scene.json` + `avm_scene.json` + `coverage.json` + filament 兼容 config |

---

## 9. 模块划分与文件清单

放在 `opencv_camera` 插件内（不新建插件）：可直接复用 `camera_factory` / `apply` / `shader`，
且 Blender 扩展之间没有稳定的跨插件依赖机制。**目录结构见 §17**（场景统一收编到 `core/scenes/` 与 `bl/scenes/`）。

| 文件 | 状态 | 职责 |
| --- | --- | --- |
| `core/scenes/avm_layout.py` | 新增 | 纯 Python 场地方程、**`points(camera)` 生成契约（§15.2，逐字复刻 `PlaneScenePerfs.Model`）**、4 个标定块矩形、预设、Store JSON 编解码。**禁止 import bpy** |
| `core/scenes/avm_coverage.py` | 新增 | 纯 Python **地面覆盖足迹**（`ray_from_pixel` + 与 z=0 求交）、并集/重叠/盲区、标定块可见性矩阵（§16） |
| `bl/scenes/__init__.py` | 新增 | 场景注册表 `SCENES` + `register()` / `unregister()` 编排（§17.3） |
| `bl/scenes/base.py` | 新增 | `SceneDefinition`、`has_scene`、root/集合命名、`ScenePanelMixin`、公共算子模板 |
| `bl/scenes/debounce.py` | 新增 | 通用去抖定时器（从 `preview.py` 抽出，多场景共用） |
| `bl/scenes/camera_scene.py` | **迁移** | ← `bl/scene_builder.py`（行为/命名不变，§17.5） |
| `bl/scenes/avm_scene/properties.py` | 新增 | PropertyGroup（场地/车辆/地面/标定块 + **相机位姿**集合 + `root` 指针；**内参不在此**，§4.2）+ update 回调 |
| `bl/scenes/avm_scene/builder.py` | 新增 | 建/重建对象、材质、mesh；创建 `AVM_Root` 并回写 `settings.root`；4 台相机创建、内参与位姿写入；覆盖曲线对象 `AVM_Coverage_*` |
| `bl/scenes/avm_scene/controller.py` | 新增 | 去抖重建（复用 `bl/scenes/debounce.py`） |
| `bl/scenes/avm_scene/io.py` | 新增 | 参数序列化 / 反序列化：完整 + 精简格式、JSON/YAML、校验、版本迁移（§8） |
| `bl/scenes/avm_scene/coverage.py` | 新增 | 覆盖曲线对象 + 报告（调用 `core/scenes/avm_coverage.py`） |
| `bl/scenes/avm_scene/operators.py` | 新增 | `opencv_cam.avm_add_scene` / `avm_rebuild` / `avm_reset` / `avm_remove_scene` / `avm_import_params` / `avm_export_params` / `avm_export_json` / `avm_apply_json` / `avm_render_cameras` / `avm_analyze_coverage` / `avm_export_materials` |
| `bl/scenes/avm_scene/ui.py` | 新增 | Scene 面板 + 3D 视口 N 面板（两者 poll 均为「`root` 存在」，§5.2） |
| `bl/scene_builder.py` | **兼容转发** | `from .scenes.camera_scene import *`，保留一个版本后删除 |
| `bl/menus.py` | 改 | 遍历场景注册表生成 `Add ▸ VisionSim` 条目（§17.4） |
| `bl/icons.py` + `scripts/make_icon.py` | 改 | 新增 `avm_scene` 图标（4 角黑块 + 车辆俯视轮廓） |
| `__init__.py` | 改 | 注册编排加入 `bl.scenes`（相机属性之后、UI 之前） |
| `scripts/solve_avm_defaults.py` | 新增 | **离线**反算脚本（cv2）：`vehicle_avm_*.json` → `location`/`rotation`/`K`/`D` → 写预设 |
| `presets/avm_scene/default.json` | 新增 | 内置默认预设（场地 + 4 台相机位姿/内参），随包分发 |
| `tests/test_core.py` | 改 | `core.scenes.avm_layout` + `core.scenes.avm_coverage` 单测（零依赖） |
| `tests/test_release_scripts.py` | 改 | 离线脚本的产物与 `camera_pose.cc` 注释比对（**无 cv2 则 SKIP**） |
| `tests/run_blender_tests.py` | 改 | 建场景 / 改参数重建 / 相机内外参 / JSON 往返；**场景注册表与菜单条目**（§17.7） |
| `docs/roadmap.md`、`README.md`、`blender_manifest.toml` | 改 | 状态、说明、version bump（minor） |

---

## 10. 测试计划

| 层 | 用例 |
| --- | --- |
| `test_core.py` | 场地方程与 HTML 一致；**`points(camera)` 四组输出与 minibus 配置的 `points_3d` 逐点一致（§15.2）**；Store JSON 与 App `Size` 格式互认（`"WxH"`、int cm）；**4 个标定块矩形与 `points(camera)` 的 8 点一致、边长均为 `corner`**；`border` 不影响块位；**参数往返**（完整/精简 × JSON/YAML，导出→导入→再导出逐字节稳定）；`sizeFrom` 容错；精简导入只改场地；版本迁移；**内置预设 `presets/avm_scene/default.json` 可解析**；**覆盖足迹**（射线与 z=0 求交、边界闭合、无解时的退化处理）与**可见性矩阵**（标定块是否被各相机覆盖） |
| `test_release_scripts.py` | **离线反算脚本**：跑 `scripts/solve_avm_defaults.py`（cv2），断言解出的相机中心与 `camera_pose.cc` 注释一致（<1 mm）、渲染内参用原始 K、PnP 用 K'（§6.1）；**无 cv2 则 SKIP**（CI 不受影响） |
| `run_blender_tests.py` | `avm_add_scene` 建出 1 地面 + 1 车 + 4 标定块 + 4 相机且都在 `AVM Scene` 集合、`settings.root` 指向 `AVM_Root`；**面板可见性**：建前 `poll=False`、建后 `poll=True`、删除 `AVM_Root` 后 `poll=False`；改 `core_w` / `corner` 后标定块/相机随之更新；4 台相机是 `CUSTOM` + `opencv_fisheye.osl` 且已编译、内外参各自独立；**参数文件只有 `location`/`rotation`/`K`/`D`（无 R/t）**；**文件导入导出算子**（`avm_export_params` → 改参数 → `avm_import_params` 还原）；**`avm_render_cameras` 按各相机输出尺寸落盘 4 张 PNG**；**`avm_analyze_coverage` 生成 4 条贴地覆盖曲线**、**`avm_export_materials` 产出 5 类文件**；导入失败时不改场景；卸载无残留 |
| 手测 | 拖滑块实时重建、N 面板显隐、F12 看 4 路鱼眼畸变、图层开关、标定块是否落在预期位置 |

---

## 11. 实施分期

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| **P0 ✅** | **离线反算一次**：`scripts/solve_avm_defaults.py` + `presets/avm_scene/default.json` | ✅ 预设与 `camera_pose.cc` 注释一致（<1 mm）；`tests/test_release_scripts.py` 新增用例（无 cv2 则 SKIP） |
| **P1 ✅** | `core/scenes/avm_layout.py` + `core/scenes/avm_coverage.py` + 单测 | ✅ `python3 tests/test_core.py` 全绿（零依赖）；`points(camera)` 与 minibus 配置逐点一致、可见性矩阵符合预期 |
| **P1b ✅** | **场景框架收编**（§17）：`bl/scenes/{base,debounce}.py` + 注册表 + `menus.py` 改遍历；**迁移 `scene_builder.py` → `bl/scenes/camera_scene.py`**（含兼容转发） | ✅ 注册表列出 `Camera Scene`；菜单遍历注册表；`test_scene_registry` 覆盖注册表/shim/debounce；旧测试全绿 |
| **P2 ✅** | `avm_scene/{properties,controller,builder,operators}.py` + 图标：一键建静态场景 | ✅ `Add ▸ VisionSim ▸ AVM Scene` 出 10 个对象（地面/车/4 块/4 相机 + `AVM_Root`）；相机是已编译的 `opencv_fisheye.osl`，K/D/位姿取自预设；改参数 + `[重建]` 幂等更新；`test_avm_scene_builder` 覆盖 |
| **P3 ✅** | 控制器面板：Scene 面板 + 3D 视口 N 面板（`root` 存在才显示） | ✅ 两个面板均已注册且 `poll` 随建/删切换；N 面板**只放布局**（尺寸/位姿/图层），内参留在 `CV Intrinsics`；`[选中该相机]` 引导到内参面板；`test_avm_panels` 覆盖 |
| **P4 ✅** | 预设 / 图层 / **参数导入导出（完整 + 精简、文件 + 文本框）** / **导出 4 路渲染图** | ✅ `io.py` 完整+精简格式、JSON 精确往返、校验失败**不半套用**；`avm_apply_preset` 5 个预设；`avm_render_cameras` 按各相机输出尺寸落盘 4 张 PNG；`test_avm_io` 覆盖 |
| **P5 ✅** | **覆盖评估与素材导出**（§16）：覆盖曲线 + 可见性矩阵 + `avm_export_materials` | ✅ `avm_analyze_coverage` 出 4 条贴地 POLY 曲线 + 摘要/矩阵；`avm_export_materials` 产出 4 PNG + `plane_scene.json` + `avm_scene.json` + `coverage.json` + `vehicle_avm_scene.json`（骨架，`points_2d` 留空）+ `scene_spec.md`；`test_avm_coverage_and_export` 覆盖 |
| P6 | 文档（含 `docs/architecture.md` 目录树）、README、roadmap、版本号 | `scripts/run_tests.sh` 全绿 |
| P7（后续，可选） | 真实 GLB 车模、BEV 拼图、外部标定回环对接、**DMS Scene**（走 §17 的框架） | — |

---

## 12. 风险与注意点

1. **去抖必须做**：滑块 `update` 每帧触发，直接重建会卡死 UI；沿用 `preview.py` 的 timer 去抖。
2. **不污染场景**：重建只动 `AVM Scene` 集合内的对象。
3. **相机 pose 双向同步**：写 `matrix_world` 后回填 `opencv_cam.pose`，加重入守卫，避免回调成环。
4. **分辨率所有权**：四台相机共用 `scene.render.resolution_*`，用 `active_camera` 指定唯一驱动者。
5. **Z-fighting**：标定块与地面要有毫米级抬升（`block_lift`）。
6. **单位**：内部 m，JSON/面板 cm，边界处统一换算。
7. **CI 依赖**：`core` 不得引入 numpy/cv2；cv2 只出现在「可选对照测试」里。
8. **保真度**：相机位置/姿态与 filament_avm 一致，但图像落点有 6–33 px 残差（§6.5），文档需明确。
9. **扩展模式回归**：`sys.path` 与 `bl_ext.*` 两种加载路径都要跑（历史问题）。
10. **覆盖计算的退化**：鱼眼有效域外（θ>90°）与地平线以上（`d_z ≥ 0`）的方向无地面交点，
    足迹多边形可能开口；报告里必须标注，不能当成正常闭合区域算面积。
11. **覆盖评估的开销**：栅格化 + 射线求交随网格精度上升；网格步长可调（默认 2 cm），
    并随重建去抖，不要每帧重算。

---

## 13. 决议清单（全部已确认）

| # | 问题 | 结论 |
| --- | --- | --- |
| 1 | 标定块是「独立对象」还是「并入布面的 mesh」 | **独立对象**（`AVM_Block_FL/FR/RL/RR`），易选中/替换/导出 |
| 2 | 车长 `core_h` / `core_w` 的来源 | **面板滑杆**（决策 H），不从 GLB 反推；默认 240×480 cm |
| 3 | 地面是否要程序化网格 | 要（对照 HTML 的「地面网格」图层，便于看尺度） |
| 4 | 是否需要在 Blender 内直接渲染 4 路拼图 | 本期不做拼图，但**导出 4 路渲染图**（见 #13） |
| 5 | 一键建场景的默认参数 | 读内置预设 `presets/avm_scene/default.json`（场地 + 4 台相机位姿/内参），**不做解算** |
| 6 | 参数文件默认格式 | JSON（精确往返）为默认，YAML 作可读备选；两者都支持 |
| 7 | 完整格式是否内嵌源配置路径 | `meta.source` 记录，便于追溯默认值的来源 |
| 8 | 标定布形态 | **布 = 块**：4 个规格相同的纯黑方块放在车四周四对角（决策 A），无单独布面 |
| 9 | `core` 与车模的关系 | 车模长宽默认 = `core`（对齐 HTML 的 `car ≡ core`），可独立覆盖；`core` 不影响标定几何 |
| 10 | ~~改内参后是否自动重解位姿~~ | **作废**（§6.6）：场景不做位姿解算（§1.1）；改内参相机不动，`[恢复默认位姿]` 只重读内置预设 |
| 11 | 纯黑块的可检测性（只有外轮廓） | **本期只渲染**，不做检测；地面浅色、黑块无阴影，P2 目视验证轮廓可辨 |
| 12 | ~~纯 Python PnP 精度~~ | **转为离线**：PnP 只在 `scripts/solve_avm_defaults.py`（cv2）里跑一次，产物是固定预设；**不再是运行时风险** |
| 13 | 2D 检测回环 | **本场景不需要 2D 点**：只做 `points_3d` 生成；PnP 用配置里已有的 `points_2d`；仿真**导出 4 路图像**后由**外部程序检测对角点**。Blender 内不做检测、不存 `points_2d` |
| 14 | 相机朝向未做图像级验证 | **P2 渲染一张对照**：检查标定块投影落在 `points_2d` 附近（残差见 §6.5） |
| 15 | int cm 取整 | **面板/导出 int cm，内部 float m**；导出时取整，避免 `100.0` vs `100` |
| 16 | `ba_opt` 导致 left/right 固有偏差 | **文档写明，非 bug**（§14） |
| 17 | 相机内参归属与面板分工 | **内参复用既有自定义相机配置**（`camera.data.opencv_cam` + `CV Intrinsics/Presets/Output`），AVM 面板只管**安装位姿**；**N 面板只放各「大件」的布局属性**（尺寸/位置/姿态/图层），不含内参、不含导入导出 |

> **#13 的影响**：删掉原 P6 的「检测回环」；新增一个「导出 4 路渲染图」的小能力
> （`opencv_cam.avm_render_cameras`，按各相机自身的 K/D/输出尺寸渲染到 PNG 目录），
> 交给 `mediapipe_avm_calib` / `filament_avm` 那侧做角点检测与反标定。

---

## 14. 附：坐标与位姿速查

```text
HTML (x 右, y 上, 原点居中, cm)  ──►  Blender 车辆系 (X 右, Y 前, Z 上, m)
    x_cm / 100  ──► X        y_cm / 100  ──► Y

filament_avm:  solvePnP(points_3d, points_2d, K)  →  (R, t)
    world = Blender 车辆系（X 右, Y 前, Z 上）
    camera = OpenCV 相机系（X 右, Y 下, Z 前）
    ⇒ Blender 相机物体矩阵 = core/transform.object_matrix_from_opencv(R, t)   （无需额外 world_matrix）
    ⇒ 仅在离线脚本 `solve_avm_defaults.py` 里用一次，转成 location + rotation 后 R/t 即丢弃（§6.6）
```

minibus 默认参数（车辆系，单位 m / 度；这就是内置默认值，不是运行时输出）：

| 相机 | `location` (m) | `rotation` XYZ 欧拉 (°) | 俯角 |
| --- | --- | --- | --- |
| front | (-0.031, +2.467, +2.691) | (+20.4, −1.1, +1.7) | 69.6° |
| back | (-0.071, −2.437, +2.834) | (+23.9, +1.2, −179.6) | 66.1° |
| left | (−1.280, −0.025, +2.340) | (+48.0, +0.4, +89.6) | 42.0° |
| right | (+1.185, +0.044, +2.321) | (+42.2, +0.9, −91.0) | 47.8° |

（`rotation` 由 §6.1 解出的 R 经 `transform.object_matrix_from_opencv` 转成 Blender 相机物体矩阵后取欧拉角，
俯角 = 90° − `rotation.x`。）

**渲染内参**（四台相同，来自配置的原始 K，`cy` 不含 ba_opt 偏移）：
`fx=317.776, fy=318.025, cx=636.233, cy=477.820`；`D=[0.0848, 0.0432, -0.0380, 0.0094]`，1280×960。

**PnP 用的 K'**（仅求解默认位姿时使用，不写进相机）：`cy` left=523.526 / right=522.603（+ba_opt 偏移），
front/back 与原始 K 相同。

> 由此产生一个**固有现象**：left/right 用「原始 K 渲染 + K' 解出的位姿」，标定块在图像里会偏离
> `points_2d` 数十像素（§6.5）。这是复刻 filament_avm 生产路径的必然结果，不是 Blender 的错。

---

## 15. 参考实现对照（`mediapipe_avm_calib`）

标定工具 App（Android/Kotlin）是场地方程与标定流程的**权威实现**，HTML 工具与
`vehicle_avm_*.json` 都是它的产物。落地时以下契约必须**照搬**，不要自创。

### 15.1 权威文件

| 关注点 | 文件 | 说明 |
| --- | --- | --- |
| 场地方程 / `points_3d` 生成 | `persist/PlaneScenePerfs.kt: Model.points()/sizing()/mask()/bev()` | `points(camera)` 是 `points_3d` 的唯一真源 |
| 场地方程（另一份等价实现） | `ui/scene/PlaneSceneConfig.kt: PlaneSceneGeometry.of()` | 与 `Model.sizing()` 完全等价 |
| 参数持久化 / Store | `persist/PlaneScenePerfs.kt: Store` / `sink()` / `restore()` | `{border:"WxH", corner:int, inner:"WxH", car:"WxH"}` |
| Store 字符串格式 | `comm/Size.kt` | `toString() = "${width}x${height}"`，`from()` 按 `"x"` 切分 |
| 检测点 / 尺寸 / 检测参数 | `persist/CalibProjPerfs.kt` | 每相机 `points_2d` + `input_size` + `Factor` + `Orbit` |
| 内参 | `persist/IntrinsicPerfs.kt` | 每相机 K/D |
| 配置生成 | `util/JsonGenerator.kt: obtain()/build()` | `points_2d/3d/K/D/input_size/model_scene/bev_bound/orbit/enable/ba_opt` |
| 2D 角点检测 | `lib/AutoCalibrator.kt: detectCorners()` → native `FalconNative.corners()` | `Factor(area, x-range, y-range, black_ratio=0.7)` |
| PnP / 位姿 | `filament_avm/falcon/core/camera/camera_pose.cc` + `ba_optimization.cc` | 见 §6 |

### 15.2 `points_3d` 生成契约（必须逐字复刻）

`PlaneScenePerfs.Model.points(camera)`（`x1 = core/2 + inner`，`x2 = x1 + corner`）：

```text
front / back : (-x2, y2) (-x1, y2) ( x1, y2) ( x2, y2)
               (-x2, y1) (-x1, y1) ( x1, y1) ( x2, y1)
left  / right: (-x2,-y2) (-x2,-y1) (-x2, y1) (-x2, y2)
               (-x1,-y2) (-x1,-y1) (-x1, y1) (-x1, y2)

后处理：front/left 用原值；back/right 把每个点的 (x, y) 取负
输出：8 个 [x, y, 0.0]（单位 m，Blender 车辆系）
```

> 这与 minibus 配置里 front/back/left/right 四组 `points_3d` 完全一致（已核对）。
> **顺序即 PnP 的 2D↔3D 对应关系**，`core/scenes/avm_layout.py` 必须原样实现，不能只给「4 个矩形」。

### 15.3 标定流程（参考项目的完整链路）

```text
① 场地尺寸  ← 用户在 PlaneScene 面板/HTML 工具里调滑杆（border/corner/inner/core，int cm）
                ↓ Model.points(camera)
② points_3d  ← 由场地尺寸生成（不是从图像反解）
③ points_2d  ← 实拍 4 路图 → native 角点检测 → 人工微调（ProjectionCalibrator 放大镜）
④ K/D        ← 每相机内参（IntrinsicPerfs）
⑤ 生成 config.json（JsonGenerator.build）
⑥ 运行时 PnP（camera_pose.cc）→ 外参 → 渲染
```

**关键结论**：参考项目里**场地尺寸是输入，不是标定输出**；「标定」指的是
**角点检测 + PnP 求外参**。本方案对应到这条链路的**前半段**（决议 #13）：

| 步骤 | 本方案 |
| --- | --- |
| ① 场地尺寸 | 面板滑杆（决策 H），并可从 HTML/App 的 `plane_scene` Store 导入 |
| ② `points_3d` | `core/scenes/avm_layout.py` 的 `points(camera)` 生成（**唯一的生成物**，用于导出的 filament 骨架） |
| ③ `points_2d` | **本场景不需要**：`points_2d` 只在离线反算里用一次（§6），场景内**不检测、不存储** |
| ④ K/D | 从内置预设导入（每台独立），可在 `CV Intrinsics` 面板编辑 |
| ⑤ 默认位姿 | **离线**由 `scripts/solve_avm_defaults.py` 反算 → 写进预设（§6.4/§6.6） |
| ⑥ 渲染 | Cycles 渲染 4 路鱼眼图，`avm_render_cameras` 导出 PNG |

**回环在外部**：导出的 4 路 PNG 交给 `mediapipe_avm_calib` / `filament_avm` 侧做角点检测与反标定，
Blender 侧**不做解算、不做检测**（§1.1）。因此本方案只保留两个「纯参数」方向：

1. **离线反算**（开发期，§6）→ 从 filament config 反推场地尺寸 + 反算默认位姿 → 固化进内置预设；
2. **从 `plane_scene` Store 导入**（HTML/App 导出）→ 直接设场地尺寸 → 生成 `points_3d`。

### 15.4 导入时需要忽略的字段

`model_scene` / `bev_bound` / `orbit` / `mask_overlay` / `steering_line` / `glb_file` / `ibl_file` /
`filamat_path` 都是 filament 渲染侧的参数，Blender 场景不使用，导入时应**跳过而不报错**。

### 15.5 可复用的后续能力（P7，可选）

- 若将来想在 Blender 内做检测回环，可搬 `FalconNative.corners()` 的检测参数 `Factor`
  （area / x-range / y-range / black_ratio）对渲染图跑同样的黑色区域检测——**本期不做**（决议 #13）。
- `Model.mask(radius)` / `Model.bev()` 可作为地面 BEV 视图的参考边界。

---

## 16. 覆盖评估与素材导出（用途驱动，§1.1）

这是本场景的**主要价值**：变尺寸看 4 路视野、判场地是否够用；无实车时产出 AVM 原始素材。

### 16.1 地面覆盖足迹（coverage footprint）

**精确覆盖判定**（面积/可见性都用它）：把地面点反投回图像，判断像素是否落在图像内。
对鱼眼相机，地平线以下的地面点都满足 θ<90°，落在有效域内，所以这一步是精确的：

```text
d_world = P - C                                # 相机中心 -> 地面点
d_local = R_objectᵀ · d_world                  # 世界 -> Blender 相机局部
d_cam   = M · d_local                          # M = diag(1,-1,-1) -> OpenCV 相机系
(u, v)  = camera_model.project_point(d_cam, intr, dist)
covered = 0 <= u <= width and 0 <= v <= height
```

**可视化足迹**：沿图像矩形边界采样 N 个像素（默认 256），逐个转成贴地射线并与 `z = 0` 求交：

```text
d_cam   = camera_model.ray_from_pixel(u, v, intr, dist)
d_world = R_object · (M · d_cam)
t = -C_z / d_world_z          (需 d_world_z < 0，即朝下)
P = C + t · d_world
```

- 采样边界可取 **图像矩形边界**（`domain="image"`）或 **鱼眼有效域圆**
  （`domain="valid"`，`fisheye_valid_radius_px` 与图像矩形求交）；
- 退化处理：`d_world_z ≥ 0`（指向地平线以上）或 `t ≤ 0` 的方向**无地面交点**，
  该段被丢弃并把足迹标记为 **open**（前/后相机因为俯角不够大，图像上缘越过地平线，
  足迹天然是「开口扇形」）；报告里列出 `open_polygons`；
- 输出：每台相机 1 条贴地折线（世界系，单位 m），用于 3D 视口画覆盖曲线。

> 因此：**面积与可见性走 16.1 的精确反投影**，**曲线只用足迹折线做可视化**——
> 开口扇形不做多边形面积统计（`Footprint.area` 为 `nan`）。

### 16.2 评估量

| 量 | 说明 |
| --- | --- |
| 每相机覆盖面积 | 各自覆盖的地面面积（m²） |
| 并集 / 重叠 | 4 台覆盖的并集面积、≥2 台的重叠面积 |
| 盲区 | 场地内未被任何相机覆盖的地面面积 |
| 场地覆盖率 | `1 - 盲区 / 场地面积` |
| **标定块可见性矩阵** | 块 × 相机：块的 4 个角是否都落在该相机的图像内（✓/✗） |
| **场地够用判据** | 4 个标定块是否各自至少被 1 台相机完整看到（`CoverageReport.field_ok`） |

> 面积统计用**栅格化**：把场地按固定网格（默认 5 cm，可调）逐格调用精确覆盖判定，
> 简单、稳健、无需多边形布尔运算；精度由网格步长控制。

**已验证（minibus 默认参数）**：4 台相机的可见性矩阵为
`front_left{front,left}`、`front_right{front,right}`、`back_left{back,left}`、
`back_right{back,right}`，覆盖率 100%、`field_ok = true`。

### 16.3 展示

- 3D 视口叠加：贴地 `POLY` 曲线 `AVM_Coverage_Front/Back/Left/Right`（每台一色，抬升 2 mm，
  由图层开关 `show_coverage` 控制）；可选并集填充面；
- 面板：数值 + 可见性矩阵（✓/✗）+ 「场地是否够用」结论；
- 可选：正交俯视渲染出 BEV 图（P7）。

### 16.4 一键出素材 `avm_export_materials`

```text
<out_dir>/
├── front.png / back.png / left.png / right.png   # 各相机原生 K/D 与输出尺寸（决策 G）
├── plane_scene.json                              # HTML / App 兼容 Store（int cm）
├── avm_scene.json                                # 完整参数（§8.1）
├── coverage.json                                 # 足迹多边形 + 可见性矩阵 + 数值
├── vehicle_avm_<name>.json                       # filament 兼容骨架（见下）
└── scene_spec.md                                 # 人读：尺寸 cm/m、相机安装、评估结论
```

`vehicle_avm_<name>.json` 是**骨架**：写出 `points_3d`（由 `points(camera)` 生成）、
`K` / `D` / `input_size` / `enable` / `ba_opt`，**`points_2d` 留空**，
由外部程序在导出的 PNG 上检测后回填（决议 #13）。

### 16.5 实现与依赖

- 全部计算放 `core/scenes/avm_coverage.py`（纯 Python，复用 `core/camera_model` 的射线/有效域），
  因此可在 `python3` 下单测，不需要 Blender；
- 曲线对象与出图由 `bl/scenes/avm_scene/builder.py` / `operators.py` 负责；
- 覆盖评估随尺寸/位姿变化重算（并入 §5.1 的去抖重建流程，或按需点 `[覆盖评估]`）。

---

## 17. Scene 目录结构与多场景扩展

### 17.1 目标

`opencv_camera` 会持续新增「算法场景」（本次 **AVM Scene**，后续 **DMS** 等），
已有的 **Camera Scene** 也要一并收编。所以把「场景」提升为一等公民：

> **一个场景 = 一个自包含模块 + 一条注册表记录**；新增场景只增文件，不改既有代码。

### 17.2 目录结构

```text
addons/opencv_camera/
├── core/                                   # 纯 Python（禁止 import bpy）
│   ├── camera_model.py / transform.py / calibration_io.py / paths.py / presets.py   （现有，共用）
│   └── scenes/                             # 场景专属的几何/模型/IO
│       ├── __init__.py
│       ├── avm_layout.py                   # 场地方程 + points(camera) + Store JSON（§3/§15.2）
│       └── avm_coverage.py                 # 覆盖足迹 / 可见性矩阵（§16）
├── bl/                                     # Blender 集成
│   ├── properties.py / apply.py / shader.py / preview.py / icons.py /
│   │   panels_patch.py / ui.py             （现有，相机相关，不动）
│   ├── menus.py                            # 改为遍历场景注册表（§17.4）
│   └── scenes/                             # 每个场景一个模块/包
│       ├── __init__.py                     # 注册表 + register()/unregister() 编排
│       ├── base.py                         # SceneDefinition / has_scene / root / 公共面板骨架
│       ├── debounce.py                     # 通用去抖（从 preview.py 抽出复用）
│       ├── camera_scene.py                 # ← 迁移自 bl/scene_builder.py（行为不变，§17.5）
│       └── avm_scene/
│           ├── __init__.py                 # 元数据 + register 编排
│           ├── properties.py               # 场地/车辆/地面/标定块 + 相机位姿集合（内参不在此，§4.2）+ root 指针
│           ├── builder.py                  # 建/重建对象、材质、相机、覆盖曲线
│           ├── controller.py               # 去抖重建
│           ├── io.py                       # 参数导入导出（完整/精简）
│           ├── coverage.py                 # 覆盖曲线对象 + 报告
│           ├── operators.py                # opencv_cam.avm_*
│           └── ui.py                       # Scene 面板 + N 面板
├── presets/
│   └── avm_minibus.yaml                    # AVM 内置默认参数（§6.4）
└── icons/
    ├── camera_scene.png                    # 现有
    └── avm_scene.png                       # 新增（未来 dms_scene.png ...）
```

未来新增 DMS：只加 `bl/scenes/dms_scene/`（+ 如需 `core/scenes/dms_*.py`、`icons/dms_scene.png`），
**其余文件一律不动**。

### 17.3 场景注册表与基类（`bl/scenes/base.py`）

```python
@dataclass(frozen=True)
class SceneDefinition:
    id: str                 # "avm_scene"（snake_case，全局唯一）
    label: str              # "AVM Scene"（菜单/面板显示名）
    icon: str               # "avm_scene"（icons/<icon>.png，带内置回退）
    order: int              # 菜单排序
    add_operator: str       # "opencv_cam.avm_add_scene"
    root_name: str          # "AVM_Root"（根空物体）
    collection_name: str    # "AVM Scene"（集合）
```

`bl/scenes/__init__.py` 维护 `SCENES: list[SceneDefinition]`，`register()` 依次注册
各场景的属性/算子/面板，`unregister()` 反序；`menus.py` 遍历它生成菜单。

**基类提供的公共能力**：

| 能力 | 说明 |
| --- | --- |
| `has_scene(context, defn)` | 统一的「已建立」判定：root 指针非空且对象仍在（§5.2） |
| `root_pointer` / `collection` | 统一的根空物体与集合命名/查找 |
| `ScenePanelMixin` | Scene Properties + 3D 视口 N 面板的公共骨架（poll/标题/根指针/布局辅助） |
| 公共算子模板 | `rebuild` / `remove_scene` / `reset_defaults`，按 `scene_id` 参数化 |
| `debounce` | 通用去抖（把 `preview.py` 的 timer 模式抽出来，两个场景共用） |

### 17.4 菜单与图标

`menus.py` 不再硬编码条目：

```text
Add ▸ VisionSim
├── Camera                    （现有，四种模型）
├── Camera Scene              （注册表）
├── AVM Scene                 （注册表）
└── DMS Scene ...             （未来，注册表）
```

每个场景自带图标 `icons/<scene_id>.png`（`scripts/make_icon.py` 生成，无 UI 时回退内置图标）。

### 17.5 Camera Scene 迁移

| 现在 | 迁移后 |
| --- | --- |
| `bl/scene_builder.py`（`COLLECTION_NAME = "OpenCV Camera Scene"`） | `bl/scenes/camera_scene.py` |
| `operators.OPENCV_CAM_OT_add_camera_scene` | 迁到 `bl/scenes/camera_scene.py`（或改用公共 add 模板） |
| `menus.py` 里硬编码 `Camera Scene` 条目 | 由注册表驱动 |

- **行为与命名保持不变**：集合仍叫 `OpenCV Camera Scene`，对象仍叫 `CheckerCube` /
  `CheckerGround` / `ColorBlock*` / `KeyLight` / `SunLight`，`prepare_render` / `apply_and_build` 语义不变；
- `bl/scene_builder.py` 保留为**兼容转发**（`from .scenes.camera_scene import *`）一个版本，
  避免外部脚本与旧测试立刻失效；后续版本删除。

### 17.6 命名约定

| 项 | 规则 | 例 |
| --- | --- | --- |
| 场景 id | `<name>_scene` | `avm_scene` |
| 集合 | `<Name> Scene` | `AVM Scene` |
| 根空物体 | `<NAME>_Root` | `AVM_Root` |
| 场景属性 | `Scene.<id>` | `scene.avm_scene` |
| 面板 | `OPENCV_CAM_PT_<id>` | `OPENCV_CAM_PT_avm_scene` |
| 算子 | `opencv_cam.<id>_<action>` | `opencv_cam.avm_add_scene` |
| 图标 | `icons/<id>.png` | `icons/avm_scene.png` |
| 核心模块 | `core/scenes/<name>_*.py` | `core/scenes/avm_layout.py` |

### 17.7 测试与文档影响

- `tests/run_blender_tests.py`：`from opencv_camera.bl import scene_builder` 改到新路径；
  断言集合名/对象名不变；新增「注册表列出全部场景」「菜单条目数 = 场景数 + 1」两条用例；
- `tests/test_core.py`：导入路径改为 `core.scenes.avm_layout` / `core.scenes.avm_coverage`；
- `docs/architecture.md` 的目录树与 `scripts/make_icon.py` 的图标表同步更新；
- 迁移作为独立提交（`refactor: collect scenes under bl/scenes`），便于回溯。
