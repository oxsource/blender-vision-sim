# 架构与开发约定

## 1. 分层：`core` / `bl` / `shaders`

```text
addons/opencv_camera/
├── __init__.py          register() / unregister() 编排，不写业务逻辑
├── core/                纯 Python，禁止 import bpy
│   ├── camera_model.py    内参、畸变、正投影与反投影（固定点迭代）
│   ├── transform.py       OpenCV ↔ Blender 坐标/矩阵/内参换算
│   ├── calibration_io.py  标定文件读写（含无依赖 YAML 子集解析器）
│   └── paths.py           包内文件定位（基于 __file__，兼容扩展/legacy 两种安装）
├── core/
│   └── presets.py          presets/ 目录的扫描与加载
├── bl/                  Blender 集成层
│   ├── properties.py      PropertyGroup / PointerProperty 定义 + Live Apply / 输出尺寸回调
│   ├── shader.py          OSL Text 数据块安装、编译校验、强制重编译
│   ├── apply.py           apply_values（快，供 Live Apply）/ apply_settings（含编译）
│   ├── camera_factory.py  Add Camera 的相机创建（模型 + 预设 + 可选 rig 空物体）
│   ├── preview.py         预览渲染（按标定宽高比、渲染设置用完即还原）+ 防抖定时器
│   ├── scene_builder.py   相机场景（棋盘方块/地面/灯光）
│   ├── selftest.py        渲染自检（隐藏其他对象，跑完还原）
│   ├── menus.py           Add ▸ VisionSim 子菜单（Camera / Camera Scene）
│   ├── icons.py           图标集载入（每个菜单项一个单色线条 PNG，带内置图标回退）
│   ├── panels_patch.py    隐藏 Cycles 自动生成的裸参数面板（可开关，卸载时还原）
│   ├── ui.py              五个顶层面板（bl_order -50..-46 最前）：CV Intrinsics
│   │                      （模型/内参/畸变/动作/状态）/ CV Extrinsics / CV Presets /
│   │                      CV Preview / CV Output
│   ── operators.py       算子：薄壳，只做 context 解析、调用 bl 逻辑、report
└── shaders/opencv_camera.osl   权威着色器源文件
```

为什么这样分：

- **可测试**：`core` 不依赖 Blender，`python3 tests/test_core.py` 就能跑（畸变往返、坐标变换、标定文件解析）。
- **可复用**：数据集导出、标定回环、参数校验工具都能直接 import `core`，不必启动 Blender。
- **可移植**：插件换成别的渲染器（或改为导出 OSL/GLSL）时，只需替换 `bl/` 与 `shaders/`。

## 2. Blender 规范要点（4.2+ Extension）

| 约定 | 做法 |
| --- | --- |
| 插件元数据 | 只写 `blender_manifest.toml`（`schema_version`/`id`/`version`/`type`/`blender_version_min`/`license`），不写 `bl_info` |
| 导入方式 | 插件内部**只用相对导入**；扩展模式下模块名是 `bl_ext.<repo>.<id>` |
| 资源路径 | 用 `__file__` 定位（`core/paths.py`），不要依赖 `__package__` 或当前工作目录 |
| 注册顺序 | `properties → operators → ui`，卸载时反序（UI 依赖算子/属性，属性依赖属性组） |
| 面板挂载 | 每个模块都是**顶层**面板（`bl_space_type='PROPERTIES'`、`bl_context='data'`、**不设** `bl_parent_id`），统一 `CV ` 前缀命名（`CV Intrinsics` / `CV Extrinsics` / `CV Output` / `CV Presets` / `CV Preview`）；用**负** `bl_order`（-50..-46）排到 Blender 自带面板（默认 0 / 1000）**之前** |
| 自定义相机入口 | **不能**扩展 `Camera.type`（C 侧 RNA 枚举）；用 `Add ▸ Camera` 菜单算子创建已配置好的 Custom 相机（`VIEW3D_MT_camera_add.append`） |
| 属性即时生效 | 属性 `update=` 回调 → `bl/apply.apply_values()`（只写 `cycles_custom`，快）；切换模型时走完整的 `apply_settings()`（要换着色器并重编译） |
| 双向同步要防递归 | `R/t` 与 Euler、相机物体三者互为镜像（`_update_pose` / `_update_euler`），用模块级 `_SYNCING` 重入守卫包住写回，避免 update 回调互相触发成环 |
| 插件入口 | 所有入口集中在 `Add ▸ VisionSim`（`Camera` 四个模型 + `Camera Scene`）；不额外往 `Add ▸ Camera` 里塞条目（Blender 不允许扩展 `Camera.type`，塞进去也只能建 Custom 相机，容易误导）。相机骨架用 `add_camera(use_rig=True)` 的选项而不是单独的菜单项 |
| 与别的插件共存 | 隐藏 Cycles 裸参数面板用的是**运行时替换 poll**（Python 面板的 poll 每次绘制都会重新查找），卸载时还原；Cycles 之后重新注册面板会导致补丁失效，此时面板会提示 "patch inactive"，功能不受影响 |
| 预览 | Blender 4.x 无面板内嵌图片 API（`template_preview`/`Image.preview` 已移除）→ 预览渲染后用 `bpy.ops.render.view_show()` 显示在 Image Editor；渲染设置与内参都要还原 |
| 分辨率所有权 | 输出尺寸由插件管理（`output.*`）并驱动 `scene.render.resolution_*`；面板/算子必须用同一处逻辑（`apply.output_resolution`），不要在别处硬编码分辨率 |
| 算子 | `bl_idname = "opencv_cam.<action>"`；需要撤销的加 `{'REGISTER','UNDO'}`；文件对话框用 `ImportHelper`/`ExportHelper` |
| 不污染用户场景 | 自检/预览类操作要保存并还原 `scene.render.*`、`view_settings`、`scene.camera`、`view_layer.objects.active`、各对象 `hide_render` |
| 持久化 | 安装的 OSL Text 数据块加 `use_fake_user = True`，随 `.blend` 保存 |
| 版本管理 | `blender_manifest.toml` 的 `version`；行为/接口变化时提升并记入 `docs/roadmap.md` |

### 2.0 属性注解的坑（会导致属性被静默丢弃）

bpy 会在注册时用 `typing.get_type_hints` 重新求值注解字符串，任何 `NameError` 都会让 Blender
**只打印一条警告并丢掉该属性**（表现为 `Convert py args to operator properties:: keyword ... unrecognized`）。因此：

- 注解里引用的名字必须在**模块作用域**可解析（例如别忘了 `from bpy.props import EnumProperty`）；
- 注解里避免复杂表达式（推导式等），把 items 提取成模块级常量；
- `items=` 传**函数**时，`default` 必须给整数索引；要让 `default` 用字符串标识符，items 必须是**列表**。

### 2.1 两个必须记住的 Cycles 行为

1. **编译失败会静默沿用旧字节码**。`custom_shader` 赋值后若 oslc 报错，`custom_bytecode` 保持旧值，渲染继续用旧着色器，Python 侧不抛异常（错误只写系统控制台）。因此 `bl/shader.py: ensure_compiled()` 是所有写入路径的必经关口，`tests` 里专门有一条「broken shader detected」用例。
2. **编译由 RNA update 回调触发**，回调里会查当前场景的渲染引擎；在脚本/无场景上下文中不一定触发。`bl/shader.py: force_compile()` 直接调用 Cycles 插件的 `osl.update_custom_camera_shader()` 作为兜底（注意这是 Cycles 内部 API，升级 Blender 后需回归）。

### 2.2 自定义图标

- 图标是 64×64 单色线条 PNG（`icons/<entry>.png`，由 `scripts/make_icon.py` 用 SDF 画线生成），
  刻意用最少笔画（眼睛+瞳孔、机身+镜头、方/圆 + 十字/弓形十字、立方体轮廓），保证 16 px 菜单尺寸可辨，用
  `bpy.utils.previews.new()` / `pcoll.load(name, path, 'IMAGE')` 载入，`icon_value` 用在**算子按钮**上；
- `bpy.utils.previews` 是惰性子模块，必须写 `import bpy.utils.previews`（直接 `bpy.utils.previews` 会 AttributeError）；
- `UILayout.menu()` 在 4.5 **支持 `icon_value`**，所以 `Add ▸ VisionSim` 这一级也用自绘图标（拿不到 id 时回退内置 `TRACKING`，避免和相机图标混淆）；
- 后台/无 UI 会话 `icon_id` 为 0（无效），`icons.operator()` 会自动回退到内置图标；
- 图标是**原创**线条标识（眼睛/相机/畸变网格/立方体/坐标轴），不要分发 OpenCV 官方 logo（商标）；
  每个菜单项一个图标名，`icons.kwargs(name)` / `icons.operator(..., name=...)` 统一处理回退。

## 3. 参数流向

```text
addons/opencv_camera/core/*            （数学、IO，纯 Python）
        ▲                                    │
        │ 读取核心模型对象                     │ 标定文件导入
        │                                    ▼
Camera.opencv_cam（PropertyGroup）  ← 唯一真源，用户/脚本都改这里
        │  Apply to Camera（算子）
        ▼
bl/apply.apply_settings()
   1. apply_render_resolution()    按 output.* 把输出尺寸写进 scene.render（可关）
   2. bl/shader.attach()           写/刷新 Text，Lens Type = Custom / Internal
   3. bl/shader.ensure_compiled()  校验 custom_bytecode（失败即报错中止）
   4. apply_values(): camera.cycles_custom[param] = value
                                    （参数由 Cycles 从 OSL 形参自动创建）
```

顺序不可颠倒：内参要按**当前渲染分辨率**换算，所以第 1 步必须在第 4 步之前。
`apply_values()` 是给 Live Apply 用的轻量路径（不碰着色器）；切换畸变模型走完整的
`apply_settings()`（需要换着色器并重编译）。

参数名与类型由 OSL 形参决定（float/int/bool/数组/字符串），列表在 `bl/apply.py: SHADER_PARAMS` 与 `shaders/opencv_camera.osl` 之间必须保持同步。

## 3.1 打包与发版

| 工具 | 说明 |
| --- | --- |
| `scripts/package.py` | 纯 Python 打包（zip 根目录放 `blender_manifest.toml` + 包内容），排除 `__pycache__`/`.pyc`/`.DS_Store`/`*.zip`，**固定时间戳**因此可复现（同源两次构建 sha256 相同）；同时输出 `.sha256` 并做清单基本校验（id 必须等于目录名、必填字段、`schema_version`） |
| `scripts/package.sh` | 默认转调 `package.py`（无需 Blender，输出 `dist/`）；`--blender` 时改用官方 `blender --command extension build`（权威校验，输出 `dist-official/`，两个目录**刻意分开**避免互相覆盖） |
| `scripts/version.sh` | npm version 风格：bump manifest 里的 `version` → commit `chore(release): vX.Y.Z` → 打注释 tag `vX.Y.Z`；`--push` 顺带推送；`--dry-run`/`--show`；脏工作区拒绝执行 |
| `.github/workflows/ci.yml` | 唯一的工作流，**仅 `v*` tag 触发**，**不依赖 Blender**：job `release` 校验 tag↔manifest → 快速检查（编译/核心单测/发布工具测试）→ `package.py` 出包 → `gh release create/upload` 附 zip + sha256 + artifact |

版本号的**唯一真源**是 `blender_manifest.toml` 的 `version`，CI 会断言 tag 与之匹配，避免"tag 与包不一致"。

> 坑：官方 `blender --command extension build --output-dir X` **要求 X 已存在**，否则报
> `FATAL_ERROR: Error creating archive ... No such file or directory`（且不会创建目录）。
> `scripts/package.sh --blender` 与两个工作流都会先 `mkdir -p`。

## 4. 测试策略

| 层次 | 文件 | 覆盖内容 |
| --- | --- | --- |
| 核心单测 | `tests/test_core.py` | 内参缩放、畸变正反解往返、投影往返、坐标变换不变式、镜头/主点换算、标定文件读写（OpenCV YAML / ROS camera_info / Kalibr / JSON） |
| 集成测试 | `tests/run_blender_tests.py` | 注册与面板、着色器编译、参数下发（含 float32 精度）、**坏着色器检测**、渲染自检、与自带透视相机逐像素等价、主点偏移等价、分辨率换算、位姿往返、算子端到端 |
| 手测 | Blender GUI | 面板交互、导入导出对话框、自检在真实场景中的表现 |

```bash
scripts/run_tests.sh                     # 全部
python3 tests/test_core.py               # 只跑核心
BLENDER=/path/to/blender scripts/run_tests.sh
```

集成测试通过 `sys.path` 直接导入插件包（最接近 legacy add-on 的加载方式），另有扩展安装路径的验证流程：

```bash
scripts/dev_install.sh
# 然后在 Blender 中 enable bl_ext.user_default.opencv_camera 并再次运行自检
```

> 扩展模式与 `sys.path` 模式的**代码路径相同但上下文不同**，两种方式都要跑一遍自检：曾经出现过「默认灰色世界 + film_transparent=False 导致自检质心偏到画面中心」的问题，只在扩展模式下暴露。

## 5. 新增插件时的清单

1. `addons/<id>/blender_manifest.toml`（`id` 与目录名一致，`blender_version_min = "4.5.0"`）；
2. `core/` + `bl/` 分层，`__init__.py` 只做注册编排；
3. 面板挂在合适的 `bl_parent_id` 下，算子命名 `<id>.<action>`；
4. `tests/test_core.py` 或 `tests/run_blender_tests.py` 增加用例；
5. `scripts/package.sh` 能构建出 `dist/<id>-<version>.zip`；
6. 更新根 `README.md` 插件表与 `docs/roadmap.md` 状态。