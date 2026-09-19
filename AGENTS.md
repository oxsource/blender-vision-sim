# 项目宪法

本文件是仓库的**最高约定**，人和 AI 的每次改动都必须遵守；与其它文档冲突时以本文件为准。
每条只给结论，展开与理由见 `docs/architecture.md`（架构与开发约定）。

## 1. 分层

- `core/` 是纯 Python，**禁止 `import bpy`**；`bl/` 才能访问 `bpy` / `mathutils`。
- 插件内部**只用相对导入**（扩展模式下包名是 `bl_ext.<repo>.<id>`）。
- 插件自有 PropertyGroup 是参数的**唯一真源**；`camera.cycles_custom[...]` 只作写入目标。
- 任何写 Cycles 参数的路径都必须校验 `custom_bytecode` 非空（编译失败会**静默沿用旧着色器**）。

## 2. 版本适配（全局机制）

同一套代码支持 **Blender 4.5 LTS 与 5.x**。跨版本 API 差异必须按"机制"处理，而不是就地打补丁：

- 所有跨版本差异**只写在 `bl/compat.py`** 一个模块里，对外是**按意图命名**的函数。
- **特性探测优先**于版本号：读 RNA 枚举、`getattr` 试新形状并回退；**禁止 `bpy.app.version` 分支**。
- 其它模块**禁止**直接使用跨版本改名的标识符 / 算子 / 按名字取节点
  （`action.fcurves`、`BLENDER_EEVEE(_NEXT)`、`blend_method` / `surface_render_method`、
  `import_scene.*` / `wm.obj_import` / `export_scene.gltf`、`nodes["Principled BSDF"]` …）。
- 新增差异时只动 `bl/compat.py` 与它的用例；每个 shim 都要有测试，集成测试在 4.5 与 5.2 上各跑一遍。
- `tests/test_version_policy.py` 用 AST 静态强制本规则，并随 CI 运行。

## 3. 质量门

- 新增/修改行为都要有测试：核心逻辑进 `tests/test_core.py`，需要 Blender 的进 `tests/run_blender_tests.py`。
- 提交前跑 `scripts/run_tests.sh`（核心 + 版本策略 + 发布工具 + 无头 Blender）。
- 场景遵循 `bl/scenes/` 的注册表机制：一个场景 = 一个自包含模块 + 一条注册表记录。

## 4. 发版

- 版本唯一真源是 `addons/<id>/blender_manifest.toml` 的 `version`。
- 发版走 `scripts/version.sh`（npm version 风格）；CI 仅由 `v*` tag 触发，且不依赖 Blender。
