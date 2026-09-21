# 路线图

## 阶段目标

| 阶段 | 内容 | 验收标准 |
| --- | --- | --- |
| M1 ✅ | `opencv_camera` 插件骨架：OSL 着色器、参数下发、编译校验、自检算子 | 面板可用；自检误差 < 0.3 px（实测 0.06 px） |
| M2 ✅ | 标定文件导入导出（OpenCV YAML / ROS camera_info / Kalibr / JSON）、外参设置、镜头反推、分辨率换算 | 核心单测 + 集成测试全绿 |
| M3 ✅ | 工程化补齐：README/文档、`scripts/package.py`（可复现打包）+ `package.sh`、`scripts/version.sh`（npm version 风格发版）、GitHub Actions（ci + tag 自动出包发 Release） | `dist/opencv_camera-*.zip` 可直接安装；push/PR 跑测试，打 tag 自动发版 |
| M4 ✅ | 畸变模型扩展：OpenCV fisheye（equidistant，θ 多项式） | 自检通过（θ≈72° 误差 0.048 px、θ≈87° 误差 0.026 px）；导入导出保留模型标识 |
| M4b | 其余模型：thin-prism/tilted sensor（s1..s4）、双鱼眼/超广角（EUCM/DS） | 每个模型都有自检用例 |
| M5 | 端到端回归：渲染 → 角点/ArUco 检测 → `cv2.calibrateCamera` 反标定回环 | 反标定内参相对误差 < 1%，畸变系数趋势一致 |
| M5b ✅ | 交互补齐：`Add ▸ VisionSim`（Camera 四种模型 + Camera Scene）、预设加载、Live Apply、Euler 外参、Preview（Image Editor）、Recompile | 一键建相机 / 改参数即时生效 / 预览可看 |
| M5c ✅ | **算法场景框架 + AVM Scene**：`bl/scenes/` 注册表（Camera Scene 收编）、AVM 平面场景（地面 / 车 / 4 标定块 / 4 鱼眼相机）、尺寸控制器、覆盖评估、参数与 Falcon 配置导出 | 见 [`avm-scene.md`](avm-scene.md)：4 路预览 + 覆盖/可见性矩阵 + `Export Falcon` 打包 |
| M5d ✅ | **Drive Scene**：室内停车场行驶素材（**四摄**逐帧渲染 + 每帧车速/位姿真值 + `clip.json` v2 + `vehicle` 块）| 见 [`drive-scene.md`](drive-scene.md)：`[Export Clip]` 出每路一条 mp4 + PNG 序列 + 逐帧真值（车体与每台相机位姿）打包成 zip，关键帧与纯 Python 运动模型逐帧一致 |
| M6 | `camera_rig`：多相机刚体、同步渲染、多相机标定导出（含双目/HFOV 组合） | 双目极线几何验证通过；同步渲染输出可复现 |
| M7 | `dataset_export`：渲染 + 真值（位姿/内参/深度/实例分割），KITTI / COLMAP / EuRoC 布局 | 导出的数据集能被参考工具链直接读取 |
| M8 | `sensor_sim`：IMU/GNSS/LiDAR 轨迹与噪声（可选引入第三方仿真中间件） | 与视觉时间戳对齐；噪声参数可配置 |

## 已知待办

- [ ] 视口预览（Viewport Render）是否支持自定义相机：本次尝试用 Rendered shading 对比 **标准相机与自定义相机**，两者在无头 MCP 会话里都没有渲染画面（视口需要交互刷新），**结论未定**；当前预览走 Image Editor，UI 已注明该限制。
- [ ] `Add ▸ Camera` 菜单条目目前是四个模型各一条；后续可改成一条 + 弹窗选择模型/预设。
- [ ] DoF 与自定义相机组合：需要着色器输出 `position` 并读取 `cam:aperture_position`（`advanced_camera.osl` 有参考实现）。
- [ ] 运动模糊/滚动快门：按 `time` 采样，暂未实现。
- [ ] Blender 5.x 复核：`custom_mode` / `custom_shader` / `custom_bytecode` 命名与行为是否变化。
- [ ] 导出「渲染实际使用的内参」（分辨率换算后）以便回写 OpenCV 端流程。
- [ ] 标定文件 XML（OpenCV FileStorage XML）读写。
- [x] fisheye 模型的正/反解与着色器（θ 多项式 + Newton 反解）—— 已完成（`shaders/opencv_fisheye.osl`）。
- [ ] fisheye 的 `cv2.fisheye.calibrate` 端到端回环（M5）与超 180° 的 EUCM/DS 模型。
- [x] AVM Scene 真实地面 mesh：内置 Falcon 碗形 `unlit_round_bowls.glb` 作为 `AVM_Ground`
  （`use_ground_model` / `ground_model`，P5d）。
- [ ] **AVM Scene 后续**（见 [`avm-scene.md`](avm-scene.md) P7）：真实 GLB 车模、BEV 拼图、
  与外部角点检测/反标定工具的回环对接、以及按 `bl/scenes/` 框架新增 **DMS** 等场景。
- [ ] **Drive Scene 后续**（见 [`drive-scene.md`](drive-scene.md) §9）：
  ~~四路相机~~ **已实施**（[`drive-scene-multicam.md`](drive-scene-multicam.md)：相机与车体定义
  收进 `core/scenes/avm_cameras.py` / `vehicle.py` 单一真源、四台 `DRIVE_Cam_*`、产物契约 v2、
  可选的 `[Sync Cameras from AVM Scene]`），曲线路径与航向、
  可中断的模态录制与 mp4 已实施，剩深度/分割真值（与 M7 `dataset_export` 合流）。
- [ ] 渲染农场场景：支持 EXTERNAL 模式（`.osl`/`.oso` 落盘 + 相对路径），避免依赖 Text 数据块。

## 设计原则

1. **数学与 Blender 解耦**：能在 `python3` 下验证的东西不放进 `bl/`。
2. **可验证优先**：任何渲染相关特性都必须有「渲染 → 与解析模型比对」的测试，而不是仅凭肉眼。
3. **失败要响**：宁可报错中止，也不要静默产出用旧着色器渲染的结果。
4. **不污染用户场景**：仿真类操作在临时对象/临时设置上完成，结束后完整还原。
5. **真实标定可回环**：生成的图像必须能被真实相机标定/检测流程消费（同一套 `K, D` 语义）。