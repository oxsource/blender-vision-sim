# 路线图

## 阶段目标

| 阶段 | 内容 | 验收标准 |
| --- | --- | --- |
| M1 ✅ | `opencv_camera` 插件骨架：OSL 着色器、参数下发、编译校验、自检算子 | 面板可用；自检误差 < 0.3 px（实测 0.06 px） |
| M2 ✅ | 标定文件导入导出（OpenCV YAML / ROS camera_info / Kalibr / JSON）、外参设置、镜头反推、分辨率换算 | 核心单测 + 集成测试全绿 |
| M3 | 工程化补齐：README/文档、`scripts/package.sh` 打包、CI（无头 Blender 跑测试） | `dist/opencv_camera-*.zip` 可直接安装；CI 一键跑测试 |
| M4 | 畸变模型扩展：OpenCV fisheye（equidistant，θ 多项式）、thin-prism/tilted sensor | 每个模型都有自检用例；导入导出保留模型标识 |
| M5 | 端到端回归：渲染 → 角点/ArUco 检测 → `cv2.calibrateCamera` 反标定回环 | 反标定内参相对误差 < 1%，畸变系数趋势一致 |
| M6 | `camera_rig`：多相机刚体、同步渲染、多相机标定导出（含双目/HFOV 组合） | 双目极线几何验证通过；同步渲染输出可复现 |
| M7 | `dataset_export`：渲染 + 真值（位姿/内参/深度/实例分割），KITTI / COLMAP / EuRoC 布局 | 导出的数据集能被参考工具链直接读取 |
| M8 | `sensor_sim`：IMU/GNSS/LiDAR 轨迹与噪声（可选引入第三方仿真中间件） | 与视觉时间戳对齐；噪声参数可配置 |

## 已知待办

- [ ] 视口预览（Viewport Render / `render_preview`）是否支持自定义相机：需要实测，若不支持要在 UI 明确提示「仅 F12 渲染生效」。
- [ ] DoF 与自定义相机组合：需要着色器输出 `position` 并读取 `cam:aperture_position`（`advanced_camera.osl` 有参考实现）。
- [ ] 运动模糊/滚动快门：按 `time` 采样，暂未实现。
- [ ] Blender 5.x 复核：`custom_mode` / `custom_shader` / `custom_bytecode` 命名与行为是否变化。
- [ ] 导出「渲染实际使用的内参」（分辨率换算后）以便回写 OpenCV 端流程。
- [ ] 标定文件 XML（OpenCV FileStorage XML）读写。
- [ ] fisheye 模型的正/反解与着色器（θ 多项式 + Newton 反解）。
- [ ] 渲染农场场景：支持 EXTERNAL 模式（`.osl`/`.oso` 落盘 + 相对路径），避免依赖 Text 数据块。

## 设计原则

1. **数学与 Blender 解耦**：能在 `python3` 下验证的东西不放进 `bl/`。
2. **可验证优先**：任何渲染相关特性都必须有「渲染 → 与解析模型比对」的测试，而不是仅凭肉眼。
3. **失败要响**：宁可报错中止，也不要静默产出用旧着色器渲染的结果。
4. **不污染用户场景**：仿真类操作在临时对象/临时设置上完成，结束后完整还原。
5. **真实标定可回环**：生成的图像必须能被真实相机标定/检测流程消费（同一套 `K, D` 语义）。