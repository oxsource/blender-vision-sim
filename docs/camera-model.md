# 相机模型：OpenCV 内参与畸变

本文记录 `opencv_camera` 插件的模型推导、坐标约定与实测验证数据。数学实现在
`addons/opencv_camera/core/camera_model.py`，着色器在 `addons/opencv_camera/shaders/opencv_camera.osl`。

## 1. Blender 侧能力（4.5.3 实测）

| 项 | 值 |
| --- | --- |
| 取景类型 | `Camera.type = 'CUSTOM'`（Lens Type = Custom），4.5 起即有 |
| 着色器绑定 | `custom_mode = 'INTERNAL'` + `custom_shader = <Text>`；或 `'EXTERNAL'` + `custom_filepath` |
| 编译产物 | `custom_bytecode` / `custom_bytecode_hash`（由 Cycles 插件在 RNA 更新时编译并回写） |
| 参数存储 | `camera.cycles_custom`（Cycles 注册的 PropertyGroup），键由 OSL 形参自动创建 |
| 可用属性 | `cam:sensor_size`、`cam:image_resolution`、`cam:focal_distance`、`cam:aperture_size`、`cam:aperture_aspect_ratio`、`cam:aperture_position` |
| 渲染后端 | 仅 CPU / OptiX |
| 缺失功能 | 无射线→像素逆映射（Vector pass、Window 纹理坐标不可用）；自定义相机下自适应细分有已知崩溃规避 |

着色器契约：输入 `camera_shader_raster_position()`（0–1，像素中心，Y 向上），输出
`position`（射线起点）、`direction`（**归一化**方向）、`throughput`（黑色即丢弃该射线）。
相机坐标系：+X 右、+Y 上、+Z 为视线方向。

## 2. 投影模型

### 2.1 前向（OpenCV）

相机系 X 右、Y 下、Z 前；`x = X/Z, y = Y/Z`：

```
r² = x² + y²
num = 1 + k1r² + k2r⁴ + k3r⁶
den = 1 + k4r² + k5r⁴ + k6r⁶
x_d = x·num/den + 2p1xy + p2(r²+2x²)
y_d = y·num/den + p1(r²+2y²) + 2p2xy
u = fx·x_d + cx ,  v = fy·y_d + cy
```

`k4=k5=k6=0` 即经典 5 参（`plumb_bob`/`radtan`）模型；非零即 rational 模型。
系数顺序统一为 `(k1, k2, p1, p2, k3, k4, k5, k6)`。

### 2.2 反向（着色器所需）

着色器拿到像素，需要还原射线方向，即求畸变逆（等价 `cv2.undistortPoints`）：

```
x = x_d, y = y_d
repeat N:
    icdist = (1 + k4r² + k5r⁴ + k6r⁶) / (1 + k1r² + k2r + k3r)
    dx = 2p1xy + p2(r²+2x²);  dy = p1(r²+2y²) + 2p2xy
    x = (x_d - dx)·icdist;    y = (y_d - dy)·icdist
```

实现要点：固定点迭代（默认 10 次）、`den` 过小或坐标发散时跳出、
`allow_off_sensor=0` 时把发散射线 `throughput = color(0)` 丢弃。

## 3. 坐标与像素约定

| 量 | OpenCV | Blender / OSL 相机空间 |
| --- | --- | --- |
| X | 右 | 右 `+X` |
| Y | **下** | **上** `+Y` |
| Z | 前（光轴） | OSL：前 `+Z`；Blender 相机局部：后 `−Z` |
| 图像原点 | 左上，v 向下 | 栅格 0–1，Y 向上，像素中心在 `(i+0.5, j+0.5)` |

着色器实现：

```c
point r = camera_shader_raster_position();
float px = r.x * W;            /* u: 左→右 */
float py = (1.0 - r.y) * H;    /* v: 上→下 */
...
direction = normalize(vector(x, -y, 1.0));   /* 把 OpenCV 的 y 下翻回 +Y 上 */
```

注意：`u`、`v` 是 **OSL 保留全局变量**（surface 的纹理坐标），着色器里不能用它们做局部变量，否则
`error: "u" already declared in this scope`。

## 4. 与 Blender 自带相机的换算

水平 fit（`sensor_fit='AUTO'` 且 `W >= H`，或 `'HORIZONTAL'`）：

```
fx = lens_mm / sensor_width_mm · W
lens_mm = fx · sensor_width_mm / W
```

垂直 fit（`'VERTICAL'`，或 `'AUTO'` 且 `H > W`）把 `sensor_width_mm/W` 换成 `sensor_height_mm/H`。
两种情况都得到 `fx = fy`（方形像素）。

主点与 shift（128×128、50mm/36mm 实测标定）：

```
cx = (0.5 - shift_x) · W
cy = (0.5 + shift_y) · H      # 注意 y 方向符号相反：Blender 图像坐标自下而上
```

实测：`shift=(0.1, 0.15)` → 主点 `(51.107, 83.230)`，与上式 `(51.2, 83.2)` 一致（0.03 px 内）；
用该映射切换相机后，自定义相机与自带相机渲染结果**逐像素完全一致**。

## 5. 外参转换

`cv2.solvePnP` 给出 world→camera 的 `(R_cv, t_cv)`（相机系 Y 向下、Z 向前）。
Blender 相机局部系由 `M = diag(1, -1, -1)` 关联：

```
R_b = M · R_cv            # world → Blender camera local
t_b = M · t_cv
C   = -R_bᵀ · t_b         # 光心在世界坐标
R_wc = R_bᵀ               # camera → world，即 object 旋转
```

若标定世界系不是 Blender 世界系（ROS: x 前/y 左/z 上；OpenCV 视觉系: y 下），
插件提供可选的 `world_matrix`（4×4）左乘，不靠猜。

## 6. 两种工作模式

| 模式 | 设置 | 用途 |
| --- | --- | --- |
| A 畸变栅格 | `enable_distortion=1`，填标定得到的 `K, D` | 与实拍图像**逐像素可比**，可喂给同一套 OpenCV 检测/标定/VIO 流程 |
| B 理想/去畸变 | `enable_distortion=0`；需要去畸变视图时填 `cv2.getOptimalNewCameraMatrix()` 的结果 | 作为虚拟理想针孔相机使用（无黑边的 rectify 视图） |

## 7. 分辨率行为

内参是像素量纲：插件把标定分辨率 `image_width/height` 与当前渲染分辨率一起保存，
`scale_to_render=True`（默认）时按比例缩放 fx/fy 与显式主点，保证 **FOV 不变**；关闭则原样使用
（适合已经按目标分辨率标定的场景）。自动主点（`cx=cy=-1`）始终跟随图像中心，与分辨率无关。
非方形像素（`pixel_aspect_x != pixel_aspect_y`）会破坏 OpenCV 的像素模型，插件在面板上给出提示。

## 8. 实测验证数据（Blender 4.5.3 LTS / Cycles CPU）

测试场景：黑色世界 + 棋盘发射平面（`z=-3`）或发射小球；128×128、4 samples、无降噪、Standard view transform。

| 用例 | 期望 | 实测 |
| --- | --- | --- |
| 零畸变自定义相机 vs 自带透视相机（50mm/36mm/128px） | 完全一致 | `max\|Δ\| = 0`，16384/16384 像素相同（图像 std 0.203） |
| 主点偏移（shift 0.1 / 0.15，cx=51.2 cy=83.2） | 完全一致 | `max\|Δ\| = 0` |
| 畸变 k1=-0.25, k2=0.06, p1=2e-4 目标点成像位置 | 与 OpenCV 前向模型一致 | 误差 0.058 px（自检算子） |
| 畸变（k1=-0.2）目标点成像位置（另一组 fx/fy） | 同上 | 误差 0.094 px |
| 分辨率换算 1920×1080 → 128×128 | fx 按 1/15 缩放 | 1500 → 100.0 |
| 坏着色器（故意语法错误）后 `ensure_compiled` | 必须失败 | 检出并中止（旧字节码被静默保留） |

## 9. 参考

- Blender Manual · Custom Camera · <https://docs.blender.org/manual/en/5.2/render/cycles/osl/camera.html>
- Blender Manual · OSL · <https://docs.blender.org/manual/en/5.2/render/cycles/osl/index.html>
- 手册源码：`blender-manual` 仓库 `manual/render/cycles/osl/camera.rst`
- Blender 源码：`source/blender/makesrna/intern/rna_camera.cc`（`rna_Camera_custom_update`）
- Blender 源码：`intern/cycles/blender/addon/osl.py`（`update_custom_camera_shader`、`_cycles.osl_compile`、`oslquery`）
- Blender 源码：`intern/cycles/blender/camera.cpp`（`BlenderCameraParamQuery`：按名从 `cycles_custom` 取参）
- Cycles 模板：`scripts/templates_osl/advanced_camera.osl`（径向畸变反解、DoF 写法）
- OpenCV：`cv2.calibrateCamera` / `cv2.undistortPoints` / `cv2.getOptimalNewCameraMatrix`