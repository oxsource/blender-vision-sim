# 相机模型：OpenCV 内参与畸变

本文记录 `opencv_camera` 插件的模型推导、坐标约定与实测验证数据。数学实现在
`addons/opencv_camera/core/camera_model.py`，着色器在 `addons/opencv_camera/shaders/`
（`opencv_camera.osl` = 多项式模型，`opencv_fisheye.osl` = 鱼眼模型）。

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

插件支持三种 OpenCV 畸变模型，按 `distortion.model` 选择对应的着色器与参数集：

| 模型 | 系数 | 着色器 | 反解方式 |
| --- | --- | --- | --- |
| `brown_conrady` | k1,k2,p1,p2,k3 | `opencv_camera.osl` | 固定点迭代（同 `cv2.undistortPoints`） |
| `rational` | 追加 k4,k5,k6（分母项） | `opencv_camera.osl` | 同上 |
| `fisheye` | k1,k2,k3,k4（θ 多项式） | `opencv_fisheye.osl` | 牛顿迭代（同 `cv2.fisheye.undistortPoints`） |

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

### 2.1b 前向（OpenCV fisheye / Kannala-Brandt）

```
r = |(X/Z, Y/Z)| ,  theta = atan(r)
theta_d = theta (1 + k1 t^2 + k2 t^4 + k3 t^6 + k4 t^8)
x_d = theta_d / r * (X/Z) ,  y_d = theta_d / r * (Y/Z)
u = fx x_d + cx ,  v = fy y_d + cy
```

注意该模型的**有效域**：`theta = atan(r)` 只能表示 `theta < 90°`，因此正向模型只在
`r_px <= fisheye_valid_radius_px()`（`theta = 90°` 对应的像素半径）内有定义。广角 AVM
镜头（本文档实测的参考相机）在画面角落处 `theta ≈ 101°`，已经超出正向模型的定义域——
**渲染（像素→射线）不受限制**，但用正向模型做"点投影预测"时只能取有效域内的像素。

### 2.2 反向（着色器所需）

着色器拿到像素，需要还原射线方向，即求畸变逆（等价 `cv2.undistortPoints`）：

```
x = x_d, y = y_d
repeat N:
    icdist = (1 + k4r² + k5r⁴ + k6r⁶) / (1 + k1r² + k2r + k3r)
    dx = 2p1xy + p2(r²+2x²);  dy = p1(r²+2y²) + 2p2xy
    x = (x_d - dx)·icdist;    y = (y_d - dy)·icdist
```

实现要点：固定点迭代（默认 20 次）、`den` 过小或坐标发散时跳出；发散时默认保留 best effort 方向，
当 `discard_invalid_rays=1` 时把该射线 `throughput = color(0)` 丢弃（对应像素渲染为黑）。

鱼眼模型的反解是**牛顿迭代**：

```
solve  theta_d = theta (1 + k1 t^2 + k2 t^4 + k3 t^6 + k4 t^8)  for theta
f'(theta) = 1 + 3 k1 t^2 + 5 k2 t^4 + 7 k3 t^6 + 9 k4 t^8
direction = (sin(theta) * x_d / r_d, -sin(theta) * y_d / r_d, cos(theta))
```

用 `sin/cos` 而不是斜率 `tan(theta)`：广角鱼眼在画面角落 `theta > 90°`（射线略向后），
`tan` 会翻号而 `sin/cos` 保持正确；返回的方向本身即单位向量。`Distortion.enabled = False`
时两种模型都退化为理想针孔（同一套 fx/fy/cx/cy）。

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
插件提供可选的 `world_matrix`（4×4）左乘，不靠猜（在 `CV Extrinsics ▸ World Frame` 折叠块里）。

面板里除 `R`/`t` 外还提供 **Euler（XYZ，Blender 世界系）三轴角度输入**，与 `R` 双向同步、直接旋转
相机物体；注意约定：**OpenCV 下的单位位姿（X 右 / Y 下 / Z 前）在 Blender 里是绕 X 轴 180°**，
所以 R=单位矩阵时 Euler 显示为 (180°, 0, 0) 而不是全 0。

## 6. 两种工作模式

| 模式 | 设置 | 用途 |
| --- | --- | --- |
| A 畸变栅格 | `enable_distortion=1`，填标定得到的 `K, D` | 与实拍图像**逐像素可比**，可喂给同一套 OpenCV 检测/标定/VIO 流程 |
| B 理想/去畸变 | `enable_distortion=0`；需要去畸变视图时填 `cv2.getOptimalNewCameraMatrix()` 的结果 | 作为虚拟理想针孔相机使用（无黑边的 rectify 视图） |

## 7. 分辨率行为

内参是像素量纲，所以有**两个**分辨率概念，插件把两者分开管理：

| 概念 | 存放位置 | 说明 |
| --- | --- | --- |
| 标定分辨率 | `intrinsics.image_width/height` | K/D 是在这个尺寸下标出来的 |
| 输出分辨率 | `output.mode` + `output.width/height` | 实际渲染/输出的图像尺寸；`mode='calibration'` 时等于标定分辨率，`'custom'` 用手填/预设，`'scene'` 跟随 Blender |

`output.lock_scene_resolution`（默认开）会在每次应用参数时把输出分辨率写进 `scene.render.resolution_*`，
因此 F12 的输出就是设定的相机原生尺寸；关闭则只作为插件里的记录，渲染尺寸仍由场景控制。

把内参从标定分辨率换算到输出分辨率时：

| 情况 | 行为 |
| --- | --- |
| 宽高比一致（如 1920×1080 → 640×360） | 按比例缩放 fx/fy 与主点偏移，**FOV 不变**（`scale_to_render=False` 时原样使用） |
| 宽高比不一致（如 1920×1080 → 1280×1280） | 无法同时匹配几何：改为**保持像素尺度**（fx/fy 不变）+ 主点相对图像中心的像素偏移不变，即"原始像素尺度的中心裁剪"，面板给出 aspect mismatch 提示 |

> 顺序很重要：**先设渲染分辨率，再下发内参**。内参是按渲染分辨率换算的，顺序反了会用旧分辨率
> 换算，FOV 就错了（预览曾有这个 bug，已修并有测试守护）。

自动主点（`cx=cy=-1`）始终跟随图像中心；显式主点在缩放/裁剪时按上表处理。
非方形像素（`pixel_aspect_x != pixel_aspect_y`）会破坏 OpenCV 的像素模型，插件在面板上给出提示。

自检（`Run Self Test`）会自动按标定宽高比选择测试分辨率（长边 256，短边按比例），避免裁剪/拉伸。

## 8. 实测验证数据（Blender 4.5.3 LTS / Cycles CPU）

测试场景：黑色世界 + 棋盘发射平面（`z=-3`）或发射小球；128×128、4 samples、无降噪、Standard view transform。

| 用例 | 期望 | 实测 |
| --- | --- | --- |
| 零畸变自定义相机 vs 自带透视相机（50mm/36mm/128px） | 完全一致 | `max\|Δ\| = 0`，16384/16384 像素相同（图像 std 0.203） |
| 主点偏移（shift 0.1 / 0.15，cx=51.2 cy=83.2） | 完全一致 | `max\|Δ\| = 0` |
| 多项式畸变（k1=-0.2, k2=0.03, p1=2e-4，fx=64/128px，θ≤45°） | 与 OpenCV 前向模型一致 | 误差 0.018 px（自检算子） |
| 鱼眼（参考 AVM 前相机 1280×960，θ≈72°） | 同上 | 误差 0.048 px |
| 鱼眼贴近 90° 边界（θ≈87°，射线需用 sin/cos 表示） | 同上 | 误差 0.026 px |
| 分辨率换算 1920×1080 → 960×540（同比） | fx 按 1/2 缩放 | 1500 → 750.0 |
| 分辨率宽高比不一致 1920×1080 → 128×128 | 保持像素尺度（裁剪） | fx 1500 不变，主点回到图像中心 |
| 坏着色器（故意语法错误）后 `ensure_compiled` | 必须失败 | 检出并中止（旧字节码被静默保留） |

补充：Cycles 为自定义相机生成的参数是**数值型 ID 属性**（`widget="boolean"` 只影响存入的值类型，
4.5 的 `id_properties_ui` 没有复选画法），所以插件把那张裸参数表默认隐藏，改由自己的面板提供
真正的 `BoolProperty` 复选框（`distortion.enabled` / `discard_invalid_rays`）。

### 8.1 可视化验证（`Add  VisionSim ▸ Camera Scene`）

一键生成棋盘方块 + 棋盘地面 + 灯光（集合 `OpenCV Camera Scene`：`CheckerCube` / `CheckerGround` /
`ColorBlock0-3` / `KeyLight` / `SunLight`），并把相机内参应用到 Cycles：

| 鱼眼模型（`fisheye`，k1..k4） | 关闭畸变（理想针孔） |
| --- | --- |
| ![鱼眼](images/fisheye_on.png) | ![针孔](images/pinhole_off.png) |
| 地面网格弯曲、边缘压缩、物体向中心收缩 | 网格线为直线、边缘拉伸 |

两张图使用同一套 K（fx=317.78, cx=636.23, cy=477.82，1280×960）与同一场景，仅切换
`enable_distortion`。

## 9. 导入其它工程的相机配置

除标准标定文件外，`Import Calibration` 也接受多相机应用配置（如 `filament_avm` 的
`configs/vehicle_avm_minibus.json`：`cameras: [{name, K, D, input_size}]`），可用
`camera_name` 选择具体相机：

```python
from opencv_camera.core import calibration_io
calib = calibration_io.load_calibration("vehicle_avm_minibus.json", camera_name="front")
```

注意：该类配置**没有** `distortion_model` 字段，插件会按默认的 Brown-Conrady 读取 4 个系数并给出
提示——AVM 这类广角镜头应把面板里的 `Model` 切到 `Fisheye`（仓库自带的
`presets/default_camera.yaml` 已显式写为 `distortion_model: equidistant`，可直接导入）。

## 10. 参考

- Blender Manual · Custom Camera · <https://docs.blender.org/manual/en/5.2/render/cycles/osl/camera.html>
- Blender Manual · OSL · <https://docs.blender.org/manual/en/5.2/render/cycles/osl/index.html>
- 手册源码：`blender-manual` 仓库 `manual/render/cycles/osl/camera.rst`
- Blender 源码：`source/blender/makesrna/intern/rna_camera.cc`（`rna_Camera_custom_update`）
- Blender 源码：`intern/cycles/blender/addon/osl.py`（`update_custom_camera_shader`、`_cycles.osl_compile`、`oslquery`）
- Blender 源码：`intern/cycles/blender/camera.cpp`（`BlenderCameraParamQuery`：按名从 `cycles_custom` 取参）
- Cycles 模板：`scripts/templates_osl/advanced_camera.osl`（径向畸变反解、DoF 写法）
- OpenCV：`cv2.calibrateCamera` / `cv2.undistortPoints` / `cv2.getOptimalNewCameraMatrix`