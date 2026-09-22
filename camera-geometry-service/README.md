# camera-geometry-service

针孔投影 + Brown–Conrady 畸变 + 双视图三角化的后端服务。标定/重建流水线把一批点作为
一个**作业**提交上来，拿回投影结果或三角化结果。纯 HTTP/JSON，无前端。

## 投影模型（钉死的约定）

1. 相机坐标系三维点 `(X, Y, Z)` 归一化到 `z=1` 平面：`x = X/Z, y = Y/Z`（`Z ≤ 0` 拒绝）。
2. **在归一化平面上**施加 Brown–Conrady 畸变（`r² = x² + y²`）：
   ```
   radial = 1 + k1·r² + k2·r⁴
   x_d = x·radial + 2·p1·x·y       + p2·(r² + 2x²)
   y_d = y·radial + p1·(r² + 2y²)  + 2·p2·x·y
   ```
   ⚠️ 径向畸变的 `r` 必须取自归一化平面。拿像素坐标凑 `r` 会差出 fx/fy（数百倍）的
   量级，结果直接报废——本服务的畸变实现只接受归一化坐标，从结构上杜绝这个错误。
3. 内参映射到像素：`u = cx + fx·x_d`，`v = cy + fy·y_d`。
4. 像面判定：`0 ≤ u < width` 且 `0 ≤ v < height` 为在像面内，逐点如实标注。

单点接口与批量作业**共用同一个投影函数**（`PinholeProjector.project`），同一个点
从哪个入口进来结果都一致。

## 三角化方法：DLT（固定）

选用 **DLT（Direct Linear Transform）** 而非中点法：它最小化代数重投影残差，对噪声
匹配的推广性更好。流程：匹配像素 → 归一化平面 → 迭代去畸变（恢复理想针孔光线）→
每个视图贡献两条方程 `x·P3−P1=0, y·P3−P2=0`（P=[R|t]）→ 4×4 法方程 AᵀA 的最小
特征值对应特征向量（Jacobi 旋转）→ 齐次归一化得三维点。

**红线**：绝不平均两个视图的像素坐标冒充三维点；三维点只来自两条反投影光线的
最小二乘交会。

每对匹配重投影回两个视图（含畸变），逐对报告两个视图的重投影误差及二者均值；
作业级报告有效匹配上的**最大/平均重投影误差**。三角化结果落在某台相机后方的
匹配对标记 `valid=false`（这是计算结果而非输入错误），其余匹配继续。

## 作业与校验策略

- **投影作业**：一组内参+畸变系数 + 一批相机系三维点 → 每点像素坐标与在像面内标记
  + 像面外点数。
- **三角化作业**：两台相机（内参+畸变+外参）+ 一组匹配像点对 → 每对三维点与重投影
  误差 + 作业级最大/平均误差。
- **非法点策略（全局一致）**：fail-fast，整作业拒绝。所有点在第一次投影/三角化之前
  全部校验完毕，任一非法点（如 Z≤0）以带类型的 422 拒绝整个作业，不产生部分结果。

## API

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/project` | 单点投影 `{intrinsics, distortion?, point}` → `{u, v, inBounds}` |
| POST | `/api/jobs/projection` | 投影作业 `{intrinsics, distortion?, points[]}` → `{results[], totalPoints, outOfBoundsCount}` |
| POST | `/api/jobs/triangulation` | 三角化作业 `{camera1, camera2, matches[]}` → `{pairs[], pairCount, validPairCount, maxReprojectionError, meanReprojectionError}` |
| GET | `/api/presets` | 只读回显已注册内参预设 + 立方体标定算例（可直接 POST 回去） |
| GET | `/api/status` | 运行状态：版本、启动时间、已完成作业计数 |

- `distortion` 整个对象可省略（等价于零畸变）；一旦给出，`k1/k2/p1/p2` 缺一不可。
- 外参格式：`{"rotation": [[..],[..],[..]], "translation": [x,y,z]}`，世界→相机：`X_cam = R·X_world + t`。
- 匹配点对格式：`{"view1": {"x","y"}, "view2": {"x","y"}}`。

### 结构化错误

非法输入在开算前拦下，返回 422（JSON 无法解析为 400），无未捕获异常、无空结果：

```json
{"error": {"type": "POINT_BEHIND_CAMERA", "message": "...", "details": {"point": "points[2]", "z": -0.5}}}
```

| type | 触发 |
|---|---|
| `POINT_BEHIND_CAMERA` | 任一点 Z ≤ 0 |
| `INVALID_FOCAL_LENGTH` | fx 或 fy 非正/非有限 |
| `INVALID_IMAGE_SIZE` | 图像宽或高 ≤ 0 |
| `MISSING_INTRINSICS_FIELD` | 内参缺项（含整个 intrinsics 缺失） |
| `MISSING_DISTORTION_FIELD` | 畸变对象给了但缺系数 |
| `MISSING_EXTRINSICS` | 三角化某台相机缺外参 |
| `INVALID_EXTRINSICS` | 旋转非 3×3 / 平移非 3 维 / 含非有限值 |
| `EMPTY_MATCH_SET` | 匹配点对数量为零 |
| `MISSING_FIELD` / `INVALID_VALUE` | 其它缺项 / 非有限数值 |
| `MALFORMED_REQUEST` | 请求体不是合法 JSON（400） |

## 预置算例

`GET /api/presets` 返回两个内参预设（`hd720-f800`、`vga-f520`）和一个已知立方体算例
`cubeDemo`：单位立方体 8 个角点、两台相机（相机 1 在 (0,0,5)、相机 2 在 (2,1,4)，
均朝向原点），以及两份可直接 POST 的作业体。8 个角点在两个视图中的投影全部落在
1280×720 像面内，方便人工核对。

```bash
curl -s localhost:8080/api/presets | jq .cubeDemo.projectionJob > job.json
curl -s -XPOST localhost:8080/api/jobs/projection -H 'Content-Type: application/json' -d @job.json
# 期望：totalPoints=8, outOfBoundsCount=0
```

## 构建与运行

```bash
# Docker（构建阶段会跑完整测试套件，测试不过镜像不出）
docker build -t camera-geometry-service .
docker run --rm -p 8080:8080 camera-geometry-service

# 本地（需要 JDK 17 + Maven）
mvn test
mvn spring-boot:run
```

## 验收不变量 ↔ 测试映射

| 不变量 | 测试 |
|---|---|
| k1=k2=p1=p2=0 时畸变前后像素逐点相同 | `PinholeProjectorTest.zeroDistortionLeavesPixelsExactlyIdentical`、`ProjectionApiTest.zeroDistortionCoefficientsProduceSamePixelsAsNoDistortion` |
| 点沿光轴远移 → 像点向主点收缩 | `PinholeProjectorTest.pointsRecedingAlongOpticalAxisShrinkTowardPrincipalPoint` |
| fx、fy 同时加倍 → 半径加倍 | `PinholeProjectorTest.doublingBothFocalLengthsDoublesRadiusFromPrincipalPoint` |
| 方向性 X/Z（而非 Z/X） | `PinholeProjectorTest.normalizationDividesByZNotTheOtherWayRound` |
| 径向畸变在归一化平面（非像素） | `PinholeProjectorTest.radialDistortionUsesNormalizedRadiusNotPixels` |
| 立方体角点三角化重投影贴回原像素（< 1e-6 px） | `DltTriangulatorTest.cubeCornersTriangulateAndReprojectBackOntoOriginalPixels`、`TriangulationApiTest.cubeDemoTriangulationReprojectsBackOntoOriginalPixels` |
| Z≤0 / 焦距非正 / 外参缺失 / 零匹配分别被拒 | `ProjectionApiTest`、`TriangulationApiTest` 各 `*Rejected*` 用例 |
| 单点与批量同点结果一致 | `ProjectionApiTest.singlePointAndBatchJobShareTheSameProjectionResult` |
| 多作业并发结果隔离 | `ConcurrencyIsolationTest.concurrentJobsNeverMixTheirResults` |

## 代码结构

```
com.acme.camera
├── core          纯几何内核：针孔投影、Brown–Conrady 畸变、DLT 三角化、Jacobi 特征分解
├── validation    输入校验 + 带类型的结构化错误
├── job           作业编排（无状态服务，天然并发隔离）+ 作业计数
├── api           HTTP 路由（投影/三角化/预设/状态四个 Controller）+ 全局异常映射 + DTO
├── preset        内参预设注册表 + 已知立方体标定算例
└── config        内核 Bean 装配
```
