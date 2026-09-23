# camera-geometry-service

针孔投影 + Brown–Conrady 畸变 + 双视图三角化 + **相机标定（从观测反解内参/畸变/位姿）** 的后端
服务。标定/重建流水线把一批点作为一个**作业**提交上来，拿回投影结果、三角化结果或标定结果。
纯 HTTP/JSON，无前端。

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

## 标定方法：线性初值 + LM 迭代重投影细化（两段式，固定）

标定作业把同一块已知几何的控制点（世界系三维位置）在若干张照片里被点出的像素提交上来，
反解出**一台**相机的内参（fx、fy、cx、cy）、Brown–Conrady 系数（k1、k2、p1、p2）以及
**每张观测各自的世界→相机位姿**。绝不允许退化成一次线性求解就交差：

1. **粗初值（仅线性闭式解）**：
   - 非平面三维控制点：每视图解 DLT 相机矩阵（输入坐标做 Hartley 归一化），RQ 分解
     （Gram–Schmidt）劈出 K，多视图平均；位姿由 K 固定的归一化 DLT + cheirality 定符号。
   - 平面标定板：张正友线性法（零斜切，每视图单应贡献两条关于 ω=K⁻ᵀK⁻¹ 的方程），
     位姿由单应分解 + 平面深度 cheirality 二选一。
   - 畸变初值恒为零（线性阶段无法估计非线性项）。
2. **迭代细化（Levenberg–Marquardt，唯一的参数来源）**：内参、四个畸变系数、全部位姿
   （位姿用 Rodrigues 3 向量参数化，天然正交）打包进**同一个**参数向量，雅可比取中心
   数值差分，阻尼信赖域自适应伸缩，直接最小化**所有观测、所有点的像素重投影残差平方和**。
   残差走的就是本服务那条「归一化平面 r² → 径向 (1+k1r²+k2r⁴) → p1/p2 切向 → 内参映射像素」
   的投影管线（`BrownConradyDistortion` + 内参），标定结果与投影内核严丝合缝。

**红线**：不平均像素、不用解析公式凑内参；解出来的参数只来自「让重投影误差最小」这一
迭代过程的收敛结果。最终最大/平均重投影误差由 `PinholeProjector` 把每个世界点重投影回去
逐点算出，既是质量指标也是验收卡口。

**停机条件（回包如实标注，调用方永远看得出这次标定到底怎样）**：残差 RMS ≤ 阈值
（`RESIDUAL_THRESHOLD_REACHED`，此时 `converged=true`）；相邻两轮改善微乎其微
（`IMPROVEMENT_TOO_SMALL`）；参数步长微乎其微（`STEP_TOO_SMALL`）；到迭代上限
（`MAX_ITERATIONS_REACHED`）；信赖域缩到底仍无法下降（`DIVERGED`）——后四种
`converged=false`，但仍返回参数、最终残差、停机原因、跑了多少轮，不假装成功。

**观测充分性**（开算前以带类型错误拒绝，绝不返回「看着像样其实没意义」的数）：
平面标定板至少 **2 张**不同姿态的视图（单视图无法把焦距和板深/尺度分开，错误的内参也能
零误差重投影）；非平面三维控制点单视图即可辨识；每视图平面 ≥4 点 / 三维 ≥6 点；
且 `2×总观测点数 ≥ 未知量(4 内参 [+4 畸变] + 6×视图数) + 余量`。

## 作业与校验策略

- **投影作业**：一组内参+畸变系数 + 一批相机系三维点 → 每点像素坐标与在像面内标记
  + 像面外点数。
- **三角化作业**：两台相机（内参+畸变+外参）+ 一组匹配像点对 → 每对三维点与重投影
  误差 + 作业级最大/平均误差。
- **标定作业**：一组观测（每张 = 同一块控制点的世界三维点 + 对应像素）+ 像面宽高 →
  反解内参、畸变（可选）、每视图位姿 + 收敛诊断 + 作业级最大/平均重投影误差。
- **非法点策略（全局一致）**：fail-fast，整作业拒绝。所有点在第一次投影/三角化/求解
  之前全部校验完毕，任一非法点以带类型的 422 拒绝整个作业，不产生部分结果。

## API

| 方法 | 路径 | 说明 |
|---|---|---|
| POST | `/api/project` | 单点投影 `{intrinsics, distortion?, point}` → `{u, v, inBounds}` |
| POST | `/api/jobs/projection` | 投影作业 `{intrinsics, distortion?, points[]}` → `{results[], totalPoints, outOfBoundsCount}` |
| POST | `/api/jobs/triangulation` | 三角化作业 `{camera1, camera2, matches[]}` → `{pairs[], pairCount, validPairCount, maxReprojectionError, meanReprojectionError}` |
| POST | `/api/jobs/calibration` | 标定作业 `{width, height, estimateDistortion?, observations[]}` → 内参+畸变+每视图位姿+收敛诊断+最大/平均误差 |
| GET | `/api/presets` | 只读回显已注册内参预设 + 立方体标定算例（可直接 POST 回去） |
| GET | `/api/status` | 运行状态：版本、启动时间、已完成作业计数（投影/三角化/**标定**） |

- `distortion` 整个对象可省略（等价于零畸变）；一旦给出，`k1/k2/p1/p2` 缺一不可。
- 外参格式：`{"rotation": [[..],[..],[..]], "translation": [x,y,z]}`，世界→相机：`X_cam = R·X_world + t`。
- 匹配点对格式：`{"view1": {"x","y"}, "view2": {"x","y"}}`。

### 标定作业请求/回包

```json
{
  "width": 1280, "height": 720, "estimateDistortion": true,
  "observations": [
    {"worldPoints": [{"x","y","z"}, ...],
     "pixels":      [{"x","y"}, ...]}
  ]
}
```

- 每张观测的 `worldPoints` 与 `pixels` 必须等长且按下标对齐（第 i 个三维点在第 i 个像素被看到）。
  世界坐标无 Z>0 限制（那是相机系规则，标定输入是世界系），但必须全部有限。
- `estimateDistortion` 省略或为 true → 估计 k1/k2/p1/p2；为 false → 固定为零（纯针孔标定）。

回包（`POST /api/jobs/calibration`）：

```json
{
  "calibrationMethod": "BUNDLE_REPROJECTION_LM",
  "intrinsics": {"fx","fy","cx","cy","width","height"},
  "distortion": {"k1","k2","p1","p2"},
  "poses": [{"observation": 0, "extrinsics": {"rotation","translation"},
             "maxReprojectionError","meanReprojectionError"}, ...],
  "convergence": {"converged": true, "stopReason": "RESIDUAL_THRESHOLD_REACHED",
                  "iterations": 9, "initialRmsError": 0.21,
                  "finalRmsError": 7.3e-10, "rmsThresholdPixels": 1e-9},
  "observationCount": 5, "totalPoints": 100,
  "maxReprojectionError": 2.3e-9, "meanReprojectionError": 9.2e-10
}
```

位姿回包约定与三角化一致：`{"rotation","translation"}`，世界→相机 `X_cam = R·X_world + t`。

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
| `EMPTY_OBSERVATION_SET` | 标定作业一张观测都没有 |
| `OBSERVATION_SIZE_MISMATCH` | 某张观测的三维点数与像素点数对不上 |
| `INSUFFICIENT_CONSTRAINTS` | 观测/控制点太少或退化（平面仅 1 视图、每视图点数不足、方程数少于未知量） |
| `MISSING_FIELD` / `INVALID_VALUE` | 其它缺项 / 非有限数值（含非有限世界坐标或像素） |
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
| 真值回环：投影→观测→标定，解回原内参/位姿且重投影近零（平面/三维、零畸变） | `CameraCalibratorTest.planarBoardWithZeroDistortionRecoversGroundTruthParameters`、`...nonPlanarVolumeRecoversGroundTruthParameters`、`CalibrationApiTest.nonZeroDistortionRoundTripRecoversParametersAndNearZeroErrors` |
| 非零 k1/k2/p1/p2 被认出而非解成零还硬说收敛 | `CameraCalibratorTest.planarBoardWithNonZeroDistortionRecoversTheCoefficients`、`...nonPlanarVolumeWithNonZeroDistortionRecoversTheCoefficients` |
| 观测不足（零观测/点太少/单平面视图/方程欠定）被结构化拒绝 | `CameraCalibratorTest.zeroObservationsAreRejected` 等、`CalibrationApiTest.emptyObservationSetIsRejected`、`...singlePlanarViewIsRejectedAsUnderConstrained`、`...tooFewEquationsForTheUnknownCountAreRejected` |
| 收敛状态如实上报（达标才 converged；不自洽数据不谎称达标） | `CameraCalibratorTest.convergenceReportShowsThresholdStopAndNonZeroIterationCount`、`...inconsistentObservationsDoNotClaimThresholdConvergence` |
| 多个标定作业并发，参数与误差报告互不串味 | `CalibrationConcurrencyTest.concurrentCalibrationJobsNeverMixTheirParametersOrErrorReports` |

## 代码结构

```
com.acme.camera
├── core          纯几何内核：针孔投影、Brown–Conrady 畸变、DLT 三角化、Jacobi 特征分解
├── calibration   标定内核（与投影/三角化/校验各自独立成包）：
│                 ├─ CameraCalibrator         两段式编排（线性初值 → LM 迭代 → 经投影内核复核误差）
│                 ├─ CalibrationInitializer   线性粗初值（三维 DLT+RQ / 平面 Zhang、Hartley 归一化）
│                 ├─ LevenbergMarquardt       通用 LM 迭代优化（独立放置，雅可比取中心数值差分）
│                 ├─ CalibrationResidualModel 参数打包/解包 + 重投影残差（复用现有投影管线）
│                 ├─ RotationMath / RqDecomposition / DenseSolver   Rodrigues、RQ 分解、Cholesky/Jacobi
│                 └─ CalibrationConstraintChecker                  观测充分性（不足即结构化拒绝）
├── validation    输入校验 + 带类型的结构化错误
├── job           作业编排（无状态服务，天然并发隔离）+ 作业计数
├── api           HTTP 路由（投影/三角化/标定/预设/状态五个 Controller）+ 全局异常映射 + DTO
├── preset        内参预设注册表 + 已知立方体标定算例
└── config        内核 Bean 装配
```
