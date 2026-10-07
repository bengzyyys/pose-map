# 本地定位与地图

这是一个在本机运行的本地定位与地图。`posemap` 包提供二维轨迹与路标
能力：按段导入有序帧、维护路标观测合并、查询历史位姿与矩形区域内路标，
并把全部数据持久化到本地文件，关闭后重新打开结果一致。

## 使用

```bash
go test ./...
```

## 基本流程

1. `posemap.Create(path, Config)` 用初始时间、位置、朝向、位置方差、
   最大帧间隔与路标合并距离创建一份地图（已有文件不会被覆盖）；
   已存在的地图用 `posemap.Open(path)` 打开。
2. `m.ImportSegment(Segment{ID, Frames})` 按段导入。整段校验通过才
   生效并落盘；失败返回可区分的 `*posemap.RejectError`（含从零开始的
   帧序号；路标/段冲突含标识），已提交数据不受影响。同一内容重复
   导入直接返回首次结果，同一标识不同内容会被拒绝。
3. 查询：`CurrentPose()`、`PoseAt(t)`（不晚于 t 的最后一份位姿）、
   `LandmarksInRect(Rect)`（含边界，按标识排序）。
4. 已确认回环校正 `m.Correct(Correction{ID, Anchor, Target})`：
   `Anchor` 必须准确命中某个已导入帧（不能是初始位姿，也不借用最近
   帧），`Target` 给出该帧应有的位置、朝向与非负方差。锚点帧采用目标
   位姿，锚点至末帧的轨迹整体平移并旋转、保持相对位姿与时间、朝向
   归一；方差取目标方差加锚点之后原运动方差的累计值。受影响帧中的
   路标观测随位姿重放（按原时间与同帧次序、沿用合并距离限制），更早
   观测固定，标识与观测次数不变。校正跨越同一路标的多次出现时，各次
   观测归入各自出现分别重放、分别遵守合并距离；任一次出现冲突就整次
   拒绝（错误指出路标、出现编号与冲突帧时间）。冲突或非法输入整次
   拒绝（`anchor_not_found`、`no_basis`、`correction_mismatch` 等
   可区分原因），任何失败都不留部分更新。`m.Corrections()` 按提交
   次序返回校正记录（含受影响位姿与路标各次出现的校正前后值），记录
   只增不改。同标识同内容重复提交返回首次结果，同标识不同内容被拒绝；
   校正后重复导入原轨迹段仍返回首次导入结果。
5. 路标失效与历次出现：`m.Invalidate(Invalidation{ID, Reason, Landmarks})`
   把一组路标的当前有效记录同时撤下，失效时间取提交时的当前位姿时间。
   操作标识/原因为空、列表为空、路标标识为空或重复、路标不存在或当前
   已失效时整次拒绝（`empty_reason`、`empty_landmark_list`、
   `duplicate_landmark_entry`、`landmark_not_found`、
   `landmark_inactive`、`invalidation_mismatch` 等可区分原因）。成功
   返回失效时间与各路标的出现编号；区域查询立即排除被撤下的路标，位姿
   与已接受观测不删除。同一标识每次失效后在新的成功导入轨迹中再次
   出现，出现编号从 1 起递增、观测计数从 1 开始，旧出现不参与合并；整
   段被拒绝不留下新记录也不消耗编号。同一操作标识以相同原因、相同
   路标集合重复提交返回首次结果（即使路标后来再次出现也不再撤下），
   同标识不同内容被拒绝。`m.LandmarkAppearances(id)` 按编号递增返回
   各次出现的位置、观测计数、有效状态、首次观测时间以及失效时间与
   原因（仍有效时不携带失效信息）；未知标识返回可
   `errors.Is(ErrLandmarkNotFound)` 区分的未找到结果。
6. `m.Close()` 关闭。之后用 `Open` 重新打开同一文件，查询、校正记录、
   失效/再现记录与各类重复提交的结果均与关闭前一致；旧文件（无逐帧
   观测来源）仍可打开、查询、继续导入，其中路标视为第 1 次有效记录且
   首次观测时间未知，只对缺少依据的历史范围拒绝校正（`no_basis`），
   在其上新追加的完整依据范围仍可校正。文件损坏报 `ErrCorrupt`，版本
   不支持报 `ErrUnsupportedVersion`，都不会被当作新地图覆盖。

## 轨迹导入示例：自身坐标如何影响位姿与路标

下面这份程序走完整条导入流程：创建地图、导入一段两帧轨迹（每帧都平移，
第一帧同时转向，两帧各观测同一个路标一次），查询各帧位姿、两帧之间的
历史位姿与矩形区域内路标，再提交一段必然被拒绝的轨迹并检查拒绝后的状态。
位置单位是米、时间是整数毫秒、朝向是弧度。看懂结果有三个坐标约定：

- **平移使用运动前的朝向**：帧内先按进入该帧时的朝向把 `DX,DY` 转到
  地图坐标，再累加 `DHeading`。
- **观测使用运动完成后的位姿**：同帧的路标观测在平移、转向都完成之后
  才转换到地图坐标。
- 路标查询返回的是**地图坐标**，不会把输入时的机器人自身坐标原样返回。
- 路标合并比较的是**新观测与此前所接受观测的平均位置**之间的距离（不是
  与某一条原始观测比）；距离恰好等于 `MergeDistance` 上限仍可接纳，严格
  超过才拒绝。

### 这组输入在地图上的结果

配置取初始位姿 `(t=1000, x=0, y=0, heading=0)`、初始位置方差 `0.01`、
最大帧间隔 `1000` 毫秒、路标合并距离 `2` 米。

| | 第 0 帧 t=1100 | 第 1 帧 t=1200 |
|---|---|---|
| 帧输入 | 沿自身前方 `DX=3`、转向 `+π/2`、运动方差 0.02、观测 `door` 自身坐标 `(5,0)` | 沿自身前方 `DX=3`、不转向、运动方差 0.03、观测 `door` 自身坐标 `(0,0)` |
| 平移（按运动前朝向） | 运动前朝向 0（地图 +X）：`(0,0)→(3,0)` | 运动前朝向 π/2（自身 +X 即地图 +Y）：`(3,0)→(3,3)` |
| 运动完成后朝向 | π/2 | π/2（本帧 `DHeading=0`） |
| 观测转换（按运动完成后位姿） | 位姿 `(3,0,π/2)` 下自身 `(5,0)` → 地图 `(3,5)`；路标首次出现，平均位置 `(3,5)`、次数 1 | 位姿 `(3,3,π/2)` 下自身 `(0,0)` → 地图 `(3,3)`；与平均位置 `(3,5)` 的距离恰好等于上限 2m，仍接纳，平均位置变为 `(3,4)`、次数 2 |
| 累计位置方差 | 0.01+0.02=**0.03** | 0.03+0.03=**0.06** |

两次观测转换到地图后是两个不同的点 `(3,5)` 与 `(3,3)`，相距恰好等于
合并上限，所以仍合并为同一个路标：平均位置 `(3,4)`、观测次数 2。

历史查询 `PoseAt(1150)` 选在两帧时间 1100 与 1200 之间，返回的是
**不晚于 1150 的最后一份位姿**，即 1100 帧，返回值携带的是它自己的
帧时间 1100（而不是查询参数 1150）。区域查询用矩形
`(MinX=-3,MinY=3)-(MaxX=3,MaxY=5)`：door 的 x=3 正好压在 MaxX
边界上，**边界包含**所以仍被返回；区域查询只返回当前有效路标。

随后再提交第二段：第 0 帧（t=1300，继续前进 1m、无观测）本身合法，
第 1 帧（t=1400）对 `door` 的观测转换到地图后是 `(-3,4)`，与此前
平均位置 `(3,4)` 相距 6m，超过 2m 上限。于是整段被拒绝——超距发生在
段内已有一帧合法输入之后，但可区分的拒绝原因（`landmark_conflict`）、
**从零开始**的帧序号（1）和路标标识（`door`）都能从 `*RejectError`
取出，且不会部分生效：拒绝后当前位姿仍是 t=1200 那帧，路标仍在
`(3,4)`，次数仍为 2。

### 完整程序

程序随仓库放在 `examples/importtrack/main.go`，内容如下。运行时只需在
命令行指定你自己的本地地图文件路径；该路径必须尚不存在——`Create` 是
排他创建，创建失败（例如文件已存在）时程序立即终止，不要为了运行示例
去删除或覆盖已有文件。需要再次运行时换一个不存在的路径即可，已有地图
则用 `posemap.Open` 打开。

```go
// Command importtrack 演示 posemap 的轨迹导入流程：创建地图、导入两帧
// （同时平移、转向并观测路标）、查询历史位姿与矩形区域路标，以及一次
// 因路标观测超出合并距离而被整体拒绝的导入。
//
// 用法：
//
//	go run ./examples/importtrack <你自己的本地地图文件路径>
//
// 指定的文件必须尚不存在：posemap.Create 不会覆盖或删除已有文件，
// 创建失败时程序直接终止，本次示例不再继续。
package main

import (
	"fmt"
	"log"
	"math"
	"os"

	"github.com/bengzyyys/pose-map/posemap"
)

func main() {
	if len(os.Args) != 2 {
		fmt.Fprintf(os.Stderr, "用法: %s <地图文件路径（必须尚不存在）>\n", os.Args[0])
		os.Exit(2)
	}

	// 位置单位为米，时间为整数毫秒，朝向为弧度。
	m, err := posemap.Create(os.Args[1], posemap.Config{
		InitialTime:     1000, // 初始时间（毫秒）
		InitialX:        0,    // 初始位置 (0,0)（米，地图坐标）
		InitialY:        0,
		InitialHeading:  0,    // 初始朝向 0（弧度，地图 +X 方向）
		InitialVariance: 0.01, // 初始位置方差
		MaxInterval:     1000, // 相邻帧时间间隔上限（毫秒）
		MergeDistance:   2,    // 同标识路标观测合并距离上限（米）
	})
	if err != nil {
		// 路径上已有文件时会走到这里；不要为了运行示例去删除或覆盖它。
		log.Fatalf("创建地图失败，示例终止: %v", err)
	}
	defer func() {
		if err := m.Close(); err != nil {
			log.Fatalf("关闭地图失败: %v", err)
		}
	}()

	// 第一段：两帧，每帧都沿自身前方移动并各观测同一个路标 door 一次。
	// 平移一律按运动前的朝向转换到地图，观测一律按运动完成后的位姿转换。
	res, err := m.ImportSegment(posemap.Segment{
		ID: "seg-1",
		Frames: []posemap.Frame{
			// 第 0 帧：先沿运动前朝向 0（地图 +X）前进 3m 到 (3,0)，
			// 再逆时针转向 π/2；观测按运动完成后的位姿 (3,0,π/2) 转换，
			// 自身坐标 (5,0)（正前方 5m）对应地图坐标 (3,5)。
			{
				Time: 1100, DX: 3, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.02,
				Observations: []posemap.Observation{{ID: "door", X: 5, Y: 0}},
			},
			// 第 1 帧：沿当前朝向 π/2（地图 +Y）前进 3m 到 (3,3)；
			// 自身坐标 (0,0) 转换到地图坐标 (3,3)，与此前平均位置 (3,5)
			// 的距离恰好等于合并上限 2m，仍被接纳；平均位置变为 (3,4)、
			// 观测次数变为 2。
			{
				Time: 1200, DX: 3, DY: 0, DHeading: 0, MoveVariance: 0.03,
				Observations: []posemap.Observation{{ID: "door", X: 0, Y: 0}},
			},
		},
	})
	if err != nil {
		log.Fatalf("seg-1 导入失败: %v", err)
	}
	fmt.Printf("seg-1 导入结果: 末位姿 time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f, 路标 %v\n",
		res.EndPose.Time, res.EndPose.X, res.EndPose.Y, res.EndPose.Heading,
		res.EndPose.Variance, res.LandmarkIDs)

	p1, err := m.PoseAt(1100)
	if err != nil {
		log.Fatalf("查询 1100 帧失败: %v", err)
	}
	printPose("第 0 帧地图位姿", p1)

	cur, err := m.CurrentPose()
	if err != nil {
		log.Fatalf("查询当前位姿失败: %v", err)
	}
	printPose("末帧地图位姿", cur)

	// 历史查询选在两帧时间 1100 与 1200 之间：返回不晚于 1150 的最后
	// 一份位姿，即 1100 帧，并携带它自己的帧时间 1100。
	pmid, err := m.PoseAt(1150)
	if err != nil {
		log.Fatalf("历史位姿查询失败: %v", err)
	}
	printPose("PoseAt(1150) 返回的是 1100 帧", pmid)

	// 矩形覆盖示例路标 (3,4)：MaxX 取 3，door 的 x 正好压在边界上，
	// 边界包含所以仍会返回；区域查询只返回当前有效路标，坐标是地图坐标。
	rect := posemap.Rect{MinX: -3, MinY: 3, MaxX: 3, MaxY: 5}
	lms, err := m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("矩形查询失败: %v", err)
	}
	printLandmarks("矩形 (-3,3)-(3,5) 内当前有效路标", lms)

	// 第二段：第 0 帧合法（继续前进 1m，无观测），第 1 帧对 door 的
	// 观测转换到地图 (-3,4)，距此前平均位置 (3,4) 为 6m，超过 2m 上限，
	// 整段被拒绝，不会有任何部分生效。
	_, err = m.ImportSegment(posemap.Segment{
		ID: "seg-2",
		Frames: []posemap.Frame{
			{Time: 1300, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.01},
			{
				Time: 1400, DX: 0, DY: 0, DHeading: 0, MoveVariance: 0.01,
				Observations: []posemap.Observation{{ID: "door", X: 0, Y: 6}},
			},
		},
	})
	rj, ok := posemap.AsRejectError(err)
	if !ok {
		log.Fatalf("seg-2 应当被拒绝，实际返回 err=%v", err)
	}
	// Frame 是从零开始的帧序号；Landmark 给出涉及的路标标识。
	fmt.Printf("seg-2 被拒绝: 原因=%s 帧序号=%d 路标=%s（%v）\n",
		rj.Kind, rj.Frame, rj.Landmark, err)

	cur, err = m.CurrentPose()
	if err != nil {
		log.Fatalf("拒绝后查询当前位姿失败: %v", err)
	}
	printPose("拒绝后当前位姿（仍是 seg-1 末帧）", cur)

	lms, err = m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("拒绝后矩形查询失败: %v", err)
	}
	// 被拒观测本会落在 (-3,4)（矩形另一头的角上），但它没有生效，
	// 查询结果仍只有此前成功提交的 door，次数仍为 2。
	printLandmarks("拒绝后矩形查询（结果不变）", lms)
}

func printPose(label string, p posemap.Pose) {
	fmt.Printf("%s: time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f\n",
		label, p.Time, p.X, p.Y, p.Heading, p.Variance)
}

func printLandmarks(label string, lms []posemap.Landmark) {
	fmt.Printf("%s: %d 个\n", label, len(lms))
	for _, lm := range lms {
		fmt.Printf("  id=%s x=%.2f y=%.2f count=%d\n", lm.ID, lm.X, lm.Y, lm.Count)
	}
}
```

在仓库根目录运行（把最后一个参数换成你自己的地图文件路径）：

```bash
go run ./examples/importtrack /path/to/your-map.bin
```

预期输出如下，可与上表逐项对应：各帧地图位置、末帧朝向 π/2、累计位置
方差 0.03 与 0.06、路标的平均位置 `(3,4)` 与观测次数 2；拒绝后查询
仍得到此前成功提交的结果，且没有任何部分生效：

```text
seg-1 导入结果: 末位姿 time=1200 x=3.00 y=3.00 heading=1.5707963268 variance=0.06, 路标 [door]
第 0 帧地图位姿: time=1100 x=3.00 y=0.00 heading=1.5707963268 variance=0.03
末帧地图位姿: time=1200 x=3.00 y=3.00 heading=1.5707963268 variance=0.06
PoseAt(1150) 返回的是 1100 帧: time=1100 x=3.00 y=0.00 heading=1.5707963268 variance=0.03
矩形 (-3,3)-(3,5) 内当前有效路标: 1 个
  id=door x=3.00 y=4.00 count=2
seg-2 被拒绝: 原因=landmark_conflict 帧序号=1 路标=door（posemap: frame 1 observation of landmark door is beyond merge distance）
拒绝后当前位姿（仍是 seg-1 末帧）: time=1200 x=3.00 y=3.00 heading=1.5707963268 variance=0.06
拒绝后矩形查询（结果不变）: 1 个
  id=door x=3.00 y=4.00 count=2
```

## 回环校正示例：只覆盖后半段轨迹

下面这份程序演示一次已确认回环校正（`m.Correct`）。它和导入示例使用
同一套坐标约定：**位置与查询结果用地图坐标**（米），**帧内的平移与路标
观测用机器人自身坐标**（米），观测按运动完成后的位姿转换到地图；时间是
整数毫秒，朝向是弧度（校正后归一到 `[-π,π)`），位置方差单位是米²，
路标合并距离 `MergeDistance` 单位是米，比较的是新观测与**当时平均位置**
的距离（恰好等于上限仍接纳，严格超过才拒绝）。

程序放在 `examples/loopcorrection/main.go`，可以独立执行：创建地图、
导入一段四帧轨迹、先撞一次“锚点未命中”，再提交一次同时改变位置与朝向
的成功校正，最后提交一次必然因超合并距离被拒绝的校正，并把拒绝后的
状态与已有校正记录打印出来。

### 这组输入：四帧轨迹与一个路标

配置取初始位姿 `(t=2000, x=-2, y=0, heading=0)`、初始位置方差 `0.01`、
最大帧间隔 `1000` 毫秒、合并距离 `2` 米。一段四帧轨迹如下（平移按运动
前朝向、观测按运动完成后位姿转换）：

| 帧 | 运动完成后地图位姿 | 对 `door` 的观测（自身坐标 → 地图坐标） |
|---|---|---|
| t=2100 | `(0,-2)`，heading `0`，方差 `0.01+0.02=0.03` | 自身 `(5,-4)` → 地图 **`(5,-6)`** |
| t=2200（锚点） | `(0,0)`，heading `π/2`，方差 `0.06` | 无观测 |
| t=2300 | `(4,-5)`，heading 归一为 `-π`，方差 `0.10` | 自身 `(0,0)` → 地图 **`(4,-5)`** |
| t=2400 | `(6,-5)`，heading 归一为 `-π/2`，方差 `0.15` | 无观测（但相对上一帧实际前进了 2m） |

`door` 的两条观测在锚点之前（t=2100）与锚点之后（t=2300）各一条，
相距 `√2 ≈ 1.41m`，在 2m 上限内，合并为同一次出现（出现编号 1）：
平均位置 **`(4.5,-5.5)`**、观测次数 **2**。锚点帧 t=2200 本身没有观测，
这样后面超距拒绝时，冲突观测的帧时间会明显区别于锚点时间。

历史查询 `PoseAt(2150)` 选在 2100 与 2200 之间，返回的是不晚于 2150
的最后一份位姿，即 **2100 帧**，返回值携带它自己的帧时间 2100，而不是
查询参数 2150。

### 锚点必须准确命中已导入帧

先故意把锚点时间给成两帧之间的 `2250`：

```go
m.Correct(posemap.Correction{ID: "corr-miss", Anchor: 2250,
    Target: posemap.CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 0.02}})
```

2250 不是任何已导入帧的时间，校正也**不会借用最近的 2200 帧**，于是以
`anchor_not_found` 拒绝，`RejectError.Time` 是提交的锚点时间 2250。
锚点同样不能取初始位姿 t=2000。这类业务拒绝之后，当前位姿仍是 t=2400
那帧。

### 成功校正：锚点采用目标，后续帧保持相对关系

随后把锚点准确指向 **t=2200**，目标位姿 `(10,0)`、heading `0`、方差
`0.02`：

```go
rec, err := m.Correct(posemap.Correction{ID: "corr-loop", Anchor: 2200,
    Target: posemap.CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 0.02}})
```

锚点从 `(0,0,π/2)` 落到 `(10,0,0)`，**位置平移 10m、朝向顺时针旋转
π/2，两者同时改变**。锚点到末帧作为同一次刚体重定位，保持各帧相对锚点
的位置与朝向，时间不变：

| 查询 | 校正前 | 校正后 |
|---|---|---|
| t=2100（锚点之前） | `(0,-2)`，h=`0`，v=`0.03` | **不变**：`(0,-2)`，h=`0`，v=`0.03` |
| t=2200（锚点） | `(0,0)`，h=`π/2`，v=`0.06` | 精确采用目标：`(10,0)`，h=`0`，v=`0.02` |
| t=2300（后续帧） | `(4,-5)`，h=`-π`，v=`0.10` | `(5,-4)`，h=`π/2`，v=`0.06` |
| t=2400（末帧） | `(6,-5)`，h=`-π/2`，v=`0.15` | `(5,-6)`，h=`-π`，v=`0.11` |

可以逐项验证刚体关系：t=2300 相对锚点原是 `(4,-5)`、朝向差 `-3π/2`，
随锚点旋转 `-π/2` 后相对偏移变为 `(-5,-4)`，加到目标 `(10,0)` 得
`(5,-4)`；t=2400 同理得 `(5,-6)`。**方差从目标方差接续**：锚点取
`0.02`，之后每帧加上锚点之后的**原**运动方差，即 t=2300 为
`0.02+0.04=0.06`、t=2400 为 `0.06+0.05=0.11`；锚点之前的 t=2100
连方差都保持原值 `0.03`。

校正后再做两帧之间的查询 `PoseAt(2250)`，仍返回**实际帧时间 2200**，
但那份位姿已经是校正后的目标位姿 `(10,0,0)`。

### 路标：早期观测固定，受影响观测重新平均，次数不增加

校正不能把路标的平均位置“整个搬过去”。`door` 的两条观测里：

- t=2100 的观测在**锚点之前**，它转换到地图的位置 **`(5,-6)` 固定
  不变**；
- t=2300 的观测在受影响范围内，它的机器人自身坐标 `(0,0)` 不变，但要
  按**校正后**的位姿 `(5,-4,π/2)` 重新转换，落点变为 **`(5,-4)`**。

于是新平均是 `((5,-6)+(5,-4))/2 = ` **`(5,-5)`**，观测次数仍是 **2**，
并不增加新观测。本例的旋转中心恰好在 `(5,-5)`，所以路标平均只移动了
`√0.5 ≈ 0.71m`，而锚点移动了 10m——这正说明平均位置**不是**跟随锚点
做同一次平移旋转：若把旧平均 `(4.5,-5.5)` 当成地图上的一个点整体刚体
搬运（同样旋转 `-π/2` 再平移），它会被搬到 `(4.5,-4.5)`，与实际结果
`(5,-5)` 不符。规则是“锚点前的观测固定、受影响观测按校正后位姿重新
参与等权平均”，整体搬运会错误地改掉那条根本不在校正范围内的早期观测；
而且受影响观测是用**不变的自身坐标按新位姿重新投影**，并不是把它原来
的地图点当刚体搬运（本例该观测自身坐标恰为 `(0,0)`，重投影后就落在新
机器人位置 `(5,-4)`）。重放仍逐条遵守合并距离：本例重放后两条观测相距
还是 `√2 m`，所以校正被接纳。

要区分三类数值：

- **提交目标**：`Correction.Target`，只描述锚点帧“应该在的位姿”；
- **当前查询值**：`PoseAt`/`CurrentPose`/`LandmarksInRect` 校正后立即
  返回新结果（路标的地图坐标是 `(5,-5)`）；
- **记录中的历史快照**：`m.Corrections()` 返回的 `CorrectionRecord` 保存
  提交那一刻每个受影响位姿、每个受影响路标出现的**校正前与校正后值**，
  只增不改；例如记录里 `door` 的快照是
  `(4.5000,-5.5000,count=2) → (5.0000,-5.0000,count=2)`，可与校正前后
  的查询值一一对应。

### 再提交一次必然超距的校正：业务拒绝，不是程序异常

最后用同一锚点 t=2200、同样的旋转角，把目标位置改成 `(30,0)`：

```go
m.Correct(posemap.Correction{ID: "corr-far", Anchor: 2200,
    Target: posemap.CorrectionTarget{X: 30, Y: 0, Heading: 0, Variance: 0.02}})
```

这次重放时，固定的早期观测仍是 `(5,-6)`，而 t=2300 的观测随新位姿变到
`(25,-4)`，两者相距 `√404 ≈ 20.1m`，严格超过 2m。校正整次被拒绝，
从 `*RejectError` 可以取到：

- 原因 `Kind = landmark_conflict`；
- `Time = 2300`：**冲突观测所在帧的实际时间**（不是锚点 2200，也不是
  查询时间）；
- `Landmark = door`、`Occurrence = 1`：冲突的路标标识与其出现编号。

这是一次**业务拒绝**，程序不应当把它当作异常退出：示例打印拒绝原因后
继续查询，可以看到当前位姿仍是 corr-loop 的末帧 `(5,-6)`、`door` 仍在
`(5,-5)` 且次数仍为 2，`m.Corrections()` 里也仍只有 corr-loop 这一条
成功记录（被拒绝的 corr-far 不留任何记录），记录中的历史快照保持成功
校正后的值。与之相对，**创建地图、导入基础轨迹这类准备步骤一旦失败就
明确报错并停止**——例如 `Create` 是排他创建，指定的地图文件已存在时
直接终止，程序不会删除或覆盖那个已有文件。

### 完整程序

程序随仓库放在 `examples/loopcorrection/main.go`，内容如下。运行时在
命令行指定你自己的本地地图文件路径；该路径必须尚不存在，需要再次运行
时换一个路径即可，已有地图请用 `posemap.Open` 打开。

```go
// Command loopcorrection 演示 posemap 的已确认回环校正：创建地图、导入
// 一段四帧轨迹（同一个路标 door 在锚点之前与锚点之后都有观测，锚点
// 之后还保留一帧有实际位移），先演示一次锚点没有命中已导入帧的拒绝，再
// 提交一次同时改变位置与朝向的成功校正，最后提交一次会让 door 的观测
// 超过合并距离的校正并确认它只是业务拒绝、状态保持成功校正后的结果。
//
// 用法：
//
//	go run ./examples/loopcorrection <你自己的本地地图文件路径>
//
// 指定的文件必须尚不存在：posemap.Create 不会覆盖或删除已有文件，
// 创建失败时程序直接终止，本次示例不再继续。
package main

import (
	"fmt"
	"log"
	"math"
	"os"

	"github.com/bengzyyys/pose-map/posemap"
)

func main() {
	if len(os.Args) != 2 {
		fmt.Fprintf(os.Stderr, "用法: %s <地图文件路径（必须尚不存在）>\n", os.Args[0])
		os.Exit(2)
	}

	// 位置单位为米，时间为整数毫秒，朝向为弧度，位置方差为米²。
	m, err := posemap.Create(os.Args[1], posemap.Config{
		InitialTime:     2000, // 初始时间（毫秒）
		InitialX:        -2,   // 初始位置 (-2,0)（米，地图坐标）
		InitialY:        0,
		InitialHeading:  0,    // 初始朝向 0（弧度，地图 +X 方向）
		InitialVariance: 0.01, // 初始位置方差（米²）
		MaxInterval:     1000, // 相邻帧时间间隔上限（毫秒）
		MergeDistance:   2,    // 同标识路标观测合并距离上限（米）
	})
	if err != nil {
		// 路径上已有文件时会走到这里；不要为了运行示例去删除或覆盖它。
		log.Fatalf("创建地图失败，示例终止: %v", err)
	}
	defer func() {
		if err := m.Close(); err != nil {
			log.Fatalf("关闭地图失败: %v", err)
		}
	}()

	// 四帧轨迹（运动学与导入示例相同：平移按运动前朝向、观测按运动完成后
	// 位姿转换）。door 在锚点之前与锚点之后各被观测一次（锚点帧本身没有
	// 观测），两次观测在地图上相距 √2 m，落在 2m 合并上限内，属于同一次
	// 出现（出现编号 1）。
	//
	//	t=2100 位姿 (0,-2)  h=0     观测 door → 地图 (5,-6)
	//	t=2200 位姿 (0,0)   h=π/2   （无观测）                  ← 校正锚点
	//	t=2300 位姿 (4,-5)  h=-π    观测 door → 地图 (4,-5)
	//	t=2400 位姿 (6,-5)  h=-π/2  （前进 2m，无观测，有实际位移）
	//
	// 两次观测平均位置 (4.5,-5.5)，次数 2。
	_, err = m.ImportSegment(posemap.Segment{
		ID: "seg-loop",
		Frames: []posemap.Frame{
			// 第 0 帧：沿朝向 0 移动 (2,-2) 到 (0,-2)，不转向；
			// door 的自身坐标 (5,-4) 按完成后位姿转换到地图 (5,-6)。
			{
				Time: 2100, DX: 2, DY: -2, DHeading: 0, MoveVariance: 0.02,
				Observations: []posemap.Observation{{ID: "door", X: 5, Y: -4}},
			},
			// 第 1 帧：沿朝向 0 向自身 +Y 移动 2m 到 (0,0)，再逆时针
			// 转向 π/2；本帧没有路标观测。这一帧是后面的校正锚点。
			{Time: 2200, DX: 0, DY: 2, DHeading: math.Pi / 2, MoveVariance: 0.03},
			// 第 2 帧：沿朝向 π/2 的自身坐标移动 (-5,-4)（地图 +X 4m、
			// -Y 5m）到 (4,-5)，再转向 π（归一后为 -π）；door 恰好在
			// 机器人位置，自身 (0,0) → 地图 (4,-5)，与上一条观测 (5,-6)
			// 相距 √2 m，接纳。
			{
				Time: 2300, DX: -5, DY: -4, DHeading: math.Pi / 2, MoveVariance: 0.04,
				Observations: []posemap.Observation{{ID: "door", X: 0, Y: 0}},
			},
			// 第 3 帧：沿朝向 π 前进 2m（地图 +X）到 (6,-5)，再转向
			// -π/2；没有路标观测，但锚点之后保留一帧有实际位移的轨迹。
			{Time: 2400, DX: -2, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.05},
		},
	})
	if err != nil {
		log.Fatalf("seg-loop 导入失败，示例终止: %v", err)
	}

	// ---- 校正前：锚点帧、后续帧与 door 的当前查询值 ----
	p2200, err := m.PoseAt(2200)
	if err != nil {
		log.Fatalf("查询锚点帧失败: %v", err)
	}
	printPose("校正前 t=2200 锚点帧", p2200)
	p2300, err := m.PoseAt(2300)
	if err != nil {
		log.Fatalf("查询 t=2300 帧失败: %v", err)
	}
	printPose("校正前 t=2300 后续帧", p2300)
	p2400, err := m.PoseAt(2400)
	if err != nil {
		log.Fatalf("查询 t=2400 末帧失败: %v", err)
	}
	printPose("校正前 t=2400 末帧", p2400)
	printDoor(m, "校正前 door（2 次观测平均，次数 2）")

	// 历史查询选在两帧之间：返回的仍是实际帧时间。2150 位于 2100 与
	// 2200 之间，PoseAt 返回不晚于它的最后一份位姿，即 2100 帧。
	pmid, err := m.PoseAt(2150)
	if err != nil {
		log.Fatalf("两帧之间历史查询失败: %v", err)
	}
	printPose("PoseAt(2150) 返回的是 2100 帧（注意 Time 是实际帧时间）", pmid)

	// ---- 第一次提交：锚点给在两帧之间，必须被拒绝 ----
	// 2250 没有对应任何已导入帧；校正不会借用“最近一帧”2200。
	_, err = m.Correct(posemap.Correction{
		ID:     "corr-miss",
		Anchor: 2250,
		Target: posemap.CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 0.02},
	})
	rj, ok := posemap.AsRejectError(err)
	if !ok {
		log.Fatalf("锚点 2250 应当被拒绝，实际返回 err=%v", err)
	}
	fmt.Printf("锚点未命中被拒绝: 原因=%s 提交锚点时间=%d hasTime=%v（%v）\n",
		rj.Kind, rj.Time, rj.HasTime, err)
	cur, err := m.CurrentPose()
	if err != nil {
		log.Fatalf("拒绝后查询当前位姿失败: %v", err)
	}
	printPose("拒绝后当前位姿不变", cur)

	// ---- 第二次提交：锚点准确命中 t=2200，成功校正 ----
	// 目标让锚点从 (0,0,π/2) 落到 (10,0,0)：位置平移 10m、朝向顺时针
	// 旋转 π/2，成功后位置与朝向同时改变。锚点至末帧作为同一次刚体重
	// 定位：锚点之前 t=2100 的观测 (5,-6) 固定不动，它距这次旋转的旋转
	// 中心 (5,-5) 只有 1m；锚点之后 t=2300 的观测 (4,-5) 转到 (5,-4)，
	// 与固定观测仍相距 √2 m，重放成功，新平均为 (5,-5)。目标方差 0.02
	// （米²）从锚点接续，之后每帧累加锚点之后的原运动方差。
	rec, err := m.Correct(posemap.Correction{
		ID:     "corr-loop",
		Anchor: 2200,
		Target: posemap.CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 0.02},
	})
	if err != nil {
		log.Fatalf("成功校正意外失败: %v", err)
	}
	fmt.Printf("校正 corr-loop 成功: 锚点=%d 受影响末帧=%d 受影响位姿 %d 份、路标出现 %d 个\n",
		rec.Anchor, rec.EndTime, len(rec.Poses), len(rec.Landmarks))

	// 锚点之前的位姿完全不变；锚点采用目标位姿；后续帧保持与锚点的相对
	// 关系整体刚体重定位。
	p2100, err := m.PoseAt(2100)
	if err != nil {
		log.Fatalf("校正后查询 2100 帧失败: %v", err)
	}
	printPose("校正后 t=2100（锚点之前，保持不变）", p2100)
	p2200, err = m.PoseAt(2200)
	if err != nil {
		log.Fatalf("校正后查询锚点帧失败: %v", err)
	}
	printPose("校正后 t=2200 锚点帧（即提交目标）", p2200)
	p2300, err = m.PoseAt(2300)
	if err != nil {
		log.Fatalf("校正后查询 2300 帧失败: %v", err)
	}
	printPose("校正后 t=2300 后续帧", p2300)
	p2400, err = m.PoseAt(2400)
	if err != nil {
		log.Fatalf("校正后查询 2400 帧失败: %v", err)
	}
	printPose("校正后 t=2400 末帧", p2400)
	// 两帧之间的查询仍返回实际帧时间，且拿到的是校正后的 2200 帧。
	pmid, err = m.PoseAt(2250)
	if err != nil {
		log.Fatalf("校正后两帧之间查询失败: %v", err)
	}
	printPose("PoseAt(2250) 返回的是校正后的 2200 帧", pmid)
	printDoor(m, "校正后 door（早期观测固定，受影响观测按新位姿重新平均）")

	// ---- 第三次提交：同样锚定 t=2200，但平移到 x=30，观测必然超距 ----
	// 锚点帧本身没有观测；重放时锚点之前 t=2100 的观测固定在 (5,-6)，
	// t=2300 的观测随这次校正变到 (25,-4)，与固定观测相距 √404 ≈
	// 20.1m，超过 2m 上限。第一条冲突观测在 t=2300（不是锚点时间），
	// 整次拒绝、不留部分更新。
	_, err = m.Correct(posemap.Correction{
		ID:     "corr-far",
		Anchor: 2200,
		Target: posemap.CorrectionTarget{X: 30, Y: 0, Heading: 0, Variance: 0.02},
	})
	rj, ok = posemap.AsRejectError(err)
	if !ok {
		log.Fatalf("corr-far 应当被拒绝，实际返回 err=%v", err)
	}
	// 业务拒绝不是程序异常：打印原因、冲突观测所在的实际帧时间、路标
	// 标识与出现编号，然后继续用查询证明状态没有被改动。
	fmt.Printf("超距校正被拒绝: 原因=%s 冲突帧时间=%d 路标=%s 出现编号=%d hasOccurrence=%v（%v）\n",
		rj.Kind, rj.Time, rj.Landmark, rj.Occurrence, rj.HasOccurrence, err)

	cur, err = m.CurrentPose()
	if err != nil {
		log.Fatalf("拒绝后查询当前位姿失败: %v", err)
	}
	printPose("拒绝后当前位姿（仍是 corr-loop 的末帧结果）", cur)
	printDoor(m, "拒绝后 door（位置与次数不变）")

	// 已有校正记录保持成功校正后的结果：被拒绝的 corr-far 没有产生记录，
	// 记录里保存的是提交时的历史快照（校正前后值），不会随当前状态改写。
	recs, err := m.Corrections()
	if err != nil {
		log.Fatalf("查询校正记录失败: %v", err)
	}
	fmt.Printf("校正记录共 %d 条（被业务拒绝的提交不留记录）:\n", len(recs))
	for _, r := range recs {
		fmt.Printf("  记录 id=%s 锚点=%d 受影响末帧=%d 提交目标={x=%.2f y=%.2f heading=%.10f variance=%.2f}\n",
			r.ID, r.Anchor, r.EndTime, r.Target.X, r.Target.Y, r.Target.Heading, r.Target.Variance)
		for _, pc := range r.Poses {
			fmt.Printf("    位姿 time=%d 校正前 (%.2f,%.2f,h=%.10f,v=%.2f) → 快照校正后 (%.2f,%.2f,h=%.10f,v=%.2f)\n",
				pc.Before.Time, pc.Before.X, pc.Before.Y, pc.Before.Heading, pc.Before.Variance,
				pc.After.X, pc.After.Y, pc.After.Heading, pc.After.Variance)
		}
		for _, lc := range r.Landmarks {
			fmt.Printf("    路标 %s 出现 %d 校正前 (%.4f,%.4f,count=%d) → 快照校正后 (%.4f,%.4f,count=%d)\n",
				lc.ID, lc.Occurrence, lc.Before.X, lc.Before.Y, lc.Before.Count,
				lc.After.X, lc.After.Y, lc.After.Count)
		}
	}
}

// printDoor 用一个足以覆盖本示例的大矩形取出 door，打印的是当前查询值
// （地图坐标的平均位置与观测次数），区别于校正记录里保存的历史快照。
func printDoor(m *posemap.Map, label string) {
	lms, err := m.LandmarksInRect(posemap.Rect{MinX: -100, MinY: -100, MaxX: 100, MaxY: 100})
	if err != nil {
		log.Fatalf("查询 door 失败: %v", err)
	}
	for _, lm := range lms {
		if lm.ID == "door" {
			fmt.Printf("%s: x=%.4f y=%.4f count=%d\n", label, lm.X, lm.Y, lm.Count)
			return
		}
	}
	log.Fatalf("查询范围内没有 door")
}

func printPose(label string, p posemap.Pose) {
	fmt.Printf("%s: time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f\n",
		label, p.Time, p.X, p.Y, p.Heading, p.Variance)
}
```

在仓库根目录运行（把最后一个参数换成你自己的、尚不存在的地图文件路径）：

```bash
go run ./examples/loopcorrection /path/to/your-loop-map.bin
```

预期输出如下，可与上面的表格逐项对应：校正前锚点 `(0,0,π/2)`、door
`(4.5,-5.5)` 次数 2；两帧之间查询返回实际帧时间；锚点 2250 以
`anchor_not_found` 拒绝且状态不变；成功校正后锚点精确为目标
`(10,0,0)`、t=2100 完全不变、后续两帧保持相对关系、方差从 0.02 接续、
door 变为 `(5,-5)` 且次数仍为 2；超距校正报出冲突帧时间 2300、路标
door、出现编号 1，随后位姿、路标与校正记录都保持成功校正后的结果：

```text
校正前 t=2200 锚点帧: time=2200 x=0.00 y=0.00 heading=1.5707963268 variance=0.06
校正前 t=2300 后续帧: time=2300 x=4.00 y=-5.00 heading=-3.1415926536 variance=0.10
校正前 t=2400 末帧: time=2400 x=6.00 y=-5.00 heading=-1.5707963268 variance=0.15
校正前 door（2 次观测平均，次数 2）: x=4.5000 y=-5.5000 count=2
PoseAt(2150) 返回的是 2100 帧（注意 Time 是实际帧时间）: time=2100 x=0.00 y=-2.00 heading=0.0000000000 variance=0.03
锚点未命中被拒绝: 原因=anchor_not_found 提交锚点时间=2250 hasTime=true（posemap: anchor time 2250 does not match an imported frame）
拒绝后当前位姿不变: time=2400 x=6.00 y=-5.00 heading=-1.5707963268 variance=0.15
校正 corr-loop 成功: 锚点=2200 受影响末帧=2400 受影响位姿 3 份、路标出现 1 个
校正后 t=2100（锚点之前，保持不变）: time=2100 x=0.00 y=-2.00 heading=0.0000000000 variance=0.03
校正后 t=2200 锚点帧（即提交目标）: time=2200 x=10.00 y=0.00 heading=0.0000000000 variance=0.02
校正后 t=2300 后续帧: time=2300 x=5.00 y=-4.00 heading=1.5707963268 variance=0.06
校正后 t=2400 末帧: time=2400 x=5.00 y=-6.00 heading=-3.1415926536 variance=0.11
PoseAt(2250) 返回的是校正后的 2200 帧: time=2200 x=10.00 y=0.00 heading=0.0000000000 variance=0.02
校正后 door（早期观测固定，受影响观测按新位姿重新平均）: x=5.0000 y=-5.0000 count=2
超距校正被拒绝: 原因=landmark_conflict 冲突帧时间=2300 路标=door 出现编号=1 hasOccurrence=true（posemap: correction conflicts landmark door occurrence 1 at frame time 2300）
拒绝后当前位姿（仍是 corr-loop 的末帧结果）: time=2400 x=5.00 y=-6.00 heading=-3.1415926536 variance=0.11
拒绝后 door（位置与次数不变）: x=5.0000 y=-5.0000 count=2
校正记录共 1 条（被业务拒绝的提交不留记录）:
  记录 id=corr-loop 锚点=2200 受影响末帧=2400 提交目标={x=10.00 y=0.00 heading=0.0000000000 variance=0.02}
    位姿 time=2200 校正前 (0.00,0.00,h=1.5707963268,v=0.06) → 快照校正后 (10.00,0.00,h=0.0000000000,v=0.02)
    位姿 time=2300 校正前 (4.00,-5.00,h=-3.1415926536,v=0.10) → 快照校正后 (5.00,-4.00,h=1.5707963268,v=0.06)
    位姿 time=2400 校正前 (6.00,-5.00,h=-1.5707963268,v=0.15) → 快照校正后 (5.00,-6.00,h=-3.1415926536,v=0.11)
    路标 door 出现 1 校正前 (4.5000,-5.5000,count=2) → 快照校正后 (5.0000,-5.0000,count=2)
```

`Ready()` 仍返回 true，表示基线包可以加载。
