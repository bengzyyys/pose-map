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

`Ready()` 仍返回 true，表示基线包可以加载。
