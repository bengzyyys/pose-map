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

## 回环校正示例：只覆盖后半段轨迹的重定位

下面这份程序走完整条校正流程：创建地图、导入一段三帧轨迹（同一个路标
在锚点前后都有观测，锚点之后仍有一帧实际位移），提交一次只覆盖后半段
轨迹的成功校正（位置与朝向同时改变），再提交一次会让路标观测超出合并
距离而被整体拒绝的校正，最后确认拒绝后状态不变。坐标约定与导入示例
相同：位置是**地图坐标**（米），观测输入是**机器人自身坐标**（米），
时间是整数毫秒，朝向是弧度（归一到 [-π,π)），方差是位置方差，合并
距离单位是米。

### 这组输入在地图上的结果

配置与导入示例相同：初始位姿 `(t=1000, x=0, y=0, heading=0)`、初始
位置方差 `0.01`、最大帧间隔 `1000` 毫秒、路标合并距离 `2` 米。

| | 第 0 帧 t=1100 | 第 1 帧 t=1200（校正锚点） | 第 2 帧 t=1300 |
|---|---|---|---|
| 帧输入 | 沿自身前方 `DX=3`、不转向、运动方差 0.02、观测 `door` 自身坐标 `(5,0)` | 沿自身前方 `DX=3`、转向 `+π/2`、运动方差 0.03、观测 `door` 自身坐标 `(1,-1)` | 沿自身前方 `DX=2`、不转向、运动方差 0.04、观测 `door` 自身坐标 `(-1,0)` |
| 校正前位姿 | `(3,0)` 朝向 0，方差 0.01+0.02=**0.03** | `(6,0)` 朝向 π/2，方差 0.03+0.03=**0.06** | `(6,2)` 朝向 π/2，方差 0.06+0.04=**0.10** |
| 观测转换到地图 | `(8,0)`；首次出现，平均 `(8,0)`、次数 1 | `(7,1)`；与 `(8,0)` 相距 √2m，平均 `(7.5,0.5)`、次数 2 | `(6,1)`；与 `(7.5,0.5)` 相距 √2.5m，平均 `(7,0.67)`、次数 3 |

### 成功校正 loop-1：锚点采用目标位姿，后续帧保持相对关系

提交 `Correct(Correction{ID: "loop-1", Anchor: 1200, Target: {X: 6.5, Y: 1, Heading: π/4, Variance: 0.05}})`。
锚点 `1200` 准确命中已导入的第 1 帧；目标同时改变位置与朝向。结果：

- **锚点之前的位姿不变**：1100 帧仍是 `(3,0)` 朝向 0、方差 0.03。
- **锚点采用目标位姿**：1200 帧变为 `(6.5,1)` 朝向 π/4、方差取目标
  方差 0.05。
- **后续帧保持与锚点的相对关系**：1300 帧校正前相对锚点的偏移 `(0,2)`
  随锚点旋转 `-π/4` 后变为 `(√2,√2)`，落在新锚点上得 `(7.91,2.41)`，
  朝向叠加同一旋转角后为 π/4；方差从目标方差接续锚点之后的运动方差，
  即 0.05+0.04=**0.09**。帧时间不变。
- **路标重新平均但次数不增**：1100 帧的早期观测在锚点之前，地图位置
  `(8,0)` 固定不动；1200、1300 两帧的受影响观测按校正后位姿重新转换，
  分别落在 `(7.91,1)` 与 `(7.21,1.71)`，与固定观测依原时间次序重新
  做增量平均（仍遵守 2m 合并距离），平均位置从 `(7,0.67)` 变为
  `(7.71,0.90)`，观测次数仍为 3——重放只是重新计算位置，不新增观测。

这里能看出为什么不能把整个路标的平均位置一起搬到新位置：平均值由
三次观测组成，其中锚点之前的观测固定不动，只有锚点及之后的观测随
校正改变。若把平均值整体按锚点的平移旋转搬走，等于把固定观测也搬了，
区域查询的结果就与校正记录里"早期观测不变、受影响观测重放"的前后值
对不上。正确结果只移动一部分——`(7,0.67)→(7.71,0.90)`，而不是跟随
锚点走完全部位移。

历史查询 `PoseAt(1250)` 选在校正后的 1200 与 1300 两帧之间，返回
**不晚于 1250 的最后一份位姿**，即校正后的 1200 帧，返回值携带的是
它自己的帧时间 1200。注意 1250 只是查询参数：**校正锚点必须准确命中
已导入帧的时间**，把两帧之间的查询时间（如 1250）或初始位姿时间
（1000）当作锚点提交，会被以 `anchor_not_found` 整次拒绝。

读输出时注意区分三类数值：**提交目标**（`Target`，调用者声明锚点应有
的位姿与方差）、**当前查询值**（`PoseAt`/`CurrentPose`/
`LandmarksInRect` 返回的最新状态）、**记录中的历史快照**
（`Corrections()` 里每条 `CorrectionRecord` 的 `Before`/`After`，
生成后永不改写，后续校正或拒绝都不会回头修改它）。

### 会被拒绝的第二次校正：超出合并距离

随后提交 `loop-2`：锚点仍为 1200，目标 `(6.5,5,π/4)`——把锚点沿地图
+Y 再平移 4m。重放后 1200 帧对 `door` 的观测会落在 `(7.91,5)`，与
固定的早期观测 `(8,0)` 相距约 5m，超过 2m 合并上限，**整次校正被
拒绝**。这是业务拒绝而非程序异常：程序用 `AsRejectError` 取出可区分
的拒绝原因（`landmark_conflict`）、冲突观测所在帧的**实际时间**
（1200）、路标标识（`door`）与出现编号（1）后照常继续；位姿、路标
位置与已有校正记录都保持 loop-1 成功校正后的结果，不留部分更新。

### 完整程序

程序随仓库放在 `examples/loopclose/main.go`，内容如下。与导入示例
一样，运行时只需在命令行指定你自己的本地地图文件路径；该路径必须
尚不存在——`Create` 是排他创建，创建失败（例如文件已存在）时程序
立即终止，不要为了运行示例去删除或覆盖已有文件。创建地图、导入基础
轨迹等准备步骤失败时程序同样明确报错并停止后续操作；只有校正被业务
拒绝属于预期流程，不作为程序异常退出。

```go
// Command loopclose 演示 posemap 的回环校正流程：创建地图、导入一段三帧
// 轨迹（锚点前后都观测同一个路标，锚点之后仍有实际位移），提交一次只覆盖
// 后半段轨迹的成功校正（位置与朝向同时改变），再提交一次会让路标观测超出
// 合并距离而被整体拒绝的校正，并展示拒绝后位姿、路标与已有校正记录都保持
// 成功校正后的结果。
//
// 用法：
//
//	go run ./examples/loopclose <你自己的本地地图文件路径>
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

	// 基础轨迹：三帧，每帧都沿自身前方移动，并各观测同一个路标 door 一次。
	// 平移一律按运动前的朝向转换到地图，观测一律按运动完成后的位姿转换。
	// 这是校正前的准备步骤，失败属于程序错误，直接终止。
	res, err := m.ImportSegment(posemap.Segment{
		ID: "seg-1",
		Frames: []posemap.Frame{
			// 第 0 帧：沿运动前朝向 0（地图 +X）前进 3m 到 (3,0)，不转向；
			// 观测按运动完成后的位姿 (3,0,0) 转换，自身坐标 (5,0) 对应
			// 地图坐标 (8,0)。路标首次出现，平均位置 (8,0)、次数 1。
			{
				Time: 1100, DX: 3, DY: 0, DHeading: 0, MoveVariance: 0.02,
				Observations: []posemap.Observation{{ID: "door", X: 5, Y: 0}},
			},
			// 第 1 帧（后来的校正锚点）：沿运动前朝向 0 前进 3m 到 (6,0)，
			// 再逆时针转向 π/2；观测按运动完成后的位姿 (6,0,π/2) 转换，
			// 自身坐标 (1,-1) 对应地图坐标 (7,1)，与平均位置 (8,0) 相距
			// √2m，在 2m 上限内；平均位置变为 (7.5,0.5)、次数 2。
			{
				Time: 1200, DX: 3, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.03,
				Observations: []posemap.Observation{{ID: "door", X: 1, Y: -1}},
			},
			// 第 2 帧：沿运动前朝向 π/2（地图 +Y）前进 2m 到 (6,2)，不转向；
			// 自身坐标 (-1,0) 对应地图坐标 (6,1)，与平均位置 (7.5,0.5)
			// 相距 √2.5m，仍在上限内；平均位置变为 (7,0.67)、次数 3。
			{
				Time: 1300, DX: 2, DY: 0, DHeading: 0, MoveVariance: 0.04,
				Observations: []posemap.Observation{{ID: "door", X: -1, Y: 0}},
			},
		},
	})
	if err != nil {
		log.Fatalf("seg-1 导入失败，示例终止: %v", err)
	}
	fmt.Printf("seg-1 导入结果: 末位姿 time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f, 路标 %v\n",
		res.EndPose.Time, res.EndPose.X, res.EndPose.Y, res.EndPose.Heading,
		res.EndPose.Variance, res.LandmarkIDs)

	// 校正前的完整状态：三帧位姿与路标平均位置。
	p1100, err := m.PoseAt(1100)
	if err != nil {
		log.Fatalf("查询 1100 帧失败: %v", err)
	}
	printPose("校正前 1100 帧（锚点之前，校正后保持不变）", p1100)

	p1200, err := m.PoseAt(1200)
	if err != nil {
		log.Fatalf("查询 1200 帧失败: %v", err)
	}
	printPose("校正前锚点 1200 帧", p1200)

	cur, err := m.CurrentPose()
	if err != nil {
		log.Fatalf("查询当前位姿失败: %v", err)
	}
	printPose("校正前末帧 1300", cur)

	// 矩形覆盖 door 校正前后的位置 (7,0.67) 与 (7.71,0.90)。
	rect := posemap.Rect{MinX: 5, MinY: -1, MaxX: 9, MaxY: 2}
	lms, err := m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("矩形查询失败: %v", err)
	}
	printLandmarks("校正前矩形 (5,-1)-(9,2) 内路标", lms)

	// 第一次校正：锚点必须准确命中已导入帧的时间（这里是 1200，不能是
	// 初始位姿 1000，也不能是两帧之间的查询时间）。目标同时改变位置与
	// 朝向：锚点从 (6,0,π/2) 重定位到 (6.5,1,π/4)，方差重置为 0.05。
	// 锚点至末帧整体平移旋转、保持相对位姿；锚点之前的 1100 帧不变。
	rec, err := m.Correct(posemap.Correction{
		ID:     "loop-1",
		Anchor: 1200,
		Target: posemap.CorrectionTarget{X: 6.5, Y: 1, Heading: math.Pi / 4, Variance: 0.05},
	})
	if err != nil {
		log.Fatalf("loop-1 校正应当成功，实际返回 err=%v", err)
	}
	fmt.Printf("校正 loop-1 已接受: 锚点=%d 目标 x=%.2f y=%.2f heading=%.10f variance=%.2f, 受影响末帧=%d\n",
		rec.Anchor, rec.Target.X, rec.Target.Y, rec.Target.Heading, rec.Target.Variance, rec.EndTime)
	for _, pc := range rec.Poses {
		fmt.Printf("  位姿 %d: 前 x=%.2f y=%.2f heading=%.10f variance=%.2f → 后 x=%.2f y=%.2f heading=%.10f variance=%.2f\n",
			pc.Before.Time, pc.Before.X, pc.Before.Y, pc.Before.Heading, pc.Before.Variance,
			pc.After.X, pc.After.Y, pc.After.Heading, pc.After.Variance)
	}
	for _, lc := range rec.Landmarks {
		fmt.Printf("  路标 %s 第 %d 次出现: 前 x=%.2f y=%.2f → 后 x=%.2f y=%.2f, 次数 %d→%d\n",
			lc.ID, lc.Occurrence, lc.Before.X, lc.Before.Y, lc.After.X, lc.After.Y,
			lc.Before.Count, lc.After.Count)
	}

	// 校正后的当前查询值：1100 帧不变；锚点采用目标位姿；1300 帧保持与
	// 锚点的相对关系（校正前相对锚点的偏移 (0,2) 随锚点旋转 -π/4 后变为
	// (√2,√2)，朝向同为 π/4）；方差从目标方差 0.05 接续 1300 帧的运动
	// 方差 0.04，得到 0.09。
	p1100, err = m.PoseAt(1100)
	if err != nil {
		log.Fatalf("校正后查询 1100 帧失败: %v", err)
	}
	printPose("校正后 1100 帧（不变）", p1100)

	p1200, err = m.PoseAt(1200)
	if err != nil {
		log.Fatalf("校正后查询 1200 帧失败: %v", err)
	}
	printPose("校正后锚点 1200 帧（采用目标位姿）", p1200)

	cur, err = m.CurrentPose()
	if err != nil {
		log.Fatalf("校正后查询当前位姿失败: %v", err)
	}
	printPose("校正后末帧 1300（保持与锚点的相对关系）", cur)

	// 历史查询选在两帧时间 1200 与 1300 之间：返回不晚于 1250 的最后
	// 一份位姿，即校正后的 1200 帧，并携带它自己的帧时间 1200。注意
	// 1250 只是查询参数，不能作为校正锚点——锚点必须准确命中已导入帧，
	// 用 1250 提交校正会被以 anchor_not_found 拒绝。
	pmid, err := m.PoseAt(1250)
	if err != nil {
		log.Fatalf("历史位姿查询失败: %v", err)
	}
	printPose("PoseAt(1250) 返回的是 1200 帧", pmid)

	// 路标的早期观测（1100 帧，地图位置 (8,0)）保持不动，受影响帧的
	// 观测按校正后位姿重新参与平均：平均位置从 (7,0.67) 变为 (7.71,0.90)，
	// 观测次数仍为 3，不会因为重放而增加。
	lms, err = m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("校正后矩形查询失败: %v", err)
	}
	printLandmarks("校正后矩形 (5,-1)-(9,2) 内路标", lms)

	// 第二次校正：把锚点沿地图 +Y 平移 4m。重放后 1200 帧对 door 的
	// 观测会落在 (7.91,5)，与固定的早期观测 (8,0) 相距约 5m，超过 2m
	// 合并上限，整次校正被拒绝。这是业务拒绝，不是程序异常：用
	// AsRejectError 取出可区分的原因、冲突观测所在帧的实际时间、路标
	// 标识与出现编号，程序继续运行。
	_, err = m.Correct(posemap.Correction{
		ID:     "loop-2",
		Anchor: 1200,
		Target: posemap.CorrectionTarget{X: 6.5, Y: 5, Heading: math.Pi / 4, Variance: 0.05},
	})
	rj, ok := posemap.AsRejectError(err)
	if !ok {
		log.Fatalf("loop-2 应当被业务拒绝，实际返回 err=%v", err)
	}
	fmt.Printf("loop-2 被拒绝: 原因=%s 冲突帧时间=%d 路标=%s 出现编号=%d（%v）\n",
		rj.Kind, rj.Time, rj.Landmark, rj.Occurrence, err)

	// 拒绝不留部分更新：位姿、路标位置与已有校正记录都保持 loop-1
	// 成功校正后的结果。
	cur, err = m.CurrentPose()
	if err != nil {
		log.Fatalf("拒绝后查询当前位姿失败: %v", err)
	}
	printPose("拒绝后当前位姿（仍是 loop-1 校正后的末帧）", cur)

	lms, err = m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("拒绝后矩形查询失败: %v", err)
	}
	printLandmarks("拒绝后矩形查询（结果不变）", lms)

	recs, err := m.Corrections()
	if err != nil {
		log.Fatalf("查询校正记录失败: %v", err)
	}
	fmt.Printf("已有校正记录: %d 条\n", len(recs))
	for _, r := range recs {
		fmt.Printf("  记录 %s: 锚点=%d 受影响末帧=%d\n", r.ID, r.Anchor, r.EndTime)
	}
	// 记录中的校正前后值是历史快照：即使之后又有别的校正或拒绝，loop-1
	// 记录里锚点的 Before 仍是最初导入时的 (6,0,π/2)，与当前查询值
	// (6.5,1,π/4) 区分开。
	first := recs[0].Poses[0]
	fmt.Printf("记录中 loop-1 锚点校正前快照（历史值，不被后续操作改写）: time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f\n",
		first.Before.Time, first.Before.X, first.Before.Y, first.Before.Heading, first.Before.Variance)
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
go run ./examples/loopclose /path/to/your-map.bin
```

预期输出如下，可与上文逐项对应：锚点采用提交目标 `(6.5,1,π/4)`、
1300 帧保持相对关系到 `(7.91,2.41)`、方差接续为 0.05 与 0.09、1100
帧不变；路标平均位置只移动一部分到 `(7.71,0.90)`、次数仍为 3；
`PoseAt(1250)` 返回的是 1200 帧并携带实际帧时间；loop-2 被
`landmark_conflict` 拒绝（冲突帧时间 1200、路标 door、出现编号 1），
拒绝后位姿、路标与校正记录都保持 loop-1 成功校正后的结果：

```text
seg-1 导入结果: 末位姿 time=1300 x=6.00 y=2.00 heading=1.5707963268 variance=0.10, 路标 [door]
校正前 1100 帧（锚点之前，校正后保持不变）: time=1100 x=3.00 y=0.00 heading=0.0000000000 variance=0.03
校正前锚点 1200 帧: time=1200 x=6.00 y=0.00 heading=1.5707963268 variance=0.06
校正前末帧 1300: time=1300 x=6.00 y=2.00 heading=1.5707963268 variance=0.10
校正前矩形 (5,-1)-(9,2) 内路标: 1 个
  id=door x=7.00 y=0.67 count=3
校正 loop-1 已接受: 锚点=1200 目标 x=6.50 y=1.00 heading=0.7853981634 variance=0.05, 受影响末帧=1300
  位姿 1200: 前 x=6.00 y=0.00 heading=1.5707963268 variance=0.06 → 后 x=6.50 y=1.00 heading=0.7853981634 variance=0.05
  位姿 1300: 前 x=6.00 y=2.00 heading=1.5707963268 variance=0.10 → 后 x=7.91 y=2.41 heading=0.7853981634 variance=0.09
  路标 door 第 1 次出现: 前 x=7.00 y=0.67 → 后 x=7.71 y=0.90, 次数 3→3
校正后 1100 帧（不变）: time=1100 x=3.00 y=0.00 heading=0.0000000000 variance=0.03
校正后锚点 1200 帧（采用目标位姿）: time=1200 x=6.50 y=1.00 heading=0.7853981634 variance=0.05
校正后末帧 1300（保持与锚点的相对关系）: time=1300 x=7.91 y=2.41 heading=0.7853981634 variance=0.09
PoseAt(1250) 返回的是 1200 帧: time=1200 x=6.50 y=1.00 heading=0.7853981634 variance=0.05
校正后矩形 (5,-1)-(9,2) 内路标: 1 个
  id=door x=7.71 y=0.90 count=3
loop-2 被拒绝: 原因=landmark_conflict 冲突帧时间=1200 路标=door 出现编号=1（posemap: correction conflicts landmark door occurrence 1 at frame time 1200）
拒绝后当前位姿（仍是 loop-1 校正后的末帧）: time=1300 x=7.91 y=2.41 heading=0.7853981634 variance=0.09
拒绝后矩形查询（结果不变）: 1 个
  id=door x=7.71 y=0.90 count=3
已有校正记录: 1 条
  记录 loop-1: 锚点=1200 受影响末帧=1300
记录中 loop-1 锚点校正前快照（历史值，不被后续操作改写）: time=1200 x=6.00 y=0.00 heading=1.5707963268 variance=0.06
```

`Ready()` 仍返回 true，表示基线包可以加载。
