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

`Ready()` 仍返回 true，表示基线包可以加载。

## 导入示例：自身坐标如何影响位姿与路标

下面的程序创建一份地图、导入一段两帧轨迹（同时包含平移、转向与路标
观测），再查询位姿与路标。位置单位是**米**，时间是**整数毫秒**，朝向是
**弧度**（本例 π/2 即 90°）。每一帧内部的次序是：

1. **平移使用运动前的朝向**：先按该帧运动开始前（上一帧结束后）的朝向，
   把自身坐标下的 `(DX, DY)` 旋转到地图坐标；
2. 再把 `DHeading` 累加到朝向上（归一到 [-π, π)），累计位置方差为初始
   方差与沿途各帧 `MoveVariance` 之和；
3. **观测使用运动完成后的位姿**：路标观测先按该帧运动后的位姿从机器人
   自身坐标转换到地图坐标，再参与合并；路标查询返回的始终是**地图坐标**，
   不直接返回输入的自身坐标。
4. 同一路标一次出现的第一条观测直接建点；此后每条新观测都与**此前全部
   观测的平均位置**比较距离，距离**恰好等于合并上限仍可接纳**，严格超过
   才拒绝，接纳后的位置是等权平均。

把顶部 `mapPath` 改成你本机上一个**尚不存在**的文件路径后运行即可。
创建失败（典型情况是该文件已存在）时程序立即停止：`Create` 不会删除或
覆盖任何已有文件，已有的地图请改用 `posemap.Open` 打开。

```go
package main

import (
	"fmt"
	"log"
	"math"

	"github.com/bengzyyys/pose-map/posemap"
)

// mapPath 是本地地图文件路径，运行前改成你自己的路径。
// 该路径已存在时 Create 会失败：示例不会删除或覆盖任何已有文件，
// 而是直接停止；已有的地图请改用 posemap.Open 打开。
const mapPath = "example-map.bin"

func main() {
	// 单位：位置为米，时间为整数毫秒，朝向为弧度。
	// 初始位姿在地图原点、朝向 0；相邻帧间隔上限 1000ms，路标合并距离 1m。
	m, err := posemap.Create(mapPath, posemap.Config{
		InitialTime:     0,
		InitialX:        0,
		InitialY:        0,
		InitialHeading:  0,
		InitialVariance: 0,
		MaxInterval:     1000,
		MergeDistance:   1.0,
	})
	if err != nil {
		// 创建失败（例如文件已存在）时停止本次示例流程。
		log.Fatalf("创建地图失败：%v", err)
	}
	defer func() {
		if err := m.Close(); err != nil {
			log.Printf("关闭地图失败：%v", err)
		}
	}()

	// 第一段：两帧轨迹，每帧都观测同一个路标 pole 一次。
	// 帧内先按“运动前朝向”平移，再更新朝向；观测按“运动完成后的位姿”
	// 从机器人自身坐标转换到地图坐标。
	seg1 := posemap.Segment{
		ID: "seg-1",
		Frames: []posemap.Frame{
			{
				// 第 0 帧：沿自身前方（朝向 0，即地图 +X）前进 1m，
				// 同时左转 90°；运动方差 0.01。
				Time:         100,
				DX:           1,
				DY:           0,
				DHeading:     math.Pi / 2,
				MoveVariance: 0.01,
				// 在自身左侧 1m 处观测 pole，运动后位姿为 (1,0,π/2)，
				// 转换到地图为 (0,0)。
				Observations: []posemap.Observation{
					{ID: "pole", X: 0, Y: 1},
				},
			},
			{
				// 第 1 帧：朝向已是 π/2（地图 +Y），继续沿自身前方
				// 前进 1m，朝向不变；运动方差 0.02。
				Time:         200,
				DX:           1,
				DY:           0,
				DHeading:     0,
				MoveVariance: 0.02,
				// 同样的自身坐标 (0,1)，运动后位姿为 (1,1,π/2)，
				// 转换到地图为 (0,1)：与上一帧的地图点不同，
				// 两点距离恰好等于合并上限 1m，仍可合并。
				Observations: []posemap.Observation{
					{ID: "pole", X: 0, Y: 1},
				},
			},
		},
	}
	res, err := m.ImportSegment(seg1)
	if err != nil {
		log.Fatalf("导入 seg-1 失败：%v", err)
	}
	fmt.Printf("段 seg-1 导入成功：末位姿 time=%d x=%.3f y=%.3f heading=%.6f variance=%.2f，路标=%v\n",
		res.EndPose.Time, res.EndPose.X, res.EndPose.Y, res.EndPose.Heading, res.EndPose.Variance, res.LandmarkIDs)

	p1, err := m.PoseAt(100)
	if err != nil {
		log.Fatalf("查询第 1 帧位姿失败：%v", err)
	}
	printPose("第 1 帧位姿 PoseAt(100)", p1)

	cur, err := m.CurrentPose()
	if err != nil {
		log.Fatalf("查询当前位姿失败：%v", err)
	}
	printPose("当前位姿 CurrentPose", cur)

	// 查询时间 150ms 落在两帧之间：返回不晚于 150ms 的最后一份位姿，
	// 即时间为 100ms 的第 1 帧（携带的是它自己的帧时间，不是 150）。
	pmid, err := m.PoseAt(150)
	if err != nil {
		log.Fatalf("历史位姿查询失败：%v", err)
	}
	printPose("历史查询 PoseAt(150)，返回不晚于它的最后一份位姿", pmid)

	// 矩形区域查询，单位米，边界包含。pole 的平均位置 (0,0.5) 恰好
	// 压在 MinX=0 边界上：仍被返回；当前有效路标只有 pole 一个。
	rect := posemap.Rect{MinX: 0, MinY: 0, MaxX: 1, MaxY: 1}
	lms, err := m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("区域查询失败：%v", err)
	}
	printLandmarks(fmt.Sprintf("区域查询矩形 [0,0]-[1,1]（含边界），共 %d 个", len(lms)), lms)

	// 第二段：两帧。第 0 帧是一帧合法输入（不动，再次观测 (0,1)，
	// 地图点 (0,1)，离当前平均位置 0.5m）；第 1 帧的观测转到地图为
	// (0,2)，离此前平均位置 1.333m，严格超过 1m 上限。
	seg2 := posemap.Segment{
		ID: "seg-2",
		Frames: []posemap.Frame{
			{
				Time:         300,
				DX:           0,
				DY:           0,
				DHeading:     0,
				MoveVariance: 0,
				Observations: []posemap.Observation{
					{ID: "pole", X: 0, Y: 1},
				},
			},
			{
				Time:         400,
				DX:           0,
				DY:           0,
				DHeading:     0,
				MoveVariance: 0,
				Observations: []posemap.Observation{
					{ID: "pole", X: 1, Y: 1},
				},
			},
		},
	}
	if _, err := m.ImportSegment(seg2); err != nil {
		if rj, ok := posemap.AsRejectError(err); ok {
			// 可区分的拒绝原因、从零开始的帧序号、相关路标标识。
			fmt.Printf("段 seg-2 被整体拒绝：原因=%s 帧序号=%d 路标=%s（%v）\n",
				rj.Kind, rj.Frame, rj.Landmark, rj)
		} else {
			log.Fatalf("导入 seg-2 返回非拒绝错误：%v", err)
		}
	} else {
		log.Fatal("seg-2 本应被拒绝")
	}

	// 拒绝是整段生效：此前成功提交的位姿与路标原样保留，
	// seg-2 的第 0 帧也不会部分生效。
	cur, err = m.CurrentPose()
	if err != nil {
		log.Fatalf("拒绝后查询当前位姿失败：%v", err)
	}
	printPose("拒绝后当前位姿（仍停在 seg-1 末帧）", cur)

	lms, err = m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("拒绝后区域查询失败：%v", err)
	}
	printLandmarks(fmt.Sprintf("拒绝后区域查询，共 %d 个（观测次数没有增加，无部分生效）", len(lms)), lms)
}

func printPose(label string, p posemap.Pose) {
	fmt.Printf("%s：time=%d x=%.3f y=%.3f heading=%.6f variance=%.2f\n",
		label, p.Time, p.X, p.Y, p.Heading, p.Variance)
}

func printLandmarks(label string, lms []posemap.Landmark) {
	fmt.Println(label)
	for _, lm := range lms {
		// 路标查询返回的始终是地图坐标，不是输入里的自身坐标。
		fmt.Printf("  %s：x=%.3f y=%.3f 观测次数=%d\n", lm.ID, lm.X, lm.Y, lm.Count)
	}
}
```

### 输入与结果逐项对应

初始位姿是 `time=0, (0,0), heading=0, variance=0`。

- **第 0 帧（time=100）**：平移按运动前朝向 0 计算，自身前方 `(1,0)`
  就是地图 `(1,0)`；之后朝向变为 `0+π/2=π/2`，累计方差 `0+0.01=0.01`。
  观测按运动后的位姿 `(1,0,π/2)` 转换：自身坐标 `(0,1)` 旋到地图为
  `(1+cos(π/2)·0−sin(π/2)·1, 0+sin(π/2)·0+cos(π/2)·1)=(0,0)`。
  这是 `pole` 的第一条观测，直接建点，观测次数为 1。
- **第 1 帧（time=200）**：平移按运动前朝向 π/2 计算，自身前方 `(1,0)`
  此时指向地图 +Y，新位置为 `(1,1)`；朝向保持 π/2，累计方差
  `0.01+0.02=0.03`。观测按位姿 `(1,1,π/2)` 转换，同样的自身坐标 `(0,1)`
  这次落到地图 `(0,1)`——同一自身坐标在不同位姿下给出不同地图点。它与
  此前平均位置 `(0,0)` 的距离恰好为 1m，等于合并上限，仍被接纳；接纳后
  平均位置为 `(0,0.5)`，观测次数为 2。
- **历史查询 `PoseAt(150)`**：150ms 落在两帧之间，返回的是不晚于 150ms
  的最后一份位姿，即 time=100 的第 0 帧，结果携带它自己的帧时间 100。
- **区域查询 `LandmarksInRect({0,0},{1,1})`**：矩形边界包含，
  `(0,0.5)` 恰好压在 MinX=0 边界上仍被返回；区域查询只返回当前有效
  路标，这里只有 `pole` 一个，位置是地图坐标 `(0,0.5)`、次数 2。
- **第二段被整体拒绝**：seg-2 第 0 帧（time=300）本身合法——不动，
  观测 `(0,1)` 转到地图仍是 `(0,1)`，离当前平均位置 0.5m，暂存均值会
  变成 `(0,2/3)`、次数 3；但第 1 帧（time=400，从零开始的帧序号 1）的
  自身观测 `(1,1)` 按位姿 `(1,1,π/2)` 转到地图为 `(0,2)`，与**此前平均
  位置** `(0,2/3)` 相距 4/3 ≈ 1.333m，严格超过 1m 上限。整段因此被拒绝，
  `AsRejectError` 可取到原因 `landmark_conflict`、帧序号 1 与路标标识
  `pole`。
- **拒绝后**：第 0 帧的暂存结果不会落盘，没有任何部分生效。当前位姿
  仍是 time=200 的 `(1,1), π/2, 方差 0.03`；`pole` 仍在 `(0,0.5)`，
  观测次数仍是 2。

### 预期输出

```text
段 seg-1 导入成功：末位姿 time=200 x=1.000 y=1.000 heading=1.570796 variance=0.03，路标=[pole]
第 1 帧位姿 PoseAt(100)：time=100 x=1.000 y=0.000 heading=1.570796 variance=0.01
当前位姿 CurrentPose：time=200 x=1.000 y=1.000 heading=1.570796 variance=0.03
历史查询 PoseAt(150)，返回不晚于它的最后一份位姿：time=100 x=1.000 y=0.000 heading=1.570796 variance=0.01
区域查询矩形 [0,0]-[1,1]（含边界），共 1 个
  pole：x=0.000 y=0.500 观测次数=2
段 seg-2 被整体拒绝：原因=landmark_conflict 帧序号=1 路标=pole（posemap: frame 1 observation of landmark pole is beyond merge distance）
拒绝后当前位姿（仍停在 seg-1 末帧）：time=200 x=1.000 y=1.000 heading=1.570796 variance=0.03
拒绝后区域查询，共 1 个（观测次数没有增加，无部分生效）
  pole：x=0.000 y=0.500 观测次数=2
```
