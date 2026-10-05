package posemap

import (
	"math"
	"testing"
)

// 本文件为“成功导入的内容不受调用方后续改写影响”这一已有行为提供回归
// 保障：调用方把一段轨迹交给 ImportSegment 并收到成功结果后，可能复用
// 这批帧与观测数据准备下一段轨迹（就地改写观测标识/坐标、调整运动方差、
// 替换观测列表）。已接受的位姿、路标与逐帧依据必须从此保持稳定：改写
// 不得暗中改变历史位姿与路标，也不得影响以后以这些帧为依据进行的回环
// 校正。只使用现有公开入口（ImportSegment/CurrentPose/PoseAt/
// LandmarksInRect/Correct/Corrections），不改变任何公开使用方式。

// 三帧轨迹（含初始位姿），含非零朝向、运动后观测、跨帧同一路标与同帧
// 有先后次序的重复观测：
//
//	p0 (t=0)  (1,2)   h=0     v=0.5
//	p1 (t=100)(3,2)   h=π/2   v=0.7   观测 L0 本地(1,0) → 地图(3,3)
//	p2 (t=200)(3,3)   h=π/2   v=1.0   观测 L1 本地(0,0) → (3,3)，再 L1 本地(0,1) → (2,3)
//	p3 (t=300)(3,4)   h=0     v=1.4   观测 L1 本地(0,-0.5) → (3,3.5)；L2 本地(2,1) → (5,5)
//
// p2 同帧第二条 L1 与当时的均值 (3,3) 距离恰为合并距离 1，依赖前一条接纳
// 后的结果才能被接纳。L1 三次观测均值 (8/3, 19/6)，计数 3；L0、L2 计数 1。
func mutableSegment() Segment {
	return Segment{
		ID: "s1",
		Frames: []Frame{
			{
				Time: 100, DX: 2, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.2,
				Observations: []Observation{{ID: "L0", X: 1, Y: 0}},
			},
			{
				Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.3,
				Observations: []Observation{{ID: "L1", X: 0, Y: 0}, {ID: "L1", X: 0, Y: 1}},
			},
			{
				Time: 300, DX: 1, DY: 0, DHeading: -math.Pi / 2, MoveVariance: 0.4,
				Observations: []Observation{{ID: "L1", X: 0, Y: -0.5}, {ID: "L2", X: 2, Y: 1}},
			},
		},
	}
}

// tamperImportedSegment 模拟调用方在导入成功后复用原数据：就地改写原观测
// 的标识（含空标识）与自身坐标（含远超合并距离的取值）、调整各帧运动方差，
// 并整列替换最后一帧的观测列表（引入地图从未见过的陌生路标）。
func tamperImportedSegment(seg *Segment) {
	seg.Frames[0].Observations[0].ID = "" // 空路标标识：若被重新读取必然拒绝
	seg.Frames[1].Observations[0].X = 1e9 // 远超合并距离：若被重新读取必然冲突
	seg.Frames[1].Observations[1].ID = "ghost"
	seg.Frames[0].MoveVariance = 50
	seg.Frames[1].MoveVariance = 60
	seg.Frames[2].MoveVariance = 7.5
	seg.Frames[2].Observations = []Observation{{ID: "ghost2", X: 1e9, Y: 1e9}}
}

// 导入成功后调用方改写原数据：当前位姿、相应时间的历史位姿、路标位置与
// 观测次数仍是已接受的结果；随后对已导入帧提交一次同时改变位置、朝向与
// 方差的合法校正，校正按成功导入时的观测与运动方差进行，不受改写影响。
func TestImportIsolationFromCallerMutation(t *testing.T) {
	m := newMap(t, baseConfig())
	seg := mutableSegment()
	res, err := m.ImportSegment(seg)
	if err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	if res.EndPose.Time != 300 || res.EndPose.X != 3 || res.EndPose.Y != 4 || res.EndPose.Variance != 1.4 {
		t.Fatalf("import end pose = %+v, want (3,4) v=1.4", res.EndPose)
	}
	if len(res.LandmarkIDs) != 3 || res.LandmarkIDs[0] != "L0" || res.LandmarkIDs[1] != "L1" || res.LandmarkIDs[2] != "L2" {
		t.Fatalf("import landmarks = %v, want [L0 L1 L2]", res.LandmarkIDs)
	}

	// 调用方复用原数据：就地改写观测与方差、替换观测列表。
	tamperImportedSegment(&seg)

	// 当前位姿与历史位姿保持已接受的结果。
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	if cur.Time != 300 || cur.X != 3 || cur.Y != 4 || cur.Heading != 0 || cur.Variance != 1.4 {
		t.Fatalf("current = %+v, want accepted (3,4) h=0 v=1.4", cur)
	}
	p, err := m.PoseAt(100)
	if err != nil {
		t.Fatal(err)
	}
	if p.X != 3 || p.Y != 2 || p.Heading != math.Pi/2 || p.Variance != 0.7 {
		t.Fatalf("pose at 100 = %+v, want accepted (3,2) h=π/2 v=0.7", p)
	}
	p, err = m.PoseAt(250)
	if err != nil {
		t.Fatal(err)
	}
	if p.Time != 200 || p.X != 3 || p.Y != 3 || p.Variance != 1.0 {
		t.Fatalf("pose at 250 = %+v, want accepted frame at t=200", p)
	}

	// 路标位置与观测次数保持已接受的结果，陌生路标不出现。
	lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 3 {
		t.Fatalf("landmarks = %v, want L0/L1/L2 only", lms)
	}
	l0, _ := findLandmark(lms, "L0")
	if l0.X != 3 || l0.Y != 3 || l0.Count != 1 {
		t.Fatalf("L0 = %+v, want (3,3) count 1", l0)
	}
	l1, _ := findLandmark(lms, "L1")
	approxEq(t, "L1 x", l1.X, 8.0/3.0)
	approxEq(t, "L1 y", l1.Y, 9.5/3.0)
	if l1.Count != 3 {
		t.Fatalf("L1 count = %d, want 3", l1.Count)
	}
	l2, _ := findLandmark(lms, "L2")
	if l2.X != 5 || l2.Y != 5 || l2.Count != 1 {
		t.Fatalf("L2 = %+v, want (5,5) count 1", l2)
	}

	// 对已导入帧提交合法校正：锚点 t=200，目标同时改变位置、朝向与方差。
	// 原数据已被改成空标识、超距坐标与陌生路标，也不能让本次校正被误拒绝。
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 200, Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 2}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if rec.Anchor != 200 || rec.EndTime != 300 || rec.Target.X != 10 || rec.Target.Y != 20 || rec.Target.Variance != 2 {
		t.Fatalf("record header = %+v", rec)
	}

	// 位姿记录：δ = 0 - π/2 = -π/2。
	// p2 → (10,20) h=0 v=2；p3 相对锚点 (0,1) 旋转 -π/2 → (1,0)，
	// 即 (11,20) h=-π/2，方差按成功导入时的运动方差累加 2+0.4=2.4，
	// 不能采用调用方后来填写的 7.5。
	if len(rec.Poses) != 2 {
		t.Fatalf("record poses = %+v, want 2 changes", rec.Poses)
	}
	if rec.Poses[0].Before.X != 3 || rec.Poses[0].Before.Y != 3 || rec.Poses[0].Before.Variance != 1.0 {
		t.Fatalf("anchor before = %+v, want accepted (3,3) v=1.0", rec.Poses[0].Before)
	}
	approxEq(t, "anchor after x", rec.Poses[0].After.X, 10)
	approxEq(t, "anchor after y", rec.Poses[0].After.Y, 20)
	approxEq(t, "anchor after heading", rec.Poses[0].After.Heading, 0)
	approxEq(t, "anchor after variance", rec.Poses[0].After.Variance, 2)
	if rec.Poses[1].Before.X != 3 || rec.Poses[1].Before.Y != 4 {
		t.Fatalf("p3 before = %+v, want accepted (3,4)", rec.Poses[1].Before)
	}
	approxEq(t, "p3 after x", rec.Poses[1].After.X, 11)
	approxEq(t, "p3 after y", rec.Poses[1].After.Y, 20)
	approxEq(t, "p3 after heading", rec.Poses[1].After.Heading, -math.Pi/2)
	approxEq(t, "p3 after variance", rec.Poses[1].After.Variance, 2.4)

	// 路标记录：只含受影响的 L1、L2（按标识排序），不含调用方改出的
	// 陌生路标；观测次数不变。L1 按原观测与原同帧次序重放：
	// (10,20)、(10,21)（距离恰为 1 仍接纳）、(10.5,20)，均值
	// (30.5/3, 61/3)；L2 本地(2,1) 随 p3 校正后位姿 → (12,18)。
	if len(rec.Landmarks) != 2 || rec.Landmarks[0].ID != "L1" || rec.Landmarks[1].ID != "L2" {
		t.Fatalf("record landmarks = %+v, want L1 and L2 only", rec.Landmarks)
	}
	lc := rec.Landmarks[0]
	if lc.Occurrence != 1 {
		t.Fatalf("L1 occurrence = %d, want 1", lc.Occurrence)
	}
	approxEq(t, "L1 before x", lc.Before.X, 8.0/3.0)
	approxEq(t, "L1 before y", lc.Before.Y, 9.5/3.0)
	approxEq(t, "L1 after x", lc.After.X, 30.5/3.0)
	approxEq(t, "L1 after y", lc.After.Y, 61.0/3.0)
	if lc.Before.Count != 3 || lc.After.Count != 3 {
		t.Fatalf("L1 count changed: %d -> %d", lc.Before.Count, lc.After.Count)
	}
	lc = rec.Landmarks[1]
	approxEq(t, "L2 before x", lc.Before.X, 5)
	approxEq(t, "L2 before y", lc.Before.Y, 5)
	approxEq(t, "L2 after x", lc.After.X, 12)
	approxEq(t, "L2 after y", lc.After.Y, 18)
	if lc.Before.Count != 1 || lc.After.Count != 1 {
		t.Fatalf("L2 count changed: %d -> %d", lc.Before.Count, lc.After.Count)
	}

	// 返回的记录与查询结果相互一致。
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 || !recordsEqual(recs[0], rec) {
		t.Fatalf("corrections = %+v, want the returned record", recs)
	}
	cur, _ = m.CurrentPose()
	approxEq(t, "current x", cur.X, rec.Poses[1].After.X)
	approxEq(t, "current y", cur.Y, rec.Poses[1].After.Y)
	approxEq(t, "current heading", cur.Heading, rec.Poses[1].After.Heading)
	approxEq(t, "current variance", cur.Variance, rec.Poses[1].After.Variance)
	p, _ = m.PoseAt(200)
	approxEq(t, "pose at 200 x", p.X, rec.Poses[0].After.X)
	approxEq(t, "pose at 200 variance", p.Variance, rec.Poses[0].After.Variance)

	// 更早的帧保持不变：初始位姿与 t=100 的位姿、只在 t=100 被观测的
	// L0 都不受校正影响，也不出现在校正记录中。
	p0, err := m.PoseAt(0)
	if err != nil || p0.X != 1 || p0.Y != 2 || p0.Variance != 0.5 {
		t.Fatalf("initial pose changed: %+v %v", p0, err)
	}
	p, _ = m.PoseAt(100)
	if p.X != 3 || p.Y != 2 || p.Heading != math.Pi/2 || p.Variance != 0.7 {
		t.Fatalf("pose at 100 changed by correction: %+v", p)
	}
	lms, _ = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 3 {
		t.Fatalf("landmarks after correction = %v, want L0/L1/L2 only", lms)
	}
	l0, _ = findLandmark(lms, "L0")
	if l0.X != 3 || l0.Y != 3 || l0.Count != 1 {
		t.Fatalf("L0 changed by correction: %+v", l0)
	}
	l1, _ = findLandmark(lms, "L1")
	approxEq(t, "queried L1 x", l1.X, 30.5/3.0)
	approxEq(t, "queried L1 y", l1.Y, 61.0/3.0)
	if l1.Count != 3 {
		t.Fatalf("queried L1 count = %d, want 3", l1.Count)
	}
	l2, _ = findLandmark(lms, "L2")
	approxEq(t, "queried L2 x", l2.X, 12)
	approxEq(t, "queried L2 y", l2.Y, 18)
	if l2.Count != 1 {
		t.Fatalf("queried L2 count = %d, want 1", l2.Count)
	}
}

// 原观测会引起校正冲突时，即使调用方事后把原数据改成看似可以合并的
// 位置，校正仍按已接受的观测拒绝：错误给出原路标、出现编号与冲突帧
// 时间，已有位姿、路标与校正记录均保持不变。
func TestCorrectionConflictUsesAcceptedObservations(t *testing.T) {
	m := newMap(t, baseConfig())
	// 两帧轨迹：p1 (t=100) (3,2) h=π/2 观测 L1 → (3,3)；
	// p2 (t=200) (3,3) h=π/2 观测 L1 → (3,3)。L1 均值 (3,3)，计数 2。
	seg := Segment{
		ID: "s1",
		Frames: []Frame{
			{
				Time: 100, DX: 2, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.2,
				Observations: []Observation{{ID: "L1", X: 1, Y: 0}},
			},
			{
				Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.3,
				Observations: []Observation{{ID: "L1", X: 0, Y: 0}},
			},
		},
	}
	if _, err := m.ImportSegment(seg); err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}

	// 调用方事后把原观测改成看似可以合并的位置：若校正采用这份改写，
	// p2 校正到 (10,20) h=0 后该观测会映射到 (3,3)，与既有均值距离为 0。
	seg.Frames[1].Observations[0].X = -7
	seg.Frames[1].Observations[0].Y = -17

	// 校正仍按已接受的观测（本地 (0,0) → 地图 (10,20)，与 (3,3) 距离
	// 远超合并距离）拒绝，并指出原路标、出现编号与冲突帧时间。
	_, err := m.Correct(Correction{ID: "c1", Anchor: 200, Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 2}})
	rej, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("Correct err = %v, want *RejectError", err)
	}
	if rej.Kind != RejectLandmarkConflict {
		t.Fatalf("reject kind = %q, want %q", rej.Kind, RejectLandmarkConflict)
	}
	if !rej.HasLandmark || rej.Landmark != "L1" {
		t.Fatalf("reject landmark = %q (has=%v), want L1", rej.Landmark, rej.HasLandmark)
	}
	if !rej.HasOccurrence || rej.Occurrence != 1 {
		t.Fatalf("reject occurrence = %d (has=%v), want 1", rej.Occurrence, rej.HasOccurrence)
	}
	if !rej.HasTime || rej.Time != 200 {
		t.Fatalf("reject time = %d (has=%v), want conflict frame time 200", rej.Time, rej.HasTime)
	}

	// 已有位姿、路标与校正记录均保持不变。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 3 || cur.Y != 3 || cur.Heading != math.Pi/2 || cur.Variance != 1.0 {
		t.Fatalf("current = %+v, want unchanged (3,3) h=π/2 v=1.0", cur)
	}
	p, _ := m.PoseAt(100)
	if p.X != 3 || p.Y != 2 || p.Variance != 0.7 {
		t.Fatalf("pose at 100 = %+v, want unchanged (3,2) v=0.7", p)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "L1" || lms[0].X != 3 || lms[0].Y != 3 || lms[0].Count != 2 {
		t.Fatalf("landmarks = %v, want unchanged L1 (3,3) count 2", lms)
	}
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 0 {
		t.Fatalf("corrections = %v, want none after rejection", recs)
	}
}
