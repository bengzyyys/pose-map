package posemap

import (
	"errors"
	"math"
	"testing"
)

// 本文件为“成功导入的内容不受调用方后续改写影响”这一已有行为提供回归
// 保障。调用方把一段带运动、方差和路标观测的轨迹交给地图并收到成功结果
// 后，常会复用同一批帧与观测切片去准备下一段轨迹；地图中已接受的历史
// 位姿、路标位置与观测次数必须从此独立于调用方内存，也不能影响以后以
// 这些帧为依据进行的回环校正（校正重放只使用导入时保存下来的逐帧观测
// 与原运动方差）。只使用现有公开入口（Create/ImportSegment/CurrentPose/
// PoseAt/LandmarksInRect/LandmarkAppearances/Correct/Corrections），
// 不改变任何公开使用方式与返回结果。
//
// 统一几何（初始位姿 (0,0)、朝向 0、初始方差 1、合并上限 5）：
//
//	p0   t=0    (0,0)          朝向 0
//	f50  t=50   DX=1（无观测） → (1,0)，朝向 0，方差 1.1
//	f100 t=100  DX=3、转 90°   → (4,0)，朝向 π/2，方差 1.3
//	      运动后观测：L 本地 (0,-1) → 世界 (5,0)；M 本地 (0,1) → 世界 (3,0)
//	f200 t=200  沿自身 X 走 4  → (4,4)，朝向 π/2，方差 1.7
//	      同帧按序两条 L：本地 (0,0) → (4,4)、(1,0) → (4,5)
//	f300 t=300  原地不动、无观测 → (4,4)，朝向 π/2，方差 2.2
//
// 导入时 L 三次观测 (5,0)/(4,4)/(4,5) 逐次相距 4.123…、3.041… 均不超过
// 合并上限 5，均值 (13/3, 3)，计数 3；M 一次观测 (3,0)。
//
// 回环校正以 t=100 为锚点，目标 (10,20)、朝向 0、方差 0.3，同时改变
// 位置、朝向与方差。锚点之后整体刚体变换（-90° 旋转加平移）：
//
//	t=100 → (10,20)，朝向 0，方差 0.3
//	t=200 → (14,20)，朝向 0，方差 0.3+0.4 = 0.7
//	t=300 → (14,20)，朝向 0，方差 0.3+0.4+0.5 = 1.2
//
// 重放后的观测（同帧次序不变）：
//
//	t=100：L (0,-1) → (10,19)；M (0,1) → (10,21)
//	t=200：L (0,0) → (14,20)，L (1,0) → (15,20)
//	L 均值 = (13, 59/3)，计数仍为 3；M = (10,21)，计数 1。
//
// 刚体变换保持观测间距离，故各步距离（4.123…、3.041…）与导入时相同，
// 校正合法；t=50 及初始位姿在锚点之前，保持不变。

// mutationIsolationConfig 是这组场景的地图配置：原点初始位姿、非零
// 初始方差、合并上限 5。
func mutationIsolationConfig() Config {
	return Config{InitialTime: 0, InitialVariance: 1.0, MaxInterval: 1000, MergeDistance: 5.0}
}

// mutationIsolationSegment 构造上述带非零朝向、运动后观测、跨帧同一
// 路标与同帧有序重复观测的合法轨迹，并返回各帧指针供调用方事后改写。
func mutationIsolationSegment() (Segment, *Frame, *Frame, *Frame, *Frame) {
	seg := Segment{
		ID: "seg",
		Frames: []Frame{
			{Time: 50, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
			{
				Time: 100, DX: 3, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.2,
				Observations: []Observation{
					{ID: "L", X: 0, Y: -1},
					{ID: "M", X: 0, Y: 1},
				},
			},
			{
				Time: 200, DX: 4, DY: 0, DHeading: 0, MoveVariance: 0.4,
				Observations: []Observation{
					{ID: "L", X: 0, Y: 0},
					{ID: "L", X: 1, Y: 0},
				},
			},
			{Time: 300, DX: 0, DY: 0, DHeading: 0, MoveVariance: 0.5},
		},
	}
	return seg, &seg.Frames[0], &seg.Frames[1], &seg.Frames[2], &seg.Frames[3]
}

// expectPose 断言查询到的位姿在时间与各数值字段上都接近期望。
func expectPose(t *testing.T, what string, got, want Pose) {
	t.Helper()
	if got.Time != want.Time {
		t.Fatalf("%s time = %d, want %d (pose %+v)", what, got.Time, want.Time, got)
	}
	approxEq(t, what+" x", got.X, want.X)
	approxEq(t, what+" y", got.Y, want.Y)
	approxEq(t, what+" heading", got.Heading, want.Heading)
	approxEq(t, what+" variance", got.Variance, want.Variance)
}

// expectPoseChange 断言校正记录中一帧的校正前后值与期望一致。
func expectPoseChange(t *testing.T, i int, got PoseChange, before, after Pose) {
	t.Helper()
	if got.Before.Time != before.Time || got.After.Time != after.Time {
		t.Fatalf("pose change %d times = %d -> %d, want %d -> %d",
			i, got.Before.Time, got.After.Time, before.Time, after.Time)
	}
	approxEq(t, "before x", got.Before.X, before.X)
	approxEq(t, "before y", got.Before.Y, before.Y)
	approxEq(t, "before heading", got.Before.Heading, before.Heading)
	approxEq(t, "before variance", got.Before.Variance, before.Variance)
	approxEq(t, "after x", got.After.X, after.X)
	approxEq(t, "after y", got.After.Y, after.Y)
	approxEq(t, "after heading", got.After.Heading, after.Heading)
	approxEq(t, "after variance", got.After.Variance, after.Variance)
}

// 调用方成功导入后就地改写原观测标识与自身坐标、调整原帧运动方差并
// 替换原观测列表：地图的当前位姿、各时间历史位姿、路标位置与观测次数
// 仍全部是导入时已接受的结果，改出的空标识/陌生路标与巨大坐标不会出现。
func TestImportedTrajectoryIsolatedFromCallerMutation(t *testing.T) {
	m := newMap(t, mutationIsolationConfig())
	seg, f50, f100, f200, f300 := mutationIsolationSegment()

	res, err := m.ImportSegment(seg)
	if err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	// 导入结果即已接受的末位姿与路标集合。
	expectPose(t, "import end pose", res.EndPose, Pose{Time: 300, X: 4, Y: 4, Heading: math.Pi / 2, Variance: 2.2})
	if len(res.LandmarkIDs) != 2 || res.LandmarkIDs[0] != "L" || res.LandmarkIDs[1] != "M" {
		t.Fatalf("import landmark ids = %v, want [L M]", res.LandmarkIDs)
	}

	// 记录导入时已接受的位姿快照，供改写后及校正记录比对。
	wantP0 := Pose{Time: 0, X: 0, Y: 0, Heading: 0, Variance: 1.0}
	wantP50 := Pose{Time: 50, X: 1, Y: 0, Heading: 0, Variance: 1.1}
	wantP100 := Pose{Time: 100, X: 4, Y: 0, Heading: math.Pi / 2, Variance: 1.3}
	wantP200 := Pose{Time: 200, X: 4, Y: 4, Heading: math.Pi / 2, Variance: 1.7}
	wantP300 := Pose{Time: 300, X: 4, Y: 4, Heading: math.Pi / 2, Variance: 2.2}

	// ---- 调用方复用这批输入准备下一段：就地改写一切可写字段 ----
	// 原观测标识改成空标识、自身坐标改成远超合并距离的巨大值。
	f100.Observations[0] = Observation{ID: "", X: 1e6, Y: 1e6}
	f100.Observations[1] = Observation{ID: "", X: -1e6, Y: 1e6}
	f200.Observations[0] = Observation{ID: "", X: 1e6, Y: 1e6}
	f200.Observations[1] = Observation{ID: "", X: 1e6, Y: -1e6}
	// 原帧运动方差改成准备下一段时填写的新值。
	f50.MoveVariance, f100.MoveVariance, f200.MoveVariance, f300.MoveVariance = 500, 500, 500, 500
	// 整表替换原观测列表：陌生路标标识与空标识、远离一切的坐标。
	f100.Observations = []Observation{{ID: "ZZ", X: 1e9, Y: 1e9}}
	f200.Observations = []Observation{{ID: "", X: 1e9, Y: 1e9}}

	// 当前位姿与相应时间的历史位姿仍是已接受结果。
	cur, _ := m.CurrentPose()
	expectPose(t, "current after mutation", cur, wantP300)
	for _, want := range []Pose{wantP0, wantP50, wantP100, wantP200} {
		p, err := m.PoseAt(want.Time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", want.Time, err)
		}
		expectPose(t, "history after mutation", p, want)
	}

	// 路标位置与观测次数仍是已接受结果；陌生标识 ZZ 不在地图中。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 2 {
		t.Fatalf("landmarks after mutation = %v, want exactly L and M", lms)
	}
	lBefore, ok := findLandmark(lms, "L")
	if !ok {
		t.Fatalf("L missing after mutation: %v", lms)
	}
	approxEq(t, "L x after mutation", lBefore.X, 13.0/3.0)
	approxEq(t, "L y after mutation", lBefore.Y, 3)
	if lBefore.Count != 3 {
		t.Fatalf("L count after mutation = %d, want 3", lBefore.Count)
	}
	mBefore, ok := findLandmark(lms, "M")
	if !ok {
		t.Fatalf("M missing after mutation: %v", lms)
	}
	approxEq(t, "M x after mutation", mBefore.X, 3)
	approxEq(t, "M y after mutation", mBefore.Y, 0)
	if mBefore.Count != 1 {
		t.Fatalf("M count after mutation = %d, want 1", mBefore.Count)
	}
	if _, err := m.LandmarkAppearances("ZZ"); !errors.Is(err, ErrLandmarkNotFound) {
		t.Fatalf("stranger landmark ZZ appeared after mutation: %v", err)
	}
	hL, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(hL.Appearances) != 1 || !hL.Appearances[0].Active ||
		!hL.Appearances[0].HasFirstSeen || hL.Appearances[0].FirstSeenTime != 100 ||
		hL.Appearances[0].Landmark.Count != 3 {
		t.Fatalf("L appearance after mutation = %+v", hL.Appearances)
	}

	// ---- 对已导入帧提交一次原本合法的回环校正 ----
	// 空标识与 1e9 坐标若被误当作观测来源，会以 empty_id/non_finite/
	// landmark_conflict 等原因拒绝；使用内部已接受副本时校正必须成功。
	req := Correction{
		ID:     "c-loop",
		Anchor: 100,
		Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 0.3},
	}
	rec, err := m.Correct(req)
	if err != nil {
		t.Fatalf("Correct after caller mutated input: %v", err)
	}
	if rec.ID != "c-loop" || rec.Anchor != 100 || rec.EndTime != 300 || rec.Target != req.Target {
		t.Fatalf("record header = %+v", rec)
	}

	// 更早帧不在记录中；记录覆盖锚点及之后三帧，前后值与已接受轨迹/
	// 校正目标一致。
	if len(rec.Poses) != 3 {
		t.Fatalf("pose changes = %+v, want 3 (t=100,200,300)", rec.Poses)
	}
	wantP100After := Pose{Time: 100, X: 10, Y: 20, Heading: 0, Variance: 0.3}
	wantP200After := Pose{Time: 200, X: 14, Y: 20, Heading: 0, Variance: 0.7}
	wantP300After := Pose{Time: 300, X: 14, Y: 20, Heading: 0, Variance: 1.2}
	expectPoseChange(t, 0, rec.Poses[0], wantP100, wantP100After)
	expectPoseChange(t, 1, rec.Poses[1], wantP200, wantP200After)
	expectPoseChange(t, 2, rec.Poses[2], wantP300, wantP300After)

	// 受影响路标按原观测及原同帧次序重放：L 三次观测 (10,19)/(14,20)/
	// (15,20) 等权平均 (13, 59/3)，计数不变；M 一次观测 (10,21)。
	if len(rec.Landmarks) != 2 {
		t.Fatalf("landmark changes = %+v, want L and M", rec.Landmarks)
	}
	lc, mc := rec.Landmarks[0], rec.Landmarks[1]
	if lc.ID != "L" || lc.Occurrence != 1 || mc.ID != "M" || mc.Occurrence != 1 {
		t.Fatalf("landmark change order = %+v %+v", lc, mc)
	}
	if lc.Before.ID != "L" || lc.Before.Count != 3 || lc.After.Count != 3 {
		t.Fatalf("L change counts = %+v, want 3 -> 3", lc)
	}
	approxEq(t, "L before x in record", lc.Before.X, 13.0/3.0)
	approxEq(t, "L before y in record", lc.Before.Y, 3)
	approxEq(t, "L after x in record", lc.After.X, 13)
	approxEq(t, "L after y in record", lc.After.Y, 59.0/3.0)
	if mc.Before.Count != 1 || mc.After.Count != 1 {
		t.Fatalf("M change counts = %+v, want 1 -> 1", mc)
	}
	approxEq(t, "M before x in record", mc.Before.X, 3)
	approxEq(t, "M before y in record", mc.Before.Y, 0)
	approxEq(t, "M after x in record", mc.After.X, 10)
	approxEq(t, "M after y in record", mc.After.Y, 21)

	// 查询结果与校正记录的校正后值相互一致。
	cur, _ = m.CurrentPose()
	expectPose(t, "current after correction", cur, wantP300After)
	for _, want := range []Pose{wantP100After, wantP200After, wantP300After} {
		p, _ := m.PoseAt(want.Time)
		expectPose(t, "history after correction", p, want)
	}
	for _, pc := range rec.Poses {
		p, _ := m.PoseAt(pc.After.Time)
		expectPose(t, "pose query vs record", p, pc.After)
	}
	// 锚点之前的帧保持导入时结果，不采用调用方后来填写的运动方差。
	p0, _ := m.PoseAt(0)
	expectPose(t, "initial pose after correction", p0, wantP0)
	p50, _ := m.PoseAt(50)
	expectPose(t, "earlier frame after correction", p50, wantP50)

	// 区域查询只含 L、M 的校正后结果；陌生路标 ZZ 不出现。
	lms, _ = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 2 {
		t.Fatalf("landmarks after correction = %v, want exactly L and M", lms)
	}
	lAfter, ok := findLandmark(lms, "L")
	if !ok {
		t.Fatalf("L missing after correction: %v", lms)
	}
	approxEq(t, "queried L x", lAfter.X, lc.After.X)
	approxEq(t, "queried L y", lAfter.Y, lc.After.Y)
	if lAfter.Count != 3 {
		t.Fatalf("queried L count = %d, want 3", lAfter.Count)
	}
	mAfter, ok := findLandmark(lms, "M")
	if !ok {
		t.Fatalf("M missing after correction: %v", lms)
	}
	approxEq(t, "queried M x", mAfter.X, mc.After.X)
	approxEq(t, "queried M y", mAfter.Y, mc.After.Y)
	if mAfter.Count != 1 {
		t.Fatalf("queried M count = %d, want 1", mAfter.Count)
	}
	if _, err := m.LandmarkAppearances("ZZ"); !errors.Is(err, ErrLandmarkNotFound) {
		t.Fatalf("stranger landmark ZZ appeared in map/records: %v", err)
	}
	hL, _ = m.LandmarkAppearances("L")
	if len(hL.Appearances) != 1 || !hL.Appearances[0].Active ||
		hL.Appearances[0].FirstSeenTime != 100 || hL.Appearances[0].Landmark.Count != 3 {
		t.Fatalf("L appearance after correction = %+v", hL.Appearances)
	}
	approxEq(t, "appearance L x", hL.Appearances[0].Landmark.X, 13)
	approxEq(t, "appearance L y", hL.Appearances[0].Landmark.Y, 59.0/3.0)

	// 保存的校正记录与返回记录一致，且只有这一条；调用方对原输入的
	// 改写没有向记录中引入任何陌生路标或空标识。
	recs, _ := m.Corrections()
	if len(recs) != 1 || !recordsEqual(recs[0], rec) {
		t.Fatalf("stored records = %+v, want single record equal to returned %+v", recs, rec)
	}
}

// 原观测本应在重放时引起校正冲突时，调用方事后把原数据改成看似可以
// 合并的位置（乃至整表替换成陌生路标）都不能改变结论：校正仍按已接受
// 的观测拒绝，错误给出原路标、出现编号与冲突帧时间；已有位姿、路标与
// 校正记录均保持不变，冲突的提交不消耗校正标识。
//
// 几何（初始位姿 (0,0)、朝向 0、合并上限 2）：
//
//	t=100：L 本地 (0,0) → 世界 (0,0)（首次建点）
//	t=200：L 本地 (1,0) → 世界 (1,0)（距 (0,0) 为 1，合法导入；均值 (0.5,0)）
//
// 以 t=200 为锚点平移到 (3,0)：t=100 在锚点之前，观测固定为 (0,0)；
// t=200 的已接受观测 (1,0) 随新位姿重放为 (4,0)，与 (0,0) 相距 4 > 2，
// 必须冲突。调用方若把本地坐标改成 (-2,0)，重放只会得到 (1,0)、看似
// 可以合并；地图必须无视这次改写。
func TestCorrectionConflictUsesAcceptedObservationsNotCallerMutation(t *testing.T) {
	cfg := Config{InitialTime: 0, InitialVariance: 1.0, MaxInterval: 1000, MergeDistance: 2.0}
	m := newMap(t, cfg)
	seg := Segment{
		ID: "seg",
		Frames: []Frame{
			{Time: 100, DX: 0, DY: 0, DHeading: 0, MoveVariance: 0,
				Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
			{Time: 200, DX: 0, DY: 0, DHeading: 0, MoveVariance: 0,
				Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
		},
	}
	if _, err := m.ImportSegment(seg); err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	f100, f200 := &seg.Frames[0], &seg.Frames[1]

	// 导入后：L 均值 (0.5,0)、计数 2；位姿停在 (0,0)。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("landmarks before correction = %v", lms)
	}
	approxEq(t, "L x before", lms[0].X, 0.5)
	approxEq(t, "L y before", lms[0].Y, 0)
	if lms[0].Count != 2 {
		t.Fatalf("L count before = %d, want 2", lms[0].Count)
	}

	req := Correction{
		ID:     "c-loop",
		Anchor: 200,
		Target: CorrectionTarget{X: 3, Y: 0, Heading: 0, Variance: 0},
	}
	expectConflict := func(stage string) {
		t.Helper()
		_, err := m.Correct(req)
		r, ok := AsRejectError(err)
		if !ok {
			t.Fatalf("%s: correction err = %v, want *RejectError", stage, err)
		}
		if r.Kind != RejectLandmarkConflict || !r.HasTime || r.Time != 200 ||
			!r.HasLandmark || r.Landmark != "L" || !r.HasOccurrence || r.Occurrence != 1 {
			t.Fatalf("%s: reject = %+v, want landmark_conflict at time 200 for L occurrence 1", stage, r)
		}
	}

	// 原数据未被改写时，按已接受观测 (1,0) 重放得到 (4,0)，冲突。
	expectConflict("before caller mutation")

	// 调用方把原观测改成本地 (-2,0)：重放只会得到 (1,0)，与固定观测
	// (0,0) 相距 1，看似可以合并；同时篡改运动方差。
	f200.Observations[0] = Observation{ID: "L", X: -2, Y: 0}
	f100.MoveVariance, f200.MoveVariance = 90, 90
	expectConflict("after caller mutated accepted observation")

	// 进一步把整份原观测列表替换为陌生路标 ZZ：拒绝信息仍指向原路标 L
	// 与原冲突帧 t=200、出现 1，而不是接受校正或改报 ZZ。
	f100.Observations = []Observation{{ID: "ZZ", X: 1e9, Y: 1e9}}
	f200.Observations = []Observation{{ID: "ZZ", X: 0, Y: 0}}
	expectConflict("after caller replaced observation lists")

	// 已有位姿、路标与校正记录均保持导入时结果，无任何部分更新。
	cur, _ := m.CurrentPose()
	expectPose(t, "current after rejected corrections", cur,
		Pose{Time: 200, X: 0, Y: 0, Heading: 0, Variance: 1.0})
	for _, tc := range []struct {
		time int64
		x    float64
	}{
		{0, 0}, {100, 0}, {200, 0},
	} {
		p, err := m.PoseAt(tc.time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", tc.time, err)
		}
		if p.Time != tc.time || p.X != tc.x || p.Y != 0 || p.Heading != 0 {
			t.Fatalf("PoseAt(%d) = %+v, want pose at x=%v unchanged", tc.time, p, tc.x)
		}
		approxEq(t, "variance unchanged", p.Variance, 1.0)
	}
	lms, _ = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("landmarks after rejected corrections = %v, want only L", lms)
	}
	approxEq(t, "L x unchanged", lms[0].X, 0.5)
	approxEq(t, "L y unchanged", lms[0].Y, 0)
	if lms[0].Count != 2 {
		t.Fatalf("L count after rejected corrections = %d, want 2", lms[0].Count)
	}
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 || !h.Appearances[0].Active ||
		!h.Appearances[0].HasFirstSeen || h.Appearances[0].FirstSeenTime != 100 ||
		h.Appearances[0].Landmark.Count != 2 {
		t.Fatalf("L appearance after rejected corrections = %+v", h.Appearances)
	}
	if _, err := m.LandmarkAppearances("ZZ"); !errors.Is(err, ErrLandmarkNotFound) {
		t.Fatalf("stranger landmark ZZ leaked into map: %v", err)
	}
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("correction records after rejections = %v, want none", recs)
	}

	// 同一校正标识未被失败的提交消耗：把目标改成不再冲突的恒等位置
	// 校正后仍可用同一标识成功，证明此前拒绝确实只针对已接受观测引起
	// 的冲突，而非标识或锚点状态被污染。
	okReq := Correction{
		ID:     "c-loop",
		Anchor: 200,
		Target: CorrectionTarget{X: 0, Y: 0, Heading: 0, Variance: 0},
	}
	rec, err := m.Correct(okReq)
	if err != nil {
		t.Fatalf("identity correction after conflicts: %v", err)
	}
	if len(rec.Landmarks) != 1 || rec.Landmarks[0].ID != "L" || rec.Landmarks[0].After.Count != 2 {
		t.Fatalf("identity correction record = %+v", rec.Landmarks)
	}
	approxEq(t, "L x after identity correction", rec.Landmarks[0].After.X, 0.5)
}
