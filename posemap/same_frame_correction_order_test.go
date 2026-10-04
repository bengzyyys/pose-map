package posemap

import (
	"math"
	"testing"
)

// 本文件为“同帧多条观测同一路标时，回环校正重放仍按输入次序逐条接纳”
// 这一已有行为提供回归保障，只使用现有公开入口（Create/ImportSegment/
// Correct/Corrections/LandmarkAppearances/PoseAt 等），不改变任何公开
// 使用方式与返回结果。
//
// 统一几何（只沿世界 X 轴分布，合并上限 1，初始位姿 (0,0)、朝向 0）：
//
//	p0 (t=0)   (0,0)   朝向 0   —— 锚点之前对 L 的一次固定观测：本地 (0,0)
//	                             转换到地图为 0.0
//	帧 t=50：DX=0，姿态不变，对 L 一次观测 → 世界 X=0（计数 1）
//	帧 t=100（锚点帧）：DX=0，同帧四条 L 观测，本地 X 依次为 -0.5/-1/0.5/0
//	帧 t=200：DX=0，无观测，便于检查更早位姿在校正后不变
//
// 锚点之前的观测使用原地图位置且在校正中固定；锚点及之后的观测按校正后
// 位姿转换。校正只平移不旋转（目标朝向保持 0），因此每条观测转换后的
// 世界 X = shift + 本地 X， admitted 判定退化为 1 维距离。
//
// 同帧四条观测的次序至关重要（每条都与“当时含更早观测的平均位置”比较，
// 接纳后均值立即移动，下一条要与新位置比较）：
//
//	次序 A = [-0.5, -1.0, +0.5, 0]   导入时各步距离 0.5, 0.75, 1.0(恰含上限), 0.25
//	                              校正平移 -0.5 后：1.0(恰含), 1.0(恰含), 0.833…, 0.125 → 全部接纳
//	次序 B = [0, -1.0, +0.5, -0.5]   导入时各步距离 0, 1.0(恰含), 0.833…, 0.375
//	                              校正平移 -0.5 后：0.5, 1.25 → 第二条即超合并距离被拒
//
// 两份原轨迹都合法（导入时两组各步距离均不超过合并上限，最终均值同为
// -0.2、总观测 5 次），差别完全来自校正后的逐条接纳过程。

// sameFrameConfig 是这组场景的地图配置：原点初始位姿、初始位置方差
// 0.5、合并上限 1。
func sameFrameConfig() Config {
	return Config{InitialTime: 0, InitialVariance: 0.5, MaxInterval: 1000, MergeDistance: 1.0}
}

// sameFrameLGeometry 按给定同帧次序构造统一几何的合法原轨迹：锚点之前对
// L 一次固定观测，锚点帧同帧四条 L 观测，另有一帧无观测的后续帧。
func sameFrameLGeometry(orderA bool) Segment {
	var localXs [4]float64
	if orderA {
		localXs = [4]float64{-0.5, -1.0, 0.5, 0.0}
	} else {
		localXs = [4]float64{0.0, -1.0, 0.5, -0.5}
	}
	obs := make([]Observation, 4)
	for i, x := range localXs {
		obs[i] = Observation{ID: "L", X: x, Y: 0}
	}
	return Segment{
		ID: "seg",
		Frames: []Frame{
			{Time: 50, DX: 0, DY: 0, DHeading: 0, MoveVariance: 0.1,
				Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
			{Time: 100, DX: 0, DY: 0, DHeading: 0, MoveVariance: 0.1,
				Observations: obs},
			{Time: 200, DX: 0, DY: 0, DHeading: 0, MoveVariance: 0.1},
		},
	}
}

// 同帧多条观测的位置校正成功：每条都与当时平均位置比较，距离恰好等于
// 合并上限仍接纳；全部接纳后路标位置是所有已接受观测的等权平均，同帧
// 每条观测单独计数（5 条观测没有被合并成一次）。成功记录与区域/历次
// 出现查询一致，早期位姿与既有记录保持不变。
func TestCorrectionSameFrameObservationsAcceptedInOrder(t *testing.T) {
	cfg := sameFrameConfig()
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(sameFrameLGeometry(true)); err != nil {
		t.Fatalf("import order A: %v", err)
	}

	// 校正前：5 条观测等权平均 X = (0 - 0.5 - 1 + 0.5 + 0)/5 = -0.2，计数 5。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("landmarks before correction = %v", lms)
	}
	approxEq(t, "L x before", lms[0].X, -0.2)
	if lms[0].Count != 5 {
		t.Fatalf("L count before = %d, want 5 (each same-frame obs counted)", lms[0].Count)
	}

	// 只改变位置、不改变朝向的合法校正：锚点 (100,0) 平移到 X=-0.5，
	// 朝向保持原朝向 0、方差非负。
	req := Correction{
		ID:     "c-order",
		Anchor: 100,
		Target: CorrectionTarget{X: -0.5, Y: 0, Heading: 0, Variance: 0},
	}
	rec, err := m.Correct(req)
	if err != nil {
		t.Fatalf("Correct order A: %v", err)
	}

	// 校正后的五条世界 X：锚点前固定为 0；锚点帧四条为 shift+本地
	// = -1.0, -1.5, 0.0, -0.5。等权平均 = (0 - 1 - 1.5 + 0 - 0.5)/5
	// = -3/5 = -0.6，仍为 5 条观测。
	if len(rec.Landmarks) != 1 {
		t.Fatalf("landmark changes = %+v", rec.Landmarks)
	}
	lc := rec.Landmarks[0]
	if lc.ID != "L" || lc.Occurrence != 1 {
		t.Fatalf("change = %+v, want L occurrence 1", lc)
	}
	approxEq(t, "L before x", lc.Before.X, -0.2)
	approxEq(t, "L before y", lc.Before.Y, 0)
	approxEq(t, "L after x", -0.6, lc.After.X)
	approxEq(t, "L after y", 0, lc.After.Y)
	// 观测次数与出现编号保持不变。
	if lc.Before.Count != 5 || lc.After.Count != 5 {
		t.Fatalf("L count = %d -> %d, want 5 -> 5", lc.Before.Count, lc.After.Count)
	}

	// 路标查询结果与校正记录中的新位置一致。
	lms, _ = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("landmarks after = %v", lms)
	}
	approxEq(t, "queried L x", lms[0].X, lc.After.X)
	approxEq(t, "queried L y", lms[0].Y, lc.After.Y)
	if lms[0].Count != lc.After.Count {
		t.Fatalf("queried count = %d, record count = %d", lms[0].Count, lc.After.Count)
	}

	// 历次出现：仍只有第 1 次出现、仍有效，位置/计数与记录一致，首次
	// 观测时间为锚点之前的 50。
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	ap := h.Appearances[0]
	if ap.Number != 1 || !ap.Active || ap.FirstSeenTime != 50 || !ap.HasFirstSeen {
		t.Fatalf("appearance = %+v", ap)
	}
	approxEq(t, "appearance L x", ap.Landmark.X, lc.After.X)
	if ap.Landmark.Count != 5 {
		t.Fatalf("appearance count = %d, want 5", ap.Landmark.Count)
	}

	// 更早的位姿（初始位姿 t=0 与锚点前一帧 t=50）不变；锚点及之后采用
	// 校正后位姿。
	checkPose := func(time int64, wantX float64) Pose {
		t.Helper()
		p, err := m.PoseAt(time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", time, err)
		}
		if p.X != wantX || p.Heading != 0 {
			t.Fatalf("PoseAt(%d) = %+v, want X=%v heading=0", time, p, wantX)
		}
		return p
	}
	p50 := checkPose(50, 0)
	approxEq(t, "t=50 variance unchanged", p50.Variance, 0.6) // 初始 0.5 + 0.1
	p0 := checkPose(0, 0)
	approxEq(t, "initial variance unchanged", p0.Variance, 0.5)
	p100, _ := m.PoseAt(100)
	approxEq(t, "anchor x after", p100.X, -0.5)
	approxEq(t, "anchor heading after", p100.Heading, 0)
	// 锚点后方差 = 目标方差 0 + 锚点后运动方差 0.1（t=200）；t=100 为目标方差。
	approxEq(t, "anchor variance after", p100.Variance, 0)
	p200, _ := m.PoseAt(200)
	approxEq(t, "t=200 x after", p200.X, -0.5)
	approxEq(t, "t=200 variance after", p200.Variance, 0.1)

	// 既有校正记录只增不改：记录保留校正前后位置与不变的次数。
	recs, _ := m.Corrections()
	if len(recs) != 1 || !recordsEqual(recs[0], rec) {
		t.Fatalf("records = %+v, want %+v", recs, rec)
	}
}

// 输入次序确实影响校正接纳结果，而不只是影响最终均值：同一份观测点
// 集合，按次序 A 导入的地图上提交平移 -0.5 的位置校正全部接纳成功；
// 另一个独立地图以次序 B 成功导入这些点后，提交完全相同的校正，却在
// 锚点帧第二条同帧观测处超过合并距离被拒。两份原轨迹都合法，差别只
// 来自校正后的逐条接纳过程。
func TestCorrectionSameFrameOrderDeterminesAcceptance(t *testing.T) {
	req := Correction{
		ID:     "c-order",
		Anchor: 100,
		Target: CorrectionTarget{X: -0.5, Y: 0, Heading: 0, Variance: 0},
	}

	// 地图 A：次序 A，同样的校正成功。
	mA := newMap(t, sameFrameConfig())
	if _, err := mA.ImportSegment(sameFrameLGeometry(true)); err != nil {
		t.Fatalf("import A: %v", err)
	}
	recA, err := mA.Correct(req)
	if err != nil {
		t.Fatalf("correction on order A rejected: %v", err)
	}
	approxEq(t, "A L after x", recA.Landmarks[0].After.X, -0.6)
	if recA.Landmarks[0].After.Count != 5 {
		t.Fatalf("A count = %d, want 5", recA.Landmarks[0].After.Count)
	}

	// 地图 B：独立文件，次序 B 成功导入（原轨迹合法）。
	mB := newMap(t, sameFrameConfig())
	if _, err := mB.ImportSegment(sameFrameLGeometry(false)); err != nil {
		t.Fatalf("import B: %v", err)
	}
	// 导入后两个地图的路标状态相同：同一观测点集合的等权平均与次数。
	bBefore, _ := mB.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(bBefore) != 1 {
		t.Fatalf("B landmarks = %v", bBefore)
	}
	approxEq(t, "B L x before", bBefore[0].X, -0.2)
	if bBefore[0].Count != 5 {
		t.Fatalf("B count before = %d, want 5", bBefore[0].Count)
	}
	// 提交同样的校正：锚点帧第二条同帧观测与接纳第一条后的新均值相距
	// 1.25 > 合并上限 1 → 整次拒绝。
	_, err = mB.Correct(req)
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("B correction err = %v, want *RejectError", err)
	}
	if r.Kind != RejectLandmarkConflict {
		t.Fatalf("B correction kind = %q, want %q", r.Kind, RejectLandmarkConflict)
	}
	// 错误指出实际冲突帧时间、路标标识及所属出现编号。
	if !r.HasTime || r.Time != 100 || !r.HasLandmark || r.Landmark != "L" ||
		!r.HasOccurrence || r.Occurrence != 1 {
		t.Fatalf("B reject = %+v, want time=100 landmark=L occurrence=1", r)
	}

	// B 整次校正不生效：路标位置/次数、位姿、校正记录均为提交前结果。
	assertUnchangedAfterRejectedOrderB(t, mB)
}

// assertUnchangedAfterRejectedOrderB 校验次序 B 地图在校正被整体拒绝后
// 没有任何部分更新：同帧前一条已暂时接纳的观测没有改变路标位置或次数，
// 位姿与既有校正记录也保持提交前结果。
func assertUnchangedAfterRejectedOrderB(t *testing.T, m *Map) {
	t.Helper()
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("landmarks after reject = %v", lms)
	}
	approxEq(t, "L x unchanged", lms[0].X, -0.2)
	approxEq(t, "L y unchanged", lms[0].Y, 0)
	if lms[0].Count != 5 {
		t.Fatalf("L count after reject = %d, want 5 (tentative admission leaked)", lms[0].Count)
	}
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 || !h.Appearances[0].Active {
		t.Fatalf("appearances after reject = %+v", h.Appearances)
	}
	approxEq(t, "appearance x unchanged", h.Appearances[0].Landmark.X, -0.2)
	if h.Appearances[0].Landmark.Count != 5 {
		t.Fatalf("appearance count after reject = %d", h.Appearances[0].Landmark.Count)
	}
	for _, tc := range []struct {
		time         int64
		wantX        float64
		wantHeading  float64
		wantVariance float64
	}{
		{0, 0, 0, 0.5},
		{50, 0, 0, 0.6},
		{100, 0, 0, 0.7},
		{200, 0, 0, 0.8},
	} {
		p, err := m.PoseAt(tc.time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", tc.time, err)
		}
		if p.X != tc.wantX || p.Heading != tc.wantHeading || math.Abs(p.Variance-tc.wantVariance) > 1e-12 {
			t.Fatalf("PoseAt(%d) = %+v, want X=%v heading=%v variance=%v",
				tc.time, p, tc.wantX, tc.wantHeading, tc.wantVariance)
		}
	}
	// 同标识校正可再次提交（冲突的那次没有消耗校正标识、没有落记录）。
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("correction records after reject = %v, want none", recs)
	}
}

// 同帧内前一条已被接纳、后一条才发生冲突：冲突观测之前的同帧第一条
// 观测（与固定观测距离恰好等于合并上限）已在暂存聚合中接纳，但整次
// 校正仍必须失败且不留任何部分更新。错误指出实际冲突帧时间、路标
// 标识与所属出现编号。
func TestCorrectionSameFrameConflictAfterTentativeAdmission(t *testing.T) {
	cfg := sameFrameConfig()
	m := newMap(t, cfg)
	// t=50：对 L 的固定观测，世界 X=0。
	// t=100：同帧两条 L 观测，本地 X 依次为 0、+1（导入时世界 X 为
	// 0、+1：第一条与固定观测重合，第二条距当时均值 0.5 恰在限内，
	// 两条都合法）。
	if _, err := m.ImportSegment(Segment{
		ID: "seg",
		Frames: []Frame{
			{Time: 50, DX: 0, MoveVariance: 0.1,
				Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
			{Time: 100, DX: 0, MoveVariance: 0.1,
				Observations: []Observation{{ID: "L", X: 0, Y: 0}, {ID: "L", X: 1, Y: 0}}},
		},
	}); err != nil {
		t.Fatalf("import: %v", err)
	}
	// 校正前 L 为 (0,0,+1)/3 即 X=1/3，计数 3。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 {
		t.Fatalf("landmarks = %v", lms)
	}
	approxEq(t, "L x before", lms[0].X, 1.0/3.0)
	if lms[0].Count != 3 {
		t.Fatalf("L count before = %d, want 3", lms[0].Count)
	}

	// 只平移不旋转 +1：锚点帧第一条世界 X=1，与固定观测 0 的距离恰好
	// 等于合并上限 1 → 暂纳（均值变为 0.5、计数 2）；第二条世界 X=2，
	// 与新均值 0.5 相距 1.5 > 1 → 冲突。
	_, err := m.Correct(Correction{
		ID:     "c-conflict",
		Anchor: 100,
		Target: CorrectionTarget{X: 1, Y: 0, Heading: 0, Variance: 0},
	})
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("err = %v, want *RejectError", err)
	}
	if r.Kind != RejectLandmarkConflict || !r.HasTime || r.Time != 100 ||
		!r.HasLandmark || r.Landmark != "L" || !r.HasOccurrence || r.Occurrence != 1 {
		t.Fatalf("reject = %+v, want landmark_conflict at time 100 L occurrence 1", r)
	}

	// 整次校正不生效：第一条已暂纳的观测没有改变路标位置或次数。
	lms, _ = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 {
		t.Fatalf("landmarks after conflict = %v", lms)
	}
	approxEq(t, "L x after conflict", lms[0].X, 1.0/3.0)
	if lms[0].Count != 3 {
		t.Fatalf("L count after conflict = %d, want 3 (tentative admission leaked)", lms[0].Count)
	}
	h, _ := m.LandmarkAppearances("L")
	if len(h.Appearances) != 1 || !h.Appearances[0].Active {
		t.Fatalf("appearances after conflict = %+v", h.Appearances)
	}
	approxEq(t, "appearance x after conflict", h.Appearances[0].Landmark.X, 1.0/3.0)
	if h.Appearances[0].Landmark.Count != 3 {
		t.Fatalf("appearance count after conflict = %d", h.Appearances[0].Landmark.Count)
	}
	// 位姿保持提交前结果。
	cur, _ := m.CurrentPose()
	if cur.Time != 100 || cur.X != 0 || cur.Heading != 0 {
		t.Fatalf("current pose after conflict = %+v", cur)
	}
	approxEq(t, "current variance after conflict", cur.Variance, 0.7)
	// 既有校正记录保持提交前结果（无记录）。
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("correction records after conflict = %v, want none", recs)
	}

	// 同一张图把平移改成恰好可接纳（0 平移的恒等位置校正）后可成功，
	// 证明被拒完全源于同帧逐条次序而非观测或轨迹本身。
	rec, err := m.Correct(Correction{
		ID:     "c-conflict",
		Anchor: 100,
		Target: CorrectionTarget{X: 0, Y: 0, Heading: 0, Variance: 0},
	})
	if err != nil {
		t.Fatalf("identity correction after conflict: %v", err)
	}
	if len(rec.Landmarks) != 1 || rec.Landmarks[0].After.Count != 3 {
		t.Fatalf("identity record = %+v", rec.Landmarks)
	}
	approxEq(t, "L x after identity", rec.Landmarks[0].After.X, 1.0/3.0)
}
