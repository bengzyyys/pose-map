package posemap

import (
	"math"
	"testing"
)

// 回环校正重放同帧多条观测时，必须沿用导入时的“按输入次序逐条接纳”规则：
// 同帧观测不是可以任意换序的一组点——每接纳一条，该路标的平均位置都会改变，
// 下一条要与这个包含更早（含本帧已接纳）观测的新平均位置比较。本文件围绕
// “锚点之前已有观测、锚点帧又包含同一路标多条观测”的已导入轨迹，保障这一
// 既有行为：只平移、不转朝向的合法校正下，同帧次序既能让逐条接纳全部成功，
// 也能让另一份仅次序不同、同样合法的轨迹在同帧后一条观测上冲突。
//
// 几何（合并上限 1；校正前各位姿均为 (0,0)、朝向 0；校正只平移）：
//
//	锚点之前 t=100 帧观测 O 本地 (0,0)，地图点 o=(0,0)，校正时固定不变；
//	锚点帧 t=200 的两条观测本地坐标为 u1=(-s,1/2)、u2=(s,-1/2)，其中
//	s=√3/4。
//
// 校正把锚点帧平移到 (s,1/2)（朝向不变），于是 u1→q1=(0,1)、
// u2→q2=(√3/2,0)；锚点之前的观测仍固定在 (0,0)。
//
//	次序一 [u1,u2]：q1 距固定观测 (0,0) 恰为 1（含上限，接纳），平均到
//	  (0,1/2)；q2 距 (0,1/2) 也恰为 1（接纳）。最终为三次观测的等权平均
//	  (√3/6, 1/3)，同帧两条各自计数，共 3 次。
//	次序二 [u2,u1]：q2 先接纳（距 (0,0) 为 √3/2≈0.866），平均到
//	  (√3/4,0)；q1 再与该平均比较，距离 √19/4≈1.0897 > 1，冲突。
//
// 关键：次序二里 q1、q2 各自单独相对固定观测 (0,0) 的距离分别是 1 与
// √3/2，都不超限；只有“先接纳 q2 改变平均、再比较 q1”的逐条规则才会拒绝。
// 因而该冲突能区分正确实现与“把同帧各点都拿去和固定均值比较”的错误换序实现。
func sameFrameOrderConfig() Config {
	return Config{
		InitialTime: 0, InitialX: 0, InitialY: 0, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 1000, MergeDistance: 1.0,
	}
}

// sameFrameOrderCorrection 是两份地图共同提交的校正：锚点 t=200 只平移到
// (s,1/2)，朝向仍为 0，不转朝向。
func sameFrameOrderCorrection() Correction {
	return Correction{
		ID:     "c",
		Anchor: 200,
		Target: CorrectionTarget{X: math.Sqrt(3) / 4, Y: 0.5, Heading: 0, Variance: 0},
	}
}

// 次序一：锚点帧先 u1 再 u2。导入与校正重放下逐条接纳都成功。
func sameFrameOrderSegmentA() Segment {
	s := math.Sqrt(3) / 4
	return Segment{ID: "s", Frames: []Frame{
		// 锚点之前的观测：地图点 (0,0)，作为校正时固定不变的更早观测。
		{Time: 100, Observations: []Observation{{ID: "O", X: 0, Y: 0}}},
		// 锚点帧：同一路标两条同帧观测，按本次序逐条接纳。
		{Time: 200, Observations: []Observation{
			{ID: "O", X: -s, Y: 0.5},
			{ID: "O", X: s, Y: -0.5},
		}},
	}}
}

// 次序二：同样三个观测点，仅把锚点帧的两条观测换序（先 u2 再 u1）。
// 导入仍合法；同一份校正重放时会在同帧后一条（u1→q1）上超过合并距离。
func sameFrameOrderSegmentB() Segment {
	s := math.Sqrt(3) / 4
	return Segment{ID: "s", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "O", X: 0, Y: 0}}},
		{Time: 200, Observations: []Observation{
			{ID: "O", X: s, Y: -0.5},
			{ID: "O", X: -s, Y: 0.5},
		}},
	}}
}

// 次序一全部接纳：更早观测固定在原地图位置；锚点帧两条观测都恰在合并上限上
// 被逐条接纳；路标为全部已接受观测（同帧各条单独计数）的等权平均，次数与
// 出现编号不变；查询结果与校正记录新位置一致，更早位姿不变。
func TestCorrectionSameFrameObservationsKeptInOrder(t *testing.T) {
	m := newMap(t, sameFrameOrderConfig())
	if _, err := m.ImportSegment(sameFrameOrderSegmentA()); err != nil {
		t.Fatalf("import order A: %v", err)
	}

	rec, err := m.Correct(sameFrameOrderCorrection())
	if err != nil {
		t.Fatalf("correction admitted all same-frame observations in order: %v", err)
	}
	if rec.Anchor != 200 || rec.EndTime != 200 || len(rec.Poses) != 1 {
		t.Fatalf("record header = %+v", rec)
	}
	// 只平移、不转朝向：锚点位姿从 (0,0) 到 (s,1/2)，朝向前后都为 0。
	pc := rec.Poses[0]
	if pc.Before.X != 0 || pc.Before.Y != 0 {
		t.Fatalf("anchor before = %+v, want (0,0)", pc.Before)
	}
	approxEq(t, "anchor after x", pc.After.X, math.Sqrt(3)/4)
	approxEq(t, "anchor after y", pc.After.Y, 0.5)
	if pc.Before.Heading != 0 || pc.After.Heading != 0 {
		t.Fatalf("heading changed: %v -> %v", pc.Before.Heading, pc.After.Heading)
	}

	// 路标条目保留原位置与校正后位置；同帧两条各计一次，共 3 次，出现编号 1。
	if len(rec.Landmarks) != 1 {
		t.Fatalf("landmark changes = %+v, want one entry for O", rec.Landmarks)
	}
	lc := rec.Landmarks[0]
	if lc.ID != "O" || lc.Occurrence != 1 {
		t.Fatalf("landmark change = %+v, want O occurrence 1", lc)
	}
	// 导入后三次观测等权平均为 (0,0)（次序 u1,u2 关于原点对称）。
	approxEq(t, "before x", lc.Before.X, 0)
	approxEq(t, "before y", lc.Before.Y, 0)
	if lc.Before.Count != 3 {
		t.Fatalf("before count = %d, want 3 (pre-anchor + two same-frame)", lc.Before.Count)
	}
	// 校正后仍为 3 次观测的等权平均 (√3/6, 1/3)，次数不被合并成一次。
	approxEq(t, "after x", lc.After.X, math.Sqrt(3)/6)
	approxEq(t, "after y", lc.After.Y, 1.0/3)
	if lc.After.Count != 3 {
		t.Fatalf("after count = %d, want 3, same-frame observations counted separately", lc.After.Count)
	}

	// 查询到的新位置与校正记录一致，次数一致。
	lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil || len(lms) != 1 {
		t.Fatalf("landmarks = %v, %v", lms, err)
	}
	approxEq(t, "queried x", lms[0].X, lc.After.X)
	approxEq(t, "queried y", lms[0].Y, lc.After.Y)
	if lms[0].ID != "O" || lms[0].Count != 3 {
		t.Fatalf("queried landmark = %+v, want O count 3", lms[0])
	}
	h, err := m.LandmarkAppearances("O")
	if err != nil || len(h.Appearances) != 1 {
		t.Fatalf("appearances = %+v, %v", h, err)
	}
	a := h.Appearances[0]
	if !a.Active || a.Number != 1 || a.Landmark.Count != 3 || !a.HasFirstSeen || a.FirstSeenTime != 100 {
		t.Fatalf("appearance = %+v, want active occurrence 1 count 3 first seen 100", a)
	}
	approxEq(t, "appearance x", a.Landmark.X, lc.After.X)
	approxEq(t, "appearance y", a.Landmark.Y, lc.After.Y)

	// 更早的位姿（初始位姿与锚点之前的 t=100 帧）保持原地图结果，不入校正记录。
	p0, _ := m.PoseAt(0)
	if p0.X != 0 || p0.Y != 0 {
		t.Fatalf("initial pose changed: %+v", p0)
	}
	pPre, _ := m.PoseAt(100)
	if pPre.X != 0 || pPre.Y != 0 || pPre.Heading != 0 {
		t.Fatalf("pre-anchor pose changed: %+v", pPre)
	}
	pAnchor, _ := m.PoseAt(200)
	approxEq(t, "queried anchor x", pAnchor.X, math.Sqrt(3)/4)
	approxEq(t, "queried anchor y", pAnchor.Y, 0.5)

	// 记录只增不改：返回记录即 Corrections() 中的唯一记录。
	recs, _ := m.Corrections()
	if len(recs) != 1 || !recordsEqual(recs[0], rec) {
		t.Fatalf("corrections = %+v, want the single returned record", recs)
	}
}

// 次序对结果确实有影响，而不只是检查最终均值：另一份独立地图以不同次序成功
// 导入同样的观测点（原轨迹同样合法、最终均值同为原点），提交同一份只平移的
// 校正却被拒绝——先接纳的同帧第一条把平均位置拉走后，第二条超过合并距离。
func TestCorrectionSameFrameOrderChangesAcceptance(t *testing.T) {
	// 次序一：成功（作为对照，确认这份校正本身合法、差别只来自同帧次序）。
	ma := newMap(t, sameFrameOrderConfig())
	if _, err := ma.ImportSegment(sameFrameOrderSegmentA()); err != nil {
		t.Fatalf("map A import: %v", err)
	}
	if _, err := ma.Correct(sameFrameOrderCorrection()); err != nil {
		t.Fatalf("map A correction should succeed in order A: %v", err)
	}

	// 次序二：独立地图、同样合法地导入这些点。
	mb := newMap(t, sameFrameOrderConfig())
	if _, err := mb.ImportSegment(sameFrameOrderSegmentB()); err != nil {
		t.Fatalf("map B import must also be legal: %v", err)
	}
	// 导入结果与次序一相同：三次观测等权平均 (0,0)、次数 3。
	before, _ := mb.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(before) != 1 || before[0].Count != 3 {
		t.Fatalf("map B landmark after import = %v", before)
	}
	approxEq(t, "map B mean x", before[0].X, 0)
	approxEq(t, "map B mean y", before[0].Y, 0)

	// 同一份校正：在同帧后一条观测（t=200）上超过合并距离，整次拒绝。
	_, err := mb.Correct(sameFrameOrderCorrection())
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("map B correction err = %v, want *RejectError", err)
	}
	if r.Kind != RejectLandmarkConflict || !r.HasTime || r.Time != 200 ||
		!r.HasLandmark || r.Landmark != "O" || !r.HasOccurrence || r.Occurrence != 1 {
		t.Fatalf("map B reject = %+v, want landmark_conflict at frame 200 O occurrence 1", r)
	}

	// 整次校正不生效：路标位置/次数保持提交前，位姿与校正记录不变。
	after, _ := mb.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(after) != 1 || after[0].Count != 3 {
		t.Fatalf("map B landmark changed after conflict: %v", after)
	}
	approxEq(t, "map B mean x after", after[0].X, 0)
	approxEq(t, "map B mean y after", after[0].Y, 0)
	for _, tm := range []int64{0, 100, 200} {
		p, _ := mb.PoseAt(tm)
		if p.X != 0 || p.Y != 0 || p.Heading != 0 {
			t.Fatalf("map B pose at %d changed after conflict: %+v", tm, p)
		}
	}
	if recs, _ := mb.Corrections(); len(recs) != 0 {
		t.Fatalf("map B correction record leaked after conflict: %+v", recs)
	}
}

// 同帧前一条已被暂时接纳、后一条才发生冲突时，不得留下部分更新：暂时接纳的
// 第一条不能改变路标位置或次数，位姿与既有校正记录保持提交前结果。
func TestCorrectionSameFrameConflictAfterFirstAdmissionAtomic(t *testing.T) {
	m := newMap(t, sameFrameOrderConfig())
	// 锚点之前 t=100 观测 O → (0,0)；锚点帧 t=200 两条观测本地 (0,1)、(0,-1/2)
	// → (0,1)、(0,-1/2)。导入时逐条接纳：第一条距 (0,0) 恰为 1，平均 (0,1/2)；
	// 第二条距 (0,1/2) 也恰为 1，平均 (0,1/6)，共 3 次，导入合法。
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "O", X: 0, Y: 0}}},
		{Time: 200, Observations: []Observation{
			{ID: "O", X: 0, Y: 1},
			{ID: "O", X: 0, Y: -0.5},
		}},
	}}); err != nil {
		t.Fatalf("import: %v", err)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].Count != 3 {
		t.Fatalf("landmark after import = %v", lms)
	}
	approxEq(t, "mean y after import", lms[0].Y, 1.0/6)

	// 只把锚点帧向下平移 1（朝向不变）：第一条校正后到 (0,0)，距固定观测
	// (0,0) 为 0 被暂时接纳（平均被拉到 (0,0)）；第二条到 (0,-3/2)，距该暂时
	// 平均 (0,0) 为 1.5 > 1，冲突。冲突发生在同帧后一条。
	_, err := m.Correct(Correction{
		ID: "c", Anchor: 200,
		Target: CorrectionTarget{X: 0, Y: -1, Heading: 0, Variance: 0},
	})
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("correction err = %v, want *RejectError", err)
	}
	if r.Kind != RejectLandmarkConflict || r.Time != 200 || !r.HasTime ||
		r.Landmark != "O" || !r.HasLandmark || !r.HasOccurrence || r.Occurrence != 1 {
		t.Fatalf("reject = %+v, want landmark_conflict at frame 200 O occurrence 1", r)
	}

	// 整次校正不生效：第一条的暂时接纳不得落路标的位置/次数上（位置仍是
	// (0,1/6) 而不是暂时平均 (0,0)，次数仍为 3），出现编号与有效状态不变。
	lms, _ = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "O" || lms[0].Count != 3 {
		t.Fatalf("landmark leaked after same-frame conflict: %v", lms)
	}
	approxEq(t, "mean x after conflict", lms[0].X, 0)
	approxEq(t, "mean y after conflict", lms[0].Y, 1.0/6)
	h, _ := m.LandmarkAppearances("O")
	if len(h.Appearances) != 1 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	a := h.Appearances[0]
	if !a.Active || a.Number != 1 || a.Landmark.Count != 3 {
		t.Fatalf("appearance changed after conflict: %+v", a)
	}
	approxEq(t, "appearance y after conflict", a.Landmark.Y, 1.0/6)

	// 位姿保持提交前结果；不新增校正记录。
	for _, tm := range []int64{0, 100, 200} {
		p, _ := m.PoseAt(tm)
		if p.X != 0 || p.Y != 0 || p.Heading != 0 {
			t.Fatalf("pose at %d changed after conflict: %+v", tm, p)
		}
	}
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("correction record leaked: %+v", recs)
	}
}
