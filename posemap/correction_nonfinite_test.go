package posemap

import (
	"math"
	"os"
	"path/filepath"
	"testing"
)

// 题目场景：机器人始终停在原点，t=50 与 t=200 各观测一次 L，本地坐标
// (1e308,0)，导入时地图坐标仍是有限的 1e308，正常接受。以没有观测的
// t=100 为锚点把目标位置改为 (1e308,0) 后，机器人位姿仍有限，但 t=200
// 的观测转换为 1e308+1e308=+Inf。必须按 non_finite 拒绝并指出 t=200、
// 路标 L、出现编号 1（该次出现的第 2 条观测），不能报与 t=50 的距离冲突，
// 也不能用锚点时间 100。
func TestCorrectionObservationOverflowNotConflict(t *testing.T) {
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 100, MergeDistance: 10})
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "L", X: 1e308}}},
		{Time: 100},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1e308}}},
	}}); err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}

	_, err := m.Correct(Correction{
		ID: "c", Anchor: 100,
		Target: CorrectionTarget{X: 1e308, Y: 0, Heading: 0, Variance: 0},
	})
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("err = %v, want *RejectError", err)
	}
	if r.Kind != RejectNonFinite {
		t.Fatalf("kind = %q, want non_finite (not landmark_conflict)", r.Kind)
	}
	if !r.HasTime || r.Time != 200 {
		t.Fatalf("time = %d(ok=%v), want the overflowing observation frame 200, not anchor 100", r.Time, r.HasTime)
	}
	if !r.HasLandmark || r.Landmark != "L" {
		t.Fatalf("landmark = %q(ok=%v), want L", r.Landmark, r.HasLandmark)
	}
	if !r.HasOccurrence || r.Occurrence != 1 {
		t.Fatalf("occurrence = %d(ok=%v), want 1", r.Occurrence, r.HasOccurrence)
	}

	// 整次校正不生效：位姿、路标位置与次数、校正记录都保持提交前的值。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 0 || cur.Y != 0 {
		t.Fatalf("current pose changed after rejection: %+v", cur)
	}
	h, _ := m.LandmarkAppearances("L")
	if len(h.Appearances) != 1 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	a := h.Appearances[0]
	if !a.Active || a.Landmark.Count != 2 || a.Landmark.X != 1e308 || a.Landmark.Y != 0 {
		t.Fatalf("landmark changed after rejection: %+v", a)
	}
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("correction records leaked: %v", recs)
	}
}

// 异常观测是其所属出现的第一条重放观测时，同样要指出帧时间、路标与出现
// 编号；这里它属于失效后再次出现的第 2 次（当前有效）出现，锚点 t=90
// 没有该路标观测，错误时间必须是异常帧 t=100 而不是锚点 90。
func TestCorrectionOverflowFirstObservationOfLaterOccurrence(t *testing.T) {
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 100, MergeDistance: 10})
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "L", X: 1e308}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "iv", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 90},
		{Time: 100, Observations: []Observation{{ID: "L", X: 1e308}}}, // 第 2 次出现的首条观测
	}}); err != nil {
		t.Fatal(err)
	}

	_, err := m.Correct(Correction{
		ID: "c", Anchor: 90,
		Target: CorrectionTarget{X: 1e308, Y: 0, Heading: 0, Variance: 0},
	})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectNonFinite {
		t.Fatalf("err = %v, want non_finite *RejectError", err)
	}
	if !r.HasTime || r.Time != 100 {
		t.Fatalf("time = %d(ok=%v), want 100 (actual frame), not anchor 90", r.Time, r.HasTime)
	}
	if !r.HasLandmark || r.Landmark != "L" || !r.HasOccurrence || r.Occurrence != 2 {
		t.Fatalf("want landmark L occurrence 2, got %+v", r)
	}

	// 两次出现的计数都不变；第 1 次仍失效，第 2 次仍有效。
	h, _ := m.LandmarkAppearances("L")
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	if h.Appearances[0].Active || h.Appearances[0].Landmark.Count != 1 {
		t.Fatalf("occurrence 1 changed: %+v", h.Appearances[0])
	}
	a2 := h.Appearances[1]
	if !a2.Active || a2.Landmark.Count != 1 || a2.Landmark.X != 1e308 {
		t.Fatalf("occurrence 2 changed: %+v", a2)
	}
}

// 校正跨越同一路标的多次出现，而异常观测实际属于当前已失效的第 1 次出现
// （在 t=60；第 2 次出现于 t=70）时，错误必须对应第 1 次出现与 t=60，
// 即使该记录已失效、即使锚点 t=50 本身没有该路标的任何观测。
func TestCorrectionOverflowBelongsToInactiveOccurrence(t *testing.T) {
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 100, MergeDistance: 10})
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 50},
		{Time: 60, Observations: []Observation{{ID: "L", X: 1e308}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "iv", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 70, Observations: []Observation{{ID: "L", X: 1e308}}},
	}}); err != nil {
		t.Fatal(err)
	}

	_, err := m.Correct(Correction{
		ID: "c", Anchor: 50,
		Target: CorrectionTarget{X: 1e308, Y: 0, Heading: 0, Variance: 0},
	})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectNonFinite {
		t.Fatalf("err = %v, want non_finite *RejectError", err)
	}
	if !r.HasTime || r.Time != 60 {
		t.Fatalf("time = %d(ok=%v), want inactive occurrence frame 60, not anchor 50", r.Time, r.HasTime)
	}
	if !r.HasLandmark || r.Landmark != "L" || !r.HasOccurrence || r.Occurrence != 1 {
		t.Fatalf("want landmark L occurrence 1 (the inactive one), got %+v", r)
	}
}

// 转换后坐标仍有限但确实超过合并距离时，继续返回距离冲突；大数值得到有限
// 结果且满足距离限制时照常接受。
func TestCorrectionHugeFiniteObservations(t *testing.T) {
	t.Run("finite but beyond merge distance stays conflict", func(t *testing.T) {
		cfg := Config{InitialTime: 0, MaxInterval: 100, MergeDistance: 1}
		m := newMap(t, cfg)
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 50, Observations: []Observation{{ID: "L", X: 1e308}}},
			{Time: 100, Observations: []Observation{{ID: "L", X: 1e308}}},
		}}); err != nil {
			t.Fatal(err)
		}
		// 锚点移到 1e307：t=50 固定在 1e308，t=100 重放为 1.1e308（有限），
		// 二者相距 1e307 > 合并距离 1，仍是距离冲突而非数值异常。
		_, err := m.Correct(Correction{
			ID: "c", Anchor: 100,
			Target: CorrectionTarget{X: 1e307, Y: 0, Heading: 0, Variance: 0},
		})
		r, ok := AsRejectError(err)
		if !ok {
			t.Fatalf("err = %v, want *RejectError", err)
		}
		if r.Kind != RejectLandmarkConflict || !r.HasTime || r.Time != 100 ||
			!r.HasLandmark || r.Landmark != "L" || !r.HasOccurrence || r.Occurrence != 1 {
			t.Fatalf("reject = %+v, want landmark_conflict at t=100 L occurrence 1", r)
		}
	})

	t.Run("finite and within merge distance accepted", func(t *testing.T) {
		cfg := Config{InitialTime: 0, MaxInterval: 100, MergeDistance: 3e307}
		m := newMap(t, cfg)
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 50, Observations: []Observation{{ID: "L", X: 1e308}}},
			{Time: 100, Observations: []Observation{{ID: "L", X: 1e308}}},
		}}); err != nil {
			t.Fatal(err)
		}
		rec, err := m.Correct(Correction{
			ID: "c", Anchor: 100,
			Target: CorrectionTarget{X: 1e307, Y: 0, Heading: 0, Variance: 0},
		})
		if err != nil {
			t.Fatalf("Correct: %v", err)
		}
		if len(rec.Landmarks) != 1 || rec.Landmarks[0].After.Count != 2 {
			t.Fatalf("record = %+v", rec.Landmarks)
		}
		// 均值 = (1e308 + 1.1e308)/2 = 1.05e308，保持有限。
		if got := rec.Landmarks[0].After.X; !isFinite(got) || math.Abs(got-1.05e308) > 1e294 {
			t.Fatalf("after X = %v, want ~1.05e308 finite", got)
		}
		lms, _ := m.LandmarksInRect(Rect{MinX: 0, MinY: -1, MaxX: math.MaxFloat64, MaxY: 1})
		if len(lms) != 1 || lms[0].Count != 2 || lms[0].X != 1.05e308 {
			t.Fatalf("queried landmark = %+v", lms)
		}
	})
}

// 校正后机器人位姿本身溢出时，沿用原有拒绝：non_finite 且只带锚点时间，
// 不携带路标/出现信息。
func TestCorrectionOverflowPoseKeepsAnchorError(t *testing.T) {
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 100, MergeDistance: 10})
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 50},
		{Time: 100, DX: 1e308}, // 导入时位姿 (1e308,0)，仍有限
	}}); err != nil {
		t.Fatal(err)
	}
	// 锚点移到 1e308：t=100 相对锚点再偏 1e308 → 1e308+1e308=+Inf。
	_, err := m.Correct(Correction{
		ID: "c", Anchor: 50,
		Target: CorrectionTarget{X: 1e308, Y: 0, Heading: 0, Variance: 0},
	})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectNonFinite {
		t.Fatalf("err = %v, want non_finite *RejectError", err)
	}
	if !r.HasTime || r.Time != 50 {
		t.Fatalf("time = %d(ok=%v), want anchor 50", r.Time, r.HasTime)
	}
	if r.HasLandmark || r.HasOccurrence {
		t.Fatalf("pose rejection unexpectedly carries landmark/occurrence: %+v", r)
	}
}

// 观测数值异常拒绝时整次校正不落盘：当前与历史位姿、路标位置与次数、
// 失效信息、已有校正记录与本地数据文件都保持提交前的值；校正标识不被
// 消耗，修正目标后同标识可提交成功，重开结果一致。
func TestCorrectionObservationOverflowAtomicity(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, Config{InitialTime: 0, MaxInterval: 100, MergeDistance: 10})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	defer m.Close()

	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 50, Observations: []Observation{
			{ID: "L", X: 1e308},
			{ID: "N", X: 1, Y: 2},
		}},
		{Time: 100},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1e308}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "iv", Reason: "gone", Landmarks: []string{"N"}}); err != nil {
		t.Fatal(err)
	}
	// 先放一条成功校正记录，确认失败的校正不会改写它。
	if _, err := m.Correct(Correction{ID: "c0", Anchor: 100, Target: CorrectionTarget{X: 0, Y: 0}}); err != nil {
		t.Fatalf("seed correction: %v", err)
	}

	before, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}

	_, err = m.Correct(Correction{
		ID: "c-bad", Anchor: 100,
		Target: CorrectionTarget{X: 1e308, Y: 0, Heading: 0, Variance: 0},
	})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectNonFinite || !r.HasTime || r.Time != 200 ||
		!r.HasLandmark || r.Landmark != "L" || !r.HasOccurrence || r.Occurrence != 1 {
		t.Fatalf("bad correction reject = %+v", r)
	}

	// 内存状态保持提交前值。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 0 || cur.Y != 0 {
		t.Fatalf("current pose changed: %+v", cur)
	}
	h, _ := m.LandmarkAppearances("L")
	if len(h.Appearances) != 1 || !h.Appearances[0].Active ||
		h.Appearances[0].Landmark.Count != 2 || h.Appearances[0].Landmark.X != 1e308 {
		t.Fatalf("L changed: %+v", h.Appearances)
	}
	hn, _ := m.LandmarkAppearances("N")
	if len(hn.Appearances) != 1 || hn.Appearances[0].Active ||
		hn.Appearances[0].InvalidReason != "gone" || hn.Appearances[0].InvalidOpID != "iv" {
		t.Fatalf("invalidation info changed: %+v", hn.Appearances)
	}
	if recs, _ := m.Corrections(); len(recs) != 1 || recs[0].ID != "c0" {
		t.Fatalf("correction history changed: %v", recs)
	}

	// 本地数据文件字节不变。
	after, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if len(after) != len(before) {
		t.Fatalf("data file changed size: %d -> %d", len(before), len(after))
	}
	for i := range before {
		if before[i] != after[i] {
			t.Fatalf("data file byte %d changed", i)
		}
	}

	// 校正标识未被消耗：同 ID 改用有限目标后提交成功（观测仍在 (1e308,0)，
	// 两次相距 0，接受；次数仍为 2）。
	rec, err := m.Correct(Correction{ID: "c-bad", Anchor: 100, Target: CorrectionTarget{X: 0, Y: 0}})
	if err != nil {
		t.Fatalf("correction id consumed by failed submit: %v", err)
	}
	if len(rec.Landmarks) != 1 || rec.Landmarks[0].After.X != 1e308 || rec.Landmarks[0].After.Count != 2 {
		t.Fatalf("retry record = %+v", rec.Landmarks)
	}

	// 重开后：两条校正记录、L 次数与失效信息均保持。
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer m2.Close()
	if recs, _ := m2.Corrections(); len(recs) != 2 || recs[0].ID != "c0" || recs[1].ID != "c-bad" {
		t.Fatalf("records after reopen = %v", recs)
	}
	h2, err := m2.LandmarkAppearances("L")
	if err != nil || len(h2.Appearances) != 1 || h2.Appearances[0].Landmark.Count != 2 {
		t.Fatalf("L after reopen: %+v %v", h2, err)
	}
	hn2, _ := m2.LandmarkAppearances("N")
	if len(hn2.Appearances) != 1 || hn2.Appearances[0].Active || hn2.Appearances[0].InvalidReason != "gone" {
		t.Fatalf("N after reopen: %+v", hn2.Appearances)
	}
}
