package posemap

import (
	"path/filepath"
	"testing"
)

// 纯平移回环校正（无旋转、两轴平移量与校正后坐标都有限）在轨迹横跨 ±1e308
// 时不得被误报为 non_finite：校正不经过相对锚点的偏移计算，逐轴平移。某轴
// 平移量为零时该轴每帧坐标保持原值；锚点准确采用提交目标，其余帧保持相对
// 位置；路标观测随位姿重放，观测次数不变；落盘重开后查询与记录一致。
func TestCorrectionPureTranslationHugeSpan(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.bin")
	m, err := Create(path, Config{
		InitialTime: 0, InitialX: 1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 100, MergeDistance: 10,
	})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	// 三帧：第一帧不移动，后两帧各沿自身 X 方向移动 -1e308；运动方差与
	// 朝向增量均为零。保存位置 (1e308,0)、(0,0)、(-1e308,0)。末帧在自身
	// 原点观测路标 L 一次。
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100},
		{Time: 200, DX: -1e308},
		{Time: 300, DX: -1e308, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}

	// 锚点 100，目标位置 (1e308,1)、朝向与方差为零：X 轴平移量为零，
	// Y 轴平移 1。三帧应改为 (1e308,1)、(0,1)、(-1e308,1)。
	rec, err := m.Correct(Correction{
		ID:     "c",
		Anchor: 100,
		Target: CorrectionTarget{X: 1e308, Y: 1, Heading: 0, Variance: 0},
	})
	if err != nil {
		t.Fatalf("pure translation correction rejected: %v", err)
	}

	wantX := []float64{1e308, 0, -1e308}
	beforeX := []float64{1e308, 0, -1e308}
	wantTime := []int64{100, 200, 300}
	if rec.EndTime != 300 || len(rec.Poses) != 3 {
		t.Fatalf("record = end %d poses %d, want end 300 and 3 poses", rec.EndTime, len(rec.Poses))
	}
	for i, p := range rec.Poses {
		if p.Before.Time != wantTime[i] || p.After.Time != wantTime[i] {
			t.Fatalf("pose %d time changed: %d -> %d", i, p.Before.Time, p.After.Time)
		}
		if p.Before.X != beforeX[i] || p.Before.Y != 0 || p.Before.Heading != 0 {
			t.Fatalf("pose %d before = %+v, want (%v,0,0)", i, p.Before, beforeX[i])
		}
		if p.After.X != wantX[i] || p.After.Y != 1 || p.After.Heading != 0 {
			t.Fatalf("pose %d after = (%v,%v,%v), want (%v,1,0)",
				i, p.After.X, p.After.Y, p.After.Heading, wantX[i])
		}
		if p.After.Variance != 0 {
			t.Fatalf("pose %d variance = %v, want 0 (target 0 + zero motion variances)", i, p.After.Variance)
		}
	}
	// 锚点准确采用提交目标。
	if rec.Poses[0].After.X != 1e308 || rec.Poses[0].After.Y != 1 {
		t.Fatalf("anchor after = (%v,%v), want exactly the target (1e308,1)",
			rec.Poses[0].After.X, rec.Poses[0].After.Y)
	}

	// 路标条目：末帧自身原点的单次观测从 (-1e308,0) 变为 (-1e308,1)，次数不变。
	if len(rec.Landmarks) != 1 {
		t.Fatalf("landmark changes = %d, want 1", len(rec.Landmarks))
	}
	lc := rec.Landmarks[0]
	if lc.ID != "L" || lc.Occurrence != 1 {
		t.Fatalf("landmark change = %+v, want L occurrence 1", lc)
	}
	if lc.Before.X != -1e308 || lc.Before.Y != 0 || lc.Before.Count != 1 {
		t.Fatalf("landmark before = %+v, want (-1e308,0) count 1", lc.Before)
	}
	if lc.After.X != -1e308 || lc.After.Y != 1 || lc.After.Count != 1 {
		t.Fatalf("landmark after = %+v, want (-1e308,1) count 1", lc.After)
	}

	// 当前位姿与按时间查询立即反映校正结果。
	check := func(m *Map) {
		t.Helper()
		cur, err := m.CurrentPose()
		if err != nil {
			t.Fatalf("CurrentPose: %v", err)
		}
		if cur.Time != 300 || cur.X != -1e308 || cur.Y != 1 || cur.Heading != 0 || cur.Variance != 0 {
			t.Fatalf("current pose = %+v, want time 300 at (-1e308,1)", cur)
		}
		for i, tm := range wantTime {
			p, err := m.PoseAt(tm)
			if err != nil {
				t.Fatalf("PoseAt(%d): %v", tm, err)
			}
			if p.X != wantX[i] || p.Y != 1 || p.Heading != 0 || p.Variance != 0 {
				t.Fatalf("PoseAt(%d) = %+v, want (%v,1) heading 0 variance 0", tm, p, wantX[i])
			}
		}
		// 初始位姿（时间 0）在锚点之前，保持 (1e308,0) 不变。
		p0, err := m.PoseAt(0)
		if err != nil {
			t.Fatalf("PoseAt(0): %v", err)
		}
		if p0.Time != 0 || p0.X != 1e308 || p0.Y != 0 {
			t.Fatalf("initial pose changed: %+v", p0)
		}
		lms, err := m.LandmarksInRect(Rect{MinX: -1e308, MinY: 0, MaxX: 0, MaxY: 2})
		if err != nil {
			t.Fatalf("LandmarksInRect: %v", err)
		}
		if len(lms) != 1 || lms[0].ID != "L" || lms[0].X != -1e308 || lms[0].Y != 1 || lms[0].Count != 1 {
			t.Fatalf("landmark query = %+v, want L at (-1e308,1) count 1", lms)
		}
	}
	check(m)

	// 落盘后重新打开同一地图：不报错损坏，查询与校正记录一致。
	if err := m.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("Open after pure translation correction: %v", err)
	}
	defer m2.Close()
	check(m2)
	recs, err := m2.Corrections()
	if err != nil {
		t.Fatalf("Corrections after reopen: %v", err)
	}
	if len(recs) != 1 || recs[0].ID != "c" || len(recs[0].Poses) != 3 || len(recs[0].Landmarks) != 1 {
		t.Fatalf("corrections after reopen = %+v, want one record c with 3 poses and 1 landmark", recs)
	}
	for i, p := range recs[0].Poses {
		if p.After.X != wantX[i] || p.After.Y != 1 || p.Before.X != beforeX[i] || p.Before.Y != 0 {
			t.Fatalf("reopened record pose %d = %+v -> %+v", i, p.Before, p.After)
		}
	}
	if recs[0].Landmarks[0].After.X != -1e308 || recs[0].Landmarks[0].After.Y != 1 {
		t.Fatalf("reopened record landmark = %+v", recs[0].Landmarks[0])
	}
}

// 纯平移中某轴平移量为零时，该轴每帧坐标逐位保持原值（含符号为零的坐标），
// 不因另一轴的平移而改变；另一轴按平移量平移。
func TestCorrectionPureTranslationZeroAxisUntouched(t *testing.T) {
	m := newMap(t, Config{
		InitialTime: 0, InitialX: 0, InitialY: -1e308, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 100, MergeDistance: 10,
	})
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100},
		{Time: 200, DY: 1e308},
		{Time: 300, DY: 1e308},
	}}); err != nil {
		t.Fatal(err)
	}
	// 只沿 X 平移 5：Y 轴平移量为零，三帧 Y 必须保持 -1e308、0、1e308。
	rec, err := m.Correct(Correction{
		ID:     "c",
		Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: -1e308, Heading: 0, Variance: 0},
	})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	wantY := []float64{-1e308, 0, 1e308}
	wantX := []float64{5, 5, 5}
	for i, p := range rec.Poses {
		if p.After.X != wantX[i] || p.After.Y != wantY[i] {
			t.Fatalf("pose %d after = (%v,%v), want (%v,%v)", i, p.After.X, p.After.Y, wantX[i], wantY[i])
		}
	}
	for i, tm := range []int64{100, 200, 300} {
		p, _ := m.PoseAt(tm)
		if p.X != wantX[i] || p.Y != wantY[i] {
			t.Fatalf("PoseAt(%d) = (%v,%v), want (%v,%v)", tm, p.X, p.Y, wantX[i], wantY[i])
		}
	}
}
