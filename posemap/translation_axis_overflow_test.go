package posemap

import (
	"path/filepath"
	"testing"
)

// 纯平移（不改变朝向）校正中，一个轴的平移量为零时，该轴每一帧已保存的
// 坐标必须原样保留：即使整条轨迹在另一轴的平移下把该轴坐标带到横跨
// ±1e308，也不能先求“相对锚点偏移”而溢出为非有限值，进而把这次合法校
// 正误报为 non_finite。锚点仍准确采用提交目标，其余帧保持原相对位置；
// 时间、朝向不变，方差沿用目标方差加锚点之后原运动方差的累计规则。
func TestCorrectionPureTranslationPreservesZeroShiftAxis(t *testing.T) {
	// 任务示例：初始 (1e308,0)、朝向零；t=100 不动，t=200/300 各沿自身
	// X 移动 -1e308，保存 (1e308,0)、(0,0)、(-1e308,0)。锚点 100 目标
	// (1e308,1)：只沿 Y 平移 1，X 不动，三帧 X 必须保持原值，末帧路标
	// 从 (-1e308,0) 变为 (-1e308,1)。
	path := filepath.Join(t.TempDir(), "map.bin")
	m, err := Create(path, Config{
		InitialTime: 0, InitialX: 1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 100, MergeDistance: 10,
	})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100},
		{Time: 200, DX: -1e308},
		{Time: 300, DX: -1e308, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}

	rec, err := m.Correct(Correction{ID: "c", Anchor: 100,
		Target: CorrectionTarget{X: 1e308, Y: 1, Heading: 0, Variance: 0}})
	if err != nil {
		t.Fatalf("pure translation across ±1e308 rejected: %v", err)
	}

	wantPoses := []struct {
		tm                       int64
		bx, by, ax, ay, variance float64
	}{
		{100, 1e308, 0, 1e308, 1, 0},
		{200, 0, 0, 0, 1, 0},
		{300, -1e308, 0, -1e308, 1, 0},
	}
	if rec.EndTime != 300 || len(rec.Poses) != 3 {
		t.Fatalf("record = end %d poses %d, want end 300 and 3 poses", rec.EndTime, len(rec.Poses))
	}
	for i, w := range wantPoses {
		pc := rec.Poses[i]
		if pc.Before.Time != w.tm || pc.After.Time != w.tm {
			t.Fatalf("pose %d time changed: %d -> %d", i, pc.Before.Time, pc.After.Time)
		}
		if pc.Before.X != w.bx || pc.Before.Y != w.by {
			t.Fatalf("pose %d before = (%v,%v), want (%v,%v)", i, pc.Before.X, pc.Before.Y, w.bx, w.by)
		}
		if pc.After.X != w.ax || pc.After.Y != w.ay {
			t.Fatalf("pose %d after = (%v,%v), want (%v,%v)", i, pc.After.X, pc.After.Y, w.ax, w.ay)
		}
		if pc.Before.Heading != 0 || pc.After.Heading != 0 {
			t.Fatalf("pose %d heading changed: %v -> %v", i, pc.Before.Heading, pc.After.Heading)
		}
		if pc.After.Variance != w.variance || pc.Before.Variance != 0 {
			t.Fatalf("pose %d variance = %v -> %v, want 0 -> %v", i, pc.Before.Variance, pc.After.Variance, w.variance)
		}
	}

	// 路标：末帧自身原点的单次观测随位姿只在 Y 上平移，X 保持 -1e308，
	// 观测次数不变。
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
	cur, _ := m.CurrentPose()
	if cur.Time != 300 || cur.X != -1e308 || cur.Y != 1 || cur.Heading != 0 || cur.Variance != 0 {
		t.Fatalf("current pose = %+v, want t300 (-1e308,1) heading 0 variance 0", cur)
	}
	for _, w := range wantPoses {
		p, err := m.PoseAt(w.tm)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", w.tm, err)
		}
		if p.X != w.ax || p.Y != w.ay || p.Heading != 0 || p.Variance != 0 {
			t.Fatalf("PoseAt(%d) = %+v, want (%v,%v)", w.tm, p, w.ax, w.ay)
		}
	}
	// 路标区域查询反映新位置 Y=1；旧位置 Y=0 不再返回。
	if lms, _ := m.LandmarksInRect(Rect{MinX: -1e308, MinY: 1, MaxX: 1e308, MaxY: 1}); len(lms) != 1 ||
		lms[0].X != -1e308 || lms[0].Y != 1 || lms[0].Count != 1 {
		t.Fatalf("landmark at new y=1 = %+v", lms)
	}
	if lms, _ := m.LandmarksInRect(Rect{MinX: -1e308, MinY: 0, MaxX: 1e308, MaxY: 0}); len(lms) != 0 {
		t.Fatalf("landmark still reported at old y=0: %+v", lms)
	}

	// 落盘后重开：不损坏，查询与校正记录（含前后值）一致。
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("reopen after pure translation: %v", err)
	}
	defer m2.Close()
	cur, _ = m2.CurrentPose()
	if cur.Time != 300 || cur.X != -1e308 || cur.Y != 1 {
		t.Fatalf("after reopen current = %+v, want t300 (-1e308,1)", cur)
	}
	for _, w := range wantPoses {
		p, _ := m2.PoseAt(w.tm)
		if p.X != w.ax || p.Y != w.ay || p.Heading != 0 || p.Variance != 0 {
			t.Fatalf("after reopen PoseAt(%d) = %+v, want (%v,%v)", w.tm, p, w.ax, w.ay)
		}
	}
	recs, _ := m2.Corrections()
	if len(recs) != 1 || recs[0].ID != "c" || len(recs[0].Poses) != 3 || len(recs[0].Landmarks) != 1 {
		t.Fatalf("after reopen record = %+v, want 1 record c with 3 poses and 1 landmark", recs)
	}
	if rl := recs[0].Landmarks[0]; rl.Before.X != -1e308 || rl.Before.Y != 0 ||
		rl.After.X != -1e308 || rl.After.Y != 1 || rl.After.Count != 1 {
		t.Fatalf("after reopen landmark record = %+v", rl)
	}
}

// 反向同样成立：沿 X 平移、Y 不动时，横跨 ±1e308 的 Y 坐标逐帧保留。
func TestCorrectionPureTranslationOrthogonalAxis(t *testing.T) {
	m := newMap(t, Config{
		InitialTime: 0, InitialX: 0, InitialY: 1e308, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 100, MergeDistance: 10,
	})
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100},
		{Time: 200, DY: -1e308},
		{Time: 300, DY: -1e308, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c", Anchor: 100,
		Target: CorrectionTarget{X: 1, Y: 1e308}}); err != nil {
		t.Fatalf("x-only translation across ±1e308 y-span rejected: %v", err)
	}
	want := []struct {
		tm   int64
		x, y float64
	}{
		{100, 1, 1e308},
		{200, 1, 0},
		{300, 1, -1e308},
	}
	for _, w := range want {
		p, _ := m.PoseAt(w.tm)
		if p.X != w.x || p.Y != w.y || p.Heading != 0 {
			t.Fatalf("PoseAt(%d) = (%v,%v,%v), want (%v,%v,0)", w.tm, p.X, p.Y, p.Heading, w.x, w.y)
		}
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: 1, MinY: -1e308, MaxX: 1, MaxY: -1e308})
	if len(lms) != 1 || lms[0].X != 1 || lms[0].Y != -1e308 || lms[0].Count != 1 {
		t.Fatalf("landmark = %+v, want (1,-1e308) count 1", lms)
	}
}

// 只沿一个轴平移时方差累计规则不变：锚点取目标方差，其后每帧加锚点之后
// 的原运动方差。
func TestCorrectionPureTranslationAccumulatesVariance(t *testing.T) {
	m := newMap(t, Config{
		InitialTime: 0, InitialX: 1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 1, MaxInterval: 100, MergeDistance: 10,
	})
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, MoveVariance: 0.25},
		{Time: 200, DX: -1e308, MoveVariance: 0.5},
		{Time: 300, DX: -1e308, MoveVariance: 0.75},
	}}); err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c", Anchor: 100,
		Target: CorrectionTarget{X: 1e308, Y: 1, Variance: 2}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	wantVar := []float64{2, 2.5, 3.25}
	for i, v := range wantVar {
		if rec.Poses[i].After.Variance != v {
			t.Fatalf("pose %d variance = %v, want %v", i, rec.Poses[i].After.Variance, v)
		}
	}
}

// 放宽只针对“平移量为零的轴”：确实发生移动的轴若校正后位置无法表示
// （远端帧相对锚点偏移溢出），仍按既有 non_finite 整次拒绝。
func TestCorrectionPureTranslationGenuineOverflowStillRejected(t *testing.T) {
	m := newMap(t, Config{
		InitialTime: 0, InitialX: -1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 100, MergeDistance: 10,
	})
	// 锚点 -1e308，三帧 -1e308、0、+1e308；沿 X 做可分辨的非零平移，
	// 末帧 before-anchor = 2e308 溢出为 +Inf。
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100},
		{Time: 200, DX: 1e308},
		{Time: 300, DX: 1e308},
	}}); err != nil {
		t.Fatal(err)
	}
	_, err := m.Correct(Correction{ID: "c", Anchor: 100,
		Target: CorrectionTarget{X: -1e308 + 1e293, Y: 0, Variance: 0}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectNonFinite || !r.HasTime || r.Time != 100 {
		t.Fatalf("err = %v, want non_finite marked at anchor 100", err)
	}
	// 原子性：轨迹与校正记录保持原样。
	if p, _ := m.PoseAt(300); p.X != 1e308 {
		t.Fatalf("pose changed after rejection: %+v", p)
	}
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("correction record leaked: %+v", recs)
	}
}

// 打开核对与提交共享同一 poseShift 规则：合法的零平移轴记录正常打开（见
// 上方用例的重开断言），而被篡改成“该轴坐标发生变化”的记录仍按损坏拒
// 绝，不会因为零平移轴走保留分支就放过几何矛盾。
func TestOpenRejectsTamperedFixedAxis(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.bin")
	m, err := Create(path, Config{
		InitialTime: 0, InitialX: 1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 100, MergeDistance: 10,
	})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100},
		{Time: 200, DX: -1e308},
		{Time: 300, DX: -1e308},
	}}); err != nil {
		t.Fatal(err)
	}
	// X 平移量为零的纯平移：三帧 X 必须分别保持 1e308、0、-1e308。
	if _, err := m.Correct(Correction{ID: "c", Anchor: 100,
		Target: CorrectionTarget{X: 1e308, Y: 1, Variance: 0}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 把中间帧（t=200）校正后 X 从应保留的 0 改成另一个有限值：违反
	// “零平移轴坐标保持原值”，按文件损坏拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Poses[1].After.X = 5
	})
	assertOpenCorrupt(t, path)
}
