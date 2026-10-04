package posemap

import (
	"math"
	"path/filepath"
	"testing"
)

// 只调整位置方差的恒等校正（目标位置与朝向取锚点帧当前值）在轨迹坐标横跨
// ±1e308 时曾被误判为 non_finite：旋转平移公式先求相对偏移 old-anchor，
// 跨度 2e308 先溢出为无穷再退化为 NaN。几何上没有任何平移或旋转时校正前后
// 位姿必须完全相同，只有方差按累计规则改变。纵向或两个坐标同时大跨度遵守
// 同一规则。
func TestCorrectionVarianceOnlyHugeSpan(t *testing.T) {
	// 三个轴向场景：初始位姿在 -1e308，三帧位移把轨迹带到 0、+1e308。
	type axis struct {
		name      string
		initX     float64
		initY     float64
		dx        []float64
		dy        []float64
		wantPoses [3][2]float64
	}
	cases := []axis{
		{
			name:      "x axis",
			initX:     -1e308,
			dx:        []float64{0, 1e308, 1e308},
			dy:        []float64{0, 0, 0},
			wantPoses: [3][2]float64{{-1e308, 0}, {0, 0}, {1e308, 0}},
		},
		{
			name:      "y axis",
			initY:     -1e308,
			dx:        []float64{0, 0, 0},
			dy:        []float64{0, 1e308, 1e308},
			wantPoses: [3][2]float64{{0, -1e308}, {0, 0}, {0, 1e308}},
		},
		{
			name:      "both axes",
			initX:     -1e308,
			initY:     -1e308,
			dx:        []float64{0, 1e308, 1e308},
			dy:        []float64{0, 1e308, 1e308},
			wantPoses: [3][2]float64{{-1e308, -1e308}, {0, 0}, {1e308, 1e308}},
		},
	}
	for _, ac := range cases {
		t.Run(ac.name, func(t *testing.T) {
			cfg := Config{
				InitialTime: 0, InitialX: ac.initX, InitialY: ac.initY, InitialHeading: 0,
				InitialVariance: 1, MaxInterval: 100, MergeDistance: 10,
			}
			m := newMap(t, cfg)
			if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
				{Time: 100, DX: ac.dx[0], DY: ac.dy[0], MoveVariance: 0.25},
				{Time: 200, DX: ac.dx[1], DY: ac.dy[1], MoveVariance: 0.5},
				{Time: 300, DX: ac.dx[2], DY: ac.dy[2], MoveVariance: 0.75},
			}}); err != nil {
				t.Fatal(err)
			}

			anchor, _ := m.PoseAt(100)
			rec, err := m.Correct(Correction{
				ID:     "c",
				Anchor: 100,
				Target: CorrectionTarget{X: anchor.X, Y: anchor.Y, Heading: anchor.Heading, Variance: 4},
			})
			if err != nil {
				t.Fatalf("variance-only correction on huge span rejected: %v", err)
			}

			// 三个受影响帧：位置、朝向、时间不变，方差依次为 4、4.5、5.25。
			wantVar := []float64{4, 4.5, 5.25}
			wantTime := []int64{100, 200, 300}
			if len(rec.Poses) != 3 {
				t.Fatalf("record poses = %d, want 3 (anchor through last frame)", len(rec.Poses))
			}
			for i, p := range rec.Poses {
				if p.Before.Time != wantTime[i] || p.After.Time != wantTime[i] {
					t.Fatalf("pose %d time changed: %d -> %d", i, p.Before.Time, p.After.Time)
				}
				if p.Before.X != p.After.X || p.Before.Y != p.After.Y || p.Before.Heading != p.After.Heading {
					t.Fatalf("pose %d geometry changed: %+v -> %+v", i, p.Before, p.After)
				}
				if p.After.X != ac.wantPoses[i][0] || p.After.Y != ac.wantPoses[i][1] || p.After.Heading != 0 {
					t.Fatalf("pose %d after = (%v,%v,%v), want (%v,%v,0)", i, p.After.X, p.After.Y, p.After.Heading,
						ac.wantPoses[i][0], ac.wantPoses[i][1])
				}
				if p.After.Variance != wantVar[i] {
					t.Fatalf("pose %d variance = %v, want %v", i, p.After.Variance, wantVar[i])
				}
			}

			// 当前位姿与按时间查询立即体现新方差，几何不变。
			cur, _ := m.CurrentPose()
			if cur.Time != 300 || cur.X != ac.wantPoses[2][0] || cur.Y != ac.wantPoses[2][1] ||
				cur.Heading != 0 || cur.Variance != 5.25 {
				t.Fatalf("current pose = %+v, want time 300 at (%v,%v) variance 5.25",
					cur, ac.wantPoses[2][0], ac.wantPoses[2][1])
			}
			for i, tm := range wantTime {
				p, err := m.PoseAt(tm)
				if err != nil {
					t.Fatalf("PoseAt(%d): %v", tm, err)
				}
				if p.X != ac.wantPoses[i][0] || p.Y != ac.wantPoses[i][1] ||
					p.Heading != 0 || p.Variance != wantVar[i] {
					t.Fatalf("PoseAt(%d) = %+v, want (%v,%v) variance %v",
						tm, p, ac.wantPoses[i][0], ac.wantPoses[i][1], wantVar[i])
				}
			}

			// 初始位姿（时间 0、原坐标、方差 1）不改变，也不进校正记录。
			p0, err := m.PoseAt(0)
			if err != nil {
				t.Fatalf("PoseAt(0): %v", err)
			}
			if p0.Time != 0 || p0.X != ac.initX || p0.Y != ac.initY || p0.Heading != 0 || p0.Variance != 1 {
				t.Fatalf("initial pose changed: %+v", p0)
			}
		})
	}
}

// 恒等校正仍生成完整校正记录：锚点至提交时末帧的前后位姿齐全；该范围内被
// 观测路标的各次出现都有条目，前后位置相同也保留，位置、观测次数、出现编号
// 与有效状态不变。之后再次校正不改写已有记录。
func TestCorrectionVarianceOnlyRecord(t *testing.T) {
	m := newMap(t, Config{
		InitialTime: 0, InitialX: -1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 1, MaxInterval: 100,
		// 相邻观测的地图坐标相隔最大 1.5e308，合并距离需覆盖。
		MergeDistance: 1.6e308,
	})
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, DX: 0, MoveVariance: 0.25, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1e308, MoveVariance: 0.5, Observations: []Observation{{ID: "L"}}},
		{Time: 300, DX: 1e308, MoveVariance: 0.75, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}

	rec, err := m.Correct(Correction{ID: "c", Anchor: 100,
		Target: CorrectionTarget{X: -1e308, Y: 0, Heading: 0, Variance: 4}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if rec.EndTime != 300 || len(rec.Poses) != 3 {
		t.Fatalf("record = end %d poses %d, want end 300 and 3 poses", rec.EndTime, len(rec.Poses))
	}
	// 路标条目保留：前后位置相同也必须有条目，次数/编号不变。
	if len(rec.Landmarks) != 1 {
		t.Fatalf("landmark changes = %d, want 1 retained even with equal before/after", len(rec.Landmarks))
	}
	lc := rec.Landmarks[0]
	if lc.ID != "L" || lc.Occurrence != 1 {
		t.Fatalf("landmark change = %+v, want L occurrence 1", lc)
	}
	if lc.Before.X != lc.After.X || lc.Before.Y != lc.After.Y {
		t.Fatalf("landmark moved on identity correction: %+v -> %+v", lc.Before, lc.After)
	}
	if lc.Before.Count != 3 || lc.After.Count != 3 {
		t.Fatalf("landmark count = %d -> %d, want 3 unchanged", lc.Before.Count, lc.After.Count)
	}

	// 查询与历次出现立即反映：位置/次数/编号/有效状态保持原值。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1, MinY: -1, MaxX: 1, MaxY: 1})
	if len(lms) != 1 || lms[0].ID != "L" || lms[0].X != 0 || lms[0].Y != 0 || lms[0].Count != 3 {
		t.Fatalf("landmark query after correction = %+v, want L at (0,0) count 3", lms)
	}
	h, err := m.LandmarkAppearances("L")
	if err != nil || len(h.Appearances) != 1 {
		t.Fatalf("appearances = %+v err=%v, want exactly 1", h, err)
	}
	a := h.Appearances[0]
	if !a.Active || a.Number != 1 || a.Landmark.Count != 3 || a.FirstSeenTime != 100 {
		t.Fatalf("appearance changed: %+v", a)
	}

	// 再次校正只追加新记录，不改写首条。
	rec2, err := m.Correct(Correction{ID: "c2", Anchor: 200,
		Target: CorrectionTarget{X: 0, Y: 0, Heading: 0, Variance: 9}})
	if err != nil {
		t.Fatalf("second correction: %v", err)
	}
	if len(rec2.Poses) != 2 {
		t.Fatalf("second record poses = %d, want 2 (200..300)", len(rec2.Poses))
	}
	all, _ := m.Corrections()
	if len(all) != 2 || all[0].ID != "c" || all[1].ID != "c2" {
		t.Fatalf("corrections = %+v, want c then c2, first never rewritten", all)
	}
	if len(all[0].Poses) != 3 || all[0].Poses[0].After.Variance != 4 {
		t.Fatalf("first record rewritten: %+v", all[0])
	}
}

// 恒等校正只放宽“几何相同但相对偏移计算溢出”这一种误拒绝；其余拒绝规则不变：
// 负方差、非有限目标、有限目标但累计后非有限（标明锚点时间）、真正改变位置
// 或朝向的校正保留原有 non_finite 行为。任何拒绝不留部分更新。
func TestCorrectionVarianceOnlyRejections(t *testing.T) {
	build := func(t *testing.T, merge float64) *Map {
		m := newMap(t, Config{
			InitialTime: 0, InitialX: -1e308, InitialY: 0, InitialHeading: 0,
			InitialVariance: 1, MaxInterval: 100, MergeDistance: merge,
		})
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 100, DX: 0, MoveVariance: 0.25},
			{Time: 200, DX: 1e308, MoveVariance: 0.5},
			{Time: 300, DX: 1e308, MoveVariance: 0.75},
		}}); err != nil {
			t.Fatal(err)
		}
		return m
	}

	// 溢出专用：锚点之后一帧携带 1e308 的运动方差。
	buildOverflow := func(t *testing.T) *Map {
		m := newMap(t, Config{
			InitialTime: 0, InitialX: -1e308, InitialY: 0, InitialHeading: 0,
			InitialVariance: 1, MaxInterval: 100, MergeDistance: 10,
		})
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 100, DX: 0, MoveVariance: 0},
			{Time: 200, DX: 1e308, MoveVariance: 1e308},
			{Time: 300, DX: 1e308, MoveVariance: 0},
		}}); err != nil {
			t.Fatal(err)
		}
		return m
	}

	t.Run("negative variance", func(t *testing.T) {
		m := build(t, 10)
		_, err := m.Correct(Correction{ID: "x", Anchor: 100,
			Target: CorrectionTarget{X: -1e308, Variance: -0.01}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNegativeVariance || !r.HasTime || r.Time != 100 {
			t.Fatalf("err = %v, want negative_variance at anchor 100", err)
		}
	})

	t.Run("non-finite target variance", func(t *testing.T) {
		m := build(t, 10)
		_, err := m.Correct(Correction{ID: "x", Anchor: 100,
			Target: CorrectionTarget{X: -1e308, Variance: math.NaN()}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite || !r.HasTime || r.Time != 100 {
			t.Fatalf("err = %v, want non_finite at anchor 100", err)
		}
	})

	t.Run("finite target variance accumulates to infinity", func(t *testing.T) {
		m := buildOverflow(t)
		// 目标方差本身有限；与锚点之后帧 200 的 1e308 运动方差累计即溢出。
		_, err := m.Correct(Correction{ID: "x", Anchor: 100,
			Target: CorrectionTarget{X: -1e308, Variance: 1e308}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite || !r.HasTime || r.Time != 100 {
			t.Fatalf("err = %v, want non_finite marked with anchor time 100", err)
		}
		// 原子性：方差、位置、校正记录保持原值。
		cur, _ := m.CurrentPose()
		if cur.Variance != 1e308 || cur.X != 1e308 {
			t.Fatalf("state changed after rejection: %+v", cur)
		}
		if p, _ := m.PoseAt(100); p.Variance != 1 {
			t.Fatalf("anchor variance changed after rejection: %v", p.Variance)
		}
		if recs, _ := m.Corrections(); len(recs) != 0 {
			t.Fatalf("correction record leaked: %+v", recs)
		}
	})

	t.Run("heading change keeps arithmetic rejection", func(t *testing.T) {
		m := build(t, 10)
		// 位置不变但朝向改变：相对偏移 2e308 旋转后溢出，保留原有 non_finite。
		_, err := m.Correct(Correction{ID: "x", Anchor: 100,
			Target: CorrectionTarget{X: -1e308, Heading: math.Pi, Variance: 4}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite || !r.HasTime || r.Time != 100 {
			t.Fatalf("err = %v, want non_finite at anchor 100 for heading change", err)
		}
		if p, _ := m.PoseAt(100); p.Heading != 0 || p.Variance != 1.25 {
			t.Fatalf("anchor changed after rejection: %+v", p)
		}
	})

	t.Run("position change keeps arithmetic rejection", func(t *testing.T) {
		m := build(t, 10)
		// 朝向不变但位置确实改变：1e308 量级下 ULP 约 1.6e292，+1 会被舍掉
		// （数值上仍是同一锚点位姿），取可分辨的 1e293；远端帧相对偏移仍溢出。
		_, err := m.Correct(Correction{ID: "x", Anchor: 100,
			Target: CorrectionTarget{X: -1e308 + 1e293, Variance: 4}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite || !r.HasTime || r.Time != 100 {
			t.Fatalf("err = %v, want non_finite at anchor 100 for position change", err)
		}
		if p, _ := m.PoseAt(300); p.X != 1e308 || p.Variance != 2.5 {
			t.Fatalf("pose changed after rejection: %+v", p)
		}
	})
}

// 恒等校正落盘后重开：位置与新方差、校正记录（锚点至末帧的前后位姿）一致。
func TestCorrectionVarianceOnlyPersists(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.bin")
	m, err := Create(path, Config{
		InitialTime: 0, InitialX: -1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 1, MaxInterval: 100, MergeDistance: 10,
	})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, DX: 0, MoveVariance: 0.25},
		{Time: 200, DX: 1e308, MoveVariance: 0.5},
		{Time: 300, DX: 1e308, MoveVariance: 0.75},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c", Anchor: 100,
		Target: CorrectionTarget{X: -1e308, Variance: 4}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer m2.Close()
	want := []struct {
		tm       int64
		x        float64
		variance float64
	}{{100, -1e308, 4}, {200, 0, 4.5}, {300, 1e308, 5.25}}
	for _, w := range want {
		p, err := m2.PoseAt(w.tm)
		if err != nil {
			t.Fatalf("PoseAt(%d) after reopen: %v", w.tm, err)
		}
		if p.X != w.x || p.Y != 0 || p.Heading != 0 || p.Variance != w.variance {
			t.Fatalf("after reopen PoseAt(%d) = %+v, want X=%v variance %v", w.tm, p, w.x, w.variance)
		}
	}
	recs, _ := m2.Corrections()
	if len(recs) != 1 || recs[0].ID != "c" || len(recs[0].Poses) != 3 {
		t.Fatalf("after reopen corrections = %+v, want one record c with 3 poses", recs)
	}
}
