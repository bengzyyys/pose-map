package posemap

import (
	"math"
	"testing"
)

// 仅调整目标方差的回环校正（目标位置与朝向等于锚点帧当前值）不应因轨迹
// 坐标幅值巨大、横跨正负两侧而被误判为 non_finite：恒等变换下各帧位置与
// 朝向保持原值，只有方差按“目标方差 + 锚点之后运动方差累计”更新。
func TestCorrectionVarianceOnlyHugeCoordinates(t *testing.T) {
	cfg := Config{
		InitialTime: 0, InitialX: -1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 1, MaxInterval: 100, MergeDistance: 10,
	}

	// 任务示例：t=100/200/300 的自身坐标 X 位移为 0、1e308、1e308，运动
	// 方差 0.25、0.5、0.75；保存位置依次为 (-1e308,0)、(0,0)、(1e308,0)。
	// 以 t=100 为锚点保持位姿不变、目标方差改为 4 后，三帧方差依次为
	// 4、4.5、5.25，位置、朝向与时间保持原值，初始位姿不变。
	t.Run("lateral span across zero", func(t *testing.T) {
		m := newMap(t, cfg)
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 100, DX: 0, MoveVariance: 0.25},
			{Time: 200, DX: 1e308, MoveVariance: 0.5},
			{Time: 300, DX: 1e308, MoveVariance: 0.75},
		}}); err != nil {
			t.Fatal(err)
		}

		rec, err := m.Correct(Correction{
			ID:     "c",
			Anchor: 100,
			Target: CorrectionTarget{X: -1e308, Y: 0, Heading: 0, Variance: 4},
		})
		if err != nil {
			t.Fatalf("Correct: %v", err)
		}

		wantPos := []Pose{
			{Time: 100, X: -1e308, Y: 0, Heading: 0, Variance: 4},
			{Time: 200, X: 0, Y: 0, Heading: 0, Variance: 4.5},
			{Time: 300, X: 1e308, Y: 0, Heading: 0, Variance: 5.25},
		}
		for _, w := range wantPos {
			p, err := m.PoseAt(w.Time)
			if err != nil {
				t.Fatalf("PoseAt(%d): %v", w.Time, err)
			}
			if p != w {
				t.Fatalf("PoseAt(%d) = %+v, want %+v", w.Time, p, w)
			}
		}
		cur, err := m.CurrentPose()
		if err != nil {
			t.Fatalf("CurrentPose: %v", err)
		}
		if cur != wantPos[2] {
			t.Fatalf("CurrentPose = %+v, want %+v", cur, wantPos[2])
		}
		// 初始位姿不属于校正范围，保持原值。
		init, err := m.PoseAt(0)
		if err != nil {
			t.Fatalf("PoseAt(0): %v", err)
		}
		if init != (Pose{Time: 0, X: -1e308, Y: 0, Heading: 0, Variance: 1}) {
			t.Fatalf("initial pose changed: %+v", init)
		}

		// 校正记录完整保留锚点至末帧的前后位姿。
		if rec.Anchor != 100 || rec.EndTime != 300 || len(rec.Poses) != 3 {
			t.Fatalf("record = %+v", rec)
		}
		wantBefore := []Pose{
			{Time: 100, X: -1e308, Y: 0, Heading: 0, Variance: 1.25},
			{Time: 200, X: 0, Y: 0, Heading: 0, Variance: 1.75},
			{Time: 300, X: 1e308, Y: 0, Heading: 0, Variance: 2.5},
		}
		for i, w := range wantBefore {
			if rec.Poses[i].Before != w || rec.Poses[i].After != wantPos[i] {
				t.Fatalf("record pose %d = %+v, want before %+v after %+v",
					i, rec.Poses[i], w, wantPos[i])
			}
		}
	})

	// 纵向大幅跨度与两个坐标同时大幅跨度时同样适用。
	t.Run("longitudinal and diagonal spans", func(t *testing.T) {
		for _, tc := range []struct {
			name   string
			initX  float64
			initY  float64
			dx, dy float64
		}{
			{"y axis", 0, -1e308, 0, 1e308},
			{"both axes", -1e308, -1e308, 1e308, 1e308},
		} {
			t.Run(tc.name, func(t *testing.T) {
				c := cfg
				c.InitialX, c.InitialY = tc.initX, tc.initY
				m := newMap(t, c)
				if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
					{Time: 100, MoveVariance: 0.25},
					{Time: 200, DX: tc.dx, DY: tc.dy, MoveVariance: 0.5},
					{Time: 300, DX: tc.dx, DY: tc.dy, MoveVariance: 0.75},
				}}); err != nil {
					t.Fatal(err)
				}
				_, err := m.Correct(Correction{
					ID:     "c",
					Anchor: 100,
					Target: CorrectionTarget{X: tc.initX, Y: tc.initY, Heading: 0, Variance: 4},
				})
				if err != nil {
					t.Fatalf("Correct: %v", err)
				}
				wantVar := []float64{4, 4.5, 5.25}
				midX, midY := tc.initX+tc.dx, tc.initY+tc.dy
				wantX := []float64{tc.initX, midX, midX + tc.dx}
				wantY := []float64{tc.initY, midY, midY + tc.dy}
				for i, tm := range []int64{100, 200, 300} {
					p, _ := m.PoseAt(tm)
					if p.X != wantX[i] || p.Y != wantY[i] || p.Heading != 0 || p.Variance != wantVar[i] {
						t.Fatalf("PoseAt(%d) = %+v, want x=%v y=%v var=%v",
							tm, p, wantX[i], wantY[i], wantVar[i])
					}
				}
			})
		}
	})

	// 涉及的路标位置、观测次数、出现编号与有效状态保持原值；前后位置相同
	// 的路标仍保留在校正记录中。
	t.Run("landmarks unchanged but recorded", func(t *testing.T) {
		wideCfg := cfg
		wideCfg.MergeDistance = 1.5e308
		m := newMap(t, wideCfg)
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 100, MoveVariance: 0.25, Observations: []Observation{{ID: "L", X: 1, Y: 2}}},
			{Time: 200, DX: 1e308, MoveVariance: 0.5, Observations: []Observation{{ID: "L", X: 1, Y: 2}}},
			{Time: 300, DX: 1e308, MoveVariance: 0.75},
		}}); err != nil {
			t.Fatal(err)
		}
		rec, err := m.Correct(Correction{
			ID:     "c",
			Anchor: 100,
			Target: CorrectionTarget{X: -1e308, Y: 0, Heading: 0, Variance: 4},
		})
		if err != nil {
			t.Fatalf("Correct: %v", err)
		}
		// 两次观测的等权平均：(-1e308+1 与 1) 按增量平均得 -5e307，Y 为 2。
		lms, err := m.LandmarksInRect(Rect{MinX: -1e308, MinY: -10, MaxX: 1e308, MaxY: 10})
		if err != nil {
			t.Fatal(err)
		}
		if len(lms) != 1 || lms[0].ID != "L" || lms[0].X != -5e307 || lms[0].Y != 2 || lms[0].Count != 2 {
			t.Fatalf("landmarks = %+v", lms)
		}
		if len(rec.Landmarks) != 1 || rec.Landmarks[0].ID != "L" || rec.Landmarks[0].Occurrence != 1 {
			t.Fatalf("record landmarks = %+v", rec.Landmarks)
		}
		lc := rec.Landmarks[0]
		if lc.Before != lc.After || lc.Before.X != -5e307 || lc.Before.Y != 2 || lc.Before.Count != 2 {
			t.Fatalf("landmark change = %+v, want identical before/after at (-5e307, 2) count 2", lc)
		}
		h, err := m.LandmarkAppearances("L")
		if err != nil {
			t.Fatal(err)
		}
		if len(h.Appearances) != 1 || !h.Appearances[0].Active || h.Appearances[0].Landmark.Count != 2 {
			t.Fatalf("appearances = %+v", h)
		}
	})

	// 目标方差有限，但与锚点之后的运动方差累计后溢出为无穷时，仍以
	// non_finite 拒绝并标明锚点时间；位姿与校正记录保持原样。
	t.Run("variance accumulation overflow still rejected", func(t *testing.T) {
		m := newMap(t, cfg)
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 100, MoveVariance: 0.25},
			{Time: 200, MoveVariance: 1e308},
			{Time: 300},
		}}); err != nil {
			t.Fatal(err)
		}
		// 目标方差本身有限，但加上 t=200 的运动方差后溢出为无穷。
		_, err := m.Correct(Correction{
			ID:     "c",
			Anchor: 100,
			Target: CorrectionTarget{X: -1e308, Y: 0, Heading: 0, Variance: math.MaxFloat64},
		})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite {
			t.Fatalf("err = %v, want non_finite", err)
		}
		if !r.HasTime || r.Time != 100 {
			t.Fatalf("time = %d(ok=%v), want anchor time 100", r.Time, r.HasTime)
		}
		p, _ := m.PoseAt(200)
		if p.Variance != 1e308 {
			t.Fatalf("pose changed after rejection: %+v", p)
		}
		if recs, _ := m.Corrections(); len(recs) != 0 {
			t.Fatalf("correction record leaked: %v", recs)
		}
	})

	// 目标方差为负或非有限数时仍按原有原因拒绝。
	t.Run("invalid target variance still rejected", func(t *testing.T) {
		m := newMap(t, cfg)
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 100, MoveVariance: 0.25},
		}}); err != nil {
			t.Fatal(err)
		}
		for _, tc := range []struct {
			name string
			v    float64
			kind string
		}{
			{"negative", -1, RejectNegativeVariance},
			{"nan", math.NaN(), RejectNonFinite},
			{"inf", math.Inf(1), RejectNonFinite},
		} {
			_, err := m.Correct(Correction{
				ID:     "c-" + tc.name,
				Anchor: 100,
				Target: CorrectionTarget{X: -1e308, Y: 0, Heading: 0, Variance: tc.v},
			})
			r, ok := AsRejectError(err)
			if !ok || r.Kind != tc.kind {
				t.Fatalf("%s: err = %v, want %s", tc.name, err, tc.kind)
			}
		}
		p, _ := m.PoseAt(100)
		if p.Variance != 1.25 {
			t.Fatalf("pose changed after rejections: %+v", p)
		}
	})
}
