package posemap

import (
	"math"
	"path/filepath"
	"testing"
)

// 范围内可表示的微小朝向是真实运动：1e-16 弧度的增量必须逐帧累积、保留
// 符号，零增量不能把已有小朝向清零。这里全部按浮点精确值比较——这些值
// 本就是输入与普通浮点加法可表示的结果。
func TestTinyHeadingAccumulates(t *testing.T) {
	for _, sign := range []float64{1, -1} {
		name := "positive"
		if sign < 0 {
			name = "negative"
		}
		t.Run(name, func(t *testing.T) {
			m := newMap(t, baseConfig())
			e := sign * 1e-16
			frames := []Frame{
				{Time: 10, DHeading: e},
				{Time: 20, DHeading: e},
				{Time: 30, DHeading: e},
			}
			res, err := m.ImportSegment(Segment{ID: "s", Frames: frames})
			if err != nil {
				t.Fatalf("ImportSegment: %v", err)
			}
			want := sign * 3e-16
			if res.EndPose.Heading != want {
				t.Fatalf("segment end heading = %v, want %v", res.EndPose.Heading, want)
			}
			cur, _ := m.CurrentPose()
			if cur.Heading != want {
				t.Fatalf("current heading = %v, want %v", cur.Heading, want)
			}
			// 本段末位姿、当前位姿与逐帧历史位姿看到一致的朝向。
			for i, wantH := range []float64{e, 2 * e, 3 * e} {
				p, err := m.PoseAt(int64(10 + 10*i))
				if err != nil {
					t.Fatalf("PoseAt: %v", err)
				}
				if p.Heading != wantH {
					t.Fatalf("PoseAt(%d) heading = %v, want %v", 10+10*i, p.Heading, wantH)
				}
			}
		})
	}
}

// 已经带小朝向的位姿再导入零增量帧，朝向必须原样保持，不能因为经过一次
// 更新就被清零。
func TestTinyHeadingSurvivesZeroIncrement(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(Segment{ID: "turn", Frames: []Frame{
		{Time: 10, DHeading: 1e-16},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "hold", Frames: []Frame{
		{Time: 20, DHeading: 0},
	}}); err != nil {
		t.Fatal(err)
	}
	cur, _ := m.CurrentPose()
	if cur.Heading != 1e-16 {
		t.Fatalf("heading after zero increment = %v, want 1e-16", cur.Heading)
	}
	p, _ := m.PoseAt(10)
	if p.Heading != 1e-16 {
		t.Fatalf("historical heading = %v, want 1e-16", p.Heading)
	}
	// 零朝向保持零（不产生 -0 之类的伪运动）。
	m2 := newMap(t, baseConfig())
	if _, err := m2.ImportSegment(Segment{ID: "z", Frames: []Frame{
		{Time: 10}, {Time: 20},
	}}); err != nil {
		t.Fatal(err)
	}
	z, _ := m2.CurrentPose()
	if z.Heading != 0 || math.Signbit(z.Heading) {
		t.Fatalf("zero heading = %v (signbit=%v), want +0", z.Heading, math.Signbit(z.Heading))
	}
}

// 小朝向必须实际参与后续定位：
//   - 自身坐标下的平移按运动前朝向转换：小转角后的下一次前进带有与该方向
//     一致的横向分量 sin(1e-16)*1 = 1e-16；
//   - 路标观测按本帧运动完成后的朝向转换：转向同帧的“正前方”观测在地图
//     坐标下同样带 1e-16 的横向分量。
func TestTinyHeadingDrivesKinematicsAndObservations(t *testing.T) {
	for _, sign := range []float64{1, -1} {
		name := "positive"
		if sign < 0 {
			name = "negative"
		}
		t.Run(name, func(t *testing.T) {
			m := newMap(t, baseConfig())
			e := sign * 1e-16
			res, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
				// 原地小转 e；同帧观测自身坐标 (1,0)：按运动后朝向转换，
				// 地图位置 (1, sin e) = (1, e)。
				{Time: 10, DX: 0, DY: 0, DHeading: e,
					Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
				// 沿自身 X 前进 1：平移按运动前朝向 e 转换，落点 (1, e)。
				{Time: 20, DX: 1, DY: 0, DHeading: 0},
			}})
			if err != nil {
				t.Fatalf("ImportSegment: %v", err)
			}
			if res.EndPose.X != 2 || res.EndPose.Y != 2+e || res.EndPose.Heading != e {
				t.Fatalf("end pose = %+v, want (2,%v,h=%v)", res.EndPose, 2+e, e)
			}
			lms, err := m.LandmarksInRect(Rect{MinX: 0, MinY: -1, MaxX: 3, MaxY: 4})
			if err != nil {
				t.Fatal(err)
			}
			if len(lms) != 1 || lms[0].X != 2 || lms[0].Y != 2+e {
				t.Fatalf("landmark = %+v, want (2,%v)", lms, 2+e)
			}
		})
	}
}

// 回环校正沿用同一角度含义：锚点原朝向为零、目标位置不变而目标朝向为
// 1e-16 时，这不是“几何完全不变”的恒等校正——锚点与后续轨迹必须包含
// 这次转向，受影响路标观测随之移动，校正记录的前后值如实反映。保存后重
// 开，小朝向不被再次清零，校正记录仍通过打开时的几何核对。
func TestTinyHeadingCorrection(t *testing.T) {
	for _, sign := range []float64{1, -1} {
		name := "positive"
		if sign < 0 {
			name = "negative"
		}
		t.Run(name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "map.pose")
			cfg := Config{
				InitialTime: 0, InitialX: 0, InitialY: 0, InitialHeading: 0,
				InitialVariance: 0.5, MaxInterval: 1000, MergeDistance: 1.0,
			}
			m, err := Create(path, cfg)
			if err != nil {
				t.Fatalf("Create: %v", err)
			}
			e := sign * 1e-16
			if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
				// t=10：前进到 (1,0)，观测自身 (1,0) → 地图 (2,0)。
				{Time: 10, DX: 1, Observations: []Observation{{ID: "L", X: 1}}},
				// t=20：再前进到 (2,0)。
				{Time: 20, DX: 1},
			}}); err != nil {
				t.Fatal(err)
			}
			rec, err := m.Correct(Correction{ID: "c", Anchor: 10,
				Target: CorrectionTarget{X: 1, Y: 0, Heading: e, Variance: 0.5}})
			if err != nil {
				t.Fatalf("Correct: %v", err)
			}
			if len(rec.Poses) != 2 {
				t.Fatalf("record covers %d poses, want 2", len(rec.Poses))
			}
			a := rec.Poses[0]
			if a.Before.X != 1 || a.Before.Y != 0 || a.Before.Heading != 0 {
				t.Fatalf("anchor before = %+v, want (1,0,h=0)", a.Before)
			}
			if a.After.X != 1 || a.After.Y != 0 || a.After.Heading != e {
				t.Fatalf("anchor after = %+v, want (1,0,h=%v)", a.After, e)
			}
			tail := rec.Poses[1]
			if tail.Before.X != 2 || tail.Before.Y != 0 || tail.Before.Heading != 0 {
				t.Fatalf("tail before = %+v, want (2,0,h=0)", tail.Before)
			}
			if tail.After.X != 2 || tail.After.Y != e || tail.After.Heading != e {
				t.Fatalf("tail after = %+v, want (2,%v,h=%v)", tail.After, e, e)
			}
			if len(rec.Landmarks) != 1 {
				t.Fatalf("landmark changes = %+v, want one entry", rec.Landmarks)
			}
			lc := rec.Landmarks[0]
			if lc.Before.X != 2 || lc.Before.Y != 0 || lc.After.X != 2 || lc.After.Y != e {
				t.Fatalf("landmark change = %+v, want before (2,0) after (2,%v)", lc, e)
			}
			cur, _ := m.CurrentPose()
			if cur.X != 2 || cur.Y != e || cur.Heading != e {
				t.Fatalf("current after correct = %+v, want (2,%v,h=%v)", cur, e, e)
			}
			if err := m.Close(); err != nil {
				t.Fatal(err)
			}

			m2, err := Open(path)
			if err != nil {
				t.Fatalf("Open: %v", err)
			}
			defer m2.Close()
			cur, _ = m2.CurrentPose()
			if cur.X != 2 || cur.Y != e || cur.Heading != e {
				t.Fatalf("current after reopen = %+v, want (2,%v,h=%v)", cur, e, e)
			}
			p10, _ := m2.PoseAt(10)
			if p10.Heading != e {
				t.Fatalf("anchor heading after reopen = %v, want %v", p10.Heading, e)
			}
			lms, _ := m2.LandmarksInRect(Rect{MinX: 0, MinY: -1, MaxX: 3, MaxY: 1})
			if len(lms) != 1 || lms[0].X != 2 || lms[0].Y != e {
				t.Fatalf("landmark after reopen = %+v, want (2,%v)", lms, e)
			}
			recs, _ := m2.Corrections()
			if len(recs) != 1 || recs[0].Poses[0].After.Heading != e {
				t.Fatalf("correction record after reopen lost tiny heading: %+v", recs)
			}
		})
	}
}

// 纯导入的小朝向保存后重开同样保持，不在打开文件时再次被清零。
func TestTinyHeadingPersists(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, DHeading: 1e-16},
		{Time: 20, DHeading: 0},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	defer m2.Close()
	cur, _ := m2.CurrentPose()
	if cur.Heading != 1e-16 {
		t.Fatalf("heading after reopen = %v, want 1e-16", cur.Heading)
	}
}

// 跨 ±π 边界的运动仍表示同一个物理方向，且正 π 仍表示为负 π：从 -π
// 再向负方向转一个该量级下可表示的小角，朝向绕回 +π 内侧的同方向表示，
// 下一次前进的横向分量与该方向一致（sin 为正）。这里的增量取 5e-16：
// 它在 π 量级下仍能被浮点加法表示（1e-16 在 π 处不足半个 ULP，正常浮点
// 加法无法表示的精度本就不要求恢复）。
func TestTinyHeadingAcrossPiBoundary(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, DHeading: math.Pi},         // 归一为 -π
		{Time: 20, DHeading: -5e-16},          // -π-5e-16 绕回 +π 内侧同方向表示
		{Time: 30, DX: 1, DY: 0, DHeading: 0}, // 平移按运动前朝向转换
	}}); err != nil {
		t.Fatal(err)
	}
	p10, _ := m.PoseAt(10)
	if p10.Heading != -math.Pi {
		t.Fatalf("heading after +pi = %v, want -pi", p10.Heading)
	}
	wantH := math.Nextafter(math.Pi, 0)
	cur, _ := m.CurrentPose()
	if cur.Heading != wantH {
		t.Fatalf("heading after crossing boundary = %v, want %v", cur.Heading, wantH)
	}
	// 从初始位置 (1,2) 沿该方向前进 1：X=1+cos≈0，横向分量 sin 为正。
	wantY := math.Sin(wantH)
	if cur.X != 0 || cur.Y != 2+wantY || wantY <= 0 {
		t.Fatalf("position = (%v,%v), want (0,%v) with positive lateral component", cur.X, cur.Y, 2+wantY)
	}
}
