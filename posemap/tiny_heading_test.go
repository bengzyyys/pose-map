package posemap

import (
	"math"
	"path/filepath"
	"testing"
)

// 微小转角（1e-16 弧度）是有限且可表示的真实运动，更新朝向时不能被当成
// 舍入误差丢掉：要逐帧累积、保留正负号、零增量不把已有小朝向清零，并且
// 实际参与后续定位（自身坐标平移按运动前朝向转换、路标观测按运动完成后
// 朝向转换），持久化重开后仍保持。回环校正沿用同一角度含义，角度始终
// 归一到 [-π,π)，正 π 表示为负 π。
const tinyHeading = 1e-16

// normalizeAngle 单元层面的不变量：范围内可表示的小角度原样保留，零保持
// 零，+π 折回 -π，略小于 -π 的可表示输入也折回 [-π,π) 而不会输出 +π。
func TestNormalizeAnglePreservesTiny(t *testing.T) {
	cases := []struct {
		name string
		in   float64
		want float64
	}{
		{"tiny positive", tinyHeading, tinyHeading},
		{"tiny negative", -tinyHeading, -tinyHeading},
		{"zero", 0, 0},
		{"plus pi", math.Pi, -math.Pi},
		{"minus pi", -math.Pi, -math.Pi},
		{"just below minus pi folds past plus pi", -math.Pi - math.Pi*math.Ldexp(1, -52), normalizeAngle(math.Pi - math.Pi*math.Ldexp(1, -52))},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := normalizeAngle(tc.in)
			if got != tc.want {
				t.Fatalf("normalizeAngle(%v) = %v, want %v", tc.in, got, tc.want)
			}
			if got >= math.Pi || got < -math.Pi {
				t.Fatalf("normalizeAngle(%v) = %v outside [-pi,pi)", tc.in, got)
			}
		})
	}
}

// 连续三帧各增加一个微小正转角：第三帧朝向按正常浮点加法累积到约 3e-16；
// 负增量累积到对应负方向。
func TestTinyHeadingAccumulates(t *testing.T) {
	cases := []struct {
		name string
		step float64
		want float64
	}{
		{"positive", tinyHeading, 3e-16},
		{"negative", -tinyHeading, -3e-16},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
			frames := make([]Frame, 3)
			for i := range frames {
				frames[i] = Frame{Time: int64(10 * (i + 1)), DHeading: tc.step}
			}
			res, err := m.ImportSegment(Segment{ID: "s", Frames: frames})
			if err != nil {
				t.Fatalf("ImportSegment: %v", err)
			}
			if res.EndPose.Heading != tc.want {
				t.Fatalf("segment end heading = %v, want %v", res.EndPose.Heading, tc.want)
			}
			// 每一帧的历史位姿都应保留累积到当时的朝向。
			for i := range frames {
				got, err := m.PoseAt(frames[i].Time)
				if err != nil {
					t.Fatalf("PoseAt(%d): %v", frames[i].Time, err)
				}
				want := float64(i+1) * tc.step
				if got.Heading != want {
					t.Fatalf("PoseAt(%d) heading = %v, want %v", frames[i].Time, got.Heading, want)
				}
			}
			cur, _ := m.CurrentPose()
			if cur.Heading != tc.want {
				t.Fatalf("current heading = %v, want %v", cur.Heading, tc.want)
			}
		})
	}
}

// 已有一个小朝向时，下一帧朝向增量为零必须保持原朝向，不能经过一次更新
// 就被清零。
func TestTinyHeadingSurvivesZeroIncrement(t *testing.T) {
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
	res, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, DHeading: tinyHeading},
		{Time: 20, DHeading: 0},
	}})
	if err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	if res.EndPose.Heading != tinyHeading {
		t.Fatalf("heading after zero increment = %v, want %v", res.EndPose.Heading, tinyHeading)
	}
}

// 小朝向必须实际参与定位：转向后的下一次前进按运动前朝向旋转，留下与该
// 方向一致的横向分量；转向帧中的路标观测按运动完成后的朝向转换，位置也
// 带同一横向分量。
func TestTinyHeadingDrivesKinematicsAndObservations(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	// 第 1 帧：原地转向 1e-16，运动完成后在自身坐标正前方 (1,0) 观测路标 L。
	// 第 2 帧：朝向增量 0，自身坐标前进 1（按运动前朝向 1e-16 转换）。
	// 第 3 帧：朝向增量 0，朝向继续保持。
	res, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, DHeading: tinyHeading, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
		{Time: 20, DX: 1, DHeading: 0},
		{Time: 30, DHeading: 0},
	}})
	if err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	wantX := math.Cos(tinyHeading)
	wantY := math.Sin(tinyHeading)
	if wantY == 0 {
		t.Fatalf("test setup: sin(%v) rounded to zero", tinyHeading)
	}
	// 前进与路标观测都必须带上与小朝向一致的横向分量。
	if res.EndPose.X != wantX || res.EndPose.Y != wantY {
		t.Fatalf("end pose = (%v,%v), want (%v,%v)", res.EndPose.X, res.EndPose.Y, wantX, wantY)
	}
	if res.EndPose.Heading != tinyHeading {
		t.Fatalf("end heading = %v, want %v", res.EndPose.Heading, tinyHeading)
	}
	for _, tm := range []int64{10, 20, 30} {
		p, err := m.PoseAt(tm)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", tm, err)
		}
		if p.Heading != tinyHeading {
			t.Fatalf("PoseAt(%d) heading = %v, want %v", tm, p.Heading, tinyHeading)
		}
	}
	cur, _ := m.CurrentPose()
	if cur.Heading != tinyHeading || cur.X != wantX || cur.Y != wantY {
		t.Fatalf("current pose = %+v, want (%v,%v,%v)", cur, wantX, wantY, tinyHeading)
	}
	lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].ID != "L" || lms[0].Count != 1 {
		t.Fatalf("landmarks = %+v", lms)
	}
	if lms[0].X != wantX || lms[0].Y != wantY {
		t.Fatalf("landmark = (%v,%v), want (%v,%v): lateral component lost", lms[0].X, lms[0].Y, wantX, wantY)
	}

	// 落盘后重开：小朝向与其定位结果保持一致，不在打开时再次被清零。
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	defer m2.Close()
	cur, _ = m2.CurrentPose()
	if cur.Heading != tinyHeading || cur.X != wantX || cur.Y != wantY {
		t.Fatalf("after reopen current pose = %+v, want (%v,%v,%v)", cur, wantX, wantY, tinyHeading)
	}
	for _, tm := range []int64{10, 20, 30} {
		p, err := m2.PoseAt(tm)
		if err != nil {
			t.Fatalf("reopen PoseAt(%d): %v", tm, err)
		}
		if p.Heading != tinyHeading {
			t.Fatalf("reopen PoseAt(%d) heading = %v, want %v", tm, p.Heading, tinyHeading)
		}
	}
	lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].X != wantX || lms[0].Y != wantY {
		t.Fatalf("after reopen landmark = %+v, want (%v,%v)", lms, wantX, wantY)
	}
}

// 回环校正：锚点原朝向为零、目标位置不变而目标朝向为 1e-16。锚点与后续
// 轨迹必须包含这次转向，受影响路标观测随之更新；不能把它误判为几何与
// 朝向完全不变的恒等校正。校正记录中的前后值如实反映结果，重开后一致。
func TestTinyHeadingLoopCorrection(t *testing.T) {
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
	// 锚点帧 t=10 前进到 (1,0)；末帧 t=20 再前进到 (2,0)，在自身正前方
	// (1,0) 观测 L，地图坐标 (3,0)。
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, DX: 1},
		{Time: 20, DX: 1, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
	}}); err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	rec, err := m.Correct(Correction{
		ID:     "c",
		Anchor: 10,
		Target: CorrectionTarget{X: 1, Y: 0, Heading: tinyHeading, Variance: 0},
	})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if len(rec.Poses) != 2 {
		t.Fatalf("pose changes = %d, want 2", len(rec.Poses))
	}
	// 锚点：位置不变，朝向从零变为 1e-16——不是恒等校正。
	if rec.Poses[0].Before.Heading != 0 || rec.Poses[0].After.Heading != tinyHeading {
		t.Fatalf("anchor heading %v -> %v, want 0 -> %v", rec.Poses[0].Before.Heading, rec.Poses[0].After.Heading, tinyHeading)
	}
	if rec.Poses[0].After.X != 1 || rec.Poses[0].After.Y != 0 {
		t.Fatalf("anchor after = (%v,%v), want (1,0)", rec.Poses[0].After.X, rec.Poses[0].After.Y)
	}
	// 后续帧随锚点一起旋转：(2,0) -> (2, sin(1e-16))，保留横向分量。
	wantY := math.Sin(tinyHeading)
	if rec.Poses[1].Before.X != 2 || rec.Poses[1].Before.Y != 0 || rec.Poses[1].Before.Heading != 0 {
		t.Fatalf("tail before = %+v, want (2,0,0)", rec.Poses[1].Before)
	}
	if rec.Poses[1].After.X != 2 || rec.Poses[1].After.Y != wantY || rec.Poses[1].After.Heading != tinyHeading {
		t.Fatalf("tail after = %+v, want (2,%v,%v)", rec.Poses[1].After, wantY, tinyHeading)
	}
	// 受影响路标观测按校正后位姿重放：(3,0) -> (3, 2*sin(1e-16))。
	if len(rec.Landmarks) != 1 || rec.Landmarks[0].ID != "L" {
		t.Fatalf("landmark changes = %+v", rec.Landmarks)
	}
	lc := rec.Landmarks[0]
	if lc.Before.X != 3 || lc.Before.Y != 0 || lc.After.X != 3 || lc.After.Y != 2*wantY {
		t.Fatalf("landmark change = (%v,%v) -> (%v,%v), want (3,0) -> (3,%v)",
			lc.Before.X, lc.Before.Y, lc.After.X, lc.After.Y, 2*wantY)
	}
	// 当前位姿实际采用校正后朝向。
	cur, _ := m.CurrentPose()
	if cur.Heading != tinyHeading || cur.X != 2 || cur.Y != wantY {
		t.Fatalf("current pose = %+v, want (2,%v,%v)", cur, wantY, tinyHeading)
	}

	// 重开后当前位姿、路标与校正记录保持一致，打开核对不把微小转向当作
	// 几何矛盾，也不重新清零。
	path := m.Path()
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	defer m2.Close()
	cur, _ = m2.CurrentPose()
	if cur.Heading != tinyHeading || cur.X != 2 || cur.Y != wantY {
		t.Fatalf("after reopen current pose = %+v, want (2,%v,%v)", cur, wantY, tinyHeading)
	}
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].X != 3 || lms[0].Y != 2*wantY {
		t.Fatalf("after reopen landmark = %+v, want (3,%v)", lms, 2*wantY)
	}
	recs, err := m2.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 || len(recs[0].Poses) != 2 ||
		recs[0].Poses[0].After.Heading != tinyHeading ||
		recs[0].Poses[1].After.Y != wantY {
		t.Fatalf("after reopen correction record = %+v", recs)
	}
}

// 回环校正：锚点朝向为 1 弧度、目标朝向为 ±1e-20 弧度——目标远小于锚点
// 朝向，但它是有限且可表示的非零角度，必须作为实际定位结果保留，不能在
// 计算旋转角时被锚点朝向的精度吞掉而退化为零。锚点及原本与锚点朝向相同
// 的帧都得到该目标朝向；有其他朝向差的帧保留相对方向；受影响路标观测的
// 地图纵坐标保留约 1e-20 的非零分量（负目标时为负）；校正后的末帧朝向继
// 续驱动随后导入的运动；重开后结果一致。
func TestTinyTargetHeadingSurvivesLargeAnchorHeading(t *testing.T) {
	for _, target := range []float64{1e-20, -1e-20} {
		name := "positive"
		if target < 0 {
			name = "negative"
		}
		t.Run(name, func(t *testing.T) {
			m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
			// t=10 锚点：原地转向 1 弧度（位置保持原点）。t=20 再转 0.5，
			// t=30 转回 -0.5（与锚点朝向同为 1）并在自身正前方 (1,0) 观测 L。
			if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
				{Time: 10, DHeading: 1},
				{Time: 20, DHeading: 0.5},
				{Time: 30, DHeading: -0.5, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
			}}); err != nil {
				t.Fatalf("ImportSegment: %v", err)
			}
			rec, err := m.Correct(Correction{
				ID:     "c",
				Anchor: 10,
				Target: CorrectionTarget{X: 0, Y: 0, Heading: target, Variance: 0},
			})
			if err != nil {
				t.Fatalf("Correct: %v", err)
			}
			if len(rec.Poses) != 3 {
				t.Fatalf("pose changes = %d, want 3", len(rec.Poses))
			}
			// 锚点：朝向从 1 变为目标小角度本身，不是零。
			if rec.Poses[0].Before.Heading != 1 || rec.Poses[0].After.Heading != target {
				t.Fatalf("anchor heading %v -> %v, want 1 -> %v", rec.Poses[0].Before.Heading, rec.Poses[0].After.Heading, target)
			}
			// 与锚点朝向差 +0.5 的帧保留相对方向。
			want20 := normalizeAngle(target + 0.5)
			if rec.Poses[1].After.Heading != want20 {
				t.Fatalf("frame 20 heading = %v, want %v", rec.Poses[1].After.Heading, want20)
			}
			// 原本与锚点朝向完全相同的末帧得到同一目标朝向。
			if rec.Poses[2].Before.Heading != 1 || rec.Poses[2].After.Heading != target {
				t.Fatalf("frame 30 heading %v -> %v, want 1 -> %v", rec.Poses[2].Before.Heading, rec.Poses[2].After.Heading, target)
			}
			// 历史查询与当前位姿都反映实际校正结果。
			for _, tm := range []int64{10, 30} {
				p, err := m.PoseAt(tm)
				if err != nil {
					t.Fatalf("PoseAt(%d): %v", tm, err)
				}
				if p.Heading != target {
					t.Fatalf("PoseAt(%d) heading = %v, want %v", tm, p.Heading, target)
				}
			}
			if p, _ := m.PoseAt(20); p.Heading != want20 {
				t.Fatalf("PoseAt(20) heading = %v, want %v", p.Heading, want20)
			}
			if cur, _ := m.CurrentPose(); cur.Heading != target {
				t.Fatalf("current heading = %v, want %v", cur.Heading, target)
			}
			// 路标观测按保留的小朝向转换：正前方 1 米的路标地图纵坐标保留
			// 与目标同号的非零分量（sin(±1e-20) 在 float64 中即 ±1e-20）。
			if len(rec.Landmarks) != 1 || rec.Landmarks[0].ID != "L" {
				t.Fatalf("landmark changes = %+v", rec.Landmarks)
			}
			if la := rec.Landmarks[0].After; la.X != 1 || la.Y != target {
				t.Fatalf("landmark after = (%v,%v), want (1,%v)", la.X, la.Y, target)
			}
			lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if err != nil {
				t.Fatal(err)
			}
			if len(lms) != 1 || lms[0].X != 1 || lms[0].Y != target {
				t.Fatalf("landmarks = %+v, want (1,%v)", lms, target)
			}
			// 校正后的末帧带有小朝向：随后导入的前进运动按它定位，留下同
			// 号的横向分量，而不是重新按零朝向定位。
			res, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
				{Time: 40, DX: 1, DHeading: 0},
			}})
			if err != nil {
				t.Fatalf("ImportSegment after correction: %v", err)
			}
			if res.EndPose.Heading != target || res.EndPose.X != 1 || res.EndPose.Y != target {
				t.Fatalf("end pose = (%v,%v,%v), want (1,%v,%v)",
					res.EndPose.X, res.EndPose.Y, res.EndPose.Heading, target, target)
			}

			// 重开后当前位姿、历史查询、路标与校正记录保持一致。
			path := m.Path()
			if err := m.Close(); err != nil {
				t.Fatal(err)
			}
			m2, err := Open(path)
			if err != nil {
				t.Fatalf("Open: %v", err)
			}
			defer m2.Close()
			if cur, _ := m2.CurrentPose(); cur.Heading != target || cur.X != 1 || cur.Y != target {
				t.Fatalf("after reopen current pose = %+v, want (1,%v,%v)", cur, target, target)
			}
			if p, _ := m2.PoseAt(10); p.Heading != target {
				t.Fatalf("after reopen PoseAt(10) heading = %v, want %v", p.Heading, target)
			}
			lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if len(lms) != 1 || lms[0].X != 1 || lms[0].Y != target {
				t.Fatalf("after reopen landmark = %+v, want (1,%v)", lms, target)
			}
			recs, err := m2.Corrections()
			if err != nil {
				t.Fatal(err)
			}
			if len(recs) != 1 || len(recs[0].Poses) != 3 ||
				recs[0].Poses[0].After.Heading != target ||
				recs[0].Poses[2].After.Heading != target {
				t.Fatalf("after reopen correction record = %+v", recs)
			}
		})
	}
}

// 跨越 ±π 边界的校正仍表示同一个物理方向：锚点朝向在 +π 附近，目标再向
// 正方向越过 +π，结果归一为 -π 一侧的等价角度，锚点位置不变。
func TestTinyHeadingCorrectionAcrossPiBoundary(t *testing.T) {
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
	anchorHeading := normalizeAngle(math.Pi - 1e-15) // +π 内侧一个可表示的小角
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, DHeading: anchorHeading},
	}}); err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	// 目标朝向沿正方向再转 3e-15，物理上越过 +π；提交值本身在范围外，
	// 由校正归一。
	targetHeading := anchorHeading + 3e-15
	rec, err := m.Correct(Correction{
		ID:     "c",
		Anchor: 10,
		Target: CorrectionTarget{X: 0, Y: 0, Heading: targetHeading, Variance: 0},
	})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	dHeading := normalizeAngle(normalizeAngle(targetHeading) - anchorHeading)
	want := normalizeAngle(anchorHeading + dHeading)
	if want >= 0 {
		t.Fatalf("test setup: expected wrap to the -pi side, got %v", want)
	}
	if rec.Poses[0].After.Heading != want {
		t.Fatalf("corrected heading = %v, want boundary-folded %v", rec.Poses[0].After.Heading, want)
	}
	if rec.Poses[0].After.X != 0 || rec.Poses[0].After.Y != 0 {
		t.Fatalf("anchor position changed: (%v,%v)", rec.Poses[0].After.X, rec.Poses[0].After.Y)
	}
	// 正 π 始终表示为负 π。
	if h := rec.Poses[0].After.Heading; h >= math.Pi || h < -math.Pi {
		t.Fatalf("heading %v outside [-pi, pi)", h)
	}
}
