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

// 锚点朝向为 1 弧度、目标朝向为 ±1e-20 弧度的校正：目标与锚点朝向的差
// 不可表示（舍入成 -1），但目标本身是有限且可表示的真实定位结果，必须作
// 为锚点校正后朝向保留下来，不能因比原朝向小很多就变成零。原本与锚点朝
// 向完全相同的帧得到同一目标朝向，其他朝向差的帧保留相对方向；受影响路
// 标观测带上同一非零横向分量；末帧的小朝向继续参与随后导入；重开后一致。
func TestTinyTargetHeadingSurvivesCorrection(t *testing.T) {
	for _, tc := range []struct {
		name   string
		target float64
	}{
		{"positive", 1e-20},
		{"negative", -1e-20},
	} {
		t.Run(tc.name, func(t *testing.T) {
			dir := t.TempDir()
			path := filepath.Join(dir, "map.pose")
			m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
			if err != nil {
				t.Fatalf("Create: %v", err)
			}
			// 全部帧位于原点：t=10 锚点转向 1 弧度并在自身正前方 (1,0) 观测 L，
			// t=20 转向相反的 0.5（与锚点朝向不同），t=30 转回与锚点完全相同的朝向。
			if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
				{Time: 10, DHeading: 1, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
				{Time: 20, DHeading: 0.5},
				{Time: 30, DHeading: -0.5},
			}}); err != nil {
				t.Fatalf("ImportSegment: %v", err)
			}
			rec, err := m.Correct(Correction{
				ID:     "c",
				Anchor: 10,
				Target: CorrectionTarget{X: 0, Y: 0, Heading: tc.target, Variance: 0},
			})
			if err != nil {
				t.Fatalf("Correct: %v", err)
			}
			if len(rec.Poses) != 3 {
				t.Fatalf("pose changes = %d, want 3", len(rec.Poses))
			}
			// 锚点：精确采用归一后的目标朝向（已在 [-π,π) 内，保留数值与正负）。
			if rec.Poses[0].After.Heading != tc.target {
				t.Fatalf("anchor heading = %v, want %v", rec.Poses[0].After.Heading, tc.target)
			}
			if rec.Poses[0].After.X != 0 || rec.Poses[0].After.Y != 0 || rec.Poses[0].After.Variance != 0 {
				t.Fatalf("anchor after = %+v, want (0,0) variance 0", rec.Poses[0].After)
			}
			// 与锚点朝向不同的帧保留相对方向：1.5 -> 0.5（1e-20 低于 0.5 的
			// 可表示精度，叠加后恰为 0.5）。
			if rec.Poses[1].After.Heading != 0.5 {
				t.Fatalf("t=20 heading = %v, want 0.5 (relative turn kept)", rec.Poses[1].After.Heading)
			}
			// 原本与锚点朝向完全相同的帧得到同一目标朝向，不是零也不统一成 0.5。
			if rec.Poses[2].After.Heading != tc.target {
				t.Fatalf("t=30 heading = %v, want %v", rec.Poses[2].After.Heading, tc.target)
			}
			// 历史查询与当前位姿立即反映实际结果。
			for _, tm := range []int64{10, 30} {
				p, err := m.PoseAt(tm)
				if err != nil {
					t.Fatalf("PoseAt(%d): %v", tm, err)
				}
				if p.Heading != tc.target {
					t.Fatalf("PoseAt(%d) heading = %v, want %v", tm, p.Heading, tc.target)
				}
			}
			cur, _ := m.CurrentPose()
			if cur.Heading != tc.target {
				t.Fatalf("current heading = %v, want %v", cur.Heading, tc.target)
			}
			// 受影响路标观测按保留的朝向转换：正前方 1 米的路标地图纵坐标
			// 保留约 1e-20 的非零分量，符号与目标一致。
			lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if err != nil {
				t.Fatal(err)
			}
			if len(lms) != 1 || lms[0].ID != "L" || lms[0].X != 1 || lms[0].Y != tc.target {
				t.Fatalf("landmark = %+v, want (1,%v): tiny lateral component lost", lms, tc.target)
			}
			if len(rec.Landmarks) != 1 || rec.Landmarks[0].After.Y != tc.target {
				t.Fatalf("landmark change = %+v, want after y %v", rec.Landmarks, tc.target)
			}
			// 末帧带着小朝向：随后导入的运动按它定位，不按零朝向。
			res, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{{Time: 40, DX: 1}}})
			if err != nil {
				t.Fatalf("import after correction: %v", err)
			}
			if res.EndPose.X != 1 || res.EndPose.Y != tc.target || res.EndPose.Heading != tc.target {
				t.Fatalf("continued end pose = %+v, want (1,%v,%v)", res.EndPose, tc.target, tc.target)
			}

			// 重开后小朝向与其定位结果、校正记录保持一致。
			if err := m.Close(); err != nil {
				t.Fatal(err)
			}
			m2, err := Open(path)
			if err != nil {
				t.Fatalf("Open: %v", err)
			}
			defer m2.Close()
			cur, _ = m2.CurrentPose()
			if cur.Heading != tc.target || cur.X != 1 || cur.Y != tc.target {
				t.Fatalf("after reopen current pose = %+v, want (1,%v,%v)", cur, tc.target, tc.target)
			}
			p, _ := m2.PoseAt(10)
			if p.Heading != tc.target {
				t.Fatalf("after reopen PoseAt(10) heading = %v, want %v", p.Heading, tc.target)
			}
			lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if len(lms) != 1 || lms[0].X != 1 || lms[0].Y != tc.target {
				t.Fatalf("after reopen landmark = %+v, want (1,%v)", lms, tc.target)
			}
			recs, err := m2.Corrections()
			if err != nil {
				t.Fatal(err)
			}
			if len(recs) != 1 || len(recs[0].Poses) != 3 ||
				recs[0].Poses[0].After.Heading != tc.target ||
				recs[0].Poses[2].After.Heading != tc.target {
				t.Fatalf("after reopen correction record = %+v", recs)
			}
		})
	}
}
