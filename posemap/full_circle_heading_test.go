package posemap

import (
	"math"
	"path/filepath"
	"testing"
)

// 跨过整圈的转向：转角 6.283185307179587 比程序使用的一整圈 2π 多
// 8.881784197001252e-16 弧度，余量有限且可表示，归一后必须保留为有符号
// 余量（整圈上方为正、下方为负），不能当作误差清零，也不能拒绝。余量
// 实际参与定位：转向后沿自身正前方前进留下同方向横向位移，转向帧观测
// 的路标带同方向横向坐标；零转角帧不清零；回环校正目标朝向处在整圈
// 附近时锚点采用同一保留朝向。
const fullCircleTurn = 6.283185307179587 // 2π + 8.881784197001252e-16

// fullCircleResidual 是 fullCircleTurn 减去一整圈后的有符号余量。必须先
// 把 2π 落到 float64 再相减：直接写 fullCircleTurn - 2*math.Pi 会按无类型
// 常量做任意精度运算，得到的不是程序实际使用的浮点整圈余量。
func fullCircleResidual() float64 {
	twoPi := 2 * math.Pi
	return fullCircleTurn - twoPi
}

// normalizeAngle 单元层面：整圈上下的余量保留符号，恰好整圈（含负整圈、
// 多倍整圈）得到 +0，范围内小角度原样保留，+π 折回 -π。
func TestNormalizeAngleFullCircleResidual(t *testing.T) {
	residual := fullCircleResidual()
	if residual != 8.881784197001252e-16 {
		t.Fatalf("test setup: residual = %v, want 8.881784197001252e-16", residual)
	}
	justBelow := math.Nextafter(2*math.Pi, 0) // 略小于一整圈
	cases := []struct {
		name string
		in   float64
		want float64
	}{
		{"just above full circle", fullCircleTurn, residual},
		{"just below full circle", justBelow, -residual},
		{"just above negative full circle", -justBelow, residual},
		{"just below negative full circle", -fullCircleTurn, -residual},
		{"exact full circle", 2 * math.Pi, 0},
		{"exact negative full circle", -2 * math.Pi, 0},
		{"exact double full circle", 4 * math.Pi, 0},
		{"plus pi", math.Pi, -math.Pi},
		{"tiny in range", 1e-16, 1e-16},
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
			if got == 0 && math.Signbit(got) {
				t.Fatalf("normalizeAngle(%v) = -0, want +0", tc.in)
			}
		})
	}
}

// 从零朝向导入一帧原地转向，转角略超一整圈：朝向保留正余量并实际驱动
// 定位——转向帧在自身正前方观测的路标带正横向坐标，之后沿自身正前方
// 前进留下同方向横向位移；中间零转角帧不清除余量；段末位姿、当前位姿
// 与历史位姿一致，重开后保持。反方向（略小于一整圈、负向整圈）对称。
func TestFullCircleTurnResidualDrivesLocalization(t *testing.T) {
	residual := fullCircleResidual()
	cases := []struct {
		name  string
		turn  float64
		wantH float64
	}{
		{"just above full circle", fullCircleTurn, residual},
		{"just below full circle", math.Nextafter(2*math.Pi, 0), -residual},
		{"just below negative full circle", -fullCircleTurn, -residual},
		{"just above negative full circle", -math.Nextafter(2*math.Pi, 0), residual},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			dir := t.TempDir()
			path := filepath.Join(dir, "map.pose")
			m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
			if err != nil {
				t.Fatalf("Create: %v", err)
			}
			// 第 1 帧：原地转向整圈余量，运动完成后在自身正前方 (1,0) 观测 L。
			// 第 2 帧：零转角，余量必须保留。
			// 第 3 帧：沿自身正前方前进 1（按运动前朝向，即余量方向转换）。
			res, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
				{Time: 10, DHeading: tc.turn, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
				{Time: 20, DHeading: 0},
				{Time: 30, DX: 1, DHeading: 0},
			}})
			if err != nil {
				t.Fatalf("ImportSegment: %v", err)
			}
			wantX := math.Cos(tc.wantH)
			wantY := math.Sin(tc.wantH)
			if wantY == 0 {
				t.Fatalf("test setup: sin(%v) rounded to zero", tc.wantH)
			}
			if res.EndPose.Heading != tc.wantH {
				t.Fatalf("end heading = %v, want signed residual %v", res.EndPose.Heading, tc.wantH)
			}
			if res.EndPose.X != wantX || res.EndPose.Y != wantY {
				t.Fatalf("end pose = (%v,%v), want (%v,%v): lateral displacement lost",
					res.EndPose.X, res.EndPose.Y, wantX, wantY)
			}
			for _, tm := range []int64{10, 20, 30} {
				p, err := m.PoseAt(tm)
				if err != nil {
					t.Fatalf("PoseAt(%d): %v", tm, err)
				}
				if p.Heading != tc.wantH {
					t.Fatalf("PoseAt(%d) heading = %v, want %v", tm, p.Heading, tc.wantH)
				}
			}
			cur, _ := m.CurrentPose()
			if cur.Heading != tc.wantH || cur.X != wantX || cur.Y != wantY {
				t.Fatalf("current pose = %+v, want (%v,%v,%v)", cur, wantX, wantY, tc.wantH)
			}
			lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if err != nil {
				t.Fatal(err)
			}
			if len(lms) != 1 || lms[0].X != wantX || lms[0].Y != wantY {
				t.Fatalf("landmark = %+v, want (%v,%v): lateral component lost", lms, wantX, wantY)
			}

			// 落盘重开：余量与其定位结果保持一致。
			if err := m.Close(); err != nil {
				t.Fatal(err)
			}
			m2, err := Open(path)
			if err != nil {
				t.Fatalf("Open: %v", err)
			}
			defer m2.Close()
			cur, _ = m2.CurrentPose()
			if cur.Heading != tc.wantH || cur.X != wantX || cur.Y != wantY {
				t.Fatalf("after reopen current pose = %+v, want (%v,%v,%v)", cur, wantX, wantY, tc.wantH)
			}
			lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if len(lms) != 1 || lms[0].X != wantX || lms[0].Y != wantY {
				t.Fatalf("after reopen landmark = %+v, want (%v,%v)", lms, wantX, wantY)
			}
		})
	}
}

// 回环校正：目标朝向略超一整圈时，锚点采用保留余量后的等价朝向（不是零），
// 后续位姿与受影响路标按该方向完成校正，校正记录后值与查询结果一致；
// 目标略小于一整圈时余量为负，对称处理。重开后结果一致。
func TestFullCircleTargetHeadingCorrection(t *testing.T) {
	residual := fullCircleResidual()
	cases := []struct {
		name   string
		target float64
		wantH  float64
	}{
		{"just above full circle", fullCircleTurn, residual},
		{"just below full circle", math.Nextafter(2*math.Pi, 0), -residual},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
			// 锚点帧 t=10 前进到 (1,0)；末帧 t=20 再前进到 (2,0)，在自身
			// 正前方 (1,0) 观测 L，地图坐标 (3,0)。
			if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
				{Time: 10, DX: 1},
				{Time: 20, DX: 1, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
			}}); err != nil {
				t.Fatalf("ImportSegment: %v", err)
			}
			rec, err := m.Correct(Correction{
				ID:     "c",
				Anchor: 10,
				Target: CorrectionTarget{X: 1, Y: 0, Heading: tc.target, Variance: 0},
			})
			if err != nil {
				t.Fatalf("Correct: %v", err)
			}
			if len(rec.Poses) != 2 {
				t.Fatalf("pose changes = %d, want 2", len(rec.Poses))
			}
			// 锚点：位置不变，朝向从零变为保留的余量——不是恒等校正。
			if rec.Poses[0].Before.Heading != 0 || rec.Poses[0].After.Heading != tc.wantH {
				t.Fatalf("anchor heading %v -> %v, want 0 -> %v",
					rec.Poses[0].Before.Heading, rec.Poses[0].After.Heading, tc.wantH)
			}
			if rec.Poses[0].After.X != 1 || rec.Poses[0].After.Y != 0 {
				t.Fatalf("anchor after = (%v,%v), want (1,0)", rec.Poses[0].After.X, rec.Poses[0].After.Y)
			}
			// 后续帧随锚点一起按余量方向旋转：(2,0) -> (2, sin(余量))。
			wantY := math.Sin(tc.wantH)
			if rec.Poses[1].After.X != 2 || rec.Poses[1].After.Y != wantY || rec.Poses[1].After.Heading != tc.wantH {
				t.Fatalf("tail after = %+v, want (2,%v,%v)", rec.Poses[1].After, wantY, tc.wantH)
			}
			// 受影响路标按校正后位姿重放：(3,0) -> (3, 2*sin(余量))。
			if len(rec.Landmarks) != 1 || rec.Landmarks[0].ID != "L" {
				t.Fatalf("landmark changes = %+v", rec.Landmarks)
			}
			lc := rec.Landmarks[0]
			if lc.Before.X != 3 || lc.Before.Y != 0 || lc.After.X != 3 || lc.After.Y != 2*wantY {
				t.Fatalf("landmark change = (%v,%v) -> (%v,%v), want (3,0) -> (3,%v)",
					lc.Before.X, lc.Before.Y, lc.After.X, lc.After.Y, 2*wantY)
			}
			// 记录后值与查询结果一致。
			cur, _ := m.CurrentPose()
			if cur.Heading != tc.wantH || cur.X != 2 || cur.Y != wantY {
				t.Fatalf("current pose = %+v, want (2,%v,%v)", cur, wantY, tc.wantH)
			}
			p, err := m.PoseAt(20)
			if err != nil {
				t.Fatalf("PoseAt(20): %v", err)
			}
			if p.Heading != rec.Poses[1].After.Heading || p.X != rec.Poses[1].After.X || p.Y != rec.Poses[1].After.Y {
				t.Fatalf("PoseAt(20) = %+v, record after = %+v", p, rec.Poses[1].After)
			}
			lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if err != nil {
				t.Fatal(err)
			}
			if len(lms) != 1 || lms[0].X != lc.After.X || lms[0].Y != lc.After.Y {
				t.Fatalf("landmarks = %+v, record after = (%v,%v)", lms, lc.After.X, lc.After.Y)
			}

			// 重开后当前位姿、路标与校正记录保持一致。
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
			if cur.Heading != tc.wantH || cur.X != 2 || cur.Y != wantY {
				t.Fatalf("after reopen current pose = %+v, want (2,%v,%v)", cur, wantY, tc.wantH)
			}
			lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if len(lms) != 1 || lms[0].X != 3 || lms[0].Y != 2*wantY {
				t.Fatalf("after reopen landmark = %+v, want (3,%v)", lms, 2*wantY)
			}
			recs, err := m2.Corrections()
			if err != nil {
				t.Fatal(err)
			}
			if len(recs) != 1 || len(recs[0].Poses) != 2 ||
				recs[0].Poses[0].After.Heading != tc.wantH ||
				recs[0].Poses[1].After.Y != wantY {
				t.Fatalf("after reopen correction record = %+v", recs)
			}
		})
	}
}
