package posemap

import (
	"errors"
	"math"
	"path/filepath"
	"testing"
)

// 整圈边界的微小余量：6.283185307179587 是比一整圈 2π 大一个 float64
// ULP 的可表示角度（运行时相减余 8.881784197001252e-16 弧度）。这些余量
// 是有限、可表示的真实转向，归一后必须作为有符号方向保留，不能被当成
// 舍入误差清零，也不能因此拒绝合法运动。
//
// 注意：下列余量必须用变量在运行时相减得到。Go 的无类型常量以任意精度
// 运算，常量 6.283185307179587 - 2π 得到的是真实数学差（约 5.23e-16），
// 而本问题关心的是两个 float64 值在运行时相减的余量 8.88e-16。
func fullTurnFixtures() (twoPi, oneAbove, eps, oneBelow float64) {
	twoPi = 2.0 * math.Pi
	oneAbove = 6.283185307179587
	eps = oneAbove - twoPi // 运行时：8.881784197001252e-16
	oneBelow = twoPi - eps
	return
}

// normalizeAngle 在整圈附近的不变量：整圈上、下沿的可表示输入保留减去
// 整圈后的有符号余量；恰好整数圈得到零；范围内角度原样；结果始终落在
// [-π,π)；多圈输入同样保留真实余量的方向。
func TestNormalizeAngleFullTurnResidue(t *testing.T) {
	twoPi, oneAbove, eps, oneBelow := fullTurnFixtures()
	cases := []struct {
		name string
		in   float64
		want float64
	}{
		{"one ULP above a full turn stays positive", oneAbove, eps},
		{"one ULP below a full turn stays negative", oneBelow, -eps},
		{"reverse turn one ULP short leaves positive", -oneBelow, eps},
		{"reverse turn one ULP over leaves negative", -oneAbove, -eps},
		{"exactly one full turn is zero", twoPi, 0},
		{"exactly minus one full turn is zero", -twoPi, 0},
		{"exactly two full turns is zero", 2 * twoPi, 0},
		{"exactly minus two full turns is zero", -2 * twoPi, 0},
		{"in-range tiny positive untouched", eps, eps},
		{"in-range tiny negative untouched", -eps, -eps},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			got := normalizeAngle(tc.in)
			if got != tc.want {
				t.Fatalf("normalizeAngle(%.17g) = %.17g, want %.17g", tc.in, got, tc.want)
			}
			if got >= math.Pi || got < -math.Pi {
				t.Fatalf("normalizeAngle(%.17g) = %v outside [-pi,pi)", tc.in, got)
			}
		})
	}

	// 多圈输入：减去整数个整圈后仍有可表示余量时，余量与方向都要保留。
	// 期望值直接取运行时 math.Mod（这些余数本身已在 [-π,π) 内）。
	const delta = 1e-14 // 在 4π 量级（ULP≈1.78e-15）仍可存活的余量
	multi := []struct {
		name string
		in   float64
		sign int
	}{
		{"two turns plus residue", 2*twoPi + delta, +1},
		{"two turns minus residue", 2*twoPi - delta, -1},
		{"reverse two turns short residue", -2*twoPi + delta, +1},
		{"reverse two turns over residue", -2*twoPi - delta, -1},
	}
	for _, tc := range multi {
		t.Run(tc.name, func(t *testing.T) {
			want := math.Mod(tc.in, twoPi)
			if want >= math.Pi {
				want -= twoPi
			} else if want < -math.Pi {
				want += twoPi
			}
			if want == 0 || (tc.sign > 0) != (want > 0) {
				t.Fatalf("test setup: unexpected residue %v for input %.17g", want, tc.in)
			}
			got := normalizeAngle(tc.in)
			if got != want {
				t.Fatalf("normalizeAngle(%.17g) = %.17g, want residue %.17g", tc.in, got, want)
			}
		})
	}
}

// 整圈余量必须实际用于定位：原地转向保留余量后沿自身正前方前进，横向位移
// 与余量方向一致；转向帧在自身正前方观测到的路标带同样方向的横向坐标；
// 之后导入零转角帧余量继续保留；段末位姿、当前位姿与历史位姿反映同一个
// 定位结果；落盘重开后保持。四个输入方向（整圈上/下沿 × 正/反方向转动）
// 各留下对应符号的余量。
func TestFullTurnResidueDrivesLocalization(t *testing.T) {
	_, oneAbove, eps, oneBelow := fullTurnFixtures()
	cases := []struct {
		name string
		turn float64
		want float64 // 归一后的有符号余量
	}{
		{"above turning forward", oneAbove, eps},
		{"below turning forward", oneBelow, -eps},
		{"above turning reverse", -oneBelow, eps},
		{"below turning reverse", -oneAbove, -eps},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "map.pose")
			m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
			if err != nil {
				t.Fatalf("Create: %v", err)
			}
			// t=10：原地转向一个整圈余量，运动完成后在自身正前方 (1,0)
			// 观测路标 L（按运动完成后的朝向转换）。
			// t=20：沿自身正前方前进 1（按运动前朝向 tc.want 转换）。
			// t=30：零转角帧，已有余量必须原样保留。
			res, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
				{Time: 10, DHeading: tc.turn, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
				{Time: 20, DX: 1, DHeading: 0},
				{Time: 30, DHeading: 0},
			}})
			if err != nil {
				t.Fatalf("ImportSegment: %v", err)
			}
			// 在 8.88e-16 量级 sin(want)==want、cos(want)==1。
			wantX, wantY := math.Cos(tc.want), math.Sin(tc.want)
			if wantY == 0 || wantY != tc.want || wantX != 1 {
				t.Fatalf("test setup: trig of %.17g gives (%v,%v)", tc.want, wantX, wantY)
			}
			if res.EndPose.Heading != tc.want {
				t.Fatalf("end heading = %.17g, want %.17g", res.EndPose.Heading, tc.want)
			}
			if res.EndPose.X != wantX || res.EndPose.Y != wantY {
				t.Fatalf("end pose = (%v,%v), want (%v,%v): lateral motion lost",
					res.EndPose.X, res.EndPose.Y, wantX, wantY)
			}
			for _, tm := range []int64{10, 20, 30} {
				p, err := m.PoseAt(tm)
				if err != nil {
					t.Fatalf("PoseAt(%d): %v", tm, err)
				}
				if p.Heading != tc.want {
					t.Fatalf("PoseAt(%d) heading = %.17g, want %.17g", tm, p.Heading, tc.want)
				}
			}
			cur, _ := m.CurrentPose()
			if cur.Heading != tc.want || cur.X != wantX || cur.Y != wantY {
				t.Fatalf("current pose = %+v, want (%v,%v,%v)", cur, wantX, wantY, tc.want)
			}
			lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if err != nil {
				t.Fatal(err)
			}
			// 转向帧的路标观测按运动完成后的朝向转换，横向坐标同号。
			if len(lms) != 1 || lms[0].ID != "L" || lms[0].Count != 1 ||
				lms[0].X != wantX || lms[0].Y != wantY {
				t.Fatalf("landmark = %+v, want (1,%v): lateral landmark coord lost", lms, wantY)
			}

			// 落盘重开：余量与其定位结果逐位保持，打开核对不再把它清零。
			if err := m.Close(); err != nil {
				t.Fatal(err)
			}
			m2, err := Open(path)
			if err != nil {
				t.Fatalf("Open: %v", err)
			}
			defer m2.Close()
			cur, _ = m2.CurrentPose()
			if cur.Heading != tc.want || cur.X != wantX || cur.Y != wantY {
				t.Fatalf("after reopen current pose = %+v, want (%v,%v,%v)", cur, wantX, wantY, tc.want)
			}
			for _, tm := range []int64{10, 20, 30} {
				if p, err := m2.PoseAt(tm); err != nil || p.Heading != tc.want {
					t.Fatalf("after reopen PoseAt(%d) = %+v, %v; want heading %.17g", tm, p, err, tc.want)
				}
			}
			lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
			if len(lms) != 1 || lms[0].X != wantX || lms[0].Y != wantY {
				t.Fatalf("after reopen landmark = %+v, want (1,%v)", lms, wantY)
			}
		})
	}
}

// 恰好一整圈是合法运动，但归一后朝向必须为零：前进回到纯 X 轴，路标横向
// 坐标为零。以整圈作为校正目标朝向时，锚点采用等价的零朝向——位置也相同
// 时属于恒等校正，校正前后位姿逐位不变。
func TestExactFullTurnNormalizesToZero(t *testing.T) {
	twoPi, _, _, _ := fullTurnFixtures()
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	res, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, DHeading: twoPi, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
		{Time: 20, DX: 1},
	}})
	if err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	if res.EndPose.Heading != 0 || res.EndPose.X != 1 || res.EndPose.Y != 0 {
		t.Fatalf("end pose = %+v, want (1,0,0)", res.EndPose)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].X != 1 || lms[0].Y != 0 {
		t.Fatalf("landmark = %+v, want (1,0)", lms)
	}

	// 目标朝向为一整圈：归一为零，与锚点原位姿等价，恒等校正逐位保持。
	rec, err := m.Correct(Correction{
		ID: "c", Anchor: 10,
		Target: CorrectionTarget{X: 0, Y: 0, Heading: twoPi, Variance: 0},
	})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if len(rec.Poses) != 2 {
		t.Fatalf("pose changes = %d, want 2", len(rec.Poses))
	}
	for i, pc := range rec.Poses {
		if pc.Before != pc.After {
			t.Fatalf("identity correction changed pose %d: %+v -> %+v", i, pc.Before, pc.After)
		}
	}
	if cur, _ := m.CurrentPose(); cur.Heading != 0 || cur.X != 1 || cur.Y != 0 {
		t.Fatalf("current pose = %+v, want (1,0,0)", cur)
	}
}

// 已确认回环校正的目标朝向处在整圈附近时，锚点采用保留下来的等价朝向，
// 后续位姿与受影响路标按该方向完成现有旋转校正；校正记录后值与查询结果
// 逐位一致，重开后一致。
func TestFullTurnResidueCorrectionTarget(t *testing.T) {
	_, oneAbove, eps, oneBelow := fullTurnFixtures()
	t.Run("target above full turn rotates to positive residue", func(t *testing.T) {
		// 锚点 t=10 前进到 (1,0)、朝向 0；t=20 再前进到 (2,0)，在自身正前
		// 方观测 L（地图 (3,0)）。目标朝向为整圈上方一个余量 → +eps：锚点
		// 朝向变为 eps，后续帧与路标整体旋转 eps。
		m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 10, DX: 1},
			{Time: 20, DX: 1, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
		}}); err != nil {
			t.Fatalf("ImportSegment: %v", err)
		}
		rec, err := m.Correct(Correction{
			ID: "c", Anchor: 10,
			Target: CorrectionTarget{X: 1, Y: 0, Heading: oneAbove, Variance: 0},
		})
		if err != nil {
			t.Fatalf("Correct: %v", err)
		}
		wantTailY := math.Sin(eps) // (2,0) 旋转 eps → (2, eps)
		wantLMY := 2 * wantTailY   // 路标 (3,0) → (3, 2eps)
		if rec.Poses[0].After.Heading != eps || rec.Poses[0].After.X != 1 || rec.Poses[0].After.Y != 0 {
			t.Fatalf("anchor after = %+v, want (1,0,%v)", rec.Poses[0].After, eps)
		}
		if rec.Poses[1].After.X != 2 || rec.Poses[1].After.Y != wantTailY || rec.Poses[1].After.Heading != eps {
			t.Fatalf("tail after = %+v, want (2,%v,%v)", rec.Poses[1].After, wantTailY, eps)
		}
		if len(rec.Landmarks) != 1 || rec.Landmarks[0].After.X != 3 || rec.Landmarks[0].After.Y != wantLMY {
			t.Fatalf("landmark after = %+v, want (3,%v)", rec.Landmarks, wantLMY)
		}
		assertCorrectionResultConsistent(t, m, eps, 2, wantTailY, 3, wantLMY)
	})

	t.Run("target below full turn rotates to negative residue", func(t *testing.T) {
		// 导入先保留一个正余量：t=10 原地转 oneAbove（朝向 +eps，位置原点），
		// 运动完成后正前方观测 L；t=20 前进 1（按 +eps）→ (1,eps)，L 地图
		// 坐标 (2,2eps)。目标朝向为整圈下方一个余量 → -eps：锚点与后续轨迹、
		// 路标按负方向校正，横向坐标翻负。
		m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1})
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 10, DHeading: oneAbove, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
			{Time: 20, DX: 1},
		}}); err != nil {
			t.Fatalf("ImportSegment: %v", err)
		}
		rec, err := m.Correct(Correction{
			ID: "c", Anchor: 10,
			Target: CorrectionTarget{X: 0, Y: 0, Heading: oneBelow, Variance: 0},
		})
		if err != nil {
			t.Fatalf("Correct: %v", err)
		}
		if rec.Poses[0].After.Heading != -eps || rec.Poses[0].After.X != 0 || rec.Poses[0].After.Y != 0 {
			t.Fatalf("anchor after = %+v, want (0,0,%v)", rec.Poses[0].After, -eps)
		}
		if rec.Poses[1].After.Heading != -eps || rec.Poses[1].After.X != 1 || rec.Poses[1].After.Y != -eps {
			t.Fatalf("tail after = %+v, want (1,%v,%v)", rec.Poses[1].After, -eps, -eps)
		}
		// 路标观测在转向帧 t=10（位置原点）：校正前 (1,+eps)，随校正后
		// 朝向 -eps 重放为 (1,-eps)；t=20 的前进帧不携带观测。
		if len(rec.Landmarks) != 1 ||
			rec.Landmarks[0].Before.X != 1 || rec.Landmarks[0].Before.Y != eps ||
			rec.Landmarks[0].After.X != 1 || rec.Landmarks[0].After.Y != -eps {
			t.Fatalf("landmark change = %+v, want (1,%v) -> (1,%v)", rec.Landmarks, eps, -eps)
		}
		// 校正后的末帧带负余量：随后导入的前进按它定位，留下负横向分量。
		// 必须在一致性核对（会关闭地图）之前完成。
		res, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{{Time: 30, DX: 1}}})
		if err != nil {
			t.Fatalf("ImportSegment after correction: %v", err)
		}
		if res.EndPose.Heading != -eps || res.EndPose.X != 2 || res.EndPose.Y != -2*eps {
			t.Fatalf("end pose = %+v, want (2,%v,%v)", res.EndPose, -2*eps, -eps)
		}
		// 当前位姿为追加后的 t=30；路标不受追加运动影响。
		assertCorrectionResultConsistent(t, m, -eps, 2, -2*eps, 1, -eps)
	})
}

// assertCorrectionResultConsistent 核对校正记录后值、当前/历史位姿查询、
// 路标区域查询与重开后的结果全部反映同一个定位结果。
func assertCorrectionResultConsistent(t *testing.T, m *Map, wantHeading, wantCurX, wantCurY, wantLMX, wantLMY float64) {
	t.Helper()
	recs, err := m.Corrections()
	if err != nil || len(recs) != 1 {
		t.Fatalf("Corrections = %v, %v", recs, err)
	}
	last := recs[0].Poses[len(recs[0].Poses)-1]
	if last.After.Heading != wantHeading {
		t.Fatalf("record last after heading = %v, want %v", last.After.Heading, wantHeading)
	}
	if cur, _ := m.CurrentPose(); cur.Heading != wantHeading || cur.X != wantCurX || cur.Y != wantCurY {
		t.Fatalf("current pose = %+v, want (%v,%v,%v)", cur, wantCurX, wantCurY, wantHeading)
	}
	if p, _ := m.PoseAt(last.After.Time); p != last.After {
		t.Fatalf("PoseAt(%d) = %+v, want record after %+v", last.After.Time, p, last.After)
	}
	lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].X != wantLMX || lms[0].Y != wantLMY {
		t.Fatalf("landmarks = %+v, want (%v,%v)", lms, wantLMX, wantLMY)
	}
	if len(recs[0].Landmarks) != 1 ||
		recs[0].Landmarks[0].After.X != wantLMX || recs[0].Landmarks[0].After.Y != wantLMY {
		t.Fatalf("record landmark after = %+v, want (%v,%v)", recs[0].Landmarks, wantLMX, wantLMY)
	}

	path := m.Path()
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	defer m2.Close()
	if cur, _ := m2.CurrentPose(); cur.Heading != wantHeading || cur.X != wantCurX || cur.Y != wantCurY {
		t.Fatalf("after reopen current pose = %+v, want (%v,%v,%v)", cur, wantCurX, wantCurY, wantHeading)
	}
	lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].X != wantLMX || lms[0].Y != wantLMY {
		t.Fatalf("after reopen landmarks = %+v, want (%v,%v)", lms, wantLMX, wantLMY)
	}
	recs2, err := m2.Corrections()
	if err != nil || len(recs2) != 1 ||
		recs2[0].Poses[0].After.Heading != recs[0].Poses[0].After.Heading ||
		recs2[0].Poses[len(recs2[0].Poses)-1].After.Heading != wantHeading {
		t.Fatalf("after reopen corrections = %+v, %v", recs2, err)
	}
}

// 保留整圈余量不改变路标冲突规则：余量真实进入观测坐标，两条观测相距超过
// 合并距离时整次导入仍被拒绝，且不留任何部分位姿或路标。
func TestFullTurnResidueDoesNotRelaxConflict(t *testing.T) {
	_, oneAbove, _, _ := fullTurnFixtures()
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 0.5})
	// t=10 转向整圈余量，在自身原点观测 L → 地图 (0,0)；t=20 前进 1 米后
	// 再在自身原点观测 L → 地图约 (1, ±eps)，相距约 1 米，超过 0.5。
	_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, DHeading: oneAbove, Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
		{Time: 20, DX: 1, Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
	}})
	var rej *RejectError
	if !errors.As(err, &rej) || rej.Kind != RejectLandmarkConflict {
		t.Fatalf("err = %v, want landmark_conflict", err)
	}
	// 整次失败：没有任何帧或路标提交。
	if cur, _ := m.CurrentPose(); cur.Time != 0 {
		t.Fatalf("current pose time = %d, want initial 0 after rejected segment", cur.Time)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 0 {
		t.Fatalf("landmarks after rejection = %+v, want none", lms)
	}
}
