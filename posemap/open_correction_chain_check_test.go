package posemap

import (
	"errors"
	"math"
	"path/filepath"
	"strings"
	"testing"
)

// 本文件的测试针对打开时的校正衔接核对：同一帧再次被校正时，本次记录的
// 校正前位置、朝向与方差必须承接此前最后一次覆盖该帧的记录的校正后值。
// 基础地图沿用 buildOverlappingCorrectionsMap（见
// open_last_correction_check_test.go）：
//
//	c1：锚点 100，覆盖 100~300，校正后 100=(5,6,v1) 200=(6,6,v1.1) 300=(7,6,v1.2)
//	c2：锚点 200，覆盖 200~400，校正前 200=(6,6,v1.1) 300=(7,6,v1.2) 400=(8,6,v1.3)
//	    校正后 200=(9,9,v2) 300=(10,9,v2.1) 400=(11,9,v2.2)
//
// 衔接关系：c2 的 200、300 校正前值必须承接 c1 的校正后值；400 此前未被
// 任何校正覆盖，没有可承接的上次结果。

// tamperC2Frame300 把 c2 中 t=300 帧的校正前后值与当前轨迹一致地移动
// delta：记录自身的几何核对与最后一次校正核对都仍然通过，只有 c2 校正前
// 值与 c1 校正后值之间的衔接被破坏，隔离出衔接核对本身。
func tamperC2Frame300(t *testing.T, path string, fn func(before, after, traj *Pose)) {
	t.Helper()
	tamperFile(t, path, func(fd *fileData) {
		if len(fd.Corrections) != 2 || len(fd.Corrections[1].Poses) != 3 {
			t.Fatalf("setup: %+v", fd.Corrections)
		}
		pc := &fd.Corrections[1].Poses[1] // c2 的 t=300
		if pc.Before.Time != 300 || fd.Trajectory[3].Time != 300 {
			t.Fatalf("setup: %+v / %+v", pc, fd.Trajectory[3])
		}
		fn(&pc.Before, &pc.After, &fd.Trajectory[3])
	})
}

// 即使校验和正确、每条记录自身的几何与方差核对都通过、当前轨迹与最后一次
// 校正一致，后一次校正的校正前值不承接前一次结果时 Open 必须拒绝：位置、
// 朝向、方差逐一覆盖，中间帧与锚点帧都在内；这些帧都没有路标观测，矛盾
// 只能由衔接核对发现。
func TestOpenRejectsBrokenCorrectionChain(t *testing.T) {
	cases := []struct {
		name string
		fn   func(before, after, traj *Pose)
	}{
		{"x does not continue previous result", func(b, a, tr *Pose) {
			b.X += 0.5
			a.X += 0.5
			tr.X += 0.5
		}},
		{"y does not continue previous result", func(b, a, tr *Pose) {
			b.Y += 0.5
			a.Y += 0.5
			tr.Y += 0.5
		}},
		{"heading does not continue previous result", func(b, a, tr *Pose) {
			b.Heading = 0.3
			a.Heading = 0.3
			tr.Heading = 0.3
		}},
		{"variance does not continue previous result", func(b, a, tr *Pose) {
			b.Variance = 1.5 // 期望承接 c1 的 1.2；校正前方差不参与其他核对
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildOverlappingCorrectionsMap(t, path, 10)
			tamperC2Frame300(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}

	// 锚点帧同样核对衔接：c2 锚点 200 的校正前方差不承接 c1 结果（1.1）。
	t.Run("anchor frame variance", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "m.pose")
		buildOverlappingCorrectionsMap(t, path, 10)
		tamperFile(t, path, func(fd *fileData) {
			pc := &fd.Corrections[1].Poses[0] // c2 的 t=200（锚点）
			if pc.Before.Time != 200 {
				t.Fatalf("setup: %+v", pc)
			}
			pc.Before.Variance = 1.5
		})
		assertOpenCorrupt(t, path)
	})
}

// 错误必须可被 errors.Is 识别为 ErrCorrupt，不返回可用地图，并指出前后
// 两次校正标识与矛盾帧时间。
func TestOpenCorrectionChainErrorDetails(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	tamperC2Frame300(t, path, func(b, a, tr *Pose) {
		b.X += 0.5
		a.X += 0.5
		tr.X += 0.5
	})
	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("map = %v, want nil", m)
	}
	msg := err.Error()
	if !strings.Contains(msg, "c2") || !strings.Contains(msg, "c1") || !strings.Contains(msg, "300") {
		t.Fatalf("error %q must name corrections c2 and c1 and frame time 300", msg)
	}
}

// 位置容差为 1e-9 乘以 1、两个坐标绝对值的最大者（量级约 7 时容差约
// 7e-9，半档放行、两倍档拒绝）；朝向按最短角度差比较，容差 1e-9 弧度，
// 跨越 ±π 的相同方向仍可接受；方差必须完全一致。放行时记录与轨迹都保留
// 文件原值，不借容差改写。
func TestOpenCorrectionChainTolerance(t *testing.T) {
	coordCases := []struct {
		name     string
		perturb  float64
		wantOpen bool
	}{
		{"within half tolerance", 3e-9, true},
		{"beyond tolerance", 2e-8, false},
	}
	for _, tc := range coordCases {
		t.Run("coord "+tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildOverlappingCorrectionsMap(t, path, 10)
			tamperC2Frame300(t, path, func(b, a, tr *Pose) {
				b.X += tc.perturb // 7+δ 对 c1 的 7
				a.X += tc.perturb
				tr.X += tc.perturb
			})
			if !tc.wantOpen {
				assertOpenCorrupt(t, path)
				return
			}
			m, err := Open(path)
			if err != nil {
				t.Fatalf("within tolerance should open: %v", err)
			}
			recs, _ := m.Corrections()
			if got := recs[1].Poses[1].Before.X; got != 7+tc.perturb {
				t.Fatalf("record before x = %v, want saved value %v", got, 7+tc.perturb)
			}
			m.Close()
		})
	}

	angleCases := []struct {
		name     string
		perturb  float64
		wantOpen bool
	}{
		{"within half tolerance", 5e-10, true},
		{"beyond tolerance", 2e-9, false},
	}
	for _, tc := range angleCases {
		t.Run("angle "+tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildOverlappingCorrectionsMap(t, path, 10)
			tamperC2Frame300(t, path, func(b, a, tr *Pose) {
				b.Heading = tc.perturb
				a.Heading = tc.perturb
				tr.Heading = tc.perturb
			})
			if tc.wantOpen {
				m, err := Open(path)
				if err != nil {
					t.Fatalf("within angle tolerance should open: %v", err)
				}
				m.Close()
			} else {
				assertOpenCorrupt(t, path)
			}
		})
	}

	// 同一方向写在 ±π 两侧：c1 校正后朝向 π-1e-10，c2 校正前朝向
	// -π+1e-10，最短角度差 2e-10 在容差内，必须放行。c1 的校正前朝向与
	// 段 s1 的末位姿一并改写，保持其余核对一致。
	t.Run("angle wraps across pi", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "m.pose")
		buildOverlappingCorrectionsMap(t, path, 10)
		tamperFile(t, path, func(fd *fileData) {
			c1 := &fd.Corrections[0].Poses[2] // c1 的 t=300
			c1.Before.Heading = math.Pi - 1e-10
			c1.After.Heading = math.Pi - 1e-10
			for i := range fd.Segments {
				if fd.Segments[i].ID == "s1" {
					fd.Segments[i].Result.EndPose.Heading = math.Pi - 1e-10
				}
			}
			c2 := &fd.Corrections[1].Poses[1] // c2 的 t=300
			c2.Before.Heading = -math.Pi + 1e-10
			c2.After.Heading = -math.Pi + 1e-10
			fd.Trajectory[3].Heading = -math.Pi + 1e-10
		})
		m, err := Open(path)
		if err != nil {
			t.Fatalf("same direction across ±pi should open: %v", err)
		}
		m.Close()
	})

	t.Run("variance must match exactly", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "m.pose")
		buildOverlappingCorrectionsMap(t, path, 10)
		tamperFile(t, path, func(fd *fileData) {
			fd.Corrections[1].Poses[1].Before.Variance = 1.2 + 1e-12 // 期望恰为 1.2
		})
		assertOpenCorrupt(t, path)
	})
}

// 中间提交但未覆盖该帧的校正不改变它的上次结果：c1 覆盖 100~300，c2 只
// 覆盖 400~500，c3 覆盖 200~500；c3 中 200、300 的校正前值承接 c1（而不
// 是时间上相邻的 c2），400、500 承接 c2。
func buildChainAcrossGapMap(t *testing.T, path string) {
	t.Helper()
	m, err := Create(path, Config{
		InitialTime: 0, MaxInterval: 1000, MergeDistance: 10,
	})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.1},
		{Time: 200, DX: 1, MoveVariance: 0.1},
		{Time: 300, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 0.1},
		{Time: 500, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 400,
		Target: CorrectionTarget{X: 20, Y: 20, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c3", Anchor: 200,
		Target: CorrectionTarget{X: 9, Y: 9, Variance: 3}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

func TestOpenChainSkipsInterveningNonCoveringCorrection(t *testing.T) {
	// 合法地图正常打开，校正记录保持提交时的快照与次序。
	path := filepath.Join(t.TempDir(), "m.pose")
	buildChainAcrossGapMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	recs, _ := m.Corrections()
	if len(recs) != 3 {
		t.Fatalf("records = %+v", recs)
	}
	// c3 的 300 校正前快照仍是 c1 的结果 (7,6)，方差为逐帧累计值，不被改写。
	wantVar := 1.0
	wantVar += 0.1
	wantVar += 0.1
	if got := recs[2].Poses[1].Before; got.X != 7 || got.Y != 6 || got.Variance != wantVar {
		t.Fatalf("c3 before snapshot = %+v, want (7,6,v%v)", got, wantVar)
	}
	m.Close()

	// c3 的 t=300 校正前值不承接 c1（c2 不覆盖 300，不参与该帧的衔接）：
	// 一致地移动 c2 够不着的 c3 前后值与当前轨迹，仍按损坏拒绝。
	path2 := filepath.Join(t.TempDir(), "m.pose")
	buildChainAcrossGapMap(t, path2)
	tamperFile(t, path2, func(fd *fileData) {
		pc := &fd.Corrections[2].Poses[1] // c3 的 t=300
		if pc.Before.Time != 300 || fd.Trajectory[3].Time != 300 {
			t.Fatalf("setup: %+v / %+v", pc, fd.Trajectory[3])
		}
		pc.Before.X += 0.5
		pc.After.X += 0.5
		fd.Trajectory[3].X += 0.5
	})
	assertOpenCorrupt(t, path2)
}

// 兼容规则：前一次覆盖该帧的记录范围内含缺少逐帧依据的 null 旧帧时，该处
// 衔接跳过比较——即使本次校正前值被改成明显矛盾值也放行，记录保持文件
// 原值。
func TestOpenSkipsChainCheckForNullBasis(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[0] = nil // t=100 只在 c1 范围内：c1 依据不完整，c2 仍完整
		pc := &fd.Corrections[1].Poses[1]
		pc.Before.X += 0.5 // 与 c1 校正后值矛盾，但衔接被跳过
		pc.After.X += 0.5
		fd.Trajectory[3].X += 0.5
	})
	m, err := Open(path)
	if err != nil {
		t.Fatalf("chain to a correction spanning a null basis frame keeps legacy rules: %v", err)
	}
	recs, _ := m.Corrections()
	if got := recs[1].Poses[1].Before.X; got != 7.5 {
		t.Fatalf("record before x = %v, want saved value 7.5", got)
	}
	m.Close()
}

// 兼容规则不得回退到更早记录：帧先被依据完整的 c1 覆盖、再被含 null 旧帧
// 的 c2 覆盖、最后被依据完整的 c3 覆盖时，c3 的衔接对象是 c2，因 c2 依据
// 缺失而跳过——不能越过 c2 改用 c1 的旧结果要求 c3 的校正前值。
func TestOpenChainDoesNotFallBackToEarlier(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, Config{
		InitialTime: 0, MaxInterval: 1000, MergeDistance: 10,
	})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.1},
		{Time: 200, DX: 1, MoveVariance: 0.1},
		{Time: 300, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	// c1 依据完整，覆盖 200~300。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 200,
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	// c2 覆盖 100~400（提交时依据完整）。
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 100,
		Target: CorrectionTarget{X: 10, Y: 10, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 500, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	// c3 依据完整，覆盖 300~500；其 300 的校正前值承接 c2 的结果
	// (15,16,v2.2)，与 c1 的旧结果 (6,6,v1.1) 不同。
	if _, err := m.Correct(Correction{ID: "c3", Anchor: 300,
		Target: CorrectionTarget{X: 20, Y: 20, Variance: 3}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// t=100 的依据置空：c2（100~400）依据不完整，成为 300 的衔接对象时
	// 跳过比较；若错误地回退到 c1，c3 的校正前值 (15,16) 会被误判矛盾。
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[0] = nil
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("must not fall back to earlier fully-based c1: %v", err)
	}
	recs, _ := m2.Corrections()
	if got := recs[2].Poses[0].Before; got.X != 15 || got.Y != 16 || got.Variance != 2.2 {
		t.Fatalf("c3 before snapshot = %+v, want (15,16,v2.2)", got)
	}
	m2.Close()
}

// 同一文件内其他依据完整的衔接仍须检查：c1 因含 null 旧帧使 c2 对它的衔
// 接被跳过，但 c2、c3 依据完整，c3 对 c2 的衔接矛盾仍按损坏拒绝。
func TestOpenChainCheckStillAppliesToOtherFullyBasedLinks(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 500, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	// c3 覆盖 300~500，校正前 300=(10,9,v2.1) 400=(11,9,v2.2) 500=(12,9,v2.3)。
	if _, err := m.Correct(Correction{ID: "c3", Anchor: 300,
		Target: CorrectionTarget{X: 30, Y: 30, Variance: 4}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// t=100 只在 c1 范围内：c1 依据不完整，c2/c3 仍完整，文件正常打开。
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[0] = nil
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	m2.Close()

	// c3 的 t=400 校正前值不承接 c2 的结果（11）：c1 的跳过不影响这条
	// 依据完整的衔接，整份文件拒绝。
	tamperFile(t, path, func(fd *fileData) {
		pc := &fd.Corrections[2].Poses[1] // c3 的 t=400
		if pc.Before.Time != 400 || fd.Trajectory[4].Time != 400 {
			t.Fatalf("setup: %+v / %+v", pc, fd.Trajectory[4])
		}
		pc.Before.X += 0.5
		pc.After.X += 0.5
		fd.Trajectory[4].X += 0.5
	})
	assertOpenCorrupt(t, path)
}
