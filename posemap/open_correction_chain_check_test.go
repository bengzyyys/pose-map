package posemap

import (
	"errors"
	"math"
	"path/filepath"
	"strings"
	"testing"
)

// 本文件覆盖“相邻校正衔接核对”：同一帧再次被校正时，本次记录的校正前
// 位置、朝向与方差必须承接此前最后一次覆盖该帧的记录的校正后值。基础场
// 景复用 buildOverlappingCorrectionsMap（见 open_last_correction_check_test.go）：
// c1 覆盖 100~300，追加 400 后 c2 覆盖 200~400，因此 200/300 的衔接为
// c1.After → c2.Before，400 没有前次校正。
//
// 为把衔接核对从其他核对中隔离出来，篡改采用“联动改法”：改某条记录某帧
// 的校正前值时，同步改该记录的校正后值与当前轨迹（锚点帧还需同步目标），
// 使每条记录自身的几何/方差核对、当前轨迹与最后一次校正的核对都仍然通
// 过，唯一剩下的矛盾就是前后两次校正之间的衔接。

// requireChainSetup 校验基础文件结构符合本组测试的假设，返回两条校正记录。
func requireChainSetup(t *testing.T, fd *fileData) {
	t.Helper()
	if len(fd.Corrections) != 2 || fd.Corrections[0].ID != "c1" || fd.Corrections[1].ID != "c2" {
		t.Fatalf("setup: corrections = %+v", fd.Corrections)
	}
	if len(fd.Trajectory) != 5 || fd.Trajectory[2].Time != 200 || fd.Trajectory[3].Time != 300 {
		t.Fatalf("setup: trajectory = %+v", fd.Trajectory)
	}
}

// 即使校验和正确、每条记录自身合法、当前轨迹与最后一次校正一致，本次记录
// 的校正前值不承接前次校正后值时 Open 必须拒绝：覆盖锚点帧与中间帧、位置
// 两个坐标、朝向与方差，以及篡改前次记录的校正后值两个方向。
func TestOpenRejectsBrokenCorrectionChain(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		// c2 锚点（t=200）校正前 x 不承接 c1 结果：锚点的校正后值即目标，
		// 目标、校正前后与当前轨迹联动改，记录自身几何保持不变。
		{"c2 anchor before x breaks chain", func(fd *fileData) {
			c2 := &fd.Corrections[1]
			c2.Target.X += 0.5
			c2.Poses[0].Before.X += 0.5
			c2.Poses[0].After.X += 0.5
			fd.Trajectory[2].X += 0.5
		}},
		// c2 锚点校正前方差不承接：方差必须完全一致，且校正前方差不参与
		// 其他任何核对，矛盾只能由衔接核对发现。
		{"c2 anchor before variance breaks chain", func(fd *fileData) {
			fd.Corrections[1].Poses[0].Before.Variance += 0.5
		}},
		// c2 中间帧（t=300）校正前 x/y/朝向不承接：校正前后与当前轨迹联动
		// 改，c2 自身几何与最后一次校正核对都仍然通过。
		{"c2 middle before x breaks chain", func(fd *fileData) {
			c2 := &fd.Corrections[1]
			c2.Poses[1].Before.X += 0.5
			c2.Poses[1].After.X += 0.5
			fd.Trajectory[3].X += 0.5
		}},
		{"c2 middle before y breaks chain", func(fd *fileData) {
			c2 := &fd.Corrections[1]
			c2.Poses[1].Before.Y += 0.5
			c2.Poses[1].After.Y += 0.5
			fd.Trajectory[3].Y += 0.5
		}},
		{"c2 middle before heading breaks chain", func(fd *fileData) {
			c2 := &fd.Corrections[1]
			c2.Poses[1].Before.Heading += 0.3
			c2.Poses[1].After.Heading += 0.3
			fd.Trajectory[3].Heading += 0.3
		}},
		{"c2 middle before variance breaks chain", func(fd *fileData) {
			fd.Corrections[1].Poses[1].Before.Variance += 0.5
		}},
		// 另一方向：篡改前次记录 c1 在 t=200 的校正后值（联动该校正前值
		// 保持 c1 自身几何一致；t=200 不是任何段的末帧，段核对不涉及）。
		{"c1 after x breaks chain", func(fd *fileData) {
			c1 := &fd.Corrections[0]
			c1.Poses[1].Before.X += 0.5
			c1.Poses[1].After.X += 0.5
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildOverlappingCorrectionsMap(t, path, 10)
			tamperFile(t, path, func(fd *fileData) {
				requireChainSetup(t, fd)
				tc.fn(fd)
			})
			assertOpenCorrupt(t, path)
		})
	}
}

// 衔接矛盾的错误必须可由 errors.Is 识别为 ErrCorrupt、不返回地图，并指出
// 前后两次校正标识及矛盾帧时间。
func TestOpenCorrectionChainErrorDetails(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	tamperFile(t, path, func(fd *fileData) {
		requireChainSetup(t, fd)
		c2 := &fd.Corrections[1]
		c2.Poses[1].Before.X += 0.5 // t=300
		c2.Poses[1].After.X += 0.5
		fd.Trajectory[3].X += 0.5
	})
	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("map = %v, want nil", m)
	}
	msg := err.Error()
	for _, want := range []string{"c1", "c2", "300"} {
		if !strings.Contains(msg, want) {
			t.Fatalf("error %q must name previous correction c1, current correction c2 and frame time 300", msg)
		}
	}
}

// 衔接比较沿用现有误差范围：坐标容差为 1e-9 乘以 1 与两值绝对值中的最大
// 者（c1 在 t=300 的校正后 x 为 7，容差约 7e-9）；朝向按最短角度差比较，
// 不超过 1e-9 弧度，跨越 ±π 的相同方向仍可接受；方差必须完全一致。
func TestOpenCorrectionChainTolerance(t *testing.T) {
	coordCases := []struct {
		name     string
		perturb  float64
		wantOpen bool
	}{
		{"within tolerance", 4e-9, true},
		{"beyond tolerance", 2e-8, false},
	}
	for _, tc := range coordCases {
		t.Run("coord "+tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildOverlappingCorrectionsMap(t, path, 10)
			tamperFile(t, path, func(fd *fileData) {
				requireChainSetup(t, fd)
				c2 := &fd.Corrections[1]
				c2.Poses[1].Before.X += tc.perturb
				c2.Poses[1].After.X += tc.perturb
				fd.Trajectory[3].X += tc.perturb
			})
			if !tc.wantOpen {
				assertOpenCorrupt(t, path)
				return
			}
			m, err := Open(path)
			if err != nil {
				t.Fatalf("within tolerance should open: %v", err)
			}
			m.Close()
		})
	}

	angleCases := []struct {
		name     string
		perturb  float64
		wantOpen bool
	}{
		{"within tolerance", 5e-10, true},
		{"beyond tolerance", 2e-9, false},
	}
	for _, tc := range angleCases {
		t.Run("angle "+tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildOverlappingCorrectionsMap(t, path, 10)
			tamperFile(t, path, func(fd *fileData) {
				requireChainSetup(t, fd)
				c2 := &fd.Corrections[1]
				c2.Poses[1].Before.Heading += tc.perturb
				c2.Poses[1].After.Heading += tc.perturb
				fd.Trajectory[3].Heading += tc.perturb
			})
			if !tc.wantOpen {
				assertOpenCorrupt(t, path)
				return
			}
			m, err := Open(path)
			if err != nil {
				t.Fatalf("within angle tolerance should open: %v", err)
			}
			m.Close()
		})
	}

	// 方差再小的差异也拒绝（精确比较，无容差）。
	t.Run("variance must match exactly", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "m.pose")
		buildOverlappingCorrectionsMap(t, path, 10)
		tamperFile(t, path, func(fd *fileData) {
			requireChainSetup(t, fd)
			fd.Corrections[1].Poses[1].Before.Variance += 1e-12
		})
		assertOpenCorrupt(t, path)
	})

	// 跨越 ±π 的相同方向仍可接受：c1 在 t=300 的校正后朝向写为 π-1e-12
	// （联动校正前值保持 c1 自身几何一致；t=300 是段 s1 的末帧，段首次
	// 导入结果同步改），c2 的校正前朝向写为 -π+1e-12，最短角度差约
	// 2e-12，在容差内。
	t.Run("heading wraps across pi", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "m.pose")
		buildOverlappingCorrectionsMap(t, path, 10)
		tamperFile(t, path, func(fd *fileData) {
			requireChainSetup(t, fd)
			nearPi := math.Pi - 1e-12
			c1 := &fd.Corrections[0]
			c1.Poses[2].Before.Heading = nearPi
			c1.Poses[2].After.Heading = nearPi
			c2 := &fd.Corrections[1]
			c2.Poses[1].Before.Heading = -nearPi
			c2.Poses[1].After.Heading = -nearPi
			fd.Trajectory[3].Heading = -nearPi
			for i := range fd.Segments {
				if fd.Segments[i].ID == "s1" {
					fd.Segments[i].Result.EndPose.Heading = nearPi
				}
			}
		})
		m, err := Open(path)
		if err != nil {
			t.Fatalf("same direction across ±π should open: %v", err)
		}
		p, _ := m.PoseAt(300)
		if p.Heading != -(math.Pi - 1e-12) {
			t.Fatalf("PoseAt(300) heading = %v, want saved value", p.Heading)
		}
		m.Close()
	})
}

// 两次校正只部分重叠时只对共同涉及的帧检查衔接：c2 的 400 帧没有前次校
// 正，其校正前值（联动校正后值、当前轨迹与段 s2 的首次导入结果，保持其
// 余核对一致）不承接任何记录也不被拒绝。
func TestOpenCorrectionChainPartialOverlapFrameWithoutPrevious(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	tamperFile(t, path, func(fd *fileData) {
		requireChainSetup(t, fd)
		c2 := &fd.Corrections[1]
		if c2.Poses[2].Before.Time != 400 {
			t.Fatalf("setup: c2 poses = %+v", c2.Poses)
		}
		c2.Poses[2].Before.X += 0.5 // t=400：没有前次校正覆盖
		c2.Poses[2].After.X += 0.5
		fd.Trajectory[4].X += 0.5
		for i := range fd.Segments {
			if fd.Segments[i].ID == "s2" {
				fd.Segments[i].Result.EndPose.X += 0.5
			}
		}
	})
	m, err := Open(path)
	if err != nil {
		t.Fatalf("frame 400 has no previous correction and must not be chained: %v", err)
	}
	p, _ := m.PoseAt(400)
	if p.X != 11.5 {
		t.Fatalf("PoseAt(400) = %+v, want saved x 11.5", p)
	}
	m.Close()
}

// 中间提交但未覆盖该帧的校正不改变它的上次结果：c1 覆盖 100~300，c2 只
// 覆盖 400~500，c3 覆盖 300~500；c3 在 300 的衔接对象是 c1 而不是 c2。
// 篡改 c3 在 300 的校正前值（锚点，联动目标、校正后值与当前轨迹）后，错
// 误必须指出 c3 与 c1。
func TestOpenCorrectionChainSkipsIntermediateNonCovering(t *testing.T) {
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
	// c2 只覆盖 400~500，不覆盖 300。
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 400,
		Target: CorrectionTarget{X: 20, Y: 20, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	// c3 覆盖 300~500：300 的上次结果是 c1，400/500 的上次结果是 c2。
	if _, err := m.Correct(Correction{ID: "c3", Anchor: 300,
		Target: CorrectionTarget{X: 9, Y: 9, Variance: 3}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	tamperFile(t, path, func(fd *fileData) {
		if len(fd.Corrections) != 3 || fd.Corrections[2].ID != "c3" {
			t.Fatalf("setup: corrections = %+v", fd.Corrections)
		}
		c3 := &fd.Corrections[2]
		if c3.Poses[0].Before.Time != 300 {
			t.Fatalf("setup: c3 poses = %+v", c3.Poses)
		}
		c3.Target.X += 0.5
		c3.Poses[0].Before.X += 0.5
		c3.Poses[0].After.X += 0.5
		fd.Trajectory[3].X += 0.5 // t=300
	})
	m2, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m2 != nil {
		t.Fatalf("map = %v, want nil", m2)
	}
	msg := err.Error()
	if !strings.Contains(msg, "c3") || !strings.Contains(msg, "c1") || !strings.Contains(msg, "300") {
		t.Fatalf("error %q must name c3, its predecessor at frame 300 (c1, not the non-covering c2) and frame time 300", msg)
	}
}

// 兼容规则：前次覆盖记录范围内含缺少逐帧依据的 null 旧帧时，跳过该处衔
// 接比较——c2 的校正前值被改成明显不承接 c1 的值也放行。
func TestOpenCorrectionChainSkipsWhenPreviousLacksBasis(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	tamperFile(t, path, func(fd *fileData) {
		requireChainSetup(t, fd)
		fd.Sources[0] = nil // t=100 在 c1 范围内：c1 依据不完整，c2 仍完整
		c2 := &fd.Corrections[1]
		c2.Target.X += 0.5
		c2.Poses[0].Before.X += 0.5 // t=200 校正前 x 不再承接 c1
		c2.Poses[0].After.X += 0.5
		fd.Trajectory[2].X += 0.5
	})
	m, err := Open(path)
	if err != nil {
		t.Fatalf("chain to a previous correction spanning a null basis frame must be skipped: %v", err)
	}
	m.Close()
}

// 兼容规则：本次记录范围内含 null 旧帧时同样跳过，且不越过它改用更早结
// 果——c2 依据不完整时，c1→c2 的衔接不检查，c2 的校正前值任意改写也放行。
func TestOpenCorrectionChainSkipsWhenCurrentLacksBasis(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	tamperFile(t, path, func(fd *fileData) {
		requireChainSetup(t, fd)
		fd.Sources[3] = nil // t=400 在 c2 范围内：c2 依据不完整
		// c2 的自身核对随依据缺失一并跳过，校正前值可任意改写。
		fd.Corrections[1].Poses[0].Before.X = 999
		fd.Corrections[1].Poses[1].Before.Y = -888
	})
	m, err := Open(path)
	if err != nil {
		t.Fatalf("chain from a correction spanning a null basis frame must be skipped: %v", err)
	}
	m.Close()
}

// 同一文件内其他依据完整的衔接仍须检查：c1 因含 null 旧帧而跳过 c1→c2
// 的衔接（c2 在 200 的校正前值被改成不承接 c1 也不因此拒绝），但依据完
// 整的 c2→c3 衔接仍然核对，矛盾时错误指出 c3、c2 与帧时间 400。
func TestOpenCorrectionChainOtherFullyBasedChainsStillChecked(t *testing.T) {
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
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 200,
		Target: CorrectionTarget{X: 9, Y: 9, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	// c3 只覆盖 400，其上次结果是依据完整的 c2。
	if _, err := m.Correct(Correction{ID: "c3", Anchor: 400,
		Target: CorrectionTarget{X: 15, Y: 15, Variance: 3}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	tamperFile(t, path, func(fd *fileData) {
		if len(fd.Corrections) != 3 || fd.Corrections[2].ID != "c3" {
			t.Fatalf("setup: corrections = %+v", fd.Corrections)
		}
		fd.Sources[0] = nil // c1 依据不完整：c1→c2 衔接跳过
		// c2 在 t=200 的校正前值不再承接 c1（联动保持 c2 自身一致）：
		// 该处衔接被跳过，不得成为拒绝理由。
		c2 := &fd.Corrections[1]
		c2.Target.X += 0.5
		c2.Poses[0].Before.X += 0.5
		c2.Poses[0].After.X += 0.5
		fd.Trajectory[2].X += 0.5
		// c3 在 t=400 的校正前值不承接 c2：依据完整的衔接仍须拒绝。
		c3 := &fd.Corrections[2]
		c3.Target.X += 0.5
		c3.Poses[0].Before.X += 0.5
		c3.Poses[0].After.X += 0.5
		fd.Trajectory[4].X += 0.5
	})
	m2, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m2 != nil {
		t.Fatalf("map = %v, want nil", m2)
	}
	msg := err.Error()
	if !strings.Contains(msg, "c3") || !strings.Contains(msg, "c2") || !strings.Contains(msg, "400") {
		t.Fatalf("error %q must name c3, c2 and frame time 400 (the fully-based chain)", msg)
	}
}
