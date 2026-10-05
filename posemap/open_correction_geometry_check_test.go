package posemap

import (
	"math"
	"path/filepath"
	"testing"
)

// buildRotatedCorrectedMap 构造含三帧（t=100、200、300，校正前位姿为
// (2,2,0)、(3,2,0)、(4,2,0)）并对 t=100 做过一次带旋转校正的地图：目标
// (5,6,π/2)，校正后三帧约为 (5,6,π/2)、(5,7,π/2)、(5,8,π/2)，方差均为 1。
func buildRotatedCorrectedMap(t *testing.T, path string) {
	t.Helper()
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
		{Time: 300, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: 6, Heading: math.Pi / 2, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 未篡改的带旋转校正记录正常打开：锚点校正后位姿即目标（朝向归一），其余帧
// 随锚点一起平移旋转，保留校正前的相对位置关系与朝向差。
func TestOpenAcceptsRotatedCorrectionGeometry(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildRotatedCorrectedMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 || len(recs[0].Poses) != 3 {
		t.Fatalf("records = %+v", recs)
	}
	want := [][2]float64{{5, 6}, {5, 7}, {5, 8}}
	for i, w := range want {
		pc := recs[0].Poses[i]
		if math.Abs(pc.After.X-w[0]) > 1e-9 || math.Abs(pc.After.Y-w[1]) > 1e-9 {
			t.Fatalf("pose %d after = (%v,%v), want (%v,%v)", i, pc.After.X, pc.After.Y, w[0], w[1])
		}
		if math.Abs(normalizeAngle(pc.After.Heading-math.Pi/2)) > 1e-9 {
			t.Fatalf("pose %d after heading = %v, want π/2", i, pc.After.Heading)
		}
	}
}

// 逐帧几何核对：锚点校正后位姿必须就是目标，其余帧必须与锚点一起平移旋转。
// 锚点或末帧被改动、只有锚点与末帧正确而中间一帧位置偏离或单独转向、整条
// 记录被整体平移而脱离目标，都按损坏拒绝；超过 1e-9 相对容差的偏离同样
// 拒绝。
func TestOpenRejectsInconsistentCorrectedGeometry(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"anchor position differs from target", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.X += 0.5
		}},
		{"anchor heading differs from target", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.Heading += 0.01
		}},
		{"middle frame position deviates", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.Y += 1e-6
		}},
		{"middle frame turns alone", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.Heading -= 1e-6
		}},
		{"last frame position wrong", func(fd *fileData) {
			fd.Corrections[0].Poses[2].After.X += 0.25
		}},
		{"last frame heading wrong", func(fd *fileData) {
			fd.Corrections[0].Poses[2].After.Heading = 0
		}},
		{"whole record shifted off target", func(fd *fileData) {
			// 三帧之间的相对关系保持，但锚点不再落在目标位置。
			for i := range fd.Corrections[0].Poses {
				fd.Corrections[0].Poses[i].After.X += 1
			}
		}},
		{"deviation beyond scaled tolerance", func(fd *fileData) {
			// 该帧坐标约 5，容差约 5e-9；1e-8 的偏离必须拒绝。
			fd.Corrections[0].Poses[1].After.X += 1e-8
		}},
		{"heading deviation beyond tolerance", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.Heading += 2e-9
		}},
		{"target moved away from recorded anchor", func(fd *fileData) {
			// 记录的快照内部一致，但目标不再描述这次校正。
			fd.Corrections[0].Target.X = 9
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildRotatedCorrectedMap(t, path)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 容差内的微小偏差不误判：坐标误差不超过 1e-9 乘以 1、预期值绝对值、保存
// 值绝对值三者中的最大值，朝向误差不超过 1e-9 弧度。
func TestOpenAcceptsGeometryWithinTolerance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildRotatedCorrectedMap(t, path)
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Poses[1].After.X += 1e-10
		fd.Corrections[0].Poses[2].After.Heading += 5e-10
	})
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
}

// 只有锚点一帧的记录同样核对几何：锚点校正后位姿必须就是目标。
func TestOpenChecksSingleFrameCorrectionGeometry(t *testing.T) {
	build := func(t *testing.T, path string) {
		t.Helper()
		m, err := Create(path, baseConfig())
		if err != nil {
			t.Fatal(err)
		}
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 100, DX: 1},
			{Time: 200, DX: 1},
		}}); err != nil {
			t.Fatal(err)
		}
		if _, err := m.Correct(Correction{ID: "c1", Anchor: 200,
			Target: CorrectionTarget{X: 5, Y: 6, Heading: 0.3, Variance: 1}}); err != nil {
			t.Fatal(err)
		}
		if err := m.Close(); err != nil {
			t.Fatal(err)
		}
	}

	// 未篡改：唯一一帧校正后位姿即目标，正常打开。
	path := filepath.Join(t.TempDir(), "m.pose")
	build(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	recs, _ := m.Corrections()
	if len(recs) != 1 || len(recs[0].Poses) != 1 {
		t.Fatalf("records = %+v", recs)
	}
	a := recs[0].Poses[0].After
	if a.X != 5 || a.Y != 6 || math.Abs(a.Heading-0.3) > 1e-9 {
		t.Fatalf("after = %+v, want (5,6,0.3)", a)
	}
	m.Close()

	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"position differs from target", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.Y += 1
		}},
		{"heading differs from target", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.Heading += 1e-6
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			build(t, path)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 只调整方差的恒等校正：校正前后位置与朝向必须完全相同，任何一帧位置或
// 朝向被改动都按损坏拒绝。
func TestOpenRejectsGeometryTamperInVarianceOnlyRecord(t *testing.T) {
	build := func(t *testing.T, path string) {
		t.Helper()
		m, err := Create(path, baseConfig())
		if err != nil {
			t.Fatal(err)
		}
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 100, DX: 1},
			{Time: 200, DX: 1},
			{Time: 300, DX: 1},
		}}); err != nil {
			t.Fatal(err)
		}
		anchor, _ := m.PoseAt(100)
		if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
			Target: CorrectionTarget{X: anchor.X, Y: anchor.Y, Heading: anchor.Heading, Variance: 1}}); err != nil {
			t.Fatal(err)
		}
		if err := m.Close(); err != nil {
			t.Fatal(err)
		}
	}
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"middle frame moved", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.X += 1e-6
		}},
		{"last frame turned", func(fd *fileData) {
			fd.Corrections[0].Poses[2].After.Heading += 1e-6
		}},
		{"anchor moved", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.Y -= 0.5
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			build(t, path)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 坐标横跨 ±1e308 的恒等校正重开时正常接受（不能为套用旋转平移公式把
// 相对偏移相减溢出）；容差随坐标量级缩放，中间帧被改成 1e300 仍按损坏
// 拒绝。
func TestOpenGeometryCheckHugeSpanIdentity(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
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
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	m2.Close()

	// 中间帧校正前位于 0，被改成 1e300：相对 0 的偏离远超缩放后的容差。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Poses[1].After.X = 1e300
	})
	assertOpenCorrupt(t, path)
}

// 几何容差与路标合并距离无关：合并距离设置得再大，超过 1e-9 容差的校正
// 后位置矛盾也不能被接受。
func TestOpenGeometryToleranceIgnoresMergeDistance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := baseConfig()
	cfg.MergeDistance = 1e6
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
		{Time: 300, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: 6, Heading: math.Pi / 2, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 0.001 远在合并距离 1e6 之内，但超出 1e-9 几何容差：必须拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Poses[1].After.X += 0.001
	})
	assertOpenCorrupt(t, path)
}

// 目标朝向未归一时按归一后的结果核对：提交 3π 与提交 -π 描述同一朝向，
// 记录保留原始目标，重开正常接受。
func TestOpenAcceptsUnnormalizedTargetHeading(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: 6, Heading: 3 * math.Pi, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m2.Close()
	recs, _ := m2.Corrections()
	if len(recs) != 1 || recs[0].Target.Heading != 3*math.Pi {
		t.Fatalf("records = %+v", recs)
	}
	for i, pc := range recs[0].Poses {
		if pc.After.Heading != -math.Pi {
			t.Fatalf("pose %d after heading = %v, want -π", i, pc.After.Heading)
		}
	}
}

// 某条校正范围内含缺少逐帧依据的 null 旧帧时，保留既有打开规则：该记录的
// 几何矛盾不核对、原样保留；同一文件中另一条依据完整的记录仍要核对，它
// 出现几何矛盾时整份文件按损坏拒绝。
func TestOpenSkipsGeometryCheckForNullBasisRecords(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildRotatedCorrectedMap(t, path)

	// 把 t=200 的逐帧依据改为缺失，并把 c1 的校正后几何改成任意矛盾值：
	// 该记录范围含 null 旧帧，按旧规则放行、原样保留。
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[1] = nil
		for i := range fd.Corrections[0].Poses {
			fd.Corrections[0].Poses[i].After.X = 100
			fd.Corrections[0].Poses[i].After.Heading = 2
		}
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("record spanning null basis frame should keep legacy open rules: %v", err)
	}

	// 之后追加依据完整的新帧，并只对该帧提交一帧校正（范围不含 null 帧）。
	if _, err := m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m2.Correct(Correction{ID: "c2", Anchor: 400,
		Target: CorrectionTarget{X: 9, Y: 9, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// c1 几何矛盾但依据不完整（跳过），c2 依据完整且合法：正常打开，c1 原样。
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	recs, _ := m3.Corrections()
	if len(recs) != 2 || recs[0].Poses[0].After.X != 100 || recs[1].Poses[0].After.X != 9 {
		t.Fatalf("records = %+v", recs)
	}
	m3.Close()

	// c2 的校正后位置与目标矛盾：其余记录被跳过不影响，整份文件仍拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[1].Poses[0].After.X = 10
	})
	assertOpenCorrupt(t, path)
}
