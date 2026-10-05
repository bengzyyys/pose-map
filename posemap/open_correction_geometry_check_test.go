package posemap

import (
	"errors"
	"math"
	"path/filepath"
	"testing"
)

// buildGeometryMap 构造一条带平移与旋转的三帧校正记录（无路标观测，排除
// 路标核对的干扰）。平移按运动前朝向换算到地图坐标，朝向增量在运动后
// 归一：
//
//	p100 校正前 (1,0,h=π/2)
//	p200 校正前 (3,0,h=π/2)
//	p300 校正前 (3,2,h=π/2)
//
// 锚点 100 校正到目标 (10,20,h=0)，整体旋转 δ=-π/2：
//
//	p100 校正后 (10,20,h=0)
//	p200 校正前相对锚点 (2,0) 旋转后 (0,-2) → (10,18,h=0)
//	p300 校正前相对锚点 (2,2) 旋转后 (2,-2) → (12,18,h=0)
func buildGeometryMap(t *testing.T, path string, merge float64) {
	t.Helper()
	m, err := Create(path, Config{
		InitialTime: 0, InitialX: 0, InitialY: 0, InitialHeading: 0,
		InitialVariance: 1, MaxInterval: 100, MergeDistance: merge,
	})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		// 运动前朝向 0：前进 1 到 (1,0)，随后朝向转到 π/2。
		{Time: 100, DX: 1, DY: 0, DHeading: math.Pi / 2},
		// 运动前朝向 π/2：自身 (0,-2) 即地图 (2,0)，到 (3,0)。
		{Time: 200, DX: 0, DY: -2},
		// 运动前朝向 π/2：自身 (2,0) 即地图 (0,2)，到 (3,2)。
		{Time: 300, DX: 2, DY: 0},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 未篡改的旋转+平移校正记录正常打开，且记录内容与落盘快照逐位一致。
func TestOpenAcceptsCorrectionGeometry(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildGeometryMap(t, path, 10)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	recs, _ := m.Corrections()
	wantAfter := []struct {
		x, y, h float64
	}{
		{10, 20, 0},
		{10, 18, 0},
		{12, 18, 0},
	}
	if len(recs) != 1 || len(recs[0].Poses) != 3 {
		t.Fatalf("records = %+v", recs)
	}
	for i, w := range wantAfter {
		got := recs[0].Poses[i].After
		if got.X != w.x || got.Y != w.y || got.Heading != w.h {
			t.Fatalf("pose %d after = (%v,%v,%v), want (%v,%v,%v)", i, got.X, got.Y, got.Heading, w.x, w.y, w.h)
		}
	}
}

// 每一帧都必须符合同一次平移和旋转：锚点、中间帧、末帧的校正后位置或朝向
// 被改成另一个有限值（时间、方差、路标列表仍满足既有检查）都按损坏拒绝；
// 只有锚点与末帧正确而中间一帧位置偏离或单独转向也不例外。
func TestOpenRejectsInconsistentCorrectionGeometry(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"anchor corrected x changed", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.X = 10.5
		}},
		{"anchor corrected y changed", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.Y = 20.5
		}},
		{"anchor corrected heading changed", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.Heading = 0.01
		}},
		{"middle frame position drifted while anchor and last stay right", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.X = 10.5
		}},
		{"middle frame y drifted", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.Y = 17.5
		}},
		{"middle frame turned independently", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.Heading = 0.3
		}},
		{"last frame position drifted", func(fd *fileData) {
			fd.Corrections[0].Poses[2].After.X = 12.2
		}},
		{"last frame turned independently", func(fd *fileData) {
			fd.Corrections[0].Poses[2].After.Heading = -0.1
		}},
		// 中间帧的“校正前”快照被改而“校正后”不动：该帧相对锚点的校正前
		// 位置关系变了，保存的校正后值不再是同一次刚体重定位的结果。
		{"middle frame before snapshot changed", func(fd *fileData) {
			fd.Corrections[0].Poses[1].Before.X = 3.5
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildGeometryMap(t, path, 10)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 范围只有锚点一帧时几何核对同样适用：锚点校正后位置/朝向必须等于归一后
// 的目标，目标被改而校正后值不随之更新也算矛盾；被改成另一个有限值即
// 损坏。等价但未归一的目标朝向合法。
func TestOpenChecksSingleFrameCorrectionGeometry(t *testing.T) {
	build := func(t *testing.T) string {
		path := filepath.Join(t.TempDir(), "m.pose")
		m, err := Create(path, Config{
			InitialTime: 0, MaxInterval: 100, MergeDistance: 10,
		})
		if err != nil {
			t.Fatal(err)
		}
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 100, DX: 1}, {Time: 200, DX: 1},
		}}); err != nil {
			t.Fatal(err)
		}
		if _, err := m.Correct(Correction{ID: "c1", Anchor: 200,
			Target: CorrectionTarget{X: 5, Y: 6, Heading: 0.2, Variance: 2}}); err != nil {
			t.Fatal(err)
		}
		if err := m.Close(); err != nil {
			t.Fatal(err)
		}
		return path
	}
	cases := []struct {
		name   string
		fn     func(fd *fileData)
		wantOK bool
	}{
		{"after x drifted", func(fd *fileData) { fd.Corrections[0].Poses[0].After.X = 5.001 }, false},
		{"after y drifted", func(fd *fileData) { fd.Corrections[0].Poses[0].After.Y = 5.999 }, false},
		{"after heading drifted", func(fd *fileData) { fd.Corrections[0].Poses[0].After.Heading = 0.21 }, false},
		{"target position changed without updating after", func(fd *fileData) {
			fd.Corrections[0].Target.X = 5.5
		}, false},
		{"target heading changed without updating after", func(fd *fileData) {
			fd.Corrections[0].Target.Heading = 0.5
		}, false},
		// 目标朝向写成等价但未归一的 0.2+2π，校正后保存归一值 0.2：合法。
		{"equivalent unnormalized target heading accepted", func(fd *fileData) {
			fd.Corrections[0].Target.Heading = 0.2 + 2*math.Pi
		}, true},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := build(t)
			tamperFile(t, path, tc.fn)
			if tc.wantOK {
				m, err := Open(path)
				if err != nil {
					t.Fatalf("equivalent normalized heading should open: %v", err)
				}
				m.Close()
				return
			}
			assertOpenCorrupt(t, path)
		})
	}
}

// 坐标容差为 1e-9 乘以 1、|期望|、|保存| 三者最大值：坐标量级约 10 时容差
// 约 1e-8，半档误差放行、两倍档误差拒绝；朝向容差为 1e-9 弧度的绝对最短
// 角度差，半档放行、两倍档拒绝。放行时原样返回被保存的值，不借容差改写
// 快照。
func TestOpenCorrectionGeometryTolerance(t *testing.T) {
	// 坐标：中间帧期望 x=10，容差 ~1e-8。
	coordCases := []struct {
		name    string
		perturb float64
		wantOK  bool
	}{
		{"within half tolerance", 5e-9, true},
		{"beyond tolerance", 2e-8, false},
	}
	for _, tc := range coordCases {
		t.Run("coord "+tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildGeometryMap(t, path, 10)
			tamperFile(t, path, func(fd *fileData) {
				fd.Corrections[0].Poses[1].After.X = 10 + tc.perturb
			})
			if tc.wantOK {
				m, err := Open(path)
				if err != nil {
					t.Fatalf("within tolerance should open: %v", err)
				}
				recs, _ := m.Corrections()
				if got := recs[0].Poses[1].After.X; got != 10+tc.perturb {
					t.Fatalf("snapshot rewritten to %v, want saved %v", got, 10+tc.perturb)
				}
				m.Close()
			} else {
				assertOpenCorrupt(t, path)
			}
		})
	}

	// 朝向：中间帧期望朝向 0，容差为绝对 1e-9 弧度。
	angleCases := []struct {
		name    string
		perturb float64
		wantOK  bool
	}{
		{"within half tolerance", 5e-10, true},
		{"beyond tolerance", 2e-9, false},
	}
	for _, tc := range angleCases {
		t.Run("angle "+tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildGeometryMap(t, path, 10)
			tamperFile(t, path, func(fd *fileData) {
				fd.Corrections[0].Poses[1].After.Heading = tc.perturb
			})
			if tc.wantOK {
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
}

// 大坐标下容差随量级放宽：坐标量级 1e10 时容差约 10，偏移 5 放行、偏移 20
// 拒绝。
func TestOpenCorrectionGeometryRelativeToleranceLargeCoordinates(t *testing.T) {
	build := func(t *testing.T) string {
		path := filepath.Join(t.TempDir(), "m.pose")
		m, err := Create(path, Config{
			InitialTime: 0, MaxInterval: 100, MergeDistance: 10,
		})
		if err != nil {
			t.Fatal(err)
		}
		// p100=(1,0,h=0) p200=(3,0,h=0) p300=(5,0,h=0)，无旋转。
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 100, DX: 1},
			{Time: 200, DX: 2},
			{Time: 300, DX: 2},
		}}); err != nil {
			t.Fatal(err)
		}
		if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
			Target: CorrectionTarget{X: 1e10, Y: 0, Heading: 0, Variance: 1}}); err != nil {
			t.Fatal(err)
		}
		if err := m.Close(); err != nil {
			t.Fatal(err)
		}
		return path
	}
	for _, tc := range []struct {
		name   string
		shift  float64
		wantOK bool
	}{
		{"shift 5 within relative tolerance", 5, true},
		{"shift 20 beyond relative tolerance", 20, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			path := build(t)
			fd := readFileData(t, path)
			base := fd.Corrections[0].Poses[1].After.X // 中间帧校正后 x ≈ 1e10+2
			fd.Corrections[0].Poses[1].After.X = base + tc.shift
			writeFileDataRaw(t, path, &fd)
			if tc.wantOK {
				m, err := Open(path)
				if err != nil {
					t.Fatalf("within relative tolerance should open: %v", err)
				}
				m.Close()
			} else {
				assertOpenCorrupt(t, path)
			}
		})
	}
}

// 超过容差的几何矛盾不能因为路标合并距离很大而被接受：合并距离只用于导入/
// 校正时的观测接纳，不是记录内部几何偏差的容许范围。
func TestOpenCorrectionGeometryRejectedRegardlessOfMergeDistance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildGeometryMap(t, path, 1e9)
	tamperFile(t, path, func(fd *fileData) {
		// 中间帧位置偏离 0.5：远小于 1e9 的合并距离，但远超 1e-8 容差。
		fd.Corrections[0].Poses[1].After.X = 10.5
	})
	assertOpenCorrupt(t, path)
}

// 朝向按最短角度差比较、容差 1e-9 弧度：归一化使 -π 与 +π 指向同一方向，
// 跨越 ±π 边界但物理等价（差在容差内）的写法放行；跨边界后差 1e-8 拒绝。
// 这里用 δ=-π 的校正，使锚点与其余帧的校正后朝向都为 -π。
func TestOpenCorrectionGeometryShortestAngle(t *testing.T) {
	build := func(t *testing.T) string {
		path := filepath.Join(t.TempDir(), "m.pose")
		m, err := Create(path, Config{
			InitialTime: 0, MaxInterval: 100, MergeDistance: 10,
		})
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
			Target: CorrectionTarget{X: 5, Y: 0, Heading: math.Pi, Variance: 1}}); err != nil {
			t.Fatal(err)
		}
		if err := m.Close(); err != nil {
			t.Fatal(err)
		}
		return path
	}
	for _, tc := range []struct {
		name   string
		fn     func(fd *fileData)
		wantOK bool
	}{
		{"minus pi and plus pi are the same direction", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.Heading = math.Pi
		}, true},
		{"seam gap within tolerance", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.Heading = math.Pi - 5e-10
		}, true},
		{"seam gap beyond tolerance", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.Heading = math.Pi - 1e-8
		}, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			path := build(t)
			tamperFile(t, path, tc.fn)
			if tc.wantOK {
				m, err := Open(path)
				if err != nil {
					t.Fatalf("shortest-angle equivalent heading should open: %v", err)
				}
				m.Close()
			} else {
				assertOpenCorrupt(t, path)
			}
		})
	}
}

// 只调方差、位置与朝向都不变的恒等校正，在轨迹坐标横跨 ±1e308 时仍正常
// 打开：几何核对与 Correct 走同一恒等分支，不重算会溢出的相对锚点偏移。
func TestOpenAcceptsIdentityCorrectionGeometryHugeSpan(t *testing.T) {
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
		Target: CorrectionTarget{X: -1e308, Y: 0, Heading: 0, Variance: 4}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("identity correction across ±1e308 must open: %v", err)
	}
	m2.Close()

	// 恒等记录中一帧的校正后位置被改成另一个有限值仍按几何矛盾拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Poses[1].After.X = 1.0
	})
	assertOpenCorrupt(t, path)
}

// 核对只针对记录自身保存的校正前后值与目标：旧记录之后对重叠范围再次校正、
// 或在结束时间之后追加轨迹，都不改变旧记录保存的快照，旧记录仍合法打开。
func TestOpenCorrectionGeometryUsesRecordSnapshotsOnly(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildGeometryMap(t, path, 10)

	// 重开后把锚点 100 再校正到另一目标，并追加一帧：旧记录 c1 的快照与
	// 当前轨迹不再一致，但其自身描述的仍是同一次平移和旋转。
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 100,
		Target: CorrectionTarget{X: -3, Y: -4, Heading: math.Pi / 4, Variance: 7}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{{Time: 400, DX: 1}}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("older record must stay valid after recorrection and appended frames: %v", err)
	}
	defer m2.Close()
	recs, _ := m2.Corrections()
	if len(recs) != 2 || recs[0].ID != "c1" || recs[1].ID != "c2" {
		t.Fatalf("records = %+v", recs)
	}
	if got := recs[0].Poses[1].After; got.X != 10 || got.Y != 18 {
		t.Fatalf("old record snapshot changed: %+v", got)
	}
}

// 校正范围内含缺少逐帧依据的 null 旧帧时，几何核对与方差核对一样保留旧
// 打开规则：即使记录的几何明显矛盾也不补造依据、不拒绝；同一文件中另一条
// 依据完整的记录仍须核对，其几何矛盾时整份文件按损坏拒绝。
func TestOpenSkipsGeometryCheckForNullBasisRecords(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, Config{
		InitialTime: 0, MaxInterval: 100, MergeDistance: 10,
	})
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
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// t=200 的逐帧依据置空，并把 c1 中间帧改成任意几何矛盾值：范围含 null
	// 旧帧，沿用旧规则放行，记录原样保留。
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[1] = nil
		fd.Corrections[0].Poses[1].After.X = 500
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("record spanning null basis frame keeps legacy open rules: %v", err)
	}

	// 追加依据完整的新帧，并只对该帧提交一帧校正（范围不含 null 帧）。
	if _, err := m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{{Time: 400, DX: 1}}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m2.Correct(Correction{ID: "c2", Anchor: 400,
		Target: CorrectionTarget{X: 9, Y: 9, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// c1 几何矛盾但依据不完整（跳过），c2 依据完整且合法：正常打开。
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	recs, _ := m3.Corrections()
	if len(recs) != 2 || recs[0].Poses[1].After.X != 500 ||
		recs[1].Poses[0].After.X != 9 {
		t.Fatalf("records = %+v", recs)
	}
	m3.Close()

	// c2 唯一一帧几何与目标矛盾：其余记录被跳过不影响，整份文件仍拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[1].Poses[0].After.X = 9.5
	})
	assertOpenCorrupt(t, path)
}

// 几何矛盾时 Open 返回的错误可被 errors.Is 识别为 ErrCorrupt。
func TestOpenGeometryErrorIsErrCorrupt(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildGeometryMap(t, path, 10)
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Poses[2].After.Heading = 1
	})
	_, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want errors.Is ErrCorrupt", err)
	}
}
