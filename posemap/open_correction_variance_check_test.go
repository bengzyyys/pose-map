package posemap

import (
	"path/filepath"
	"testing"
)

// buildVarianceCorrectedMap 构造三帧（t=100、200、300）并对 t=100 做过一次
// 校正的地图：进入三帧的原运动方差依次为 0.25、0.25、0.5，目标方差为 1，
// 故三帧校正后方差依次为 1、1.25、1.75（锚点自身的 0.25 不再累加）。
func buildVarianceCorrectedMap(t *testing.T, path string) {
	t.Helper()
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
		{Time: 300, DX: 1, MoveVariance: 0.5},
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
}

// 正常文件：校正后方差逐帧等于“目标方差 + 锚点之后原运动方差累计”，
// 锚点自身的运动方差不重复累加；重开后记录与查询都保持该结果。
func TestOpenAcceptsCorrectedVarianceAccumulation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildVarianceCorrectedMap(t, path)
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
	want := []float64{1, 1.25, 1.75}
	for i, v := range want {
		if got := recs[0].Poses[i].After.Variance; got != v {
			t.Fatalf("pose %d after variance = %v, want %v", i, got, v)
		}
	}
}

// 只有锚点一帧的记录：校正后方差等于目标方差即可；被篡改即损坏。
func TestOpenChecksSingleFrameCorrectionVariance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.4},
		{Time: 200, DX: 1, MoveVariance: 0.6},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 200,
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 未篡改：唯一一帧校正后方差恰为目标方差，正常打开。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	recs, _ := m2.Corrections()
	if len(recs) != 1 || recs[0].Poses[0].After.Variance != 2 {
		t.Fatalf("records = %+v", recs)
	}
	m2.Close()

	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Poses[0].After.Variance = 2.6
	})
	assertOpenCorrupt(t, path)
}

// 逐帧核对：锚点与末帧正确但中间帧错误、或任意一帧与累计规则不符，都按
// 损坏拒绝；以校正前方差为起点等错误写法同样被拒。
func TestOpenRejectsInconsistentCorrectedVariances(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"anchor variance differs from target", func(fd *fileData) {
			fd.Corrections[0].Poses[0].After.Variance = 1.1
		}},
		{"middle frame wrong while anchor and last are right", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.Variance = 1.3
		}},
		{"last frame wrong", func(fd *fileData) {
			fd.Corrections[0].Poses[2].After.Variance = 1.7
		}},
		{"anchor adds its own motion variance once more", func(fd *fileData) {
			// 1.25 = 目标 1 + 锚点自身运动方差 0.25：锚点不得再加一次。
			fd.Corrections[0].Poses[0].After.Variance = 1.25
		}},
		{"accumulated from pre-correction variance", func(fd *fileData) {
			// 以校正前方差（初始 0.5 累计到 0.75/1.0/1.5）为起点是错的：
			// 必须以目标方差 1 为起点。
			p := fd.Corrections[0].Poses
			p[0].After.Variance, p[1].After.Variance, p[2].After.Variance = 0.75, 1.0, 1.5
		}},
		{"negative corrected variance", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.Variance = -0.01
		}},
		{"negative target variance", func(fd *fileData) {
			fd.Corrections[0].Target.Variance = -0.5
		}},
		{"saved motion variance changed", func(fd *fileData) {
			// 锚点之后第一帧（t=200）的原运动方差被改为 0.3：真实累计
			// 应为 1.3，与保存的 1.25 矛盾。
			fd.Sources[1].MoveVariance = 0.3
		}},
		{"negative motion variance in range", func(fd *fileData) {
			fd.Sources[2].MoveVariance = -0.1
		}},
		{"accumulation overflows to non-finite while saved values are finite", func(fd *fileData) {
			// 目标与保存的校正后方差都有限，但 1 + 1e308 必为 +Inf，
			// 累计结果非有限，按损坏拒绝，不能拿保存的有限值放行。
			fd.Sources[1].MoveVariance = 1e308
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildVarianceCorrectedMap(t, path)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 目标方差与参与累计的运动方差全为零合法：每帧校正后方差都是 0。
func TestOpenAcceptsZeroCorrectedVariances(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
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
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 0}}); err != nil {
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
	for i, pc := range recs[0].Poses {
		if pc.After.Variance != 0 {
			t.Fatalf("pose %d after variance = %v, want 0", i, pc.After.Variance)
		}
	}

	// 同样全零依据下，中间帧被改成非零仍按损坏拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Poses[1].After.Variance = 0.0001
	})
	assertOpenCorrupt(t, path)
}

// 只调整方差、未改变位置与朝向的合法校正正常打开，记录保留原始前后快照
// （前方差仍为校正前累计值，位置朝向前后相同）。
func TestOpenAcceptsVarianceOnlyCorrectionSnapshots(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
		{Time: 300, DX: 1, MoveVariance: 0.5},
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

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m2.Close()
	recs, _ := m2.Corrections()
	if len(recs) != 1 || len(recs[0].Poses) != 3 {
		t.Fatalf("records = %+v", recs)
	}
	beforeVar := []float64{0.75, 1.0, 1.5} // 初始方差 0.5 按原运动方差累计
	afterVar := []float64{1, 1.25, 1.75}
	for i, pc := range recs[0].Poses {
		if pc.Before.X != pc.After.X || pc.Before.Y != pc.After.Y || pc.Before.Heading != pc.After.Heading {
			t.Fatalf("pose %d geometry changed in variance-only record: %+v -> %+v", i, pc.Before, pc.After)
		}
		if pc.Before.Variance != beforeVar[i] {
			t.Fatalf("pose %d before variance = %v, want snapshot %v", i, pc.Before.Variance, beforeVar[i])
		}
		if pc.After.Variance != afterVar[i] {
			t.Fatalf("pose %d after variance = %v, want %v", i, pc.After.Variance, afterVar[i])
		}
	}
}

// 核对范围以每条记录自己的锚点与结束时间为准：旧记录之后追加的帧（即使
// 携带很大的运动方差）不属于旧记录的累计范围，不影响旧记录合法性。
func TestOpenCorrectionVarianceRangeExcludesLaterFrames(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
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
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	// 旧记录结束于 300；之后追加一帧运动方差巨大的新帧。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 100},
	}}); err != nil {
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
	if len(recs) != 1 || recs[0].EndTime != 300 || len(recs[0].Poses) != 3 {
		t.Fatalf("records = %+v", recs)
	}
	for i, pc := range recs[0].Poses {
		if pc.After.Variance != 1 {
			t.Fatalf("pose %d after variance = %v, later frame leaked into old record", i, pc.After.Variance)
		}
	}
}

// 某条校正范围内含缺少逐帧依据的 null 旧帧时，保留既有打开规则：不推测
// 运动方差，即使该记录的校正后方差明显矛盾也不拒绝；但同一文件中另一条
// 依据完整的记录仍要核对，它出现矛盾时整份文件按损坏拒绝。
func TestOpenSkipsVarianceCheckForNullBasisRecords(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
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
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 把 t=200 的逐帧依据改为缺失，并把 c1 的方差与目标改成任意矛盾值：
	// 该记录范围含 null 旧帧，按旧规则放行、原样保留。
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[1] = nil
		c := &fd.Corrections[0]
		c.Target.Variance = -9
		for i := range c.Poses {
			c.Poses[i].After.Variance = -5
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

	// c1 矛盾但依据不完整（跳过），c2 依据完整且合法：正常打开，c1 原样。
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	recs, _ := m3.Corrections()
	if len(recs) != 2 || recs[0].Target.Variance != -9 || recs[1].Poses[0].After.Variance != 2 {
		t.Fatalf("records = %+v", recs)
	}
	m3.Close()

	// c2 的校正后方差与目标矛盾：其余记录被跳过不影响，整份文件仍拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[1].Poses[0].After.Variance = 2.5
	})
	assertOpenCorrupt(t, path)
}
