package posemap

import (
	"errors"
	"math"
	"path/filepath"
	"testing"
)

// buildVarianceCorrectedMap 构造三帧 t=100、200、300 的地图，运动方差依次
// 为 0.25、0.25、0.5（锚点 t=100 自身的入边运动方差不参与累计）；对 t=100
// 提交目标方差 1 的校正后，三帧校正后方差依次为 1、1.25、1.75（校正前为
// 0.75、1.0、1.5）。
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
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 未篡改时按规则打开：三帧校正后方差依次为 1、1.25、1.75，且校正前快照
// 保留提交时的旧值（0.75、1.0、1.5）；重复提交原校正仍返回这份记录。
func TestOpenCorrectedVarianceAccepted(t *testing.T) {
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
	if len(recs) != 1 {
		t.Fatalf("records = %+v", recs)
	}
	wantBefore := []float64{0.75, 1.0, 1.5}
	wantAfter := []float64{1, 1.25, 1.75}
	for i, pc := range recs[0].Poses {
		if pc.Before.Variance != wantBefore[i] {
			t.Fatalf("pose %d before variance = %v, want %v (snapshot must be retained)", i, pc.Before.Variance, wantBefore[i])
		}
		if pc.After.Variance != wantAfter[i] {
			t.Fatalf("pose %d after variance = %v, want %v", i, pc.After.Variance, wantAfter[i])
		}
	}
	// 重复提交原校正：返回保存的记录，当前查询不受影响。
	again, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}})
	if err != nil {
		t.Fatalf("duplicate correction: %v", err)
	}
	if len(again.Poses) != 3 || again.Poses[2].After.Variance != 1.75 {
		t.Fatalf("duplicate correction record = %+v", again)
	}
}

// 目标方差与参与累计的运动方差为零仍合法：校正后方差全部为 0。
func TestOpenCorrectedVarianceAllZeroAccepted(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0},
		{Time: 200, DX: 1, MoveVariance: 0},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 0}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open zero variances: %v", err)
	}
	defer m2.Close()
	recs, _ := m2.Corrections()
	for i, pc := range recs[0].Poses {
		if pc.After.Variance != 0 {
			t.Fatalf("pose %d after variance = %v, want 0", i, pc.After.Variance)
		}
	}
}

// 任何一帧的校正后方差不符合累计规则都按损坏拒绝：不能只凭锚点与末帧正确
// 就接受中间错误的记录，也不能以校正前方差为起点、或把锚点运动方差加两次。
func TestOpenRejectsInconsistentCorrectedVariance(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"anchor variance differs from target", func(fd *fileData) {
			// 只改目标方差，记录内锚点校正后方差仍是 1：锚点即不匹配。
			fd.Corrections[0].Target.Variance = 2
		}},
		{"middle frame wrong while anchor and last stay correct", func(fd *fileData) {
			// 锚点 1、末帧 1.75 都不动，只把 t=200 的 1.25 改成 1.5。
			fd.Corrections[0].Poses[1].After.Variance = 1.5
		}},
		{"last frame variance wrong", func(fd *fileData) {
			fd.Corrections[0].Poses[2].After.Variance = 2
		}},
		{"anchor motion variance added twice", func(fd *fileData) {
			// 锚点保持 1；把锚点入边 0.25 错加进后续累计：t=200 为 1.5、
			// t=300 为 2（正确值 1.25、1.75）。
			fd.Corrections[0].Poses[1].After.Variance = 1.5
			fd.Corrections[0].Poses[2].After.Variance = 2
		}},
		{"started from pre-correction variance", func(fd *fileData) {
			// 以校正前方差 0.75 为起点逐帧累加：0.75、1.0、1.5。
			fd.Corrections[0].Poses[0].After.Variance = 0.75
			fd.Corrections[0].Poses[1].After.Variance = 1.0
			fd.Corrections[0].Poses[2].After.Variance = 1.5
		}},
		{"negative target variance", func(fd *fileData) {
			fd.Corrections[0].Target.Variance = -0.01
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

// 只有锚点一帧的记录：校正后方差等于目标方差即合法，不等即损坏。
func TestOpenSingleFrameCorrectedVariance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.5},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 200, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open single-frame correction: %v", err)
	}
	m2.Close()

	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Poses[0].After.Variance = 1.25
	})
	assertOpenCorrupt(t, path)
}

// 非有限目标方差、负或非有限的参与运动方差、累计结果溢出为非有限，都按
// 损坏拒绝。这些数值无法经 JSON 往返（NaN/Inf 不能被 encoding/json 编
// 码），直接在解码后的 fileData 上调用 validateLoaded 核对。
func TestOpenRejectsNonFiniteCorrectedVariance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildVarianceCorrectedMap(t, path)

	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"NaN target variance", func(fd *fileData) {
			fd.Corrections[0].Target.Variance = math.NaN()
		}},
		{"+Inf target variance", func(fd *fileData) {
			fd.Corrections[0].Target.Variance = math.Inf(1)
		}},
		{"negative motion variance in range", func(fd *fileData) {
			// t=100 到 t=200 的运动（锚点后的第一份累计贡献）为负。
			fd.Sources[1].MoveVariance = -0.1
		}},
		{"+Inf motion variance in range", func(fd *fileData) {
			fd.Sources[1].MoveVariance = math.Inf(1)
		}},
		{"accumulation overflows to infinity", func(fd *fileData) {
			// 目标与锚点后值均为有限的 1e308，t=200 的运动方差 1e308 使
			// 累计成为 +Inf：即使记录里写着有限值，也必须在比较前拒绝。
			fd.Corrections[0].Target.Variance = 1e308
			fd.Corrections[0].Poses[0].After.Variance = 1e308
			fd.Sources[1].MoveVariance = 1e308
			fd.Corrections[0].Poses[1].After.Variance = 1.5e308
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			fd := readFileData(t, path)
			tc.fn(&fd)
			if err := validateLoaded(&fd); !errors.Is(err, ErrCorrupt) {
				t.Fatalf("validateLoaded err = %v, want ErrCorrupt", err)
			}
		})
	}
}

// 范围含缺少逐帧依据的 null 旧帧时保留旧打开规则：不推测运动方差、不把
// 未知贡献算成零，校正后方差与目标明显矛盾也不拒绝。
func TestOpenSkipsVarianceCheckWhenRangeLacksBasis(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "K"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	// 追加一帧有完整依据的新帧，再手写一条覆盖 t=100..300（含 null 旧帧）
	// 的校正记录，目标方差与记录内校正后方差故意完全矛盾。
	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, MoveVariance: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}
	tamperFile(t, path, func(fd *fileData) {
		cr := corrRecJSON{ID: "span", Anchor: 100, EndTime: 300,
			Target: CorrectionTarget{X: 5, Y: 6, Variance: 99}}
		for j := 1; j < len(fd.Trajectory); j++ {
			cr.Poses = append(cr.Poses, poseChangeJSON{Before: fd.Trajectory[j], After: fd.Trajectory[j]})
		}
		fd.Corrections = append(fd.Corrections, cr)
	})
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("range lacking per-frame basis keeps legacy open rules: %v", err)
	}
	defer m3.Close()

	// 同一文件中另一条依据完整的记录仍要核对：在 t=300 做恒等校正后，把
	// 该记录的目标方差改得与锚点校正后方差不一致，必须拒绝整份文件——
	// 含 null 旧帧的旧记录不能替它开脱。
	cur, err := m3.PoseAt(300)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m3.Correct(Correction{ID: "cnew", Anchor: 300, Target: CorrectionTarget{
		X: cur.X, Y: cur.Y, Heading: cur.Heading, Variance: cur.Variance,
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m3.Close(); err != nil {
		t.Fatal(err)
	}
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[1].Target.Variance = 7
	})
	assertOpenCorrupt(t, path)
}

// 只调整方差、未改变位置与朝向的合法恒等校正正常打开，前后位置/朝向快照
// 保持原值且保留在记录中。
func TestOpenVarianceOnlyCorrectionKeepsSnapshots(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, Config{
		InitialTime: 0, InitialX: 1, InitialY: 2, InitialHeading: 0,
		InitialVariance: 0.5, MaxInterval: 1000, MergeDistance: 1.0,
	})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25, Observations: []Observation{{ID: "L", X: 1}}},
		{Time: 200, DX: 1, MoveVariance: 0.5},
	}}); err != nil {
		t.Fatal(err)
	}
	anchor, err := m.PoseAt(100)
	if err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{
		X: anchor.X, Y: anchor.Y, Heading: anchor.Heading, Variance: 1,
	}})
	if err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open variance-only correction: %v", err)
	}
	defer m2.Close()
	recs, _ := m2.Corrections()
	if len(recs) != 1 || len(recs[0].Poses) != 2 {
		t.Fatalf("records = %+v", recs)
	}
	wantAfter := []float64{1, 1.5}
	for i, pc := range recs[0].Poses {
		if pc.Before.X != pc.After.X || pc.Before.Y != pc.After.Y || pc.Before.Heading != pc.After.Heading {
			t.Fatalf("pose %d geometry changed on variance-only correction: %+v -> %+v", i, pc.Before, pc.After)
		}
		if pc.After.Variance != wantAfter[i] {
			t.Fatalf("pose %d after variance = %v, want %v", i, pc.After.Variance, wantAfter[i])
		}
	}
	if len(recs[0].Landmarks) != 1 ||
		recs[0].Landmarks[0].Before.X != rec.Landmarks[0].Before.X ||
		recs[0].Landmarks[0].After.X != rec.Landmarks[0].After.X {
		t.Fatalf("landmark snapshot not retained: %+v", recs[0].Landmarks)
	}
}
