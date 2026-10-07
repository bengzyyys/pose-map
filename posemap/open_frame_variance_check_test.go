package posemap

import (
	"errors"
	"path/filepath"
	"strings"
	"testing"
)

// buildPlainVarianceMap 构造初始时间为零、初始方差为 1，随后一次导入
// 100、200、300 毫秒三帧（运动方差依次 0.25、0.5、0.75，无路标观测、无
// 校正）的地图：三帧保存方差应为 1.25、1.75、2.5。
func buildPlainVarianceMap(t *testing.T, path string) {
	t.Helper()
	cfg := baseConfig()
	cfg.InitialVariance = 1
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.5},
		{Time: 300, DX: 1, MoveVariance: 0.75},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 正常保存的未校正轨迹：各帧方差等于前一帧保存方差加原运动方差，打开
// 成功，PoseAt 原样返回各帧结果。
func TestOpenAcceptsPlainVarianceAccumulation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildPlainVarianceMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	want := map[int64]float64{0: 1, 100: 1.25, 200: 1.75, 300: 2.5}
	for ts, v := range want {
		p, err := m.PoseAt(ts)
		if err != nil {
			t.Fatal(err)
		}
		if p.Variance != v {
			t.Fatalf("PoseAt(%d) variance = %v, want %v", ts, p.Variance, v)
		}
	}
}

// 只把 200 毫秒帧的方差写成 1.5（正确值 1.75）：文件长度、校验和与段末
// 结果都合法，Open 仍须以 ErrCorrupt 拒绝并指出出错帧时间，不让 PoseAt
// 返回这份错误位姿。
func TestOpenRejectsTamperedMiddleFrameVariance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildPlainVarianceMap(t, path)
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[2].Variance = 1.5
	})
	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("map = %v, want nil", m)
	}
	if !strings.Contains(err.Error(), "200") {
		t.Fatalf("error %q does not name the offending frame time 200", err)
	}
}

// 未校正帧方差核对的各条拒绝规则：任何一帧与累计规则不符、参与累计的
// 方差为负或非有限、累计结果非有限，都按损坏拒绝。
func TestOpenRejectsInconsistentPlainFrameVariances(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"first frame wrong", func(fd *fileData) {
			fd.Trajectory[1].Variance = 1.3
		}},
		{"last frame wrong", func(fd *fileData) {
			fd.Trajectory[3].Variance = 2.4
		}},
		{"variance not accumulated from previous frame", func(fd *fileData) {
			// 每帧各自从初始方差 1 加自己的运动方差是错的：必须链式累计。
			fd.Trajectory[2].Variance = 1.5
			fd.Trajectory[3].Variance = 1.75
		}},
		{"negative saved variance", func(fd *fileData) {
			fd.Trajectory[1].Variance = -0.25
		}},
		{"negative motion variance in source", func(fd *fileData) {
			fd.Sources[1].MoveVariance = -0.1
		}},
		{"accumulation overflows to non-finite", func(fd *fileData) {
			// 前一帧保存方差与运动方差各自有限，但 1.25 + 1e308 必为
			// +Inf：累计结果非有限即损坏，不能拿保存的有限值放行。
			fd.Sources[1].MoveVariance = 1e308
			fd.Trajectory[2].Variance = 1.25
			fd.Trajectory[3].Variance = 2
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildPlainVarianceMap(t, path)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 运动方差全为零合法：各帧保存方差都等于初始方差；中间帧被改成非零仍拒。
func TestOpenAcceptsZeroMotionVariances(t *testing.T) {
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
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	p, _ := m2.PoseAt(300)
	if p.Variance != 0.5 {
		t.Fatalf("variance = %v, want initial 0.5", p.Variance)
	}
	m2.Close()

	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[2].Variance = 0.5001
	})
	assertOpenCorrupt(t, path)
}

// buildCorrectedThenAppendedMap 构造三帧（t=100、200、300，运动方差
// 0.25、0.25、0.5）经一次校正（目标方差 1，校正后方差 1、1.25、1.75）后
// 又追加一帧 t=400（运动方差 0.5）的地图：400 帧保存方差应为校正后的
// 1.75 + 0.5 = 2.25，而不是校正前累计值 1.5 + 0.5 = 2。
func buildCorrectedThenAppendedMap(t *testing.T, path string) {
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
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 0.5},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 校正结束后新追加、未被校正覆盖的帧：从前一帧当前保存的校正后方差继续
// 累计，不使用校正前的方差或首次导入结果里的旧方差。
func TestOpenChecksVarianceAccumulationAfterCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildCorrectedThenAppendedMap(t, path)
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	p, _ := m2.PoseAt(400)
	if p.Variance != 2.25 {
		t.Fatalf("variance = %v, want 2.25", p.Variance)
	}
	m2.Close()

	// 按校正前方差累计（2）或写成其他值都按损坏拒绝。
	cases := []struct {
		name string
		v    float64
	}{
		{"accumulated from pre-correction variance", 2},
		{"plain wrong value", 2.5},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildCorrectedThenAppendedMap(t, path)
			tamperFile(t, path, func(fd *fileData) {
				fd.Trajectory[4].Variance = tc.v
			})
			assertOpenCorrupt(t, path)
		})
	}
}

// 校正范围内的帧沿用校正记录核对规则，不能用相邻帧相加代替校正目标：
// 被覆盖帧的保存方差（校正后值）本就不等于前一帧加运动方差，不得因此被拒。
func TestOpenDoesNotAccumulateAcrossCorrectedFrames(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildVarianceCorrectedMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	// 校正后方差 1、1.25、1.75：若误用相邻累计会要求 0.75、1.0、1.5。
	want := map[int64]float64{100: 1, 200: 1.25, 300: 1.75}
	for ts, v := range want {
		p, err := m.PoseAt(ts)
		if err != nil {
			t.Fatal(err)
		}
		if p.Variance != v {
			t.Fatalf("PoseAt(%d) variance = %v, want %v", ts, p.Variance, v)
		}
	}
}

// 某帧来源缺失（null 旧帧）时保留兼容行为、不猜测未知运动方差；同一文件
// 中后来追加且来源完整的帧仍须接受本项检查。
func TestOpenChecksSourcedFramesAroundNullBasisFrame(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildPlainVarianceMap(t, path)

	// 把 200 毫秒帧的逐帧依据改为缺失：该帧不再核对；300 帧以来源完整的
	// 前一帧保存方差（200 帧的 1.75）为基准仍须核对，正常文件照常打开。
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[1] = nil
	})
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	// 之后追加来源完整的新帧：从 300 帧保存方差 2.5 继续累计。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 0.5},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	p, _ := m2.PoseAt(400)
	if p.Variance != 3 {
		t.Fatalf("variance = %v, want 3", p.Variance)
	}
	m2.Close()

	// 来源缺失的 200 帧方差被改：跳过该帧，且 300 帧以它为基准的累计
	// 仍与保存值一致（基准取保存值本身），不拒绝。
	// 来源完整的 300 帧方差被改：必须拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[3].Variance = 2.6
	})
	assertOpenCorrupt(t, path)
}

// 没有路标观测、也不是段末帧的中间帧同样核对：只改中间帧即拒绝（上面的
// TestOpenRejectsTamperedMiddleFrameVariance 已覆盖 200 帧；这里再确认
// 首帧——它既非末帧也无观测——同样在内）。
func TestOpenChecksFirstFrameVariance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildPlainVarianceMap(t, path)
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[1].Variance = 1.2
	})
	assertOpenCorrupt(t, path)
}
