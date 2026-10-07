package posemap

import (
	"bytes"
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// buildPlainAccumMap 构造无路标观测、无校正的三帧地图：初始方差为 1，进入
// t=100/200/300 三帧的原运动方差依次为 0.25、0.5、0.75，故三帧保存方差
// 依次为 1.25、1.75、2.5。
func buildPlainAccumMap(t *testing.T, path string) {
	t.Helper()
	cfg := Config{InitialTime: 0, InitialVariance: 1, MaxInterval: 1000, MergeDistance: 1.0}
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

// 正常保存的轨迹必须打开成功，PoseAt 拿到的仍是各帧保存的方差。
func TestOpenAcceptsPlainVarianceAccumulation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildPlainAccumMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	want := []struct {
		time int64
		v    float64
	}{{100, 1.25}, {200, 1.75}, {300, 2.5}}
	for _, w := range want {
		p, err := m.PoseAt(w.time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", w.time, err)
		}
		if p.Variance != w.v {
			t.Fatalf("PoseAt(%d) variance = %v, want %v", w.time, p.Variance, w.v)
		}
	}
}

// 规格示例：只把 t=200 帧的方差写成 1.5（初值 1、运动方差 0.25/0.5/0.75
// 时应为 1.25/1.75/2.5），即使文件长度、校验和及段末结果都合法，Open 也
// 必须以 ErrCorrupt 拒绝，错误说明指出 200，不返回可用地图，也不改写文件。
func TestOpenRejectsMiddleFrameVarianceContradiction(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildPlainAccumMap(t, path)
	fd := readFileData(t, path)
	if fd.Trajectory[2].Variance != 1.75 || fd.Trajectory[3].Variance != 2.5 {
		t.Fatalf("test setup wrong: %+v", fd.Trajectory)
	}
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[2].Variance = 1.5 // 段末帧（300）与段首次结果保持合法
	})
	saved, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("Open returned usable map: %v", m)
	}
	if !strings.Contains(err.Error(), "200") {
		t.Fatalf("error %q does not identify the offending frame time 200", err.Error())
	}
	got, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(got, saved) {
		t.Fatal("Open modified the rejected file")
	}
}

// 逐帧核对：锚点式的只看末帧不够，未被校正覆盖的每一帧都必须满足
// “前一份保存方差 + 本帧运动方差”。
func TestOpenRejectsUncoveredVarianceMismatchPerFrame(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"first frame wrong", func(fd *fileData) {
			fd.Trajectory[1].Variance = 9 // 应为 1.25
		}},
		{"last frame wrong but segment end result fixed to match", func(fd *fileData) {
			// 末帧同时是段末帧：把段首次导入结果也改成篡改值，使段末核对
			// 放行——逐帧累计核对仍必须拒绝。
			fd.Trajectory[3].Variance = 2.4 // 应为 2.5
			fd.Segments[0].Result.EndPose.Variance = 2.4
		}},
		{"negative saved variance", func(fd *fileData) {
			fd.Trajectory[2].Variance = -0.25
		}},
		{"motion variance made inconsistent", func(fd *fileData) {
			// t=200 的原运动方差改为 0.6：累计应为 1.85，与保存的 1.75 矛盾。
			fd.Sources[1].MoveVariance = 0.6
		}},
		{"negative motion variance", func(fd *fileData) {
			fd.Sources[0].MoveVariance = -0.1
		}},
		{"finite saved values but accumulation overflows", func(fd *fileData) {
			// 前一份保存方差与本帧运动方差都有限，但 1e308+1e308 = +Inf：
			// 累计结果超出有限范围，不能拿保存的有限值放行。
			fd.Trajectory[0].Variance = 1e308
			fd.Trajectory[1].Variance = 1 // 保存值有限，与无穷累计矛盾
			fd.Sources[0].MoveVariance = 1e308
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildPlainAccumMap(t, path)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 运动方差为零合法：每帧保存方差保持基准值不变；零基准上累计仍为零。
func TestOpenAcceptsZeroUncoveredVariances(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1, MoveVariance: 0},
		{Time: 300, DX: 1, MoveVariance: 0},
	}}); err != nil {
		t.Fatal(err)
	}
	m.Close()
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m2.Close()
	for _, tm := range []int64{100, 200, 300} {
		p, err := m2.PoseAt(tm)
		if err != nil {
			t.Fatal(err)
		}
		if p.Variance != 0 {
			t.Fatalf("variance at %d = %v, want 0", p.Time, p.Variance)
		}
	}

	// 全零累计下把中间帧改成非零仍按损坏拒绝（无观测、非段末帧也不跳过）。
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[2].Variance = 1e-300
	})
	assertOpenCorrupt(t, path)
}

// 与路标合并距离无关、也不因没有路标观测或不是段末帧而跳过：无观测轨迹的
// 中间帧方差矛盾一样拒绝。
func TestOpenUncoveredVarianceCheckIndependentOfLandmarks(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildPlainAccumMap(t, path)
	fd := readFileData(t, path)
	for _, s := range fd.Sources {
		if len(s.Observations) != 0 {
			t.Fatalf("test setup wrong: observations = %+v", s.Observations)
		}
	}
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[2].Variance = 1.75 + 1e-12 // 中间帧、无观测、非段末
	})
	assertOpenCorrupt(t, path)
}

// 校正结束后新追加、未被校正覆盖的帧，从前一帧当前保存的校正后方差继续
// 累计：t=100 校正目标方差 1，三帧校正后为 1/1.5/2.25；追加 t=400（运动
// 方差 0.5）应为 2.75。篡改的是追加段中的非末帧，并把它之后的帧与段结果
// 保持合法，使段末核对放行——相邻累计核对仍必须拒绝，且期望值来自校正后
// 保存值而不是首次导入旧方差。
func TestOpenAppendedFramesAccumulateFromCorrectedVariance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, InitialVariance: 0.5, MaxInterval: 1000, MergeDistance: 1.0}
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
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 0.5},
		{Time: 500, DX: 1, MoveVariance: 0.5},
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
	p400, _ := m2.PoseAt(400)
	p500, _ := m2.PoseAt(500)
	if p400.Variance != 2.75 || p500.Variance != 3.25 {
		t.Fatalf("appended variances = %v/%v, want 2.75/3.25 (continued from corrected 2.25)", p400.Variance, p500.Variance)
	}
	m2.Close()

	// 只改非末帧 t=400 的保存方差为“校正前旧值 +0.5=2.5”（正确应为校正后
	// 2.25+0.5=2.75）：段末帧 500 与段首次结果都合法，必须仍由相邻累计核对
	// 在 400 处拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[4].Variance = 2.5
	})
	assertOpenCorrupt(t, path)
}

// 被校正覆盖的帧继续使用校正记录的核对规则，不被相邻帧相加替代：篡改覆盖
// 范围内一帧的来源运动方差（保持校正记录快照合法、只改当前轨迹无关）——
// 这里直接确认覆盖帧的方差矛盾仍由“最后一次校正”核对拒绝，而未覆盖的
// 追加帧才走相邻累计。
func TestOpenCoveredFramesKeepCorrectionRules(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, InitialVariance: 1, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.5},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	m.Close()

	// 覆盖帧 t=100 的保存方差与最后一条校正记录的校正后值矛盾：由校正规则
	// 拒绝（错误来自最后一次校正核对，而非相邻累计）。
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[1].Variance = 1.4
	})
	assertOpenCorrupt(t, path)
}

// 纯旧文件没有逐帧来源：即使保存方差明显不符任何累计，也保留既有打开行为。
func TestOpenPureLegacyFileSkipsVarianceAccumulation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.5},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[2].Variance = 999
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("pure legacy file must keep legacy open rules: %v", err)
	}
	m2.Close()
}

// 旧文件后来追加、来源完整的帧仍须检查：它们以相邻前一份保存位姿（旧帧）
// 的当前保存方差为基准累计；篡改追加段中的非末帧方差即拒绝，即使段末帧与
// 段首次结果仍合法。
func TestOpenLegacyFileAppendedFramesStillChecked(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, InitialVariance: 1, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.5},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	// t=300 运动方差 0.75：合法保存值 = 旧帧 t=200 的保存方差 1.75 + 0.75
	// = 2.5；t=400 再为 2.5 + 1 = 3.5。
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.75},
		{Time: 400, DX: 1, MoveVariance: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	m2.Close()

	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen mixed file: %v", err)
	}
	p, _ := m3.PoseAt(300)
	if p.Variance != 2.5 {
		t.Fatalf("t=300 variance = %v, want 2.5", p.Variance)
	}
	m3.Close()

	// 只改非末帧 t=300：段末帧 400 与段结果保持合法，仍按相邻累计拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[3].Variance = 3
	})
	assertOpenCorrupt(t, path)
}

// 某帧来源缺失（null 旧帧）时累计链在该处断开：该帧不核对；其后来源完整
// 的帧以相邻的当前保存值为基准重新起算，仍须核对——即使这些帧不是任何段
// 的末帧。
func TestOpenNullSourceBreaksChainButLaterSourcedFramesChecked(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, InitialVariance: 1, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	// 无观测：把中间帧来源改成 null 不会影响路标点次数与归属核对。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.5},
		{Time: 300, DX: 1, MoveVariance: 0.75},
	}}); err != nil {
		t.Fatal(err)
	}
	m.Close()

	// t=200 来源缺失：该帧跳过；其余帧按相邻保存值核对，合法布局必须能打开。
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[1] = nil
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("null-source middle frame should keep compatibility: %v", err)
	}
	// 在这种布局上再追加两帧：t=400/500 应为 t=300 保存值 2.5 +2、再 +3。
	if _, err := m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 2},
		{Time: 500, DX: 1, MoveVariance: 3},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	p400, _ := m3.PoseAt(400)
	p500, _ := m3.PoseAt(500)
	if p400.Variance != 4.5 || p500.Variance != 7.5 {
		t.Fatalf("variances = %v/%v, want 4.5/7.5", p400.Variance, p500.Variance)
	}
	m3.Close()

	// 篡改追加段中的非末帧 t=400（段末帧 500 的保存值与段首次结果一致）：
	// 段末核对放行，只有相邻累计核对会发现矛盾（j 升序，先在 400 处），必须
	// 拒绝；这也证明 null 之后的链以相邻当前保存值重新起算，而不是回溯使用
	// 初始方差或首次导入旧值。
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[4].Variance = 4
	})
	assertOpenCorrupt(t, path)
}
