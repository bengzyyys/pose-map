package posemap

import (
	"errors"
	"path/filepath"
	"strings"
	"testing"
)

// buildOverlappingCorrectionsMap 构造规格示例的覆盖链（全程无路标观测，
// 排除路标核对的干扰）：
//
//	初始 (0,0,h=0,v=0)
//	t=100 (1,0,.1)  t=200 (2,0,.2)  t=300 (3,0,.3)
//	c1：锚点 100 → 目标 (5,6,h=0,v=1)（无旋转平移），覆盖 100~300
//	  校正后：100=(5,6,v1) 200=(6,6,v1.1) 300=(7,6,v1.2)
//	追加 t=400：(8,6,v1.3)
//	c2：锚点 200 → 目标 (9,9,h=0,v=2)，覆盖 200~400
//	  校正后：200=(9,9,v2) 300=(10,9,v2.1) 400=(11,9,v2.2)
//	  100 不在 c2 范围，仍为 c1 的结果 (5,6,v1)
//
// 于是当前轨迹中：100 对应 c1，200/300/400 对应 c2；c1 保存的 200
// 校正后快照 (6,6) 与当前值 (9,9) 不同，但旧记录不被改写。
func buildOverlappingCorrectionsMap(t *testing.T, path string, merge float64) {
	t.Helper()
	m, err := Create(path, Config{
		InitialTime: 0, MaxInterval: 1000, MergeDistance: merge,
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
		Target: CorrectionTarget{X: 5, Y: 6, Heading: 0, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 200,
		Target: CorrectionTarget{X: 9, Y: 9, Heading: 0, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 未篡改的覆盖链正常打开：历史查询与各帧最后一次校正的结果一致；旧记录
// c1 的快照保留原值，不被 c2 改写。
func TestOpenAcceptsPoseMatchingLastCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()

	want := []struct {
		time        int64
		x, y        float64
		h, variance float64
	}{
		{100, 5, 6, 0, 1},
		{200, 9, 9, 0, 2},
		{300, 10, 9, 0, 2.1},
		{400, 11, 9, 0, 2.2},
	}
	for _, w := range want {
		p, err := m.PoseAt(w.time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", w.time, err)
		}
		if p.X != w.x || p.Y != w.y || p.Heading != w.h || p.Variance != w.variance {
			t.Fatalf("PoseAt(%d) = %+v, want x=%v y=%v h=%v v=%v", w.time, p, w.x, w.y, w.h, w.variance)
		}
	}

	recs, _ := m.Corrections()
	if len(recs) != 2 {
		t.Fatalf("records = %+v", recs)
	}
	// c1 的 200 校正后快照仍是提交时的 (6,6)，不随后续 c2 改写。
	if got := recs[0].Poses[1].After; got.X != 6 || got.Y != 6 {
		t.Fatalf("c1 old snapshot rewritten: %+v", got)
	}
	// c2 的 200 校正后值即当前查询值。
	if got := recs[1].Poses[0].After; got.X != 9 || got.Y != 9 {
		t.Fatalf("c2 snapshot = %+v", got)
	}
}

// 即使校验和正确、每条校正自身合法，当前保存位姿与该帧最后一次校正结果
// 超出容差时 Open 必须拒绝：只属于 c1 的帧（100，c1 锚点）与属于 c2 的
// 锚点/中间/结束帧（200/300/400）逐一覆盖；这些帧都没有路标观测，矛盾
// 只能由逐帧位姿核对发现，不能依赖路标查询。
func TestOpenRejectsPoseContradictingLastCorrection(t *testing.T) {
	cases := []struct {
		name  string
		time  int64
		index int
		fn    func(p *Pose)
	}{
		{"c1-only anchor x contradicts c1", 100, 1, func(p *Pose) { p.X = 5.5 }},
		{"c1-only anchor y contradicts c1", 100, 1, func(p *Pose) { p.Y = 6.5 }},
		{"c1-only anchor heading contradicts c1", 100, 1, func(p *Pose) { p.Heading = 0.3 }},
		{"c1-only anchor variance contradicts c1", 100, 1, func(p *Pose) { p.Variance = 1.2 }},
		{"c2 anchor x contradicts c2", 200, 2, func(p *Pose) { p.X = 9.5 }},
		{"c2 anchor variance contradicts c2", 200, 2, func(p *Pose) { p.Variance = 2.5 }},
		{"middle frame y contradicts c2", 300, 3, func(p *Pose) { p.Y = 9.5 }},
		{"middle frame heading contradicts c2", 300, 3, func(p *Pose) { p.Heading = -0.2 }},
		{"end frame position contradicts c2", 400, 4, func(p *Pose) { p.X = 11.4 }},
		{"end frame variance contradicts c2", 400, 4, func(p *Pose) { p.Variance = 2.25 }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildOverlappingCorrectionsMap(t, path, 10)
			tamperFile(t, path, func(fd *fileData) {
				if fd.Trajectory[tc.index].Time != tc.time {
					t.Fatalf("setup: index %d time = %d, want %d", tc.index, fd.Trajectory[tc.index].Time, tc.time)
				}
				p := fd.Trajectory[tc.index]
				tc.fn(&p)
				fd.Trajectory[tc.index] = p
			})
			assertOpenCorrupt(t, path)
		})
	}
}

// 错误必须可被 errors.Is 识别为 ErrCorrupt，并指出对应的校正标识与矛盾
// 帧时间。
func TestOpenLastCorrectionMismatchErrorDetails(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[2].X = 9.5 // t=200，最后一次覆盖记录是 c2
	})
	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("map = %v, want nil", m)
	}
	msg := err.Error()
	if !strings.Contains(msg, "c2") || !strings.Contains(msg, "200") {
		t.Fatalf("error %q must name correction c2 and frame time 200", msg)
	}
}

// 坐标容差为 1e-9 乘以 1、两个坐标绝对值三者最大值：量级约 9 时容差约
// 9e-9，半档放行、两倍档拒绝；朝向容差为绝对 1e-9 弧度；方差必须完全
// 一致，再小的差异也拒绝。放行时查询与记录都保留文件原值，不借容差改写。
func TestOpenLastCorrectionTolerance(t *testing.T) {
	coordCases := []struct {
		name     string
		perturb  float64
		wantOpen bool
	}{
		{"within half tolerance", 4e-9, true},
		{"beyond tolerance", 2e-8, false},
	}
	for _, tc := range coordCases {
		t.Run("coord "+tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildOverlappingCorrectionsMap(t, path, 10)
			saved := 9.0 + tc.perturb
			tamperFile(t, path, func(fd *fileData) {
				fd.Trajectory[2].X = saved // t=200 期望 9
			})
			if !tc.wantOpen {
				assertOpenCorrupt(t, path)
				return
			}
			m, err := Open(path)
			if err != nil {
				t.Fatalf("within tolerance should open: %v", err)
			}
			p, _ := m.PoseAt(200)
			if p.X != saved {
				t.Fatalf("query x = %v, want saved value %v", p.X, saved)
			}
			recs, _ := m.Corrections()
			if recs[1].Poses[0].After.X != 9 {
				t.Fatalf("record after x = %v, want original 9", recs[1].Poses[0].After.X)
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
			tamperFile(t, path, func(fd *fileData) {
				fd.Trajectory[4].Heading = tc.perturb // t=400 期望 0
			})
			if tc.wantOpen {
				m, err := Open(path)
				if err != nil {
					t.Fatalf("within angle tolerance should open: %v", err)
				}
				p, _ := m.PoseAt(400)
				if p.Heading != tc.perturb {
					t.Fatalf("query heading = %v, want saved %v", p.Heading, tc.perturb)
				}
				m.Close()
			} else {
				assertOpenCorrupt(t, path)
			}
		})
	}

	t.Run("variance must match exactly", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "m.pose")
		buildOverlappingCorrectionsMap(t, path, 10)
		tamperFile(t, path, func(fd *fileData) {
			fd.Trajectory[2].Variance = 2 + 1e-12 // 期望恰为 2
		})
		assertOpenCorrupt(t, path)
	})
}

// 路标合并距离再大也不能放宽最后一次校正结果的核对：偏差 0.5 远小于
// 1e9 的合并距离，仍按损坏拒绝。
func TestOpenLastCorrectionRejectedRegardlessOfMergeDistance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 1e9)
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[3].X = 10.5 // t=300 期望 10，偏差 0.5 << 合并距离 1e9
	})
	assertOpenCorrupt(t, path)
}

// 后来追加、不属于任何校正记录的帧不参与这项核对：c2 结束于 400，之后
// 追加的 500、600 不属于 c1/c2，篡改非段末帧 500 的保存位姿仍可打开，
// 查询返回文件原值。
func TestOpenDoesNotRequireFramesOutsideCorrections(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlappingCorrectionsMap(t, path, 10)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	// 一次导入两帧：500 不是段末帧（段末为 600），段末位姿核对不涉及它，
	// 隔离出新核对本身对“范围外帧”的态度。
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 500, DX: 1, MoveVariance: 0.1},
		{Time: 600, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	tamperFile(t, path, func(fd *fileData) {
		if fd.Trajectory[5].Time != 500 {
			t.Fatalf("setup: %+v", fd.Trajectory)
		}
		fd.Trajectory[5].X = 424242
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("frames outside all corrections must not be checked: %v", err)
	}
	p, _ := m2.PoseAt(500)
	if p.X != 424242 {
		t.Fatalf("PoseAt(500) = %+v, want saved value 424242", p)
	}
	m2.Close()
}

// 兼容规则：帧的最后一条覆盖记录范围内含缺少逐帧依据的 null 旧帧时，
// 跳过该帧与当前轨迹的比较——即使当前位姿被改成明显矛盾值也放行；同一
// 文件中另一条依据完整、作为某些帧最后一次校正的记录仍须核对。
func TestOpenSkipsLastCorrectionCheckForNullBasis(t *testing.T) {
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
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// t=200 的逐帧依据置空：c1 范围 100~300 含 null 旧帧，并把当前轨迹
	// 改成任意矛盾值——锚点 100、中间帧 200、结束帧 300 全部跳过核对。
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[1] = nil
		fd.Trajectory[1].X = 500
		fd.Trajectory[2].X = 500
		fd.Trajectory[3].X = 500
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("frames whose last correction spans a null basis frame keep legacy rules: %v", err)
	}

	// 追加依据完整的新帧，并只对该帧提交一帧校正（范围不含 null 帧）。
	if _, err := m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, MoveVariance: 0.1},
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

	// c1 依据不完整（100~300 跳过），c2 依据完整（400 核对）：正常打开。
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	p, _ := m3.PoseAt(400)
	if p.X != 9 || p.Y != 9 || p.Variance != 2 {
		t.Fatalf("PoseAt(400) = %+v, want c2 result (9,9,v2)", p)
	}
	m3.Close()

	// 只属于完整记录 c2 的帧矛盾：其余记录被跳过不影响，整份文件拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Trajectory[4].X = 9.5 // t=400 期望 9
	})
	assertOpenCorrupt(t, path)
}

// 兼容规则不得回退到更早记录：帧先被依据完整的 c1 覆盖、后又被含 null
// 旧帧的 c2 覆盖时，当前位姿以“最后一次校正 c2”为准但因依据缺失跳过
// 比较——不能改用 c1 要求当前位姿回到 c1 的旧结果。
func TestOpenNullBasisLastRecordDoesNotFallBackToEarlier(t *testing.T) {
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
	}}); err != nil {
		t.Fatal(err)
	}
	// c1 依据完整，覆盖 100~200。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	// 追加 300 后，c2 覆盖 100~300（此时依据都完整，正常提交）。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 100,
		Target: CorrectionTarget{X: 7, Y: 8, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 把 t=300 的依据置空：c2（100~300）依据不完整，成为 100/200/300
	// 三帧的最后一次覆盖记录。当前 100/200 既不等于 c2 结果也不等于
	// c1 旧结果，但按兼容规则全部跳过，不回退要求 c1 的值。
	tamperFile(t, path, func(fd *fileData) {
		fd.Sources[2] = nil
		fd.Trajectory[1].X = 500 // t=100
		fd.Trajectory[2].X = 500 // t=200
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("must not fall back to earlier fully-based c1: %v", err)
	}
	m2.Close()
}
