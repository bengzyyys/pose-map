package posemap

import (
	"errors"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// trajAt 按时间找到保存轨迹中对应帧的位姿指针（trajectory[0] 为初始位
// 姿，不在查找范围内）。
func trajAt(fd *fileData, t int64) *Pose {
	for j := 1; j < len(fd.Trajectory); j++ {
		if fd.Trajectory[j].Time == t {
			return &fd.Trajectory[j]
		}
	}
	return nil
}

// buildOverlapCorrectionsMap 构造题述重叠校正场景：
//
//	p100=(1,0) p200=(2,0) p300=(3,0)，c1 锚点 100 平移到 X=5（方差 1）：
//	  100→5, 200→6, 300→7；
//	追加 p400=(8,0) 后，c2 锚点 200 平移到 X=10（方差 2），覆盖 200..400：
//	  200→10, 300→11, 400→12；100 仍是 c1 的结果 5；
//	再追加 p500=(13,0)、p600=(14,0)，均不属于任何校正。
//
// 全程无路标观测。
func buildOverlapCorrectionsMap(t *testing.T, path string) {
	t.Helper()
	m, err := Create(path, Config{
		InitialTime: 0, MaxInterval: 1000, MergeDistance: 10,
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
		Target: CorrectionTarget{X: 5, Y: 0, Heading: 0, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 200,
		Target: CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 500, DX: 1},
		{Time: 600, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 合法的重叠校正文件正常打开，历史位姿查询与“最后一次覆盖该帧的校正”
// 一致：100 对应 c1；200、300、400 对应 c2；500、600 从未被覆盖。
func TestOpenFinalPoseMatchesLastCoveringCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlapCorrectionsMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	want := []struct {
		time       int64
		x, varance float64
	}{
		{100, 5, 1},
		{200, 10, 2},
		{300, 11, 2},
		{400, 12, 2},
		{500, 13, 2},
		{600, 14, 2},
	}
	for _, w := range want {
		got, err := m.PoseAt(w.time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", w.time, err)
		}
		if got.Time != w.time || got.X != w.x || got.Variance != w.varance {
			t.Fatalf("PoseAt(%d) = %+v, want x=%v variance=%v", w.time, got, w.x, w.varance)
		}
	}
}

// 保存位姿停留在更早一次校正的结果、或被改成任意其他值，即使每条校正记
// 录自身前后关系合法、校验和正确，Open 也必须拒绝：矛盾要归到“最后一条
// 覆盖该帧”的校正标识与帧时间。覆盖范围内每一帧（含锚点与结束帧）都核
// 对；从未被任何校正覆盖的帧没有可矛盾的记录，不在本测试内。
func TestOpenRejectsSavedPoseContradictingLastCorrection(t *testing.T) {
	cases := []struct {
		name     string
		time     int64
		edit     func(p *Pose)
		wantCorr string
	}{
		// t=200 同时属于 c1[100,300] 与 c2[200,400]：最后一条是 c2，
		// 保存成 c1 的校正后值（X=6）即矛盾。
		{"overlap frame keeps earlier correction x", 200, func(p *Pose) { p.X = 6 }, "c2"},
		{"overlap frame arbitrary x", 300, func(p *Pose) { p.X = 11.5 }, "c2"},
		{"overlap frame y", 300, func(p *Pose) { p.Y = 0.5 }, "c2"},
		{"overlap frame heading", 300, func(p *Pose) { p.Heading = 0.3 }, "c2"},
		{"overlap frame variance", 300, func(p *Pose) { p.Variance = 9 }, "c2"},
		// t=100 只被 c1 覆盖：锚点帧也要核对。
		{"c1 anchor x drifted", 100, func(p *Pose) { p.X = 5.5 }, "c1"},
		// t=400 只被 c2 覆盖且是 c2 的结束帧：结束帧也要核对。
		{"c2 end frame x drifted", 400, func(p *Pose) { p.X = 12.5 }, "c2"},
		{"c2 end frame variance drifted", 400, func(p *Pose) { p.Variance = 2.5 }, "c2"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildOverlapCorrectionsMap(t, path)
			tamperFile(t, path, func(fd *fileData) {
				p := trajAt(fd, tc.time)
				if p == nil {
					t.Fatalf("setup: no frame at %d", tc.time)
				}
				tc.edit(p)
			})
			m, err := Open(path)
			if !errors.Is(err, ErrCorrupt) {
				t.Fatalf("err = %v, want ErrCorrupt", err)
			}
			if m != nil {
				t.Fatalf("Open returned usable map: %v", m)
			}
			msg := err.Error()
			if !strings.Contains(msg, tc.wantCorr) || !strings.Contains(msg, "last correction") {
				t.Fatalf("error should identify the last covering correction %s: %v", tc.wantCorr, msg)
			}
			if !strings.Contains(msg, strconv.FormatInt(tc.time, 10)) {
				t.Fatalf("error should identify frame time %d: %v", tc.time, msg)
			}
		})
	}
}

// 从未被任何校正覆盖的帧不参与本核对：篡改它的保存位姿（且该帧不是段末
// 帧、无路标观测）仍可打开，文件原值保留。
func TestOpenDoesNotRequireUncoveredFrames(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlapCorrectionsMap(t, path)
	tamperFile(t, path, func(fd *fileData) {
		p := trajAt(fd, 500) // 不属于 c1 或 c2，且不是 s3 的末帧（末帧为 600）
		p.X = 13.5
	})
	m, err := Open(path)
	if err != nil {
		t.Fatalf("uncovered frame pose is not contradicted by any record: %v", err)
	}
	defer m.Close()
	got, err := m.PoseAt(500)
	if err != nil {
		t.Fatal(err)
	}
	if got.X != 13.5 {
		t.Fatalf("saved value should be preserved, got x=%v", got.X)
	}
}

// 范围内没有任何路标观测的帧同样必须符合最后一次校正结果：不能依赖路标
// 重放间接发现矛盾。这里所有帧都无观测，矛盾直接来自保存位姿本身。
func TestOpenFinalPoseCheckAppliesToFramesWithoutObservations(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlapCorrectionsMap(t, path)
	tamperFile(t, path, func(fd *fileData) {
		trajAt(fd, 300).X = 7 // c1 的旧结果；最后覆盖 c2 要求 11
	})
	assertOpenCorrupt(t, path)
}

// 某帧没有路标观测、且不是段末帧时，路标核对与段结果核对都无从发现其位
// 姿矛盾：只有“最后一次校正结果”核对能拒绝，并指出对应校正与帧时间。
func TestOpenFinalPoseCheckFrameWithoutObservationInObservedMap(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, Config{
		InitialTime: 0, MaxInterval: 1000, MergeDistance: 10,
	})
	if err != nil {
		t.Fatal(err)
	}
	// 只有 t=100 观测 L；t=200 无观测、且不是段末帧（段末帧为 t=300）。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1},
		{Time: 300, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 5, Y: 0, Heading: 0, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	// 篡改无观测、非段末帧 t=200 的保存位置（应为 6）。
	tamperFile(t, path, func(fd *fileData) {
		trajAt(fd, 200).X = 6.5
	})
	m2, err := Open(path)
	if !errors.Is(err, ErrCorrupt) || m2 != nil {
		t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m2)
	}
	if msg := err.Error(); !strings.Contains(msg, "c1") || !strings.Contains(msg, "200") {
		t.Fatalf("error should name correction c1 and frame 200: %v", msg)
	}
}

// 坐标容差为 1e-9 乘以 1、|期望|、|保存| 三者最大值：量级约 6 时容差约
// 6e-9，半档放行、两倍档拒绝。放行时 PoseAt 与校正记录都保留文件原值，
// 不借容差改写。
func TestOpenFinalPoseCoordTolerance(t *testing.T) {
	for _, tc := range []struct {
		name   string
		shift  float64
		wantOK bool
	}{
		{"within tolerance", 2e-9, true},
		{"beyond tolerance", 2e-8, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildCorrectedMap(t, path) // c1 后 t=200 保存 X=6
			tamperFile(t, path, func(fd *fileData) {
				trajAt(fd, 200).X = 6 + tc.shift
			})
			if tc.wantOK {
				m, err := Open(path)
				if err != nil {
					t.Fatalf("within tolerance should open: %v", err)
				}
				got, _ := m.PoseAt(200)
				if got.X != 6+tc.shift {
					t.Fatalf("query returned %v, want file value %v", got.X, 6+tc.shift)
				}
				recs, _ := m.Corrections()
				if recs[0].Poses[1].After.X != 6 {
					t.Fatalf("record snapshot rewritten: %v", recs[0].Poses[1].After.X)
				}
				m.Close()
			} else {
				assertOpenCorrupt(t, path)
			}
		})
	}
}

// 朝向按最短角度差比较，容差 1e-9 弧度：半档放行、两倍档拒绝；放行时查
// 询保留文件原值。
func TestOpenFinalPoseAngleTolerance(t *testing.T) {
	for _, tc := range []struct {
		name   string
		shift  float64
		wantOK bool
	}{
		{"within tolerance", 5e-10, true},
		{"beyond tolerance", 2e-9, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildGeometryMap(t, path, 10) // c1 后 t=200 保存 heading=0
			tamperFile(t, path, func(fd *fileData) {
				trajAt(fd, 200).Heading = tc.shift
			})
			if tc.wantOK {
				m, err := Open(path)
				if err != nil {
					t.Fatalf("within angle tolerance should open: %v", err)
				}
				got, _ := m.PoseAt(200)
				if got.Heading != tc.shift {
					t.Fatalf("query returned heading %v, want file value %v", got.Heading, tc.shift)
				}
				m.Close()
			} else {
				assertOpenCorrupt(t, path)
			}
		})
	}
}

// 方差必须完全一致：远小于坐标容差的方差差异也拒绝。
func TestOpenFinalPoseVarianceMustBeExact(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildCorrectedMap(t, path) // c1 后 t=200 保存方差 1
	tamperFile(t, path, func(fd *fileData) {
		trajAt(fd, 200).Variance = 1 + 1e-12
	})
	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
	if msg := err.Error(); !strings.Contains(msg, "variance") {
		t.Fatalf("error should concern variance: %v", msg)
	}
}

// 路标合并距离不能放宽位姿一致性要求：合并距离 1e9 时 0.5 的位姿偏移仍
// 远超 1e-9 容差，必须拒绝。
func TestOpenFinalPoseRejectedRegardlessOfMergeDistance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildGeometryMap(t, path, 1e9) // c1 后 t=200 保存 X=10
	tamperFile(t, path, func(fd *fileData) {
		trajAt(fd, 200).X = 10.5
	})
	assertOpenCorrupt(t, path)
}

// 拒绝打开时原文件字节保持不变。
func TestOpenFinalPoseContradictionLeavesFileUntouched(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOverlapCorrectionsMap(t, path)
	tamperFile(t, path, func(fd *fileData) {
		trajAt(fd, 200).X = 6 // 退回 c1 旧结果
	})
	assertOpenCorrupt(t, path)
}

// 兼容规则：最后一条覆盖某帧的校正范围含缺少逐帧依据的 null 旧帧时，该
// 帧跳过与当前轨迹的比较，即使记录声称的校正后值与当前位姿不符；也不能
// 改用更早一条依据完整的覆盖记录、要求当前位姿回到它的旧结果。同一文件
// 中以依据完整记录为最后一次覆盖的其他帧仍须核对。
func TestOpenFinalPoseSkipsNullBasisLastRecordButChecksOthers(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 10}
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	// 旧段 t=100、200（无路标观测），随后转成无 sources 的旧文件。
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open legacy: %v", err)
	}
	// 追加依据完整的 t=300、400（同样无观测），并对 300..400 提交依据完
	// 整的校正 c1。
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 1},
		{Time: 400, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	cur300, _ := m2.PoseAt(300)
	if _, err := m2.Correct(Correction{ID: "c1", Anchor: 300, Target: CorrectionTarget{
		X: cur300.X + 5, Y: 0, Heading: 0, Variance: 2,
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 手写按提交次序在 c1 之后的 "span" 记录：覆盖 100..400（含 null 旧
	// 帧 100、200），Before/After 直接取当前轨迹，结构合法但依据不完整。
	tamperFile(t, path, func(fd *fileData) {
		cr := corrRecJSON{ID: "span", Anchor: 100, EndTime: 400,
			Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}
		for j := 1; j < len(fd.Trajectory); j++ {
			cr.Poses = append(cr.Poses, poseChangeJSON{Before: fd.Trajectory[j], After: fd.Trajectory[j]})
		}
		fd.Corrections = append(fd.Corrections, cr)
	})
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("null-basis span record should open: %v", err)
	}
	// 再追加依据完整的 t=500，并只对它提交一帧依据完整的校正 c2。
	if _, err := m3.ImportSegment(Segment{ID: "tail", Frames: []Frame{
		{Time: 500, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m3.Correct(Correction{ID: "c2", Anchor: 500,
		Target: CorrectionTarget{X: 20, Y: 0, Heading: 0, Variance: 3}}); err != nil {
		t.Fatal(err)
	}
	if err := m3.Close(); err != nil {
		t.Fatal(err)
	}

	// t=300 的覆盖记录按次序为 c1（依据完整）与 span（含 null 帧）：最后
	// 一条 span 依据不完整。把 span 中 300 的校正后值改成与任何真实值都
	// 无关的 999，同时把当前保存的 t=300 位姿改成既不等于 span.After、也
	// 不等于更早 c1 校正后值（c1 后 t=300 为 X=8）的 12345——该帧必须整
	// 体跳过：不按 span 拒绝，也不回退到 c1 要求当前位姿等于 c1 的旧结果。
	setup := readFileData(t, path)
	if got := setup.Corrections[0].Poses[0].After.X; got != 8 {
		t.Fatalf("test setup wrong: c1 after x at t=300 = %v, want 8", got)
	}
	tamperFile(t, path, func(fd *fileData) {
		var span *corrRecJSON
		for i := range fd.Corrections {
			if fd.Corrections[i].ID == "span" {
				span = &fd.Corrections[i]
			}
		}
		if span == nil {
			t.Fatal("setup: span record missing")
		}
		for i := range span.Poses {
			if span.Poses[i].After.Time == 300 {
				span.Poses[i].After.X = 999
			}
		}
		trajAt(fd, 300).X = 12345
	})

	m4, err := Open(path)
	if err != nil {
		t.Fatalf("frame whose last covering record lacks basis must be skipped: %v", err)
	}
	got300, _ := m4.PoseAt(300)
	if got300.X != 12345 {
		t.Fatalf("skipped frame should keep the file value, got %v", got300.X)
	}
	if recs, _ := m4.Corrections(); len(recs) != 3 {
		t.Fatalf("records = %d, want 3", len(recs))
	}
	m4.Close()

	// 同一份文件中，t=500 的最后覆盖记录 c2 依据完整：篡改其保存位姿仍须
	// 拒绝整份地图，不能因另一记录被跳过而放行。
	tamperFile(t, path, func(fd *fileData) {
		trajAt(fd, 500).X = 20.5
	})
	m5, err := Open(path)
	if !errors.Is(err, ErrCorrupt) || m5 != nil {
		t.Fatalf("fully-based last record c2 must still be enforced: err = %v, map = %v", err, m5)
	}
	if msg := err.Error(); !strings.Contains(msg, "c2") || !strings.Contains(msg, "500") {
		t.Fatalf("error should name c2 and frame 500: %v", msg)
	}
}

// 纯旧文件（无 sources、无校正）保留既有打开规则：没有可作为“最后一次
// 校正”的记录，篡改保存位姿不触发本核对（段结果数值同样沿用旧规则）。
func TestOpenFinalPosePureLegacyNoCorrections(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegmentMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	tamperFile(t, path, func(fd *fileData) {
		if len(fd.Corrections) != 0 || len(fd.Sources) != 0 {
			t.Fatalf("setup: legacy file has corrections=%d sources=%d", len(fd.Corrections), len(fd.Sources))
		}
		trajAt(fd, 200).X = 42
		fd.Segments[0].Result.EndPose.X = 42 // 段数值核对同样对旧文件跳过
	})
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("pure legacy file keeps legacy open rules: %v", err)
	}
	m2.Close()
}
