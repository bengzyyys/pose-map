package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// buildSegmentMap 构造含一段三帧（t=100、200、300，每帧 DX=1、运动方差
// 0.5）的地图：段 s1 的首次导入末位姿为 t=300、X=3、方差 1.5。
func buildSegmentMap(t *testing.T, path string) {
	t.Helper()
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.5},
		{Time: 200, DX: 1, MoveVariance: 0.5},
		{Time: 300, DX: 1, MoveVariance: 0.5},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 末位姿时间不成立——没有任何对应帧、落在两帧之间、或指向初始位姿——都
// 按损坏拒绝：Open 不返回可用地图，错误可由 errors.Is(ErrCorrupt) 判断，
// 说明中指出段标识与末位姿时间，原文件保持原样。
func TestOpenRejectsSegmentEndTimeNotOnFrame(t *testing.T) {
	for _, bad := range []int64{150, 0, 400, -5} {
		path := filepath.Join(t.TempDir(), "m.pose")
		buildSegmentMap(t, path)
		fd := readFileData(t, path)
		fd.Segments[0].Result.EndPose.Time = bad
		writeFileDataRaw(t, path, &fd)

		before, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		m, err := Open(path)
		if !errors.Is(err, ErrCorrupt) {
			t.Fatalf("time %d: err = %v, want ErrCorrupt", bad, err)
		}
		if m != nil {
			t.Fatalf("time %d: Open returned usable map: %v", bad, m)
		}
		if !strings.Contains(err.Error(), "s1") || !strings.Contains(err.Error(), strconv.FormatInt(bad, 10)) {
			t.Fatalf("time %d: error should name the segment and the end pose time: %v", bad, err)
		}
		after, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		if string(before) != string(after) {
			t.Fatalf("time %d: Open modified the rejected file", bad)
		}
	}
}

// 末帧从未受到校正时，保存的段结果必须与该帧保存的位姿一致：篡改位置、
// 朝向或方差任一项，即使文件格式与校验和都通过，也按损坏拒绝。
func TestOpenRejectsSegmentEndPoseMismatch(t *testing.T) {
	tamper := []struct {
		name string
		edit func(p *Pose)
	}{
		{"x", func(p *Pose) { p.X = 3.5 }},
		{"y", func(p *Pose) { p.Y = 0.5 }},
		{"heading", func(p *Pose) { p.Heading = 0.5 }},
		{"variance", func(p *Pose) { p.Variance = 1.6 }},
	}
	for _, tc := range tamper {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildSegmentMap(t, path)
			fd := readFileData(t, path)
			tc.edit(&fd.Segments[0].Result.EndPose)
			writeFileDataRaw(t, path, &fd)

			m, err := Open(path)
			if !errors.Is(err, ErrCorrupt) || m != nil {
				t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m)
			}
			if !strings.Contains(err.Error(), "s1") || !strings.Contains(err.Error(), "300") {
				t.Fatalf("error should name the segment and the end pose time: %v", err)
			}
		})
	}
}

// 合法文件正常打开：重复导入仍返回首次结果，当前位姿与历史查询不受影响。
func TestOpenAcceptsValidSegmentResult(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegmentMap(t, path)

	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	res, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.5},
		{Time: 200, DX: 1, MoveVariance: 0.5},
		{Time: 300, DX: 1, MoveVariance: 0.5},
	}})
	if err != nil {
		t.Fatalf("duplicate import: %v", err)
	}
	if res.EndPose.Time != 300 || res.EndPose.X != 3 || res.EndPose.Variance != 1.5 {
		t.Fatalf("duplicate import result = %+v", res.EndPose)
	}
}

// 末帧已经受过校正的段，保存的结果必须等于该帧首次受到校正前的位姿，而
// 不是校正后的当前值：把段结果篡改为校正后的当前末位姿（与当前轨迹一致）
// 同样按损坏拒绝。合法文件打开后，重复导入返回首次结果，当前位姿继续返
// 回校正结果，两者允许不同。
func TestOpenSegmentResultAfterCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegmentMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 5, Heading: 0.5, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 未篡改：打开成功，重复导入返回首次结果，当前位姿是校正后的值。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open corrected map: %v", err)
	}
	res, err := m2.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.5},
		{Time: 200, DX: 1, MoveVariance: 0.5},
		{Time: 300, DX: 1, MoveVariance: 0.5},
	}})
	if err != nil {
		t.Fatalf("duplicate import after reopen: %v", err)
	}
	if res.EndPose.X != 3 || res.EndPose.Y != 0 || res.EndPose.Heading != 0 || res.EndPose.Variance != 1.5 {
		t.Fatalf("duplicate import result = %+v, want first-import pose", res.EndPose)
	}
	cur, err := m2.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	if cur.X == 3 && cur.Y == 0 {
		t.Fatalf("current pose should reflect the correction: %+v", cur)
	}
	m2.Close()

	// 篡改：把保存的段结果换成校正后的当前末位姿——与当前轨迹一致，但
	// 不是首次导入结果，必须按损坏拒绝。
	fd := readFileData(t, path)
	fd.Segments[0].Result.EndPose = fd.Trajectory[3]
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("replaced with corrected pose: err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 只有方差被校正、位置朝向保持不变时，保存的段结果仍必须携带原始方差：
// 篡改为目标方差（与当前轨迹一致）按损坏拒绝。
func TestOpenSegmentResultVarianceOnlyCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegmentMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	// 恒等校正：目标位置/朝向与锚点当前值相同，只把方差改为 7。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 1, Y: 0, Heading: 0, Variance: 7}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open variance-only corrected map: %v", err)
	}
	res, err := m2.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.5},
		{Time: 200, DX: 1, MoveVariance: 0.5},
		{Time: 300, DX: 1, MoveVariance: 0.5},
	}})
	if err != nil {
		t.Fatalf("duplicate import after reopen: %v", err)
	}
	if res.EndPose.Variance != 1.5 {
		t.Fatalf("duplicate import variance = %v, want original 1.5", res.EndPose.Variance)
	}
	if cur, _ := m2.CurrentPose(); cur.Variance != 8 {
		t.Fatalf("current variance = %v, want corrected 8", cur.Variance)
	}
	m2.Close()

	// 篡改：把段结果方差改成校正后的当前方差 8（目标方差 7 加锚点之后
	// 运动方差 1），位置朝向不动——仍不是首次导入结果。
	fd := readFileData(t, path)
	if fd.Trajectory[3].Variance != 8 {
		t.Fatalf("setup: current end variance = %v, want 8", fd.Trajectory[3].Variance)
	}
	fd.Segments[0].Result.EndPose.Variance = 8
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("target variance in segment result: err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 对相同帧多次校正时，段结果对应的是首次校正前的位姿，不是任意一次校正
// 后的值：篡改为第二次校正前的中间值（即第一次校正后的值）同样拒绝。
func TestOpenSegmentResultWithRepeatedCorrections(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegmentMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 100, Target: CorrectionTarget{X: 20, Y: 0, Heading: 0, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open repeatedly corrected map: %v", err)
	}
	m2.Close()

	// 第一次校正后末帧位于 X=12，第二次校正后位于 X=22；首次导入值为
	// X=3。篡改为任一校正后的值都按损坏拒绝。
	for _, bad := range []float64{12, 22} {
		p := filepath.Join(t.TempDir(), "m.pose")
		buildSegmentMap(t, p)
		mm, err := Open(p)
		if err != nil {
			t.Fatal(err)
		}
		if _, err := mm.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 1}}); err != nil {
			t.Fatal(err)
		}
		if _, err := mm.Correct(Correction{ID: "c2", Anchor: 100, Target: CorrectionTarget{X: 20, Y: 0, Heading: 0, Variance: 2}}); err != nil {
			t.Fatal(err)
		}
		if err := mm.Close(); err != nil {
			t.Fatal(err)
		}
		fd := readFileData(t, p)
		fd.Segments[0].Result.EndPose.X = bad
		writeFileDataRaw(t, p, &fd)
		if m, err := Open(p); !errors.Is(err, ErrCorrupt) || m != nil {
			t.Fatalf("end x %v: err = %v, map = %v, want ErrCorrupt and nil map", bad, err, m)
		}
	}
}

// 校正范围只覆盖旧段的一部分、或校正之后继续导入新段，都不改写旧段的首
// 次结果；范围没有包含该末帧的校正也不改变它对应的值。
func TestOpenSegmentResultPartialAndLaterSegments(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1},
		{Time: 400, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	// 只覆盖 s2 的校正：s1 的末帧 200 不在范围内，s1 的首次结果不变。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 300, Target: CorrectionTarget{X: 30, Y: 0, Heading: 0, Variance: 0}}); err != nil {
		t.Fatal(err)
	}
	// 校正之后继续导入新段 s3：其首次结果记录的是校正后轨迹上的位姿。
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 500, DX: 1},
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
	r1, err := m2.ImportSegment(Segment{ID: "s1", Frames: []Frame{{Time: 100, DX: 1}, {Time: 200, DX: 1}}})
	if err != nil {
		t.Fatal(err)
	}
	if r1.EndPose.X != 2 || r1.EndPose.Time != 200 {
		t.Fatalf("s1 first result = %+v, want X=2 at t=200", r1.EndPose)
	}
	r2, err := m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{{Time: 300, DX: 1}, {Time: 400, DX: 1}}})
	if err != nil {
		t.Fatal(err)
	}
	if r2.EndPose.X != 4 || r2.EndPose.Time != 400 {
		t.Fatalf("s2 first result = %+v, want X=4 at t=400", r2.EndPose)
	}
	r3, err := m2.ImportSegment(Segment{ID: "s3", Frames: []Frame{{Time: 500, DX: 1}}})
	if err != nil {
		t.Fatal(err)
	}
	// s3 在校正之后导入：末帧从未被校正，结果与保存的该帧位姿一致。
	if r3.EndPose.X != 32 || r3.EndPose.Time != 500 {
		t.Fatalf("s3 first result = %+v, want X=32 at t=500", r3.EndPose)
	}
}

// 缺少逐帧依据的旧轨迹范围保留原有打开方式：旧文件中段结果的数值不被
// 推测核对，但末位姿时间仍须对应真实导入帧。
func TestOpenLegacySegmentResultNotInvented(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegmentMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	// 旧文件中段结果数值与轨迹不符：缺少逐帧依据，保留既有打开规则。
	fd := readFileData(t, path)
	if len(fd.Sources) != 0 {
		t.Fatalf("setup: legacy file should have no sources")
	}
	fd.Segments[0].Result.EndPose.X = 99
	writeFileDataRaw(t, path, &fd)
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("legacy open: %v", err)
	}
	m2.Close()

	// 但末位姿时间不对应真实导入帧时，即使旧文件也按损坏拒绝。
	fd = readFileData(t, path)
	fd.Segments[0].Result.EndPose.Time = 250
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("legacy bad end time: err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 旧地图上新追加且依据完整的段仍适用本次检查：篡改新段的首次结果按损坏
// 拒绝，而旧段的数值仍不被推测核对。
func TestOpenMixedLegacyNewSegmentChecked(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
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
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 篡改新追加段的首次结果：依据完整，必须按损坏拒绝。
	fd := readFileData(t, path)
	if len(fd.Segments) != 2 {
		t.Fatalf("setup: segments = %+v", fd.Segments)
	}
	idx := 0
	if fd.Segments[1].ID == "new" {
		idx = 1
	}
	if fd.Segments[idx].ID != "new" {
		t.Fatalf("setup: segments = %+v", fd.Segments)
	}
	fd.Segments[idx].Result.EndPose.X = 3.5
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("tampered new segment: err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}

	// 篡改旧段的数值（同时把新段恢复为正确值）：旧帧缺少逐帧依据，保
	// 持既有打开规则。
	fd = readFileData(t, path)
	oldIdx := 1 - idx
	fd.Segments[idx].Result.EndPose.X = 3
	fd.Segments[oldIdx].Result.EndPose.X = 77
	writeFileDataRaw(t, path, &fd)
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("legacy segment value should not be invented: %v", err)
	}
	m3.Close()
}
