package posemap

import (
	"errors"
	"math"
	"os"
	"path/filepath"
	"testing"
)

// buildTimedMap 用给定初始时间与帧时间（可跨多段）创建一份正常地图。
func buildTimedMap(t *testing.T, path string, initial int64, maxInterval int64, frameTimes ...int64) {
	t.Helper()
	m, err := Create(path, Config{
		InitialTime: initial, MaxInterval: maxInterval, MergeDistance: 1,
	})
	if err != nil {
		t.Fatal(err)
	}
	for i, ft := range frameTimes {
		if _, err := m.ImportSegment(Segment{
			ID:     segID(i),
			Frames: []Frame{frameAt(ft)},
		}); err != nil {
			t.Fatalf("import frame %d at %d: %v", i, ft, err)
		}
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

func segID(i int) string { return "seg-" + string(rune('a'+i)) }

// 篡改保存轨迹的时间并保持其他字段不变。
func setTrajectoryTimes(t *testing.T, path string, times []int64) {
	t.Helper()
	fd := readFileData(t, path)
	if len(fd.Trajectory) != len(times) {
		t.Fatalf("got %d poses, want %d", len(fd.Trajectory), len(times))
	}
	for i, tm := range times {
		fd.Trajectory[i].Time = tm
	}
	writeFileDataRaw(t, path, &fd)
}

// 仅含初始位姿、尚未导入帧的正常地图必须继续打开。
func TestOpenInitialOnlyMapOpens(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, 50, 1000)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("initial-only map: %v", err)
	}
	defer m.Close()
	if p, err := m.CurrentPose(); err != nil || p.Time != 50 {
		t.Fatalf("current pose = %+v %v", p, err)
	}
}

// 合法的多段轨迹（含负时间、从负数跨到零与正数）原样打开，查询行为不变。
func TestOpenValidTrajectoryAcrossZero(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, -100, 1000, -90, -1, 0, 1, 100)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("valid trajectory: %v", err)
	}
	defer m.Close()
	if p, err := m.PoseAt(0); err != nil || p.Time != 0 {
		t.Fatalf("PoseAt(0) = %+v %v", p, err)
	}
	if p, err := m.PoseAt(50); err != nil || p.Time != 1 {
		t.Fatalf("PoseAt(50) = %+v %v", p, err)
	}
	if cur, err := m.CurrentPose(); err != nil || cur.Time != 100 {
		t.Fatalf("current = %+v %v", cur, err)
	}
}

// 初始位姿与第一帧时间相等：打开必须按 ErrCorrupt 拒绝。
func TestOpenRejectsEqualInitialTime(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, 0, 1000, 100, 200)
	setTrajectoryTimes(t, path, []int64{0, 0, 200})
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("equal initial time: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 同一段内部时间相等：拒绝。
func TestOpenRejectsEqualFrameTimes(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, 0, 1000, 100, 200)
	setTrajectoryTimes(t, path, []int64{0, 100, 100})
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("equal frame times: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 相邻段之间时间倒退（末段第一帧不晚于前段末帧）：拒绝。
func TestOpenRejectsBackwardsAcrossSegments(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, 0, 1000, 100, 200)
	setTrajectoryTimes(t, path, []int64{0, 100, 99})
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("backwards across segments: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 段内时间倒退：拒绝。
func TestOpenRejectsBackwardsWithinSegment(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, 0, 1000, 100, 200, 300)
	setTrajectoryTimes(t, path, []int64{0, 100, 200, 150})
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("backwards within segment: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 相邻位姿间隔超过 MaxInterval：拒绝。
func TestOpenRejectsExcessiveInterval(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, 0, 1000, 100, 1000)
	// 构造文件时导入不可能超限，直接篡改：间隔 1001 > 1000。
	fd := readFileData(t, path)
	fd.Trajectory[2].Time = 1101
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("excessive interval: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 间隔恰好等于 MaxInterval 合法。
func TestOpenIntervalAtLimitAllowed(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, 0, 1000, 1000)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("interval at limit: %v", err)
	}
	m.Close()
}

// 溢出场景：初始时间 MinInt64、MaxInterval MaxInt64，第一帧为 0 时真实
// 间隔为 2^63，超过上限 2^63-1，即使有符号差值会回绕成负数也必须拒绝。
func TestOpenRejectsOverflowedIntervalToZero(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, math.MinInt64, math.MaxInt64, -1)
	setTrajectoryTimes(t, path, []int64{math.MinInt64, 0})
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("MinInt64->0: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 同一边界：MinInt64 -> -1 的真实间隔恰好为 MaxInt64，应当允许。
func TestOpenAllowsOverflowBoundaryAtMinusOne(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, math.MinInt64, math.MaxInt64, -1)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("MinInt64->-1 must open: %v", err)
	}
	m.Close()
}

// 溢出回绕不能放过轨迹中段的巨大缺口：MinInt64 -> -1 合法（间隔恰为
// MaxInt64），-1 -> MaxInt64 的真实间隔为 MaxInt64+1，必须拒绝（有符号
// 差值回绕成负数）。
func TestOpenRejectsOverflowedIntervalWithinTrajectory(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, math.MinInt64, math.MaxInt64, -1)
	fd := readFileData(t, path)
	fd.Trajectory = append(fd.Trajectory, Pose{Time: math.MaxInt64})
	fd.Sources = append(fd.Sources, nil) // 与帧数保持一一对应
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("-1->MaxInt64: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 该错误是文件损坏，不是一次新轨迹导入：不能识别为 *RejectError。
func TestOpenTimeErrorIsNotRejectError(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, 0, 1000, 100, 200)
	setTrajectoryTimes(t, path, []int64{0, 200, 200})
	_, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if r, ok := AsRejectError(err); ok {
		t.Fatalf("corrupt trajectory reported as frame rejection %+v", r)
	}
}

// 拒绝时保留原文件全部内容：不排序、不改写为新地图。
func TestOpenRejectedTrajectoryFileUntouched(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, 0, 1000, 100, 200)
	setTrajectoryTimes(t, path, []int64{0, 200, 100})
	saved, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
	got, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(got) != string(saved) {
		t.Fatal("Open modified the rejected file")
	}
}

// 缺少逐帧观测来源的旧文件同样受时间规则约束，但不能仅因来源缺失被拒：
// 时间合法的旧文件继续打开，时间异常的旧文件按损坏拒绝。
func TestOpenLegacyFileTimeRule(t *testing.T) {
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

	// 时间合法的旧文件正常打开，仍可继续导入。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("valid legacy file: %v", err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{frameAt(300)}}); err != nil {
		t.Fatalf("continue import on legacy map: %v", err)
	}
	m2.Close()

	// 篡改成倒退时间的旧文件：来源缺失不能掩盖时间损坏。
	fd := readFileData(t, path)
	fd.Trajectory[2].Time = 50
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("legacy bad times: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}
