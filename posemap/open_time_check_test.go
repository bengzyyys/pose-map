package posemap

import (
	"errors"
	"math"
	"os"
	"path/filepath"
	"testing"
)

// buildTimedMap 构造含两帧（t=100、200）的地图，无路标，便于单独篡改
// 轨迹时间。
func buildTimedMap(t *testing.T, path string, cfg Config, times ...int64) {
	t.Helper()
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	frames := make([]Frame, len(times))
	for i, tm := range times {
		frames[i] = Frame{Time: tm, DX: 1}
	}
	if len(frames) > 0 {
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: frames}); err != nil {
			t.Fatal(err)
		}
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// setTrajectoryTimes 重写文件中保存的轨迹时间（长度不变）并重算校验和。
// 段首次导入结果的末位姿时间按原命中的帧改指改时后的同一帧，保持文件
// 内部一致；时间规则本身的非法用例仍会在更早的轨迹时间检查处被拒绝。
func setTrajectoryTimes(t *testing.T, path string, times ...int64) {
	t.Helper()
	fd := readFileData(t, path)
	if len(fd.Trajectory) != len(times) {
		t.Fatalf("trajectory has %d poses, want %d", len(fd.Trajectory), len(times))
	}
	oldTimes := make([]int64, len(fd.Trajectory))
	for i, p := range fd.Trajectory {
		oldTimes[i] = p.Time
		fd.Trajectory[i].Time = times[i]
	}
	for i := range fd.Segments {
		for j, old := range oldTimes {
			if fd.Segments[i].Result.EndPose.Time == old {
				fd.Segments[i].Result.EndPose.Time = times[j]
				break
			}
		}
	}
	writeFileDataRaw(t, path, &fd)
}

// 打开时轨迹时间相等、倒退或间隔超限都按 ErrCorrupt 拒绝，不返回地图
// 对象，且原文件内容保持不变。
func TestOpenRejectsBadTrajectoryTimes(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	cases := []struct {
		name  string
		times []int64
	}{
		{"equal within segment", []int64{0, 100, 100}},
		{"backwards within segment", []int64{0, 200, 100}},
		{"equal to initial", []int64{0, 0, 100}},
		{"interval exceeded", []int64{0, 100, 1101}},
		{"interval exceeded from initial", []int64{0, 1001, 1002}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildTimedMap(t, path, cfg, 100, 200)
			setTrajectoryTimes(t, path, tc.times...)
			before, err := os.ReadFile(path)
			if err != nil {
				t.Fatal(err)
			}
			m, err := Open(path)
			if !errors.Is(err, ErrCorrupt) {
				t.Fatalf("err = %v, want ErrCorrupt", err)
			}
			if m != nil {
				t.Fatalf("map = %v, want nil", m)
			}
			after, err := os.ReadFile(path)
			if err != nil {
				t.Fatal(err)
			}
			if string(before) != string(after) {
				t.Fatal("file modified by rejected Open")
			}
		})
	}
}

// 间隔恰好等于 MaxInterval 合法；仅含初始位姿的地图也正常打开。
func TestOpenAcceptsBoundaryTrajectoryTimes(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, cfg, 100, 200)
	setTrajectoryTimes(t, path, 0, 1000, 2000)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	if p, err := m.PoseAt(1500); err != nil || p.Time != 1000 {
		t.Fatalf("PoseAt(1500) = %+v %v", p, err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	empty := filepath.Join(t.TempDir(), "empty.pose")
	buildTimedMap(t, empty, cfg)
	m, err = Open(empty)
	if err != nil {
		t.Fatalf("open initial-only map: %v", err)
	}
	if p, err := m.CurrentPose(); err != nil || p.Time != 0 {
		t.Fatalf("CurrentPose = %+v %v", p, err)
	}
	m.Close()
}

// 时间差按真实数学差值判断：初始时间 MinInt64、MaxInterval 取 MaxInt64
// 时，跳到 0 的真实间隔为 2^63，超过上限必须拒绝；跳到 -1 的间隔恰好
// 等于上限，应当允许。
func TestOpenTrajectoryIntervalOverflow(t *testing.T) {
	cfg := Config{InitialTime: math.MinInt64, MaxInterval: math.MaxInt64, MergeDistance: 1.0}

	path := filepath.Join(t.TempDir(), "m.pose")
	buildTimedMap(t, path, cfg, -1)
	setTrajectoryTimes(t, path, math.MinInt64, 0)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("MinInt64 -> 0: err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}

	ok := filepath.Join(t.TempDir(), "ok.pose")
	buildTimedMap(t, ok, cfg, -1)
	m, err := Open(ok)
	if err != nil {
		t.Fatalf("MinInt64 -> -1: %v", err)
	}
	if p, err := m.CurrentPose(); err != nil || p.Time != -1 {
		t.Fatalf("CurrentPose = %+v %v", p, err)
	}
	m.Close()
}

// 缺少逐帧来源的旧文件同样要满足时间规则：时间异常即拒绝；时间正常时
// 不因来源缺失被拒绝，原有查询与继续导入保持可用。
func TestOpenLegacyFileTrajectoryTimes(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}

	bad := filepath.Join(t.TempDir(), "bad.pose")
	buildTimedMap(t, bad, cfg, 100, 200)
	fd := readFileData(t, bad)
	fd.Sources = nil // 模拟不携带逐帧依据的旧文件
	fd.Trajectory[2].Time = 100
	writeFileDataRaw(t, bad, &fd)
	if m, err := Open(bad); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("legacy bad times: err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}

	good := filepath.Join(t.TempDir(), "good.pose")
	buildTimedMap(t, good, cfg, 100, 200)
	fd = readFileData(t, good)
	fd.Sources = nil
	writeFileDataRaw(t, good, &fd)
	m, err := Open(good)
	if err != nil {
		t.Fatalf("legacy open: %v", err)
	}
	defer m.Close()
	if p, err := m.PoseAt(150); err != nil || p.Time != 100 {
		t.Fatalf("PoseAt(150) = %+v %v", p, err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{{Time: 300, DX: 1}}}); err != nil {
		t.Fatalf("legacy continue import: %v", err)
	}
}
