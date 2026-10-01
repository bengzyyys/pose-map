package posemap

import (
	"errors"
	"math"
	"os"
	"path/filepath"
	"testing"
)

func newTestMap(t *testing.T, cfg Config) *Map {
	t.Helper()
	path := filepath.Join(t.TempDir(), "map.posemap")
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	t.Cleanup(func() { m.Close() })
	return m
}

func defaultCfg() Config {
	return Config{
		InitialTime:     1000,
		InitialX:        0,
		InitialY:        0,
		InitialTheta:    0,
		InitialVariance: 0.5,
		MaxInterval:     5000,
		MergeDistance:   1.0,
	}
}

func approx(t *testing.T, name string, got, want float64) {
	t.Helper()
	if math.Abs(got-want) > 1e-9 {
		t.Errorf("%s = %.12f, want %.12f", name, got, want)
	}
}

func TestReadyStillWorks(t *testing.T) {
	if !Ready() {
		t.Fatal("baseline not ready")
	}
}

func TestCreateAndInitialState(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	pose, err := m.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	if pose.Time != 1000 {
		t.Errorf("initial time = %d, want 1000", pose.Time)
	}
	approx(t, "initial variance", pose.Variance, 0.5)
	approx(t, "initial theta", pose.Theta, 0)

	// 初始时间点可以查到。
	if p, ok := m.HistoryAt(1000); !ok {
		t.Error("HistoryAt(1000) not found")
	} else {
		approx(t, "history x", p.X, 0)
	}
	// 早于初始时间查不到。
	if _, ok := m.HistoryAt(999); ok {
		t.Error("HistoryAt(999) unexpectedly found")
	}
}

func TestCreateRejectsExistingFile(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.posemap")
	if _, err := Create(path, defaultCfg()); err != nil {
		t.Fatal(err)
	}
	_, err := Create(path, defaultCfg())
	if !errors.Is(err, ErrFileExists) {
		t.Fatalf("expected ErrFileExists, got %v", err)
	}
}

func TestCreateRejectsBadConfig(t *testing.T) {
	bad := []Config{
		{InitialTime: 0, InitialVariance: -0.1, MaxInterval: 1, MergeDistance: 1},
		{InitialTime: 0, InitialVariance: 0, MaxInterval: 0, MergeDistance: 1},
		{InitialTime: 0, InitialVariance: 0, MaxInterval: 1, MergeDistance: 0},
		{InitialTime: 0, InitialVariance: 0, MaxInterval: 1, MergeDistance: -1},
		{InitialTime: 0, InitialVariance: math.NaN(), MaxInterval: 1, MergeDistance: 1},
		{InitialTime: 0, InitialVariance: 0, MaxInterval: 1, MergeDistance: math.Inf(1)},
	}
	for i, cfg := range bad {
		path := filepath.Join(t.TempDir(), "map.posemap")
		if _, err := Create(path, cfg); !errors.Is(err, ErrInvalidConfig) {
			t.Errorf("bad config %d: expected ErrInvalidConfig, got %v", i, err)
		}
	}
}

func TestImportBasicMotionAndLandmark(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	// 初始朝向 0：第一帧向前 1 米，朝向转到 π/2，方差 0.2。
	// 平移按运动前朝向（0）转换：位置 (1,0)。
	// 路标在机器人自身 (1,0)，按运动完成后朝向（π/2）转换：
	// 地图坐标 (1,0) + (cos π/2, sin π/2) = (1,1)。
	end, ids, err := m.Import("seg1", []Frame{
		{
			Time:     2000,
			DX:       1,
			DTheta:   math.Pi / 2,
			Variance: 0.2,
			Observations: []Observation{
				{ID: "a", X: 1, Y: 0},
			},
		},
	})
	if err != nil {
		t.Fatalf("Import: %v", err)
	}
	if end.Time != 2000 {
		t.Errorf("end time = %d, want 2000", end.Time)
	}
	approx(t, "end x", end.X, 1)
	approx(t, "end y", end.Y, 0)
	approx(t, "end theta", end.Theta, math.Pi/2)
	approx(t, "end variance", end.Variance, 0.7)
	if len(ids) != 1 || ids[0] != "a" {
		t.Errorf("ids = %v, want [a]", ids)
	}

	lms, err := m.QueryRect(-10, -10, 10, 10)
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 {
		t.Fatalf("landmarks = %v, want 1", lms)
	}
	approx(t, "landmark x", lms[0].X, 1)
	approx(t, "landmark y", lms[0].Y, 1)
	if lms[0].Count != 1 {
		t.Errorf("count = %d, want 1", lms[0].Count)
	}
}

func TestImportMotionUsesPreMotionHeading(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	// 先转到朝向 π/2，此时平移应按 π/2 转换：机器人 x 方向 -> 地图 y 方向。
	// 第一帧：原地转 π/2。第二帧：DX=1，地图位移应为 (0,1)。
	_, _, err := m.Import("seg1", []Frame{
		{Time: 2000, DTheta: math.Pi / 2},
		{Time: 3000, DX: 1},
	})
	if err != nil {
		t.Fatal(err)
	}
	pose, _ := m.CurrentPose()
	approx(t, "x", pose.X, 0)
	approx(t, "y", pose.Y, 1)
	approx(t, "theta", pose.Theta, math.Pi/2)
}

func TestImportNormalizesHeading(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	// 累计旋转超过 2π，应归一到 [-π, π)。
	end, _, err := m.Import("seg1", []Frame{
		{Time: 2000, DTheta: 3 * math.Pi},
	})
	if err != nil {
		t.Fatal(err)
	}
	// 3π 归一后为 π（math.Atan2(sin(3π), cos(3π)) ≈ π）。
	if end.Theta < -math.Pi || end.Theta >= math.Pi {
		t.Errorf("theta out of range: %v", end.Theta)
	}
	approx(t, "theta", end.Theta, math.Pi)

	// 负角度归一：第二段起点朝向为 π，转 -3π 后归一到 0。
	end2, _, err := m.Import("seg2", []Frame{
		{Time: 4000, DTheta: -3 * math.Pi},
	})
	if err != nil {
		t.Fatal(err)
	}
	approx(t, "theta2", end2.Theta, 0)
}

func TestImportVarianceAccumulates(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	end, _, err := m.Import("seg1", []Frame{
		{Time: 2000, Variance: 0.2},
		{Time: 3000, Variance: 0.3},
		{Time: 4000, Variance: 0.1},
	})
	if err != nil {
		t.Fatal(err)
	}
	approx(t, "variance", end.Variance, 0.5+0.2+0.3+0.1)
}

func TestLandmarkMergeAndAverage(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	// 第一段：路标 a 在地图 (2,0)，观测 1 次。
	_, _, err := m.Import("seg1", []Frame{
		{Time: 2000, DX: 1, Observations: []Observation{{ID: "a", X: 1}}},
	})
	if err != nil {
		t.Fatal(err)
	}

	// 第二段：位姿到 (2,0)，观测 a 在机器人 (0.5,0) -> 地图 (2.5,0)，
	// 距当前位置 0.5 <= 1，接受；平均位置 (2.25,0)，观测 2 次。
	_, ids, err := m.Import("seg2", []Frame{
		{Time: 3000, DX: 1, Observations: []Observation{{ID: "a", X: 0.5}}},
	})
	if err != nil {
		t.Fatalf("Import: %v", err)
	}
	if len(ids) != 1 || ids[0] != "a" {
		t.Errorf("ids = %v", ids)
	}

	lms, _ := m.QueryRect(-10, -10, 10, 10)
	if len(lms) != 1 {
		t.Fatalf("landmarks = %v", lms)
	}
	approx(t, "merged x", lms[0].X, 2.25)
	approx(t, "merged y", lms[0].Y, 0)
	if lms[0].Count != 2 {
		t.Errorf("count = %d, want 2", lms[0].Count)
	}
}

func TestLandmarkConflictRejectsSegment(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	// 第一段：路标 a 在地图 (2,0)。
	if _, _, err := m.Import("seg1", []Frame{
		{Time: 2000, DX: 1, Observations: []Observation{{ID: "a", X: 1}}},
	}); err != nil {
		t.Fatal(err)
	}

	// 第二段第二帧观测 a 在机器人 (5,0) -> 地图 (7,0)，距 (2,0) 为 5 > 1。
	// 整段拒绝：第一帧的位姿也不能留下。
	_, _, err := m.Import("seg2", []Frame{
		{Time: 3000, DX: 1},
		{Time: 4000, DX: 1, Observations: []Observation{{ID: "a", X: 5}}},
	})
	if !errors.Is(err, ErrLandmarkConflict) {
		t.Fatalf("expected ErrLandmarkConflict, got %v", err)
	}
	var e *Error
	if !errors.As(err, &e) {
		t.Fatalf("not *Error: %v", err)
	}
	if e.Frame != 1 {
		t.Errorf("frame = %d, want 1", e.Frame)
	}
	if e.ID != "a" {
		t.Errorf("id = %q, want a", e.ID)
	}

	// 第一帧的位姿不能留下：3500 晚于被拒段的两帧，但轨迹应停在 seg1 的 2000。
	p, _ := m.HistoryAt(3500)
	if p.Time != 2000 {
		t.Errorf("rejected segment left a pose: history at 3500 = %d", p.Time)
	}
	pose, _ := m.CurrentPose()
	if pose.Time != 2000 {
		t.Errorf("current pose time = %d, want 2000", pose.Time)
	}

	// 修正后（观测改为机器人 (0,0) -> 地图 (3,0)，距 (2,0) 为 1 <= 1）可以再次导入。
	end, _, err := m.Import("seg2", []Frame{
		{Time: 3000, DX: 1},
		{Time: 4000, DX: 1, Observations: []Observation{{ID: "a", X: 0}}},
	})
	if err != nil {
		t.Fatalf("fixed Import: %v", err)
	}
	if end.Time != 4000 {
		t.Errorf("end time = %d, want 4000", end.Time)
	}
}

func TestSameFrameMultipleObservations(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	// 同帧对同一标识观测两次：第一次新增，第二次距离为 0 接受，
	// 平均位置为两次观测的平均，观测 2 次。
	_, _, err := m.Import("seg1", []Frame{
		{
			Time: 2000,
			Observations: []Observation{
				{ID: "a", X: 1},
				{ID: "a", X: 2},
			},
		},
	})
	if err != nil {
		t.Fatal(err)
	}
	lms, _ := m.QueryRect(-10, -10, 10, 10)
	if len(lms) != 1 {
		t.Fatalf("landmarks = %v", lms)
	}
	approx(t, "x", lms[0].X, 1.5)
	if lms[0].Count != 2 {
		t.Errorf("count = %d, want 2", lms[0].Count)
	}
}

func TestRejectionReasons(t *testing.T) {
	t.Run("empty batch", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		_, _, err := m.Import("seg1", nil)
		if !errors.Is(err, ErrEmptyBatch) {
			t.Fatalf("got %v", err)
		}
	})
	t.Run("empty segment id", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		_, _, err := m.Import("", []Frame{{Time: 2000}})
		if !errors.Is(err, ErrEmptySegmentID) {
			t.Fatalf("got %v", err)
		}
	})
	t.Run("empty landmark id", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		_, _, err := m.Import("seg1", []Frame{
			{Time: 2000, Observations: []Observation{{ID: "", X: 1}}},
		})
		if !errors.Is(err, ErrEmptyLandmarkID) {
			t.Fatalf("got %v", err)
		}
		var e *Error
		errors.As(err, &e)
		if e.Frame != 0 {
			t.Errorf("frame = %d", e.Frame)
		}
	})
	t.Run("negative variance", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		_, _, err := m.Import("seg1", []Frame{
			{Time: 2000, Variance: -0.1},
		})
		if !errors.Is(err, ErrNegativeVariance) {
			t.Fatalf("got %v", err)
		}
	})
	t.Run("non finite", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		_, _, err := m.Import("seg1", []Frame{
			{Time: 2000, DX: math.NaN()},
		})
		if !errors.Is(err, ErrNonFinite) {
			t.Fatalf("got %v", err)
		}
		_, _, err = m.Import("seg2", []Frame{
			{Time: 2000, Observations: []Observation{{ID: "a", X: math.Inf(1)}}},
		})
		if !errors.Is(err, ErrNonFinite) {
			t.Fatalf("obs got %v", err)
		}
	})
	t.Run("time not increasing within segment", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		_, _, err := m.Import("seg1", []Frame{
			{Time: 2000},
			{Time: 2000},
		})
		if !errors.Is(err, ErrTimeNotIncreasing) {
			t.Fatalf("got %v", err)
		}
		var e *Error
		errors.As(err, &e)
		if e.Frame != 1 {
			t.Errorf("frame = %d, want 1", e.Frame)
		}
	})
	t.Run("first frame not after initial", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		_, _, err := m.Import("seg1", []Frame{{Time: 1000}})
		if !errors.Is(err, ErrTimeNotIncreasing) {
			t.Fatalf("got %v", err)
		}
	})
	t.Run("first frame not after previous segment", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		if _, _, err := m.Import("seg1", []Frame{{Time: 2000}}); err != nil {
			t.Fatal(err)
		}
		_, _, err := m.Import("seg2", []Frame{{Time: 2000}})
		if !errors.Is(err, ErrTimeNotIncreasing) {
			t.Fatalf("got %v", err)
		}
	})
	t.Run("interval too large", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		_, _, err := m.Import("seg1", []Frame{
			{Time: 2000},
			{Time: 7001},
		})
		if !errors.Is(err, ErrIntervalTooLarge) {
			t.Fatalf("got %v", err)
		}
		var e *Error
		errors.As(err, &e)
		if e.Frame != 1 {
			t.Errorf("frame = %d, want 1", e.Frame)
		}
	})
	t.Run("interval exactly at limit is allowed", func(t *testing.T) {
		m := newTestMap(t, defaultCfg())
		_, _, err := m.Import("seg1", []Frame{
			{Time: 2000},
			{Time: 7000},
		})
		if err != nil {
			t.Fatalf("exact limit rejected: %v", err)
		}
	})
}

func TestRejectedSegmentLeavesNoTrace(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	// seg1：路标 a 在地图 (2,0)。
	if _, _, err := m.Import("seg1", []Frame{
		{Time: 2000, DX: 1, Observations: []Observation{{ID: "a", X: 1}}},
	}); err != nil {
		t.Fatal(err)
	}

	// seg2：第二帧观测 a 在机器人 (100,0) -> 地图 (103,0)，距 (2,0) 超合并距离，整段拒绝。
	_, _, err := m.Import("seg2", []Frame{
		{Time: 3000, DX: 1},
		{Time: 4000, DX: 1, Observations: []Observation{{ID: "a", X: 100}}},
	})
	if !errors.Is(err, ErrLandmarkConflict) {
		t.Fatalf("got %v", err)
	}

	// 位姿不能留下：3500 应停在 seg1 的 2000。
	p, _ := m.HistoryAt(3500)
	if p.Time != 2000 {
		t.Errorf("pose from rejected segment left: history at 3500 = %d", p.Time)
	}
	// 路标位置与观测次数不变。
	lms, _ := m.QueryRect(-1000, -1000, 1000, 1000)
	if len(lms) != 1 || lms[0].ID != "a" || lms[0].Count != 1 {
		t.Errorf("landmarks changed after rejection: %v", lms)
	}

	// 段标识未被占用，修正后（观测改为机器人 (0,0) -> 地图 (3,0)，距离 1）可以再次导入。
	end, _, err := m.Import("seg2", []Frame{
		{Time: 3000, DX: 1},
		{Time: 4000, DX: 1, Observations: []Observation{{ID: "a", X: 0}}},
	})
	if err != nil {
		t.Fatalf("re-import after fix: %v", err)
	}
	if end.Time != 4000 {
		t.Errorf("end time = %d", end.Time)
	}
}

func TestReimportSameContentReturnsStoredResult(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	frames := []Frame{
		{Time: 2000, DX: 1, Observations: []Observation{{ID: "a", X: 1}}},
	}
	end1, ids1, err := m.Import("seg1", frames)
	if err != nil {
		t.Fatal(err)
	}

	// 再导入一段其他内容。
	if _, _, err := m.Import("seg2", []Frame{
		{Time: 3000, DX: 2, Observations: []Observation{{ID: "b", X: 1}}},
	}); err != nil {
		t.Fatal(err)
	}

	// 重复导入 seg1：返回首次结果，当前数据不变。
	end2, ids2, err := m.Import("seg1", frames)
	if err != nil {
		t.Fatalf("re-import: %v", err)
	}
	if end2 != end1 {
		t.Errorf("re-import end = %+v, want %+v", end2, end1)
	}
	if len(ids2) != len(ids1) || ids2[0] != ids1[0] {
		t.Errorf("re-import ids = %v, want %v", ids2, ids1)
	}

	// 当前位姿仍是 seg2 的末位姿。
	pose, _ := m.CurrentPose()
	if pose.Time != 3000 {
		t.Errorf("current pose time = %d, want 3000", pose.Time)
	}
}

func TestReimportSameIDDifferentContentRejected(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	if _, _, err := m.Import("seg1", []Frame{{Time: 2000, DX: 1}}); err != nil {
		t.Fatal(err)
	}
	_, _, err := m.Import("seg1", []Frame{{Time: 2000, DX: 2}})
	if !errors.Is(err, ErrSegmentConflict) {
		t.Fatalf("expected ErrSegmentConflict, got %v", err)
	}
	var e *Error
	errors.As(err, &e)
	if e.ID != "seg1" {
		t.Errorf("id = %q, want seg1", e.ID)
	}

	// 冲突不能改变数据。
	pose, _ := m.CurrentPose()
	approx(t, "x after conflict", pose.X, 1)
}

func TestHistoryQuery(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	if _, _, err := m.Import("seg1", []Frame{
		{Time: 2000, DX: 1},
		{Time: 3000, DX: 1},
	}); err != nil {
		t.Fatal(err)
	}

	cases := []struct {
		t    int64
		want float64
		ok   bool
	}{
		{999, 0, false}, // 早于初始时间
		{1000, 0, true}, // 恰为初始时间
		{1500, 0, true}, // 初始与第一帧之间
		{2000, 1, true}, // 恰为第一帧
		{2500, 1, true}, // 两帧之间
		{3000, 2, true}, // 恰为第二帧
		{9999, 2, true}, // 晚于最后一帧
	}
	for _, c := range cases {
		p, ok := m.HistoryAt(c.t)
		if ok != c.ok {
			t.Errorf("HistoryAt(%d) ok = %v, want %v", c.t, ok, c.ok)
			continue
		}
		if ok {
			approx(t, "x", p.X, c.want)
		}
	}
}

func TestQueryRect(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	// 三个路标：a(0,0)、b(3,3)、c(6,6)。
	if _, _, err := m.Import("seg1", []Frame{
		{Time: 2000, Observations: []Observation{{ID: "a", X: 0}}},
		{Time: 3000, DX: 3, Observations: []Observation{{ID: "b", X: 0}}},
		{Time: 4000, DX: 3, Observations: []Observation{{ID: "c", X: 0}}},
	}); err != nil {
		t.Fatal(err)
	}

	t.Run("inclusive bounds and sorted", func(t *testing.T) {
		lms, err := m.QueryRect(0, 0, 3, 3)
		if err != nil {
			t.Fatal(err)
		}
		if len(lms) != 2 {
			t.Fatalf("got %d landmarks: %v", len(lms), lms)
		}
		if lms[0].ID != "a" || lms[1].ID != "b" {
			t.Errorf("order = %s,%s, want a,b", lms[0].ID, lms[1].ID)
		}
	})
	t.Run("empty region", func(t *testing.T) {
		lms, err := m.QueryRect(100, 100, 200, 200)
		if err != nil {
			t.Fatal(err)
		}
		if len(lms) != 0 {
			t.Errorf("got %v, want empty", lms)
		}
	})
	t.Run("reversed x bounds", func(t *testing.T) {
		_, err := m.QueryRect(3, 0, 0, 3)
		if !errors.Is(err, ErrReversedRect) {
			t.Fatalf("got %v", err)
		}
	})
	t.Run("reversed y bounds", func(t *testing.T) {
		_, err := m.QueryRect(0, 3, 3, 0)
		if !errors.Is(err, ErrReversedRect) {
			t.Fatalf("got %v", err)
		}
	})
}

func TestPersistenceRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.posemap")
	m, err := Create(path, defaultCfg())
	if err != nil {
		t.Fatal(err)
	}

	frames := []Frame{
		{Time: 2000, DX: 1, Observations: []Observation{{ID: "a", X: 1}}},
		{Time: 3000, DX: 1, Observations: []Observation{{ID: "a", X: 1}}},
	}
	endBefore, idsBefore, err := m.Import("seg1", frames)
	if err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 重新打开。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	defer m2.Close()

	// 查询结果与关闭前一致。
	pose, err := m2.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	if pose.Time != endBefore.Time {
		t.Errorf("pose time = %d, want %d", pose.Time, endBefore.Time)
	}
	approx(t, "x", pose.X, endBefore.X)
	approx(t, "variance", pose.Variance, endBefore.Variance)

	if p, ok := m2.HistoryAt(2500); !ok {
		t.Error("HistoryAt(2500) not found after reopen")
	} else {
		approx(t, "history x", p.X, 1)
	}

	lms, err := m2.QueryRect(-10, -10, 10, 10)
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].ID != "a" || lms[0].Count != 2 {
		t.Errorf("landmarks after reopen = %v", lms)
	}
	approx(t, "landmark x", lms[0].X, 2.5)

	// 重复导入结果与首次一致。
	endAfter, idsAfter, err := m2.Import("seg1", frames)
	if err != nil {
		t.Fatalf("re-import after reopen: %v", err)
	}
	if endAfter != endBefore {
		t.Errorf("end = %+v, want %+v", endAfter, endBefore)
	}
	if len(idsAfter) != len(idsBefore) || idsAfter[0] != idsBefore[0] {
		t.Errorf("ids = %v, want %v", idsAfter, idsBefore)
	}

	// 重新打开后导入新段仍正常。
	if _, _, err := m2.Import("seg2", []Frame{{Time: 4000}}); err != nil {
		t.Fatalf("new import after reopen: %v", err)
	}
}

func TestOpenCorruptFile(t *testing.T) {
	t.Run("garbage", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "map.posemap")
		if err := os.WriteFile(path, []byte("this is not a map file at all"), 0o644); err != nil {
			t.Fatal(err)
		}
		_, err := Open(path)
		if !errors.Is(err, ErrCorruptFile) {
			t.Fatalf("got %v", err)
		}
	})
	t.Run("truncated", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "map.posemap")
		m, err := Create(path, defaultCfg())
		if err != nil {
			t.Fatal(err)
		}
		m.Close()
		data, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, data[:len(data)-5], 0o644); err != nil {
			t.Fatal(err)
		}
		_, err = Open(path)
		if !errors.Is(err, ErrCorruptFile) {
			t.Fatalf("got %v", err)
		}
	})
	t.Run("bad checksum", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "map.posemap")
		m, err := Create(path, defaultCfg())
		if err != nil {
			t.Fatal(err)
		}
		m.Close()
		data, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		// 翻转载荷中的一个字节（位于 header 与 checksum 之间）。
		data[20] ^= 0xff
		if err := os.WriteFile(path, data, 0o644); err != nil {
			t.Fatal(err)
		}
		_, err = Open(path)
		if !errors.Is(err, ErrCorruptFile) {
			t.Fatalf("got %v", err)
		}
	})
	t.Run("unsupported version", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "map.posemap")
		m, err := Create(path, defaultCfg())
		if err != nil {
			t.Fatal(err)
		}
		m.Close()
		data, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		// 版本号位于偏移 8..12。
		data[8] = 2
		if err := os.WriteFile(path, data, 0o644); err != nil {
			t.Fatal(err)
		}
		_, err = Open(path)
		if !errors.Is(err, ErrUnsupportedVersion) {
			t.Fatalf("got %v", err)
		}
	})
}

func TestOperationsAfterClose(t *testing.T) {
	m := newTestMap(t, defaultCfg())
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := m.CurrentPose(); !errors.Is(err, ErrClosed) {
		t.Fatalf("CurrentPose got %v", err)
	}
	if _, _, err := m.Import("seg", []Frame{{Time: 2000}}); !errors.Is(err, ErrClosed) {
		t.Fatalf("Import got %v", err)
	}
	if _, err := m.QueryRect(0, 0, 1, 1); !errors.Is(err, ErrClosed) {
		t.Fatalf("QueryRect got %v", err)
	}
	if _, ok := m.HistoryAt(2000); ok {
		t.Error("HistoryAt after close returned found")
	}
}

func TestInvolvedLandmarkIDsSorted(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	_, ids, err := m.Import("seg1", []Frame{
		{
			Time: 2000,
			Observations: []Observation{
				{ID: "z", X: 1},
				{ID: "a", X: 1},
				{ID: "m", X: 1},
				{ID: "a", X: 2},
			},
		},
	})
	if err != nil {
		t.Fatal(err)
	}
	want := []string{"a", "m", "z"}
	if len(ids) != len(want) {
		t.Fatalf("ids = %v, want %v", ids, want)
	}
	for i := range want {
		if ids[i] != want[i] {
			t.Errorf("ids = %v, want %v", ids, want)
			break
		}
	}
}

func TestSegmentWithNoLandmarks(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	end, ids, err := m.Import("seg1", []Frame{{Time: 2000, DX: 5}})
	if err != nil {
		t.Fatal(err)
	}
	if end.X != 5 {
		t.Errorf("end x = %v, want 5", end.X)
	}
	if len(ids) != 0 {
		t.Errorf("ids = %v, want empty", ids)
	}
}

func TestMultipleSegmentsChain(t *testing.T) {
	m := newTestMap(t, defaultCfg())

	for i, seg := range []string{"s1", "s2", "s3"} {
		_, _, err := m.Import(seg, []Frame{
			{Time: int64(2000 + i*1000), DX: 1},
		})
		if err != nil {
			t.Fatalf("segment %s: %v", seg, err)
		}
	}
	pose, _ := m.CurrentPose()
	if pose.Time != 4000 {
		t.Errorf("current time = %d, want 4000", pose.Time)
	}
	approx(t, "x", pose.X, 3)

	// 历史回溯各段末帧。
	for _, c := range []struct {
		t    int64
		want float64
	}{
		{2000, 1},
		{3000, 2},
		{4000, 3},
		{2500, 1},
		{3500, 2},
	} {
		p, ok := m.HistoryAt(c.t)
		if !ok {
			t.Errorf("HistoryAt(%d) not found", c.t)
			continue
		}
		approx(t, "x", p.X, c.want)
	}
}
