package posemap

import (
	"errors"
	"path/filepath"
	"testing"
)

// 输入有限但运动结果溢出：当前位置横坐标 1e308 再沿地图横轴正向移动
// 1e308，位置叠加得到无穷，应按 non_finite 拒绝并指出发生异常的帧；
// 朝向与累计方差溢出遵守同一规则。
func TestImportNonFinitePoseResult(t *testing.T) {
	newAt := func(t *testing.T, x, variance float64) *Map {
		t.Helper()
		m, err := Create(filepath.Join(t.TempDir(), "map.bin"), Config{
			InitialTime: 1000, InitialX: x, InitialVariance: variance,
			MaxInterval: 100, MergeDistance: 10,
		})
		if err != nil {
			t.Fatalf("Create: %v", err)
		}
		t.Cleanup(func() { m.Close() })
		return m
	}

	t.Run("position overflow", func(t *testing.T) {
		m := newAt(t, 1e308, 0)
		_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 1001, DX: 1e308},
		}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite || !r.HasFrame || r.Frame != 0 || r.HasLandmark {
			t.Fatalf("got %v, want non_finite frame 0 without landmark", err)
		}
	})

	t.Run("variance overflow", func(t *testing.T) {
		m := newAt(t, 0, 1e308)
		_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 1001, MoveVariance: 1e308},
		}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite || !r.HasFrame || r.Frame != 0 {
			t.Fatalf("got %v, want non_finite frame 0", err)
		}
	})
}

// 位姿正常而观测转换到地图坐标后溢出：按 non_finite 拒绝并携带路标
// 标识，不得误报为距离冲突；首次出现、失效后再次出现、与有效记录合并
// 三种情形结果一致。只有转换结果有限但超过合并距离时才按路标冲突拒绝。
func TestImportNonFiniteObservationResult(t *testing.T) {
	newAt := func(t *testing.T) *Map {
		t.Helper()
		m, err := Create(filepath.Join(t.TempDir(), "map.bin"), Config{
			InitialTime: 1000, MaxInterval: 100, MergeDistance: 10,
		})
		if err != nil {
			t.Fatalf("Create: %v", err)
		}
		t.Cleanup(func() { m.Close() })
		return m
	}
	check := func(t *testing.T, err error, frame int, landmark string) {
		t.Helper()
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite || !r.HasFrame || r.Frame != frame ||
			!r.HasLandmark || r.Landmark != landmark {
			t.Fatalf("got %v, want non_finite frame %d landmark %q", err, frame, landmark)
		}
	}

	t.Run("first appearance", func(t *testing.T) {
		m := newAt(t)
		// 位姿 (1e308, 0) 有限，观测 (1e308, 0) 转换后横坐标溢出。
		_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 1001, DX: 1e308, Observations: []Observation{{ID: "L", X: 1e308}}},
		}})
		check(t, err, 0, "L")
	})

	t.Run("merge with active record", func(t *testing.T) {
		m := newAt(t)
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 1001, Observations: []Observation{{ID: "L", X: 1e308}}},
		}}); err != nil {
			t.Fatalf("setup import: %v", err)
		}
		// L 有效记录在 (1e308, 0)；位姿移到 (1e308, 0) 后观测 (1e308, 0)
		// 转换溢出。若按距离判定会误报冲突。
		_, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 1002, DX: 1e308, Observations: []Observation{{ID: "L", X: 1e308}}},
		}})
		check(t, err, 0, "L")
	})

	t.Run("reappearance after invalidation", func(t *testing.T) {
		m := newAt(t)
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 1001, Observations: []Observation{{ID: "L", X: 1}}},
		}}); err != nil {
			t.Fatalf("setup import: %v", err)
		}
		if _, err := m.Invalidate(Invalidation{ID: "inv", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
			t.Fatalf("Invalidate: %v", err)
		}
		_, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 1002, DX: 1e308, Observations: []Observation{{ID: "L", X: 1e308}}},
		}})
		check(t, err, 0, "L")
		// 整段被拒绝不消耗出现编号：重新以有限观测导入，L 是第 2 次出现。
		if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 1002, Observations: []Observation{{ID: "L", X: 5}}},
		}}); err != nil {
			t.Fatalf("re-import: %v", err)
		}
		h, err := m.LandmarkAppearances("L")
		if err != nil {
			t.Fatalf("LandmarkAppearances: %v", err)
		}
		if len(h.Appearances) != 2 || !h.Appearances[1].Active || h.Appearances[1].Landmark.Count != 1 {
			t.Fatalf("appearances = %+v", h.Appearances)
		}
	})

	t.Run("finite but beyond merge distance stays conflict", func(t *testing.T) {
		m := newAt(t)
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 1001, Observations: []Observation{{ID: "L", X: 0}}},
		}}); err != nil {
			t.Fatalf("setup import: %v", err)
		}
		_, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 1002, Observations: []Observation{{ID: "L", X: 100}}},
		}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectLandmarkConflict || !r.HasLandmark || r.Landmark != "L" {
			t.Fatalf("got %v, want landmark_conflict for L", err)
		}
	})
}

// 异常出现在后面的帧时整段拒绝：前面暂时算出的位姿、新路标与观测次数
// 不可查询，段标识不被消耗，已提交地图保持原样。
func TestImportNonFiniteResultAtomic(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.bin")
	m, err := Create(path, Config{
		InitialTime: 1000, MaxInterval: 100, MergeDistance: 10,
	})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	defer m.Close()

	_, err = m.ImportSegment(Segment{ID: "bad", Frames: []Frame{
		{Time: 1001, DX: 1e308, Observations: []Observation{{ID: "A", X: 2}}},
		{Time: 1002, DX: 1e308},
	}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectNonFinite || !r.HasFrame || r.Frame != 1 {
		t.Fatalf("got %v, want non_finite frame 1", err)
	}

	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatalf("CurrentPose: %v", err)
	}
	if cur.Time != 1000 || cur.X != 0 || cur.Variance != 0 {
		t.Fatalf("pose leaked: %+v", cur)
	}
	if _, err := m.LandmarkAppearances("A"); !errors.Is(err, ErrLandmarkNotFound) {
		t.Fatalf("landmark A leaked: err = %v", err)
	}

	// 段标识未消耗：修正内容后以同一标识导入成功。
	if _, err := m.ImportSegment(Segment{ID: "bad", Frames: []Frame{
		{Time: 1001, DX: 1e308, Observations: []Observation{{ID: "A", X: 2}}},
		{Time: 1002, DX: 1},
	}}); err != nil {
		t.Fatalf("re-import with fixed content: %v", err)
	}
	cur, _ = m.CurrentPose()
	if cur.Time != 1002 || cur.X != 1e308+1 {
		t.Fatalf("after re-import pose = %+v", cur)
	}
}
