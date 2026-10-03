package posemap

import (
	"math"
	"path/filepath"
	"testing"
)

// 大数值坐标下求路标平均位置不得溢出：两次 (1e308, 0) 观测应保存为
// (1e308, 0)、次数 2，而不是因中间计算得到无穷而拒绝。
func TestImportHugeCoordinateMean(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.bin")
	m, err := Create(path, Config{
		InitialTime: 1000, MaxInterval: 100, MergeDistance: 10,
	})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	defer m.Close()

	obs := []Observation{{ID: "L", X: 1e308, Y: 0}}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 1001, Observations: obs},
		{Time: 1002, Observations: obs},
	}}); err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	lms, err := m.LandmarksInRect(Rect{MinX: 0, MinY: -1, MaxX: math.MaxFloat64, MaxY: 1})
	if err != nil {
		t.Fatalf("LandmarksInRect: %v", err)
	}
	if len(lms) != 1 {
		t.Fatalf("want 1 landmark, got %d", len(lms))
	}
	if lms[0].X != 1e308 || lms[0].Y != 0 || lms[0].Count != 2 {
		t.Fatalf("got %+v, want X=1e308 Y=0 Count=2", lms[0])
	}

	// 关闭重开后结果一致。
	if err := m.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	defer m2.Close()
	lms2, err := m2.LandmarksInRect(Rect{MinX: 0, MinY: -1, MaxX: math.MaxFloat64, MaxY: 1})
	if err != nil {
		t.Fatalf("LandmarksInRect after reopen: %v", err)
	}
	if len(lms2) != 1 || lms2[0].X != 1e308 || lms2[0].Count != 2 {
		t.Fatalf("after reopen got %+v", lms2)
	}
}

// 混合大数值观测按等权平均：两次 (8e307, 0) 加一次 (1e308, 0)，合并距离
// 3e307，三次都接受，均值约 8.666666666666667e307、次数 3，且结果有限并
// 落在参与平均的坐标范围内。纵坐标与负数坐标遵守同一规则。
func TestImportHugeCoordinateWeightedMean(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.bin")
	m, err := Create(path, Config{
		InitialTime: 1000, MaxInterval: 100, MergeDistance: 3e307,
	})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	defer m.Close()

	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 1001, Observations: []Observation{
			{ID: "L", X: 8e307, Y: 0},
			{ID: "L", X: 8e307, Y: 0},
			{ID: "L", X: 1e308, Y: 0},
			{ID: "N", X: -8e307, Y: 1e308},
			{ID: "N", X: -8e307, Y: 1e308},
			{ID: "N", X: -1e308, Y: 8e307},
		}},
	}}); err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	lms, err := m.LandmarksInRect(Rect{MinX: -math.MaxFloat64, MinY: -math.MaxFloat64, MaxX: math.MaxFloat64, MaxY: math.MaxFloat64})
	if err != nil {
		t.Fatalf("LandmarksInRect: %v", err)
	}
	if len(lms) != 2 {
		t.Fatalf("want 2 landmarks, got %d", len(lms))
	}
	want := 8e307 + (1e308-8e307)/3 // 8.666666666666667e307
	for _, lm := range lms {
		if lm.Count != 3 {
			t.Fatalf("%s: count = %d, want 3", lm.ID, lm.Count)
		}
		if !isFinite(lm.X) || !isFinite(lm.Y) {
			t.Fatalf("%s: non-finite position %+v", lm.ID, lm)
		}
	}
	if math.Abs(lms[0].X-want) > math.Abs(want)*1e-12 || lms[0].X < 8e307 || lms[0].X > 1e308 {
		t.Fatalf("L.X = %v, want ~%v within [8e307, 1e308]", lms[0].X, want)
	}
	if lms[0].ID != "L" || lms[1].ID != "N" {
		t.Fatalf("unexpected ids: %+v", lms)
	}
	if math.Abs(lms[1].X+want) > math.Abs(want)*1e-12 || lms[1].X > -8e307 || lms[1].X < -1e308 {
		t.Fatalf("N.X = %v, want ~%v", lms[1].X, -want)
	}
	wantY := 1e308 + (8e307-1e308)/3 // 9.333333333333333e307
	if math.Abs(lms[1].Y-wantY) > math.Abs(wantY)*1e-12 || lms[1].Y < 8e307 || lms[1].Y > 1e308 {
		t.Fatalf("N.Y = %v, want ~%v", lms[1].Y, wantY)
	}
}

// 回环校正在大数值坐标下重放观测：校正后观测坐标有限且每步满足合并距离
// 时不得因均值计算失败；记录中的校正后位置与查询结果一致，次数不变。
func TestCorrectionHugeCoordinateMean(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.bin")
	m, err := Create(path, Config{
		InitialTime: 1000, MaxInterval: 100, MergeDistance: 3e307,
	})
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	defer m.Close()

	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 1001, Observations: []Observation{{ID: "L", X: 8e307, Y: 0}}},
		{Time: 1002, Observations: []Observation{{ID: "L", X: 8e307, Y: 0}}},
		{Time: 1003, Observations: []Observation{{ID: "L", X: 1e308, Y: 0}}},
	}}); err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}

	// 锚点帧平移 1e307：三帧重放后的观测坐标为 9e307、9e307、1.1e308，
	// 逐步与均值比较均不超过 3e307，整次校正应成功。
	rec, err := m.Correct(Correction{
		ID:     "c1",
		Anchor: 1001,
		Target: CorrectionTarget{X: 1e307, Y: 0, Heading: 0, Variance: 0},
	})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if len(rec.Landmarks) != 1 {
		t.Fatalf("want 1 landmark change, got %d", len(rec.Landmarks))
	}
	lms, err := m.LandmarksInRect(Rect{MinX: -math.MaxFloat64, MinY: -math.MaxFloat64, MaxX: math.MaxFloat64, MaxY: math.MaxFloat64})
	if err != nil {
		t.Fatalf("LandmarksInRect: %v", err)
	}
	if len(lms) != 1 {
		t.Fatalf("want 1 landmark, got %d", len(lms))
	}
	lc := rec.Landmarks[0]
	if lc.After.Count != 3 || lms[0].Count != 3 {
		t.Fatalf("count changed: record %d, query %d", lc.After.Count, lms[0].Count)
	}
	if lc.After.X != lms[0].X || lc.After.Y != lms[0].Y {
		t.Fatalf("record after (%v, %v) != query (%v, %v)", lc.After.X, lc.After.Y, lms[0].X, lms[0].Y)
	}
	want := 9e307 + (1.1e308-9e307)/3
	if !isFinite(lms[0].X) || math.Abs(lms[0].X-want) > math.Abs(want)*1e-12 {
		t.Fatalf("X = %v, want ~%v", lms[0].X, want)
	}
}
