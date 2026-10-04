package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// 打开文件时核对“记录的观测次数”与“逐帧来源里归属该次出现的观测条数 +
// 已保存的固定旧贡献次数”：偏多偏少都按 ErrCorrupt 拒绝，不返回地图
// 对象，不改写原文件；各次出现独立结算，失效出现也要核对，纯旧文件
// （完全没有逐帧来源）保持兼容。

// 正常文件打开成功：occ1 有 t=100、200 两条来源观测，occ2 有 t=300
// 一条。
func TestOpenObservationCountsValid(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)

	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if h.Appearances[0].Landmark.Count != 2 || h.Appearances[1].Landmark.Count != 1 {
		t.Fatalf("counts = %d, %d", h.Appearances[0].Landmark.Count, h.Appearances[1].Landmark.Count)
	}
}

// 记录次数偏多（2 条来源观测却写成 3）：即使长度与校验都正确，打开即
// 拒绝，查询不可能看到这个自相矛盾的 3。
func TestOpenRejectsCountTooHigh(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].Count = 3
	writeFileDataRaw(t, path, &fd)
	saved, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("count too high: err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("Open returned usable map: %v", m)
	}
	got, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(got) != string(saved) {
		t.Fatal("Open modified the rejected file")
	}
}

// 记录次数偏少（2 条来源观测却写成 1）同样拒绝。
func TestOpenRejectsCountTooLow(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].Count = 1
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("count too low: err = %v, map = %v, want ErrCorrupt and nil", err, m)
	}
}

// 各次出现分别核对：实际 2 次、1 次被记成 1 次、2 次，总数一样也必须
// 拒绝——不能把旧出现的观测拿来补足新出现。
func TestOpenRejectsCountsSwappedBetweenOccurrences(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].Count = 1
	fd.Landmarks[0].Occurrences[1].Count = 2
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("swapped counts: err = %v, map = %v, want ErrCorrupt and nil", err, m)
	}
}

// 已失效、不再参与区域查询的旧出现次数矛盾也要拒绝，不能跳过它。
func TestOpenRejectsInactiveOccurrenceCountMismatch(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].Count = 3 // occ1 已失效，实际 2 条
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("inactive occurrence mismatch: err = %v, map = %v, want ErrCorrupt and nil", err, m)
	}
}

// 即使其他路标完全正确，单个路标记录矛盾也不能忽略继续打开。
func TestOpenRejectsOneBadLandmarkAmongGoodOnes(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "GOOD"}, {ID: "BAD"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	fd := readFileData(t, path)
	for i := range fd.Landmarks {
		if fd.Landmarks[i].ID == "BAD" {
			fd.Landmarks[i].Occurrences[0].Count = 2
		}
	}
	writeFileDataRaw(t, path, &fd)

	if mp, err := Open(path); !errors.Is(err, ErrCorrupt) || mp != nil {
		t.Fatalf("one bad landmark: err = %v, map = %v, want ErrCorrupt and nil", err, mp)
	}
}

// 同一帧重复观测同一标识不去重：保存 2 条来源观测时次数必须为 2；少存
// 一条来源而次数仍是 2 也拒绝。
func TestOpenCountsDuplicateObservationsSameFrame(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	// t=100 一帧内两次观测 L（同帧各算一次，合并后次数 2）；t=200 不
	// 观测 L（空观测帧不增加次数）。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}, {ID: "L"}}},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 未篡改：2 条同帧观测，次数 2，正常打开。
	if mp, err := Open(path); err != nil {
		t.Fatalf("duplicate observations open: %v", err)
	} else {
		h, err := mp.LandmarkAppearances("L")
		if err != nil {
			t.Fatal(err)
		}
		if h.Appearances[0].Landmark.Count != 2 {
			t.Fatalf("count = %d, want 2", h.Appearances[0].Landmark.Count)
		}
		mp.Close()
	}

	// 次数少算成 1（来源仍是 2 条）：拒绝。
	orig := readFileData(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].Count = 1
	writeFileDataRaw(t, path, &fd)
	if mp, err := Open(path); !errors.Is(err, ErrCorrupt) || mp != nil {
		t.Fatalf("undercount duplicates: err = %v, map = %v, want ErrCorrupt and nil", err, mp)
	}

	// 删掉一条同帧来源观测而次数仍记 2：来源只归属 1 条，拒绝。
	fd = orig
	fd.Sources[0].Observations = fd.Sources[0].Observations[:1]
	writeFileDataRaw(t, path, &fd)
	if mp, err := Open(path); !errors.Is(err, ErrCorrupt) || mp != nil {
		t.Fatalf("missing duplicate source: err = %v, map = %v, want ErrCorrupt and nil", err, mp)
	}
}

// 纯旧地图（完全没有 sources 列）：无法逐帧核对，按既有规则打开，即使
// 直接篡改记录次数也不补造、不猜测，保持兼容。
func TestOpenLegacyFileWithoutSourcesSkipsCountCheck(t *testing.T) {
	path := filepath.Join(t.TempDir(), "old.pose")
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

	// 未篡改的纯旧文件正常打开。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open legacy: %v", err)
	}
	m2.Close()

	// 篡改次数：纯旧文件无来源可核对，仍按旧文件规则打开（不要求补造）。
	fd := readFileData(t, path)
	fd.Landmarks[0].Count = 5
	writeFileDataRaw(t, path, &fd)
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("tampered legacy count should still open: %v", err)
	}
	defer m3.Close()
}

// 旧地图追加带来源的新帧后：次数 = 缺少来源的旧固定贡献 + 归属该出现的
// 新来源观测条数；缺来源的旧帧不能当零次，偏多偏少都拒绝。
func TestOpenMixedLegacyAndSourcesCountCheck(t *testing.T) {
	path := filepath.Join(t.TempDir(), "mixed.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	// 旧地图：K 在 t=100、200 共 2 次观测。
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "K"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	// 追加带来源的新帧 t=300，再观测 K 一次（DX=0.5：转换后与旧均值
	// (1.5,0) 恰距 1.0，含边界可接纳）：次数应为 3（2 旧贡献 + 1 来源）。
	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	fd0 := readFileData(t, path)
	if len(fd0.Sources) != 3 || fd0.Sources[0] != nil || fd0.Sources[1] != nil || fd0.Sources[2] == nil {
		t.Fatalf("unexpected sources layout: %+v", fd0.Sources)
	}

	// 混合文件本身合法（legacy 2 + 来源 1 = 3），正常打开。
	if mp, err := Open(path); err != nil {
		t.Fatalf("mixed file open: %v", err)
	} else {
		h, err := mp.LandmarkAppearances("K")
		if err != nil {
			t.Fatal(err)
		}
		if h.Appearances[0].Landmark.Count != 3 {
			t.Fatalf("mixed count = %d, want 3", h.Appearances[0].Landmark.Count)
		}
		mp.Close()
	}

	// 把旧固定贡献忽略成零（次数只与来源条数相等）：拒绝缺来源帧被当零次。
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].Count = 1
	writeFileDataRaw(t, path, &fd)
	if mp, err := Open(path); !errors.Is(err, ErrCorrupt) || mp != nil {
		t.Fatalf("mixed count ignores legacy: err = %v, map = %v, want ErrCorrupt and nil", err, mp)
	}

	// 次数偏多同样拒绝。
	fd = readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].Count = 4
	writeFileDataRaw(t, path, &fd)
	if mp, err := Open(path); !errors.Is(err, ErrCorrupt) || mp != nil {
		t.Fatalf("mixed count too high: err = %v, map = %v, want ErrCorrupt and nil", err, mp)
	}
}

// 回环校正只更新路标位置、不改变已接受观测次数；正常校正后保存的文件
// 必须继续被接受。
func TestOpenCountsValidAfterCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{
		ID:     "c1",
		Anchor: 100,
		Target: CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 0},
	}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open after correction: %v", err)
	}
	defer m2.Close()
	h, err := m2.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if h.Appearances[0].Landmark.Count != 2 || h.Appearances[1].Landmark.Count != 1 {
		t.Fatalf("counts after correction = %d, %d, want 2, 1",
			h.Appearances[0].Landmark.Count, h.Appearances[1].Landmark.Count)
	}
}
