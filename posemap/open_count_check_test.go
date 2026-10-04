package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// 记录的观测次数多于逐帧来源中归属该次出现的观测条数（无旧观测贡献）：
// 文件长度与校验和都正确，但记录 3 次、来源只有 2 条，Open 必须按
// ErrCorrupt 拒绝且不返回地图对象，而不是等到校正时才报损坏。
func TestOpenRejectsCountAboveSourceObservations(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	// 第二次出现实际只有 t=300 的 1 条观测，篡改为 3 次。
	fd.Landmarks[0].Occurrences[1].Count = 3
	writeFileDataRaw(t, path, &fd)

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("Open returned usable map: %v", m)
	}
}

// 记录的观测次数少于来源观测条数同样属于损坏。
func TestOpenRejectsCountBelowSourceObservations(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	// 第一次出现实际有 t=100、200 两条观测，篡改为 1 次。
	fd.Landmarks[0].Occurrences[0].Count = 1
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 各次出现分别核对：两次出现实际各有 2 次、1 次观测，记录成 1 次、2 次，
// 总数一样也必须拒绝——不能用另一次出现的观测补足。
func TestOpenRejectsSwappedOccurrenceCounts(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	occs := fd.Landmarks[0].Occurrences
	if occs[0].Count != 2 || occs[1].Count != 1 {
		t.Fatalf("test setup wrong: %+v", occs)
	}
	occs[0].Count, occs[1].Count = occs[1].Count, occs[0].Count
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("swapped counts: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 已失效的旧出现同样核对，不因它不再参与区域查询而跳过。
func TestOpenRejectsBadCountOnInvalidatedOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	if fd.Landmarks[0].Occurrences[0].Active {
		t.Fatal("test setup wrong: first occurrence should be invalidated")
	}
	fd.Landmarks[0].Occurrences[0].Count = 5
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("inactive occurrence count: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 次数矛盾被Open拒绝时原文件内容保持不变。
func TestOpenCountRejectionLeavesFileUntouched(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[1].Count = 2
	writeFileDataRaw(t, path, &fd)
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

// 同一帧重复观测同一标识各算一次、空观测帧不计：正常保存的此类文件
// 必须继续打开成功，且查询到全部已接受次数。
func TestOpenCountsDuplicateObservationsIndividually(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1}, // 空观测帧：不增加任何次数
		{Time: 200, DX: 0.1, Observations: []Observation{{ID: "D"}, {ID: "D"}}},
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
	lms, err := m2.LandmarksInRect(Rect{MinX: -10, MinY: -10, MaxX: 10, MaxY: 10})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].Count != 2 {
		t.Fatalf("landmarks = %+v, want D with count 2", lms)
	}
}

// 旧地图追加新帧后：旧贡献（legacy_count）与有来源的新观测共同计入各自
// 出现。正常混合文件打开成功；篡改任一部分（记录次数或旧贡献）即拒绝。
func TestOpenMixedLegacyCountConsistency(t *testing.T) {
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

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open legacy: %v", err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 旧贡献 2 次 + 新来源观测 1 条 = 记录 3 次：必须打开成功。
	fd := readFileData(t, path)
	if len(fd.Sources) != 3 || fd.Sources[0] != nil || fd.Sources[2] == nil {
		t.Fatalf("unexpected sources layout: %+v", fd.Sources)
	}
	occ := fd.Landmarks[0].Occurrences[0]
	if occ.Count != 3 || occ.LegacyCount != 2 {
		t.Fatalf("test setup wrong: %+v", occ)
	}
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen mixed legacy/new file: %v", err)
	}
	m3.Close()

	// 记录次数与（旧贡献 + 来源条数）不符：拒绝。
	fd.Landmarks[0].Occurrences[0].Count = 2
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("tampered count: err = %v, want ErrCorrupt, map=%v", err, m)
	}

	// 旧贡献被篡改（缺少来源的旧帧不能被当成零次，也不能按帧数猜测）：拒绝。
	// 注意上一次篡改已落盘，先把记录次数恢复为正确的 3 再改旧贡献。
	fd = readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].Count = 3
	fd.Landmarks[0].Occurrences[0].LegacyCount = 1
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("tampered legacy count: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 完全没有逐帧来源的旧地图仍按现有规则打开：不核对次数，不要求补造观测。
func TestOpenPureLegacyFileSkipsCountCheck(t *testing.T) {
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

	// 旧文件没有任何逐帧来源，记录次数无从核对，仍按既有规则打开。
	fd := readFileData(t, path)
	if len(fd.Sources) != 0 {
		t.Fatalf("test setup wrong: sources = %+v", fd.Sources)
	}
	fd.Landmarks[0].Count = 7
	writeFileDataRaw(t, path, &fd)
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open pure legacy file: %v", err)
	}
	m2.Close()
}

// 回环校正更新路标位置但不改变已接受观测次数：正常校正后保存的文件
// 必须继续被接受。
func TestOpenAfterCorrectionKeepsCountsAccepted(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 10.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 0, Heading: 0, Variance: 0}}); err != nil {
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
	lms, err := m2.LandmarksInRect(Rect{MinX: -100, MinY: -100, MaxX: 100, MaxY: 100})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].Count != 2 {
		t.Fatalf("landmarks = %+v, want L with count 2", lms)
	}
}
