package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// 合法文件：失效记录指向第 1 次出现，打开后旧出现保留失效状态、新出现
// 继续有效；重复提交原操作返回首次失效的编号与时间，不再次撤下新出现。
func TestOpenInvalidationRecordValid(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)

	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	res, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}})
	if err != nil {
		t.Fatalf("resubmit: %v", err)
	}
	if res.Time != 200 || len(res.Landmarks) != 1 || res.Landmarks[0].ID != "L" || res.Landmarks[0].Occurrence != 1 {
		t.Fatalf("resubmit result = %+v, want time 200 occurrence 1", res)
	}
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 || h.Appearances[0].Active || !h.Appearances[1].Active {
		t.Fatalf("appearances after resubmit = %+v", h.Appearances)
	}
}

// 失效记录指向不存在的出现编号：L 只有 2 次出现，记录却指向第 3 次。
func TestOpenRejectsInvalidationToMissingOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Invalidations[0].Landmarks[0].Occurrence = 3
	writeFileDataRaw(t, path, &fd)

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("Open returned usable map: %v", m)
	}
}

// 失效记录指向仍有效的第 2 次出现：不能因路标后来再次出现就把旧结果
// 改指最新记录。
func TestOpenRejectsInvalidationToActiveOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Invalidations[0].Landmarks[0].Occurrence = 2
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 失效记录指向从未出现的路标。
func TestOpenRejectsInvalidationToUnknownLandmark(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Invalidations[0].Landmarks[0].ID = "ghost"
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 第 1 次出现的失效时间与记录不符（200 篡改为 250）：轨迹与出现次序
// 要求各自仍满足，也必须按记录与历史的矛盾拒绝。
func TestOpenRejectsInvalidationTimeMismatch(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	occ := &fd.Landmarks[0].Occurrences[0]
	if occ.InvalidTime != 200 {
		t.Fatalf("test setup wrong: %+v", occ)
	}
	occ.InvalidTime = 250
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 第 1 次出现的失效原因与记录不符。
func TestOpenRejectsInvalidationReasonMismatch(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].InvalidReason = "moved"
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 该次出现实际由另一条操作撤下：失效时间与原因都与记录相同，仅操作
// 标识不同，也不能认可这条记录。
func TestOpenRejectsInvalidationWrongOp(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	occ := &fd.Landmarks[0].Occurrences[0]
	occ.InvalidOpID = "op-other"
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 一条操作同时撤下多个路标：任意一个条目与历史对不上，整份文件拒绝。
func TestOpenRejectsMultiLandmarkInvalidationWithOneBadEntry(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "A"}, {ID: "B", X: 0, Y: 0.5}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"A", "B"}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// A 的条目合法，B 的条目被篡改：整份文件按损坏拒绝。
	fd := readFileData(t, path)
	if len(fd.Invalidations[0].Landmarks) != 2 {
		t.Fatalf("test setup wrong: %+v", fd.Invalidations[0])
	}
	for i := range fd.Invalidations[0].Landmarks {
		if fd.Invalidations[0].Landmarks[i].ID == "B" {
			fd.Invalidations[0].Landmarks[i].Occurrence = 2
		}
	}
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 对应关系矛盾被 Open 拒绝时原文件内容保持不变。
func TestOpenInvalidationRejectionLeavesFileUntouched(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Invalidations[0].Landmarks[0].Occurrence = 2
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

// 旧地图后来产生的失效操作同样按对应关系核对：正常保存的混合文件打开
// 成功，篡改失效记录即拒绝；不要求补齐旧地图原本缺失的逐帧观测。
func TestOpenLegacyMapInvalidationChecked(t *testing.T) {
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
	if _, err := m2.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"K"}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 正常保存：旧路标的失效记录指向第 1 次出现，打开成功。
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	h, err := m3.LandmarkAppearances("K")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 || h.Appearances[0].Active || h.Appearances[0].InvalidOpID != "op1" {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	m3.Close()

	// 篡改记录指向不存在的第 2 次出现：拒绝。
	fd := readFileData(t, path)
	fd.Invalidations[0].Landmarks[0].Occurrence = 2
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("tampered record: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}
