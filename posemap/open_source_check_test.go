package posemap

import (
	"encoding/binary"
	"encoding/json"
	"errors"
	"hash/crc32"
	"os"
	"path/filepath"
	"testing"
)

// readFileData 解码地图文件的 JSON 载荷。
func readFileData(t *testing.T, path string) fileData {
	t.Helper()
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	var fd fileData
	if err := json.Unmarshal(raw[headerLen:len(raw)-4], &fd); err != nil {
		t.Fatal(err)
	}
	return fd
}

// writeFileDataRaw 用（可能被篡改的）fileData 重新组装完整文件并重算 CRC。
func writeFileDataRaw(t *testing.T, path string, fd *fileData) {
	t.Helper()
	payload, err := json.Marshal(fd)
	if err != nil {
		t.Fatal(err)
	}
	out := make([]byte, headerLen+len(payload)+4)
	copy(out[0:8], fileMagic)
	binary.BigEndian.PutUint16(out[8:10], fileVersion)
	binary.BigEndian.PutUint32(out[10:14], uint32(len(payload)))
	copy(out[headerLen:], payload)
	binary.BigEndian.PutUint32(out[len(out)-4:], crc32.ChecksumIEEE(out[:len(out)-4]))
	if err := os.WriteFile(path, out, 0o644); err != nil {
		t.Fatal(err)
	}
}

// buildOccurrenceMap 构造含同一路标两次出现的地图：L 在 t=100、200 被
// 观测，t=200 失效，t=300 再次被观测。
func buildOccurrenceMap(t *testing.T, path string) {
	t.Helper()
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
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
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 正常文件：失效端点（200）属于已失效的第一次出现，再现端点（300）属于
// 第二次出现，打开成功且记录原样保留。
func TestOpenObservationOwnershipValid(t *testing.T) {
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
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	a1, a2 := h.Appearances[0], h.Appearances[1]
	if a1.Active || a1.FirstSeenTime != 100 || a1.InvalidTime != 200 || a1.Landmark.Count != 2 {
		t.Fatalf("occ1 = %+v", a1)
	}
	if !a2.Active || a2.FirstSeenTime != 300 || a2.Landmark.Count != 1 {
		t.Fatalf("occ2 = %+v", a2)
	}
	// 查询行为不变。
	if p, err := m.PoseAt(250); err != nil || p.Time != 200 {
		t.Fatalf("PoseAt(250) = %+v %v", p, err)
	}
	if cur, err := m.CurrentPose(); err != nil || cur.Time != 300 {
		t.Fatalf("CurrentPose = %+v %v", cur, err)
	}
}

// 逐帧观测指向不存在的路标：打开即报 ErrCorrupt，不返回地图对象。
// 拒绝时不改写原文件（另见 TestOpenRejectionLeavesFileUntouched）。
func TestOpenRejectsUnknownLandmarkObservation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Sources[0].Observations[0].ID = "GHOST"
	writeFileDataRaw(t, path, &fd)

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("Open returned usable map: %v", m)
	}
}

// Open 拒绝后原文件内容必须保持不变。
func TestOpenRejectionLeavesFileUntouched(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Sources[1].Observations[0].ID = "GHOST"
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

// 观测落在两次出现之间（200 失效、300 再现时的 250 帧）：不能因为之后
// 存在有效出现就划给第二次出现，按损坏拒绝。
func TestOpenRejectsObservationBetweenOccurrences(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	// 把最后一帧时间从 300 改为 250：该帧仍观测 L，但 occ1 已于 200
	// 失效、occ2 首次观测在 300，250 没有归属。
	fd.Trajectory[3].Time = 250
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("gap observation: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 观测早于某次出现的首次观测时间，且没有更早的出现可以包含它：损坏。
func TestOpenRejectsObservationBeforeFirstSeen(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	// 第一次出现首次观测改为 150：t=100 的观测失去归属。
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 150
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("pre-first-seen observation: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 只检查当前有效路标会误杀正常保存的失效历史；这里失效出现的区间内
// （含失效端点）观测必须被接受。
func TestOpenChecksInactiveOccurrences(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	// 直接确认被检查的观测结构：t=200 帧的 L 观测属于已失效的第一次出现。
	var frame200 int64 = 200
	found := false
	for _, ob := range fd.Sources[1].Observations {
		if ob.ID == "L" {
			found = true
		}
	}
	if !found || fd.Trajectory[2].Time != frame200 {
		t.Fatalf("test setup wrong: %+v", fd.Sources[1])
	}
	// 不做任何篡改：正常文件必须打开成功（若只查有效出现，该观测会被误拒）。
	m, err := Open(path)
	if err != nil {
		t.Fatalf("inactive occurrence history rejected: %v", err)
	}
	m.Close()
}

// 旧地图兼容：null 来源帧不要求补齐归属；但后来追加的、确实带来源的帧
// 仍需检查归属。
func TestOpenLegacySourcesStillChecked(t *testing.T) {
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

	// 纯旧文件可以打开。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open legacy: %v", err)
	}
	// 旧地图上追加带来源的新帧：t=300 观测 K（旧路标无起始边界，接受）。
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen mixed legacy/new file: %v", err)
	}
	m3.Close()

	// 篡改追加帧的观测为未知路标：旧帧为 null 也不能掩盖新帧的矛盾。
	fd := readFileData(t, path)
	if len(fd.Sources) != 3 || fd.Sources[0] != nil || fd.Sources[2] == nil {
		t.Fatalf("unexpected sources layout: %+v", fd.Sources)
	}
	fd.Sources[2].Observations[0].ID = "ZZZ"
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("appended bad source: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 第一次出现的失效时间（350）晚于第二次出现的首次观测（300）：两次出现
// 的时间范围重叠，t=300 的观测同时被两次出现接纳。即使重叠范围内的观测
// 各自都能归入某次出现，出现记录本身的时间关系已矛盾，打开必须拒绝。
func TestOpenRejectsOverlappingOccurrences(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].InvalidTime = 350
	writeFileDataRaw(t, path, &fd)
	saved, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
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

// 第二次出现的首次观测与第一次出现的失效时间相等：两个端点都接纳所在
// 时刻的观测，该时刻的观测归属歧义，同样拒绝。
func TestOpenRejectsTouchingOccurrenceBounds(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	// occ1 在 200 失效；把 occ2 的首次观测改为 200，两端点相等。
	fd.Landmarks[0].Occurrences[1].FirstSeenTime = 200
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("touching bounds: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 已失效出现的首次观测时间晚于其失效时间：记录自身矛盾。
func TestOpenRejectsFirstSeenAfterInvalidation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 250 // 失效时间为 200
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("first seen after invalidation: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 第二次及以后出现的首次观测时间未知：只有旧文件的第 1 次出现允许未知。
func TestOpenRejectsLaterOccurrenceWithoutFirstSeen(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[1].HasFirstSeen = false
	fd.Landmarks[0].Occurrences[1].FirstSeenTime = 0
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("later occurrence without first seen: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 旧地图兼容：旧文件第 1 次出现首次观测时间未知，失效后再次出现仍须
// 遵守时间次序；次序合法的此类历史必须正常打开，且记录原样保留。
func TestOpenLegacyInvalidatedThenReappeared(t *testing.T) {
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
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 第 1 次出现首次观测时间未知、失效时间 200；第 2 次出现 300 首次
	// 观测。次序合法，必须打开成功且记录原样保留。
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen legacy history: %v", err)
	}
	defer m3.Close()
	h, err := m3.LandmarkAppearances("K")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	a1, a2 := h.Appearances[0], h.Appearances[1]
	if a1.Active || a1.HasFirstSeen || a1.InvalidTime != 200 {
		t.Fatalf("occ1 = %+v", a1)
	}
	if !a2.Active || !a2.HasFirstSeen || a2.FirstSeenTime != 300 {
		t.Fatalf("occ2 = %+v", a2)
	}
}
