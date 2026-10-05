package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// 记录中的首次观测时间写早（且该时间没有任何帧）：即使观测仍落在记录
// 声明的时间范围内、其余检查都通过，也必须按 ErrCorrupt 拒绝，不返回
// 地图对象，原文件保持不变。
func TestOpenRejectsFirstSeenTooEarly(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	// 第一次出现实际最早在 100 被观测，篡改为 50。
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 50
	writeFileDataRaw(t, path, &fd)

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
}

// 错误的首次时间恰好命中某个已保存的运动帧，但该帧并没有观测这一路标：
// 同样不能接受。
func TestOpenRejectsFirstSeenOnFrameWithoutObservation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 0.5, Observations: []Observation{{ID: "L"}}},
		{Time: 150, DX: 0.5}, // 该帧没有观测 L
		{Time: 200, DX: 0.5, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 150
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 首次时间写晚同样拒绝：第二次出现的首次观测在 300，篡改为 400 后
// t=300 的观测失去归属；篡改为 250（介于两次出现之间、无对应观测）时
// 归属不变但时间与最早观测不符，两种写法都必须拒绝。
func TestOpenRejectsFirstSeenTooLate(t *testing.T) {
	for _, firstSeen := range []int64{250, 400} {
		path := filepath.Join(t.TempDir(), "m.pose")
		buildOccurrenceMap(t, path)
		fd := readFileData(t, path)
		fd.Landmarks[0].Occurrences[1].FirstSeenTime = firstSeen
		writeFileDataRaw(t, path, &fd)
		if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
			t.Fatalf("firstSeen=%d: err = %v, map = %v, want ErrCorrupt and nil map", firstSeen, err, m)
		}
	}
}

// 已失效的历史出现同样核对：把第一次（已失效）出现的首次时间改晚，
// 使其错过 t=100 的观测，必须拒绝。
func TestOpenRejectsFirstSeenTamperedOnInvalidatedOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 200
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 两次出现分别依据各自的观测判断：把第二次出现的首次时间换成第一次
// 出现的首次时间（100），不能蒙混过关。
func TestOpenRejectsFirstSeenBorrowedFromPreviousOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[1].FirstSeenTime = 100
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 时间为零或负数的合法轨迹按实际时间判断：首次观测在 -100 与 0 的记录
// 都合法且打开后保持原值；零不被当作时间未知。篡改后必须拒绝。
func TestOpenFirstSeenZeroAndNegative(t *testing.T) {
	cfg := Config{InitialTime: -200, MaxInterval: 1000, MergeDistance: 1.0}
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: -100, DX: 0.5, Observations: []Observation{{ID: "N"}}},
		{Time: 0, DX: 0.5, Observations: []Observation{{ID: "Z"}, {ID: "Z"}}},
		{Time: 100, DX: 0.5, Observations: []Observation{{ID: "N"}, {ID: "Z"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m, err = Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	h, err := m.LandmarkAppearances("Z")
	if err != nil {
		t.Fatal(err)
	}
	if !h.Appearances[0].HasFirstSeen || h.Appearances[0].FirstSeenTime != 0 {
		t.Fatalf("Z first seen = %+v, want known time 0", h.Appearances[0])
	}
	h, err = m.LandmarkAppearances("N")
	if err != nil {
		t.Fatal(err)
	}
	if !h.Appearances[0].HasFirstSeen || h.Appearances[0].FirstSeenTime != -100 {
		t.Fatalf("N first seen = %+v, want known time -100", h.Appearances[0])
	}
	m.Close()

	// 篡改：N 的首次时间写成 0（恰好是另一帧的时间，但该帧没观测 N）。
	fd := readFileData(t, path)
	for i := range fd.Landmarks {
		if fd.Landmarks[i].ID == "N" {
			fd.Landmarks[i].Occurrences[0].FirstSeenTime = 0
		}
	}
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("tampered: err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 旧地图兼容：缺少旧观测依据的第 1 次出现首次时间仍未知，不补造；失效后
// 新导入的第 2 次出现依据完整、首次时间已知，必须通过核对；篡改它则拒绝。
func TestOpenLegacyMixedFirstSeen(t *testing.T) {
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
		{Time: 300, DX: 1, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 合法混合文件正常打开：第 1 次出现首次时间未知，第 2 次为 300。
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen mixed: %v", err)
	}
	h, err := m3.LandmarkAppearances("K")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	if h.Appearances[0].HasFirstSeen {
		t.Fatalf("legacy occurrence first seen must stay unknown: %+v", h.Appearances[0])
	}
	if !h.Appearances[1].HasFirstSeen || h.Appearances[1].FirstSeenTime != 300 {
		t.Fatalf("occ2 = %+v, want known first seen 300", h.Appearances[1])
	}
	m3.Close()

	// 篡改第 2 次出现的首次时间：不能因同文件存在旧历史而跳过核对。
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[1].FirstSeenTime = 250
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("tampered occ2: err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 合法文件打开后历次出现的首次时间、出现编号、观测次数与失效信息保持
// 原值；后续帧继续观测与同帧重复观测都不改变首次时间。
func TestOpenPreservesFirstSeenRecords(t *testing.T) {
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
	if a1.Number != 1 || a1.Active || !a1.HasFirstSeen || a1.FirstSeenTime != 100 ||
		a1.Landmark.Count != 2 || a1.InvalidTime != 200 || a1.InvalidReason != "gone" {
		t.Fatalf("occ1 = %+v", a1)
	}
	if a2.Number != 2 || !a2.Active || !a2.HasFirstSeen || a2.FirstSeenTime != 300 ||
		a2.Landmark.Count != 1 {
		t.Fatalf("occ2 = %+v", a2)
	}
}
