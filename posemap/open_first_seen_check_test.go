package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// buildFirstSeenMap 构造含三帧（t=100、200、300）的地图：路标 L 在
// t=200、300 被观测（首次观测时间 200），t=100 的帧不观测 L。
func buildFirstSeenMap(t *testing.T, path string) {
	t.Helper()
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 300, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 记录的首次观测时间早于实际最早观测（100 写成 50）：观测仍落在声明的
// 时间范围内、其余检查都通过，也必须按损坏拒绝，不返回地图对象，且原
// 文件保持不变。
func TestOpenRejectsFirstSeenEarlierThanObservation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildFirstSeenMap(t, path)
	fd := readFileData(t, path)
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
		t.Fatalf("Open returned usable map: %v", m)
	}
	after, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(before) != string(after) {
		t.Fatal("Open modified the rejected file")
	}
}

// 记录的首次观测时间晚于实际最早观测同样拒绝。
func TestOpenRejectsFirstSeenLaterThanObservation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildFirstSeenMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 250
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 错误时间恰好命中一个已保存的帧，但该帧没有观测这一路标：不能接受。
func TestOpenRejectsFirstSeenOnFrameWithoutObservation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildFirstSeenMap(t, path)
	fd := readFileData(t, path)
	// t=100 的帧存在但不观测 L；实际最早观测在 200。
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 100
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 错误时间没有任何对应帧：同样拒绝。
func TestOpenRejectsFirstSeenWithoutFrame(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildFirstSeenMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 150
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 已失效的历史出现同样核对：第一次出现（100 首次观测、200 失效）的首次
// 时间被写早，即使它不再参与区域查询也按损坏拒绝。
func TestOpenRejectsWrongFirstSeenForInvalidatedOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	if fd.Landmarks[0].Occurrences[0].FirstSeenTime != 100 {
		t.Fatalf("setup: occ1 first seen = %d, want 100", fd.Landmarks[0].Occurrences[0].FirstSeenTime)
	}
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 50
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 两次出现分别依据各自的观测判断：第二次出现（300 首次观测）的首次时间
// 被改成 250（落在两次出现之间、没有对应帧）或 100（第一次出现的首次
// 时间），都按损坏拒绝。
func TestOpenRejectsSecondOccurrenceFirstSeenMismatch(t *testing.T) {
	for _, firstSeen := range []int64{250, 100} {
		path := filepath.Join(t.TempDir(), "m.pose")
		buildOccurrenceMap(t, path)
		fd := readFileData(t, path)
		if fd.Landmarks[0].Occurrences[1].FirstSeenTime != 300 {
			t.Fatalf("setup: occ2 first seen = %d, want 300", fd.Landmarks[0].Occurrences[1].FirstSeenTime)
		}
		fd.Landmarks[0].Occurrences[1].FirstSeenTime = firstSeen
		writeFileDataRaw(t, path, &fd)

		if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
			t.Fatalf("first seen %d: err = %v, map = %v, want ErrCorrupt and nil map", firstSeen, err, m)
		}
	}
}

// 后续帧继续观测同一路标、同一帧连续观测多次，都不改变首次观测时间：
// 合法文件正常打开，历次出现的首次时间、出现编号、观测次数与失效信息
// 保持原值。
func TestOpenFirstSeenUnaffectedByLaterAndDuplicateObservations(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		// 同一帧连续观测两次。
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}, {ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 400, DX: 1, Observations: []Observation{{ID: "L"}}},
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
	h, err := m2.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	a1, a2 := h.Appearances[0], h.Appearances[1]
	if a1.Number != 1 || !a1.HasFirstSeen || a1.FirstSeenTime != 100 || a1.Landmark.Count != 3 ||
		a1.Active || a1.InvalidTime != 200 || a1.InvalidReason != "gone" || a1.InvalidOpID != "op1" {
		t.Fatalf("occ1 = %+v", a1)
	}
	if a2.Number != 2 || !a2.HasFirstSeen || a2.FirstSeenTime != 300 || a2.Landmark.Count != 2 || !a2.Active {
		t.Fatalf("occ2 = %+v", a2)
	}
}

// 时间为零或负数的合法轨迹按实际时间判断：首次观测时间为 0 是有效的
// 已知时间，不把零当作未知；篡改它（命中不观测该路标的帧、或没有对应
// 帧）都按损坏拒绝。
func TestOpenFirstSeenZeroAndNegativeTimes(t *testing.T) {
	cfg := Config{InitialTime: -1000, MaxInterval: 1000, MergeDistance: 1.0}
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: -100, DX: 1},
		{Time: 0, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
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
	h, err := m2.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 || !h.Appearances[0].HasFirstSeen || h.Appearances[0].FirstSeenTime != 0 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	m2.Close()

	for _, bad := range []int64{-100, -50, 50} {
		p := filepath.Join(t.TempDir(), "m.pose")
		m, err := Create(p, cfg)
		if err != nil {
			t.Fatal(err)
		}
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: -100, DX: 1},
			{Time: 0, DX: 1, Observations: []Observation{{ID: "L"}}},
			{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		}}); err != nil {
			t.Fatal(err)
		}
		if err := m.Close(); err != nil {
			t.Fatal(err)
		}
		fd := readFileData(t, p)
		fd.Landmarks[0].Occurrences[0].FirstSeenTime = bad
		writeFileDataRaw(t, p, &fd)
		if m, err := Open(p); !errors.Is(err, ErrCorrupt) || m != nil {
			t.Fatalf("first seen %d: err = %v, map = %v, want ErrCorrupt and nil map", bad, err, m)
		}
	}
}

// 旧地图兼容：缺少逐帧来源的第 1 次出现首次时间保持未知，后来追加的
// 第一条观测不给它补首次时间；但旧路标失效后新导入产生的第 2 次出现
// 依据完整、首次时间已知，必须接受核对——篡改它即损坏，不因同一文件
// 存在旧历史而跳过。
func TestOpenLegacyUnknownFirstSeenAndNewOccurrenceChecked(t *testing.T) {
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

	// 旧地图上追加观测（仍属第 1 次出现），随后失效并再次出现。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open legacy: %v", err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m2.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"K"}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "reappear", Frames: []Frame{
		{Time: 400, DX: 1, Observations: []Observation{{ID: "K"}}},
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
	h, err := m3.LandmarkAppearances("K")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	// 第 1 次出现缺少旧观测依据：首次时间保持未知，不被 t=300 的追加
	// 观测补造。
	if h.Appearances[0].HasFirstSeen {
		t.Fatalf("legacy occ1 first seen should stay unknown: %+v", h.Appearances[0])
	}
	if !h.Appearances[1].HasFirstSeen || h.Appearances[1].FirstSeenTime != 400 {
		t.Fatalf("occ2 = %+v", h.Appearances[1])
	}
	m3.Close()

	// 篡改新出现的首次时间：旧历史存在也不能跳过对它的核对。
	fd := readFileData(t, path)
	if len(fd.Sources) != 4 || fd.Sources[0] != nil || fd.Sources[3] == nil {
		t.Fatalf("unexpected sources layout: %+v", fd.Sources)
	}
	fd.Landmarks[0].Occurrences[1].FirstSeenTime = 350
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("tampered occ2 first seen: err = %v, map = %v, want ErrCorrupt and nil map", err, m)
	}
}

// 回环校正改变位姿与路标位置，但不改变观测帧时间：已正确保存的校正
// 地图仍正常打开，首次观测时间保持原值。
func TestOpenCorrectedMapKeepsFirstSeen(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
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
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 5, Heading: 0.5, Variance: 0.1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open corrected map: %v", err)
	}
	defer m2.Close()
	h, err := m2.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 || !h.Appearances[0].HasFirstSeen || h.Appearances[0].FirstSeenTime != 100 ||
		h.Appearances[0].Landmark.Count != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
}
