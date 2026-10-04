package posemap

import (
	"errors"
	"math"
	"os"
	"path/filepath"
	"testing"
)

// 规格示例：某次出现只有两条转换到地图坐标后分别位于 (1,0)、(2,0) 的
// 观测，保存位置应为 (1.5,0)。文件只把位置改成 (8,0)，即使观测次数仍是
// 两次、文件长度和校验和都正确，Open 也必须按 ErrCorrupt 拒绝。
func TestOpenRejectsPositionContradictingObservations(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	// t=100 位姿 (1,0)，L 观测本地 (0,0) → 地图 (1,0)；
	// t=200 位姿 (2,0)，L 观测 → 地图 (2,0)；等权平均 (1.5,0)、次数 2。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	fd := readFileData(t, path)
	occ := fd.Landmarks[0].Occurrences[0]
	if occ.X != 1.5 || occ.Y != 0 || occ.Count != 2 {
		t.Fatalf("test setup wrong: %+v", occ)
	}
	fd.Landmarks[0].Occurrences[0].X = 8
	writeFileDataRaw(t, path, &fd)

	saved, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m2 != nil {
		t.Fatalf("Open returned usable map: %v", m2)
	}
	got, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(got) != string(saved) {
		t.Fatal("Open modified the rejected file")
	}
}

// 合并距离不能作为保存位置偏差的容许范围：保存位置只偏离重放均值 0.1
// （远小于合并距离 1）仍必须拒绝，且不能自动改正后继续使用。
func TestOpenRejectsPositionShiftBelowMergeDistance(t *testing.T) {
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
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].X = 1.6 // 应为 1.5，偏差 0.1 < 合并距离 1
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 各次出现分别核对：两次出现的观测不能混用。occ1 均值 (1.5,0)、occ2 位于
// (3,0)，把任一次出现改成两次出现观测混算的位置都必须拒绝。
func TestOpenRejectsPositionMixedAcrossOccurrences(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	occs := fd.Landmarks[0].Occurrences
	// occ1: t=100 (1,0)、t=200 (2,0) 两条观测，均值 (1.5,0)；
	// occ2: t=300 位姿 (3,0) 的一条观测，位置 (3,0)。
	if occs[0].X != 1.5 || occs[0].Count != 2 || occs[1].X != 3 || occs[1].Count != 1 {
		t.Fatalf("test setup wrong: %+v", occs)
	}
	mixed := (1.5*2 + 3) / 3 // 假设跨出现混算会得到的“均值”

	fd.Landmarks[0].Occurrences[0].X = mixed
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("mixed position on occ1: err = %v, want ErrCorrupt, map=%v", err, m)
	}

	fd = readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].X = 1.5
	fd.Landmarks[0].Occurrences[1].X = mixed
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("mixed position on occ2: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 已经失效的出现也属于地图历史：其位置矛盾同样阻止打开，不能因为它不再
// 参与区域查询就跳过。
func TestOpenRejectsPositionMismatchOnInvalidatedOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	if fd.Landmarks[0].Occurrences[0].Active {
		t.Fatal("test setup wrong: first occurrence should be invalidated")
	}
	fd.Landmarks[0].Occurrences[0].X = 8
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("inactive occurrence position: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 篡改一条来源观测（次数不变）使重放到第二条时超出合并距离：观测依据
// 本身在当前位姿下不再满足接纳规则，同样按损坏拒绝。
func TestOpenRejectsReplayedMergeConflict(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	// t=100 的第一条观测是 occ1 的建点观测（不做距离判定），把它本地
	// X 改成 5：建点变为地图 (6,0)；t=200 的第二条观测位于 (2,0)，距
	// 当时均值 4 > 合并距离 1，重放冲突。
	fd.Sources[0].Observations[0].X = 5
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("replayed conflict: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 同帧对同一标识的多条观测按保存的输入次序逐条重放：正常文件（最终均值
// 由次序决定的增量合并产生）必须打开成功，位置与查询结果逐位一致。
func TestOpenPositionReplayHonorsSameFrameOrder(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, sameFrameConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(sameFrameLGeometry(true)); err != nil {
		t.Fatal(err)
	}
	// 再做一次平移校正，使保存位置来自校正后重放；打开时必须按当前轨迹
	// 与同帧次序复现出完全相同的均值 (-0.6,0)。
	if _, err := m.Correct(Correction{
		ID: "c-order", Anchor: 100,
		Target: CorrectionTarget{X: -0.5, Y: 0, Heading: 0, Variance: 0},
	}); err != nil {
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
	lms, err := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].Count != 5 || lms[0].X != -0.6 || lms[0].Y != 0 {
		t.Fatalf("landmark = %+v, want (-0.6,0) count 5", lms)
	}

	// 把同帧观测次序打乱后（观测集合与次数不变），按保存的同帧次序重放
	// 不再复现保存值：该次序下第二条观测重放即超出合并距离，按损坏拒绝。
	fd := readFileData(t, path)
	obs := fd.Sources[1].Observations // t=100 帧的四条 L 观测
	if len(obs) != 4 {
		t.Fatalf("test setup wrong: %+v", obs)
	}
	obs[0].X, obs[3].X = obs[3].X, obs[0].X
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("reordered same-frame observations: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 正常回环校正后保存的文件可以打开：核对的是当前轨迹重放出的位置，不
// 要求当前路标等于旧校正记录快照，也不改写那些历史快照。
func TestOpenPositionAcceptedAfterCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildCorrectedMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open corrected map: %v", err)
	}
	recs, _ := m.Corrections()
	if len(recs) != 1 {
		t.Fatalf("records = %+v", recs)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 对同一范围再做一次不同校正后重开：旧记录快照保留提交时值，位置核对
	// 只认当前轨迹。
	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m2.Correct(Correction{ID: "c2", Anchor: 100, Target: CorrectionTarget{X: -3, Y: 2, Heading: 0.5, Variance: 4}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen after recorrection: %v", err)
	}
	defer m3.Close()
	recs2, _ := m3.Corrections()
	if len(recs2) != 2 || recs2[0].ID != "c1" || recs2[1].ID != "c2" {
		t.Fatalf("records = %+v", recs2)
	}
}

// 旧地图追加有完整依据的新帧后：旧观测贡献按其原有次数与平均位置作为
// 固定起点参与，新观测共同决定最终位置，合法文件打开成功。
func TestOpenMixedLegacyPositionAccepted(t *testing.T) {
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
		t.Fatal(err)
	}
	// 旧贡献 2 次均值 (1.5,0)；新观测 (2.5,0)，最终位置 1.5+1/3。
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
	lms, _ := m3.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].Count != 3 {
		t.Fatalf("landmark = %+v, want K count 3", lms)
	}
	approxEq(t, "mixed legacy position", lms[0].X, 1.5+1.0/3.0)
	m3.Close()

	// 最终位置被篡改：旧贡献起点不变、来源观测不变，仍必须拒绝。
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].X = 8
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("tampered mixed position: err = %v, want ErrCorrupt, map=%v", err, m)
	}

	// 旧贡献均值被篡改（不能当成零、也不能按缺少依据的帧数代替）：拒绝。
	fd = readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].LegacyMX = 0
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("tampered legacy mean: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 完全没有逐帧观测依据的纯旧文件不核对位置：篡改保存位置后仍按现有规则
// 打开，不要求补造观测。
func TestOpenPureLegacyFileSkipsPositionCheck(t *testing.T) {
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

	fd := readFileData(t, path)
	if len(fd.Sources) != 0 {
		t.Fatalf("test setup wrong: sources = %+v", fd.Sources)
	}
	fd.Landmarks[0].X = 42
	writeFileDataRaw(t, path, &fd)
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("pure legacy file must still open: %v", err)
	}
	m2.Close()
}

// 现有可接受的有限大坐标不能仅因新增位置检查被误报为损坏：大坐标下增量
// 合并保存的位置必须能由同一序列逐位重放出来。
func TestOpenPositionCheckAcceptsLargeFiniteCoordinates(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.bin")
	m, err := Create(path, Config{
		InitialTime: 1000, MaxInterval: 100, MergeDistance: 3e307,
	})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 1001, Observations: []Observation{{ID: "L", X: 8e307, Y: 0}}},
		{Time: 1002, Observations: []Observation{{ID: "L", X: 8e307, Y: 0}}},
		{Time: 1003, Observations: []Observation{{ID: "L", X: 1e308, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{
		ID: "c1", Anchor: 1001,
		Target: CorrectionTarget{X: 1e307, Y: 0, Heading: 0, Variance: 0},
	}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("reopen huge-coordinate corrected map: %v", err)
	}
	defer m2.Close()
	lms, _ := m2.LandmarksInRect(Rect{MinX: -math.MaxFloat64, MinY: -math.MaxFloat64, MaxX: math.MaxFloat64, MaxY: math.MaxFloat64})
	if len(lms) != 1 || !isFinite(lms[0].X) {
		t.Fatalf("landmark = %+v", lms)
	}
	want := 9e307 + (1.1e308-9e307)/3
	if math.Abs(lms[0].X-want) > math.Abs(want)*1e-12 {
		t.Fatalf("X = %v, want ~%v", lms[0].X, want)
	}
}
