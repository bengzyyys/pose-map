package posemap

import (
	"errors"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// countTestConfig 放宽路标合并距离：多个相邻帧反复观测同一标识时，各观测
// 的地图位置随轨迹依次相距 1，等权均值会与后来观测拉开超过 1 的距离；这
// 里只关心次数核对，用足够大的合并上限让整条观测链都被接纳。
func countTestConfig() Config {
	cfg := baseConfig()
	cfg.MergeDistance = 10
	return cfg
}

// setCorrCount 篡改第 corrIdx 条校正记录中（路标 id、出现 occ）的校正前/
// 后观测次数。
func setCorrCount(fd *fileData, corrIdx int, id string, occ, before, after int) {
	lcs := fd.Corrections[corrIdx].Landmarks
	for i := range lcs {
		if lcs[i].ID == id && (lcs[i].Occurrence == occ || (lcs[i].Occurrence == 0 && occ == 1)) {
			lcs[i].Before.Count = before
			lcs[i].After.Count = after
			return
		}
	}
	panic("landmark change " + id + "/" + strconv.Itoa(occ) + " not found")
}

// findCorrCount 返回第 corrIdx 条记录中（id,occ）的校正前、后次数。
func findCorrCount(t *testing.T, fd *fileData, corrIdx int, id string, occ int) (int, int) {
	t.Helper()
	for _, lc := range fd.Corrections[corrIdx].Landmarks {
		n := lc.Occurrence
		if n == 0 {
			n = 1
		}
		if lc.ID == id && n == occ {
			return lc.Before.Count, lc.After.Count
		}
	}
	t.Fatalf("landmark change %s/%d not found in correction %d", id, occ, corrIdx)
	return 0, 0
}

// 任务给出的典型场景：某次出现到 t=300 已接纳三条观测，校正记录结束于
// 300；后来在 t=400 又接纳一条。旧记录的校正前、后次数都应保留为 3，
// 当前路标可以是 4。未篡改时正常打开；把旧记录两边都写成 4 同样不合法。
func TestOpenCorrectionCountsFreezeAtRecordEnd(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, countTestConfig())
	if err != nil {
		t.Fatal(err)
	}
	// t=100/200/300 各一条 L，相邻地图距离恰为合并距离 1，全部接纳。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 300, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	// 记录结束于 300：再追加 t=400 的一条观测，当前出现累计 4 次。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	fd := readFileData(t, path)
	if b, a := findCorrCount(t, &fd, 0, "L", 1); b != 3 || a != 3 {
		t.Fatalf("record counts = %d/%d, want 3/3", b, a)
	}
	if fd.Landmarks[0].Occurrences[0].Count != 4 {
		t.Fatalf("current landmark count = %d, want 4", fd.Landmarks[0].Occurrences[0].Count)
	}

	// 未篡改：旧记录 3/3、当前 4，正常打开。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open frozen record: %v", err)
	}
	recs, _ := m2.Corrections()
	if b, a := recs[0].Landmarks[0].Before.Count, recs[0].Landmarks[0].After.Count; b != 3 || a != 3 {
		t.Fatalf("reloaded record counts = %d/%d, want 3/3", b, a)
	}
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].Count != 4 {
		t.Fatalf("current landmark = %+v, want count 4", lms)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 把旧记录两边都写成后来的 4：即使前后相等、路标列表与位置都正常，
	// 也必须拒绝。
	tamperFile(t, path, func(fd *fileData) { setCorrCount(fd, 0, "L", 1, 4, 4) })
	assertOpenCorrupt(t, path)
}

// 次数必须是截至记录结束时间的总数，包含锚点之前属于同一次出现的观测与
// 结束帧上的观测，不能只算被移动的 [anchor,end] 那段。
func TestOpenCorrectionCountsIncludePreAnchorObservations(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, countTestConfig())
	if err != nil {
		t.Fatal(err)
	}
	// t=100、200 在锚点之前观测 L；锚点取无观测的 t=300；t=400 再观测。
	// 被移动段 [300,400] 内只有 t=400 一条，但截至结束时间共接纳 3 条。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 300, DX: 1},
		{Time: 400, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// 用锚点当前位姿做恒等校正（只固定方差）：移动段内观测位置不变，仍与
	// 锚点前固定观测合并；次数核对与几何移动无关。
	cur, err := m.PoseAt(300)
	if err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 300, Target: CorrectionTarget{
		X: cur.X, Y: cur.Y, Heading: cur.Heading, Variance: cur.Variance,
	}})
	if err != nil {
		t.Fatal(err)
	}
	if lc := rec.Landmarks[0]; lc.Before.Count != 3 || lc.After.Count != 3 {
		t.Fatalf("record counts = %d/%d, want 3/3 (whole occurrence, not moved segment)", lc.Before.Count, lc.After.Count)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 未篡改：正常打开。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	m2.Close()

	// 只按被移动段写成 1：必须拒绝。
	tamperFile(t, path, func(fd *fileData) { setCorrCount(fd, 0, "L", 1, 1, 1) })
	assertOpenCorrupt(t, path)
}

// 同帧对同一路标的多条观测各计一次；无观测帧不增加次数。
func TestOpenCorrectionCountsSameFrameDuplicates(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, countTestConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1}, // 无观测帧
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}, {ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 记录次数应为 2（无观测的 t=100 不增加）。
	fd := readFileData(t, path)
	if b, a := findCorrCount(t, &fd, 0, "L", 1); b != 2 || a != 2 {
		t.Fatalf("counts = %d/%d, want 2/2", b, a)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	m2.Close()

	tamperFile(t, path, func(fd *fileData) { setCorrCount(fd, 0, "L", 1, 1, 1) })
	assertOpenCorrupt(t, path)
}

// 校正前、校正后次数只要有一项不符即拒绝，即使另一项正确。
func TestOpenRejectsMismatchedBeforeOrAfterCount(t *testing.T) {
	cases := []struct {
		name   string
		before int
		after  int
	}{
		{"before too high", 4, 3},
		{"after too high", 3, 4},
		{"before too low", 2, 3},
		{"after too low", 3, 2},
		{"equal but wrong", 2, 2},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			m, err := Create(path, countTestConfig())
			if err != nil {
				t.Fatal(err)
			}
			if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
				{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
				{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
				{Time: 300, DX: 1, Observations: []Observation{{ID: "L"}}},
			}}); err != nil {
				t.Fatal(err)
			}
			if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
				t.Fatal(err)
			}
			if err := m.Close(); err != nil {
				t.Fatal(err)
			}
			tamperFile(t, path, func(fd *fileData) { setCorrCount(fd, 0, "L", 1, tc.before, tc.after) })
			assertOpenCorrupt(t, path)
		})
	}
}

// 各次出现分别核对：不能借另一出现的观测补足次数；已失效的旧出现只要被
// 校正涉及也要核对。
func TestOpenCorrectionCountsPerOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path) // L/1：t=100、200（200 失效，2 次）；L/2：t=300（1 次）
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	fd := readFileData(t, path)
	if b, a := findCorrCount(t, &fd, 0, "L", 1); b != 2 || a != 2 {
		t.Fatalf("L/1 counts = %d/%d, want 2/2", b, a)
	}
	if b, a := findCorrCount(t, &fd, 0, "L", 2); b != 1 || a != 1 {
		t.Fatalf("L/2 counts = %d/%d, want 1/1", b, a)
	}

	cases := []struct {
		name        string
		occ         int
		before, aft int
	}{
		{"invalidated old occurrence inflated", 1, 3, 3}, // 不能借 L/2 的一条
		{"new occurrence inflated", 2, 2, 2},             // 不能借 L/1 的观测
		{"old occurrence deflated", 1, 1, 1},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			p := filepath.Join(t.TempDir(), "m.pose")
			buildOccurrenceMap(t, p)
			m2, err := Open(p)
			if err != nil {
				t.Fatal(err)
			}
			if _, err := m2.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
				t.Fatal(err)
			}
			if err := m2.Close(); err != nil {
				t.Fatal(err)
			}
			tamperFile(t, p, func(fd *fileData) { setCorrCount(fd, 0, "L", tc.occ, tc.before, tc.aft) })
			assertOpenCorrupt(t, p)
		})
	}
}

// 旧地图保存的固定观测次数作为所属出现的既有贡献纳入总数，不能当作零：
// 两条 null 旧帧贡献 2 次、依据完整的 t=300 再贡献 1 条，仅覆盖 t=300 的
// 完整校正记录次数必须是 3；写成 1（忽略固定贡献）必须拒绝。
func TestOpenCorrectionCountsIncludeLegacyContribution(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	path := filepath.Join(t.TempDir(), "m.pose")
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
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	cur, _ := m2.PoseAt(300)
	if _, err := m2.Correct(Correction{ID: "cnew", Anchor: 300, Target: CorrectionTarget{
		X: cur.X, Y: cur.Y, Heading: cur.Heading, Variance: cur.Variance,
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	fd := readFileData(t, path)
	if b, a := findCorrCount(t, &fd, 0, "K", 1); b != 3 || a != 3 {
		t.Fatalf("counts = %d/%d, want 3/3 (legacy 2 + sourced 1)", b, a)
	}
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("open with legacy contribution: %v", err)
	}
	m3.Close()

	tamperFile(t, path, func(fd *fileData) { setCorrCount(fd, 0, "K", 1, 1, 1) })
	assertOpenCorrupt(t, path)
}

// 后来再次校正重叠轨迹不改变判断：每条记录都以自己的结束时间为准。
// c1 结束于 300（3 次），追加 t=400 后 c2 结束于 400（4 次）。
func TestOpenCorrectionCountsIndependentPerRecordEnd(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, countTestConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 300, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 100, Target: CorrectionTarget{X: 7, Y: 8, Variance: 2}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	fd := readFileData(t, path)
	if b, a := findCorrCount(t, &fd, 0, "L", 1); b != 3 || a != 3 {
		t.Fatalf("c1 counts = %d/%d, want 3/3", b, a)
	}
	if b, a := findCorrCount(t, &fd, 1, "L", 1); b != 4 || a != 4 {
		t.Fatalf("c2 counts = %d/%d, want 4/4", b, a)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open both records: %v", err)
	}
	m2.Close()

	// c1 提前结束，不能写成 c2 时的 4。
	tamperFile(t, path, func(fd *fileData) { setCorrCount(fd, 0, "L", 1, 4, 4) })
	assertOpenCorrupt(t, path)
}

// 校正范围内含缺失逐帧来源的 null 旧帧时，保留现有打开规则、不推测缺失
// 观测：即使手写记录的次数明显与可见来源不符也不按次数核对拒绝。同一文
// 件中另一条依据完整的记录被改坏次数时仍须拒绝。
func TestOpenSkipsCountCheckWhenRangeLacksBasis(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	path := filepath.Join(t.TempDir(), "m.pose")
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
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	cur, _ := m2.PoseAt(300)
	if _, err := m2.Correct(Correction{ID: "cfull", Anchor: 300, Target: CorrectionTarget{
		X: cur.X, Y: cur.Y, Heading: cur.Heading, Variance: cur.Variance,
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 手写一条覆盖 t=100..300（含 null 旧帧）的记录，故意把 K/1 次数写
	// 成与任何口径都不符的 99：依据不完整，保留旧规则照常打开。
	tamperFile(t, path, func(fd *fileData) {
		cr := corrRecJSON{ID: "span", Anchor: 100, EndTime: 300,
			Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}
		for j := 1; j < len(fd.Trajectory); j++ {
			cr.Poses = append(cr.Poses, poseChangeJSON{Before: fd.Trajectory[j], After: fd.Trajectory[j]})
		}
		cr.Landmarks = []landmarkChangeJSON{{
			ID: "K", Occurrence: 1,
			Before: Landmark{ID: "K", X: 1, Y: 1, Count: 99},
			After:  Landmark{ID: "K", X: 1, Y: 1, Count: 99},
		}}
		fd.Corrections = append(fd.Corrections, cr)
	})
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("null-frame range keeps legacy open rules: %v", err)
	}
	if recs, _ := m3.Corrections(); len(recs) != 2 {
		t.Fatalf("records = %d, want 2", len(recs))
	}
	if err := m3.Close(); err != nil {
		t.Fatal(err)
	}

	// 同一文件中依据完整的 cfull 记录改坏次数：仍须拒绝整份地图。
	tamperFile(t, path, func(fd *fileData) { setCorrCount(fd, 0, "K", 1, 7, 7) })
	assertOpenCorrupt(t, path)
}

// 次数矛盾的错误必须可 errors.Is 识别为 ErrCorrupt，且错误信息指出校正
// 标识、路标标识与出现编号。
func TestOpenCorrectionCountErrorIdentifiesRecordLandmarkOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "corr-X", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	tamperFile(t, path, func(fd *fileData) { setCorrCount(fd, 0, "L", 2, 5, 5) })

	m2, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m2)
	}
	msg := err.Error()
	for _, want := range []string{"corr-X", "L", "occurrence 2"} {
		if !strings.Contains(msg, want) {
			t.Fatalf("error %q does not identify %q", msg, want)
		}
	}
}
