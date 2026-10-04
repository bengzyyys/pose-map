package posemap

import (
	"path/filepath"
	"testing"
)

// buildCorrectedLandmarkMap 构造 t=100、200 各观测 L 一次、t=300 无观测的
// 地图，对 t=100 校正（记录覆盖三帧，路标列表只有 L 的第 1 次出现），随后
// 再导入 t=400 观测 M 的一段（M 在校正范围之外）。返回提交时的校正记录。
func buildCorrectedLandmarkMap(t *testing.T, path string) CorrectionRecord {
	t.Helper()
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 300, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, Observations: []Observation{{ID: "M"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	return rec
}

// 校正记录的路标列表必须准确描述覆盖范围内观测涉及的路标出现：漏记、
// 重复、多记（含范围外观测的路标），或把出现编号改成范围没有涉及的
// 另一编号，都按损坏拒绝，且原文件保持不变。
func TestOpenRejectsBadCorrectionLandmarkList(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"missing landmark entry", func(fd *fileData) {
			fd.Corrections[0].Landmarks = nil
		}},
		{"duplicate landmark entry", func(fd *fileData) {
			l := fd.Corrections[0].Landmarks[0]
			fd.Corrections[0].Landmarks = append(fd.Corrections[0].Landmarks, l)
		}},
		{"landmark observed only outside range", func(fd *fileData) {
			l := landmarkChangeJSON{ID: "M", Occurrence: 1}
			l.Before = Landmark{ID: "M", X: 1, Y: 2, Count: 1}
			l.After = Landmark{ID: "M", X: 1, Y: 2, Count: 1}
			fd.Corrections[0].Landmarks = append(fd.Corrections[0].Landmarks, l)
		}},
		{"occurrence number not involved in range", func(fd *fileData) {
			fd.Corrections[0].Landmarks[0].Occurrence = 2
		}},
		{"unknown landmark", func(fd *fileData) {
			l := landmarkChangeJSON{ID: "ZZZ", Occurrence: 1}
			l.Before = Landmark{ID: "ZZZ", X: 1, Y: 2, Count: 1}
			l.After = Landmark{ID: "ZZZ", X: 1, Y: 2, Count: 1}
			fd.Corrections[0].Landmarks = append(fd.Corrections[0].Landmarks, l)
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildCorrectedLandmarkMap(t, path)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 范围内没有路标观测的校正，空路标列表合法，非空列表必须拒绝。
func TestOpenCorrectionLandmarkListEmptyRange(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 空列表合法：正常打开。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	m2.Close()

	// 非空列表拒绝。
	tamperFile(t, path, func(fd *fileData) {
		l := landmarkChangeJSON{ID: "L", Occurrence: 1}
		l.Before = Landmark{ID: "L", X: 1, Y: 2, Count: 1}
		l.After = Landmark{ID: "L", X: 1, Y: 2, Count: 1}
		fd.Corrections[0].Landmarks = []landmarkChangeJSON{l}
	})
	assertOpenCorrupt(t, path)
}

// 未篡改的路标列表正常打开，记录内容与提交时一致；缺出现编号的旧条目
// 按第 1 次出现解释，同样合法。
func TestOpenAcceptsCorrectionLandmarkList(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	want := buildCorrectedLandmarkMap(t, path)
	if len(want.Landmarks) != 1 || want.Landmarks[0].ID != "L" || want.Landmarks[0].Occurrence != 1 {
		t.Fatalf("committed record = %+v", want)
	}

	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	m.Close()
	if len(recs) != 1 || !recordsEqual(recs[0], want) {
		t.Fatalf("records = %+v, want %+v", recs, want)
	}

	// 旧版本写出的记录不带出现编号：按第 1 次出现解释，正常打开。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Landmarks[0].Occurrence = 0
	})
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("open legacy occurrence-less record: %v", err)
	}
	recs, _ = m3.Corrections()
	m3.Close()
	if len(recs) != 1 || len(recs[0].Landmarks) != 1 || recs[0].Landmarks[0].Occurrence != 1 {
		t.Fatalf("legacy occurrence-less record = %+v", recs)
	}
}

// 校正范围跨越同一路标的两次出现：L 在 t=100、200 被观测（t=200 的观测
// 发生在失效时刻，仍属旧出现），失效后 t=300 再次出现。校正记录必须分别
// 列出两次出现；已失效的旧出现不因区域查询不再返回它而豁免。
func TestOpenCorrectionLandmarkListAcrossOccurrences(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
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
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}})
	if err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	if len(rec.Landmarks) != 2 ||
		rec.Landmarks[0].Occurrence != 1 || rec.Landmarks[1].Occurrence != 2 {
		t.Fatalf("committed record = %+v", rec)
	}

	// 完整记录正常打开，内容保持提交时的快照。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	recs, _ := m2.Corrections()
	m2.Close()
	if len(recs) != 1 || !recordsEqual(recs[0], rec) {
		t.Fatalf("records = %+v, want %+v", recs, rec)
	}

	// 漏记已失效的旧出现：拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Landmarks = fd.Corrections[0].Landmarks[1:]
	})
	assertOpenCorrupt(t, path)

	// 把新出现的编号改成旧出现：重复且漏记，拒绝。
	path2 := filepath.Join(t.TempDir(), "m.pose")
	build := func() {
		mm, err := Create(path2, baseConfig())
		if err != nil {
			t.Fatal(err)
		}
		if _, err := mm.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
			{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
		}}); err != nil {
			t.Fatal(err)
		}
		if _, err := mm.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
			t.Fatal(err)
		}
		if _, err := mm.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 300, DX: 1, Observations: []Observation{{ID: "L"}}},
		}}); err != nil {
			t.Fatal(err)
		}
		if _, err := mm.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
			t.Fatal(err)
		}
		if err := mm.Close(); err != nil {
			t.Fatal(err)
		}
	}
	build()
	tamperFile(t, path2, func(fd *fileData) {
		fd.Corrections[0].Landmarks[1].Occurrence = 1
	})
	assertOpenCorrupt(t, path2)
}

// 只改方差的恒等校正不改变任何出现的位置，但范围内有观测的出现仍保留
// 记录；重新打开不得因前后值相同而误判，也不得漏记。
func TestOpenAcceptsIdentityCorrectionLandmarkList(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	pose, err := m.PoseAt(100)
	if err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{
		X: pose.X, Y: pose.Y, Heading: pose.Heading, Variance: 3,
	}})
	if err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	if len(rec.Landmarks) != 1 || rec.Landmarks[0].Before != rec.Landmarks[0].After {
		t.Fatalf("identity correction record = %+v", rec)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	recs, _ := m2.Corrections()
	m2.Close()
	if len(recs) != 1 || !recordsEqual(recs[0], rec) {
		t.Fatalf("records = %+v, want %+v", recs, rec)
	}

	// 位置未变也不能漏记：删掉该条目后必须拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Landmarks = nil
	})
	assertOpenCorrupt(t, path)
}

// 缺少逐帧依据的旧地图：在其上新追加且依据完整的校正范围同样遵守路标
// 列表校验；覆盖帧含旧帧（无逐帧依据）的校正记录保留既有规则，不要求
// 补造观测。
func TestOpenCorrectionLandmarkListOnLegacyMap(t *testing.T) {
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
		{Time: 300, DX: 1, Observations: []Observation{{ID: "K", X: -1.5}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// 校正范围只有 t=300 一帧，逐帧依据完整：路标列表必须列出 K。
	if _, err := m2.Correct(Correction{ID: "c1", Anchor: 300, Target: CorrectionTarget{X: 3.5, Y: 0, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	m3.Close()

	// 依据完整的校正范围漏记 K：拒绝。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Landmarks = nil
	})
	assertOpenCorrupt(t, path)
}

// 覆盖帧含旧帧（无逐帧依据）的校正记录无法重建涉及集合，保留既有基本
// 校验，不要求补造观测：把上例校正的范围篡改为覆盖旧帧 t=200 后，路标
// 列表不再按涉及集合核对，文件仍可打开。
func TestOpenCorrectionSpanningLegacyFramesSkipsLandmarkListCheck(t *testing.T) {
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
		{Time: 300, DX: 1, Observations: []Observation{{ID: "K", X: -1.5}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m2.Correct(Correction{ID: "c1", Anchor: 300, Target: CorrectionTarget{X: 3.5, Y: 0, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 篡改范围为 [200, 300]：帧 200 来自旧文件、无逐帧依据，路标列表
	// 校验不适用，空列表也不因此被拒。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Anchor = 200
		fd.Corrections[0].EndTime = 300
		fd.Corrections[0].Poses = []poseChangeJSON{
			{Before: Pose{Time: 200, X: 2}, After: Pose{Time: 200, X: 2}},
			{Before: Pose{Time: 300, X: 3}, After: Pose{Time: 300, X: 3.5}},
		}
		fd.Corrections[0].Landmarks = nil
	})
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("open correction spanning legacy frames: %v", err)
	}
	m3.Close()
}
