package posemap

import "testing"

// 本文件为“校正记录的返回值是独立副本”这一已有行为提供回归保障：
// 调用方拿到 Correct 返回的记录或 Corrections 返回的列表后，无论
// 怎样改写其中的标识、目标、时间信息、位姿/路标前后值，或调换、
// 替换列表条目，都不能借此修改地图状态和已保存的校正历史。只使用
// 现有公开入口（ImportSegment/Correct/Corrections/Invalidate/
// LandmarkAppearances/CurrentPose/PoseAt/LandmarksInRect），不改变
// 任何公开使用方式与返回结果。

// snapshotRecord 在测试侧深拷贝一份记录作为期望快照。不复用被测
// 实现里的 cloneRecord，避免被测实现退化为浅拷贝时期望值与内部
// 状态共享底层切片、被一并改写而失去对照意义。
func snapshotRecord(rec CorrectionRecord) CorrectionRecord {
	cp := rec
	cp.Poses = append([]PoseChange(nil), rec.Poses...)
	cp.Landmarks = append([]LandmarkChange(nil), rec.Landmarks...)
	return cp
}

// scrambleRecord 模拟调用方就地改写一份校正记录的全部可写字段：
// 标识、锚点、目标、末帧时间，以及每条位姿/路标变化的出现编号与
// 校正前后值。若返回值不是独立副本，这些改写会污染地图内部状态。
func scrambleRecord(rec *CorrectionRecord) {
	rec.ID = "tampered"
	rec.Anchor = -999
	rec.Target = CorrectionTarget{X: -1, Y: -2, Heading: -3, Variance: -4}
	rec.EndTime = -1
	for i := range rec.Poses {
		rec.Poses[i].Before = Pose{Time: -1, X: -100, Y: -100, Heading: -100, Variance: -100}
		rec.Poses[i].After = Pose{Time: -2, X: -200, Y: -200, Heading: -200, Variance: -200}
	}
	for i := range rec.Landmarks {
		rec.Landmarks[i].ID = "tampered"
		rec.Landmarks[i].Occurrence = 999
		rec.Landmarks[i].Before = Landmark{ID: "tampered", X: -1, Y: -1, Count: -1}
		rec.Landmarks[i].After = Landmark{ID: "tampered", X: -2, Y: -2, Count: -2}
	}
}

// checkRecords 断言查询得到的记录列表与期望内容、次序完全一致。
func checkRecords(t *testing.T, got []CorrectionRecord, want ...CorrectionRecord) {
	t.Helper()
	if len(got) != len(want) {
		t.Fatalf("records len = %d, want %d: %v", len(got), len(want), got)
	}
	for i := range want {
		if !recordsEqual(got[i], want[i]) {
			t.Fatalf("record %d = %+v, want %+v", i, got[i], want[i])
		}
	}
}

func findLandmark(lms []Landmark, id string) (Landmark, bool) {
	for _, lm := range lms {
		if lm.ID == id {
			return lm, true
		}
	}
	return Landmark{}, false
}

// 成功校正同时包含多帧位姿变化与路标变化：改写返回记录后，再次查询
// 仍取得原始记录，当前位姿、历史位姿、路标位置与观测次数都保持成功
// 校正后的真实结果。
func TestCorrectionReturnedRecordIsolated(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(corrSegment()); err != nil {
		t.Fatal(err)
	}
	req := Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 2}}
	rec, err := m.Correct(req)
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if len(rec.Poses) != 2 || len(rec.Landmarks) != 2 {
		t.Fatalf("record should cover 2 poses and 2 landmarks: %+v", rec)
	}
	want := snapshotRecord(rec)

	// 调用方改写标识、目标、时间信息以及位姿和路标的校正前后值。
	scrambleRecord(&rec)

	// 再次查询仍取得提交时的原始记录。
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	checkRecords(t, recs, want)

	// 当前位姿与相应时间的历史位姿保持真实校正结果。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 12 || cur.Y != 20 || cur.Variance != 2.25 {
		t.Fatalf("current = %+v, want corrected (12,20) v=2.25", cur)
	}
	p, _ := m.PoseAt(100)
	if p.Time != 100 || p.X != 10 || p.Y != 20 || p.Variance != 2 {
		t.Fatalf("pose at 100 = %+v, want corrected anchor", p)
	}
	p, _ = m.PoseAt(200)
	if p.Time != 200 || p.X != 12 || p.Y != 20 {
		t.Fatalf("pose at 200 = %+v, want corrected end pose", p)
	}

	// 路标位置与观测次数保持真实校正结果，不采用调用方改写的值。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 2 {
		t.Fatalf("landmarks = %v", lms)
	}
	l1, ok := findLandmark(lms, "L1")
	if !ok {
		t.Fatalf("L1 missing: %v", lms)
	}
	approxEq(t, "L1 x", l1.X, 11.5)
	approxEq(t, "L1 y", l1.Y, 20)
	if l1.Count != 2 {
		t.Fatalf("L1 count = %d, want 2", l1.Count)
	}
	mm, ok := findLandmark(lms, "M")
	if !ok {
		t.Fatalf("M missing: %v", lms)
	}
	approxEq(t, "M x", mm.X, 12)
	approxEq(t, "M y", mm.Y, 21)
	if mm.Count != 1 {
		t.Fatalf("M count = %d, want 1", mm.Count)
	}
}

// 校正跨越同一路标的两次出现（旧出现已失效、新出现仍有效）：改写
// 返回记录中的出现编号与各次前后值后，地图中两次出现的位置、计数
// 与失效信息都不受影响，区域查询仍只包含有效出现。
func TestCorrectionRecordIsolationAcrossOccurrences(t *testing.T) {
	m := newMap(t, baseConfig())
	// 全部沿世界 X 轴、朝向 0：初始位姿 (1,2)。
	// 第 1 次出现：t=100 观测 → (1,2)，t=200 观测 → (2,2)，均值 (1.5,2) 计数 2。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 0, MoveVariance: 0.1, Observations: []Observation{{ID: "D", X: 0, Y: 0}}},
		{Time: 200, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "D", X: 0, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	inv, err := m.Invalidate(Invalidation{ID: "inv1", Reason: "stale", Landmarks: []string{"D"}})
	if err != nil {
		t.Fatalf("Invalidate: %v", err)
	}
	if inv.Time != 200 || len(inv.Landmarks) != 1 || inv.Landmarks[0].Occurrence != 1 {
		t.Fatalf("invalidation result = %+v", inv)
	}
	// 第 2 次出现：t=300 观测 → (3,2)，t=400 观测 → (4,2)，均值 (3.5,2) 计数 2。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "D", X: 0, Y: 0}}},
		{Time: 400, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "D", X: 0, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 整体平移 +10：两次出现分别重放为 (11.5,2) 与 (13.5,2)，计数不变。
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 11, Y: 2, Heading: 0, Variance: 1}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if len(rec.Poses) != 4 || len(rec.Landmarks) != 2 {
		t.Fatalf("record should cover 4 poses and 2 occurrences: %+v", rec)
	}
	if rec.Landmarks[0].Occurrence != 1 || rec.Landmarks[1].Occurrence != 2 ||
		rec.Landmarks[0].ID != "D" || rec.Landmarks[1].ID != "D" {
		t.Fatalf("occurrence order = %+v", rec.Landmarks)
	}
	want := snapshotRecord(rec)

	// 调用方改写两次出现的编号与各次前后值。
	scrambleRecord(&rec)

	// 再次查询仍取得原始记录（出现编号 1、2 与真实前后值）。
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	checkRecords(t, recs, want)

	// 地图中两次出现的位置、计数与失效信息都不受影响。
	h, err := m.LandmarkAppearances("D")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	ap1 := h.Appearances[0]
	if ap1.Number != 1 || ap1.Active || ap1.Landmark.Count != 2 {
		t.Fatalf("appearance 1 = %+v", ap1)
	}
	approxEq(t, "occ1 x", ap1.Landmark.X, 11.5)
	approxEq(t, "occ1 y", ap1.Landmark.Y, 2)
	if ap1.InvalidTime != 200 || ap1.InvalidReason != "stale" || ap1.InvalidOpID != "inv1" {
		t.Fatalf("appearance 1 invalidation info = %+v", ap1)
	}
	ap2 := h.Appearances[1]
	if ap2.Number != 2 || !ap2.Active || ap2.Landmark.Count != 2 {
		t.Fatalf("appearance 2 = %+v", ap2)
	}
	approxEq(t, "occ2 x", ap2.Landmark.X, 13.5)
	approxEq(t, "occ2 y", ap2.Landmark.Y, 2)

	// 区域查询仍只包含有效出现。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "D" {
		t.Fatalf("rect query = %v, want only active occurrence", lms)
	}
	approxEq(t, "active x", lms[0].X, 13.5)
	if lms[0].Count != 2 {
		t.Fatalf("active count = %d, want 2", lms[0].Count)
	}

	// 位姿同样保持真实校正结果。
	cur, _ := m.CurrentPose()
	if cur.Time != 400 || cur.X != 14 || cur.Y != 2 {
		t.Fatalf("current = %+v, want corrected (14,2)", cur)
	}
}

// Corrections 返回的列表同样是独立副本：改写其中记录、调换或替换
// 条目都不影响再次查询的内容与次序；先后取得的两份结果互不影响。
// 随后提交覆盖部分轨迹的新校正时，先前取得的结果保留获取时的内容，
// 保存的第一条记录仍说明第一次操作，新记录反映第二次操作实际发生
// 时的状态。
func TestCorrectionsListIsolation(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(corrSegment()); err != nil {
		t.Fatal(err)
	}
	c1 := Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 2}}
	r1, err := m.Correct(c1)
	if err != nil {
		t.Fatal(err)
	}
	// 第二次校正把末帧移到 (11.5,20)：L1 两次观测 (11,20)/(11.5,20)
	// 距离 0.5 可接受，均值 (11.25,20)；M 移到 (11.5,21)。
	c2 := Correction{ID: "c2", Anchor: 200, Target: CorrectionTarget{X: 11.5, Y: 20, Heading: 0, Variance: 7}}
	r2, err := m.Correct(c2)
	if err != nil {
		t.Fatal(err)
	}
	// 测试侧快照作为对照，与被测实现的拷贝行为无关。
	s1, s2 := snapshotRecord(r1), snapshotRecord(r2)

	listA, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	checkRecords(t, listA, s1, s2)

	// 调用方改写列表中某条记录及其位姿、路标变化，并调换、替换条目。
	scrambleRecord(&listA[0])
	listA[0], listA[1] = listA[1], listA[0]
	listA[1] = CorrectionRecord{ID: "fake"}

	// 再次查询仍保留提交时的内容与先后次序。
	listB, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	checkRecords(t, listB, s1, s2)

	// 先后取得的两份结果互不干扰：改写 listB 不污染后续查询。
	scrambleRecord(&listB[0])
	listC, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	checkRecords(t, listC, s1, s2)

	// 提交第三次校正，覆盖第一次校正涉及的部分轨迹（末帧 t=200）：
	// 末帧 (11.5,20) → (11,20)，L1 均值 (11,20)，M → (11,21)。
	c3 := Correction{ID: "c3", Anchor: 200, Target: CorrectionTarget{X: 11, Y: 20, Heading: 0, Variance: 9}}
	r3, err := m.Correct(c3)
	if err != nil {
		t.Fatal(err)
	}
	if len(r3.Poses) != 1 || r3.Poses[0].Before.X != 11.5 || r3.Poses[0].After.X != 11 {
		t.Fatalf("third record = %+v, want before from c2 result", r3)
	}
	s3 := snapshotRecord(r3)

	// 先前取得的 listC 保留获取时的内容（仍只有两条记录）。
	checkRecords(t, listC, s1, s2)

	// 保存的第一条记录仍说明第一次操作的前后变化，新记录反映第三次
	// 操作实际发生时的变化。
	listD, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	checkRecords(t, listD, s1, s2, s3)
}

// 同一标识、同一原始请求再次提交：即使地图已接受后续校正，仍返回
// 首次记录而非最新轨迹，也不再增加记录；该返回结果同样可被调用方
// 独立改写而不污染之后查询或再次取得的首次结果。
func TestCorrectionDuplicateResultIsolation(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(corrSegment()); err != nil {
		t.Fatal(err)
	}
	c1 := Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 2}}
	r1, err := m.Correct(c1)
	if err != nil {
		t.Fatal(err)
	}
	s1 := snapshotRecord(r1)
	// 地图接受后续校正，轨迹已不同于第一次校正后的状态。
	c2 := Correction{ID: "c2", Anchor: 200, Target: CorrectionTarget{X: 11.5, Y: 20, Heading: 0, Variance: 7}}
	if _, err := m.Correct(c2); err != nil {
		t.Fatal(err)
	}

	// 重复提交返回首次记录，不改为返回最新轨迹，也不增加记录。
	dup, err := m.Correct(c1)
	if err != nil {
		t.Fatalf("duplicate correction: %v", err)
	}
	if !recordsEqual(dup, s1) {
		t.Fatalf("duplicate returned %+v, want first record %+v", dup, s1)
	}
	recs, _ := m.Corrections()
	if len(recs) != 2 {
		t.Fatalf("duplicate added a record: %v", recs)
	}

	// 改写重复提交返回的结果，不污染之前取得的列表、之后查询与再次
	// 取得的首次结果。
	scrambleRecord(&dup)
	if !recordsEqual(recs[0], s1) {
		t.Fatalf("earlier query result polluted: %+v", recs[0])
	}
	recs, _ = m.Corrections()
	if len(recs) != 2 || !recordsEqual(recs[0], s1) {
		t.Fatalf("saved records polluted by scrambling duplicate result: %v", recs)
	}
	dup2, err := m.Correct(c1)
	if err != nil {
		t.Fatalf("second duplicate: %v", err)
	}
	if !recordsEqual(dup2, s1) {
		t.Fatalf("second duplicate returned %+v, want first record", dup2)
	}
	if recs, _ := m.Corrections(); len(recs) != 2 {
		t.Fatalf("second duplicate added a record: %v", recs)
	}
}

// 没有路标观测的合法校正同样保留位姿记录的隔离行为。
func TestCorrectionRecordIsolationNoLandmarks(t *testing.T) {
	m := newMap(t, baseConfig())
	// 无观测轨迹：p1 (2,2) v=0.6，p2 (3,2) v=0.7。
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.1},
		{Time: 200, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 5, Heading: 0, Variance: 1}})
	if err != nil {
		t.Fatal(err)
	}
	if len(rec.Poses) != 2 || len(rec.Landmarks) != 0 {
		t.Fatalf("record = %+v, want 2 pose changes and no landmark changes", rec)
	}
	want := snapshotRecord(rec)

	scrambleRecord(&rec)

	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	checkRecords(t, recs, want)

	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 11 || cur.Y != 5 || cur.Variance != 1.1 {
		t.Fatalf("current = %+v, want corrected (11,5) v=1.1", cur)
	}
	p, _ := m.PoseAt(100)
	if p.X != 10 || p.Y != 5 || p.Variance != 1 {
		t.Fatalf("pose at 100 = %+v, want corrected anchor", p)
	}
}
