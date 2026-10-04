package posemap

import (
	"testing"
)

// 本文件锁定校正记录的隔离语义：Correct 返回的记录与 Corrections 返回的
// 记录列表都是当时内容的独立副本，调用方改写返回值（标识、目标、时间、
// 位姿/路标前后值、出现编号、列表元素与次序）都不能改变地图状态与已保存
// 的校正历史。

// 场景轨迹（初始位姿 t=0 (1,2) h=0 v=0.5，合并距离 1）：
//
//	p1 (t=100) (2,2) v=0.6  观测 L 本地(1,0) → 地图(3,2)   L 第 1 次出现
//	p2 (t=200) (3,2) v=0.7  观测 L 本地(0,0) → 地图(3,2)   L#1 次数 2
//	inv1 在 t=200 失效 L（第 1 次出现撤下）
//	p3 (t=300) (4,2) v=0.8  观测 L 本地(1,0) → 地图(5,2)   L 第 2 次出现
//	p4 (t=400) (5,2) v=0.9  观测 L → (5,2)（L#2 次数 2）、M 本地(0,1) → (5,3)
//
// 校正 c1（锚点 t=100，目标 (10,20) h=0 v=2）为纯平移 (+8,+18)，覆盖全部
// 四帧与 L 的两次出现及 M。
func isolationFixture(t *testing.T) (*Map, CorrectionRecord) {
	t.Helper()
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
		{Time: 200, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	inv, err := m.Invalidate(Invalidation{ID: "inv1", Reason: "expire", Landmarks: []string{"L"}})
	if err != nil {
		t.Fatal(err)
	}
	if inv.Time != 200 || len(inv.Landmarks) != 1 || inv.Landmarks[0].ID != "L" || inv.Landmarks[0].Occurrence != 1 {
		t.Fatalf("invalidation = %+v", inv)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
		{Time: 400, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "L", X: 0, Y: 0}, {ID: "M", X: 0, Y: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 2}})
	if err != nil {
		t.Fatalf("correct c1: %v", err)
	}
	return m, rec
}

// snapshotRecord 在测试侧深拷贝一份记录，冻结取得返回值当时的内容；不依赖
// 被测实现内部的克隆逻辑。
func snapshotRecord(rec CorrectionRecord) CorrectionRecord {
	cp := rec
	cp.Poses = append([]PoseChange(nil), rec.Poses...)
	cp.Landmarks = append([]LandmarkChange(nil), rec.Landmarks...)
	return cp
}

// tamperRecord 模拟调用方拿到记录后肆意改写：标识、锚点、末帧时间、目标、
// 每帧位姿前后值、路标标识、出现编号、路标前后值，并调换切片元素次序。
func tamperRecord(rec *CorrectionRecord) {
	rec.ID = "tampered-id"
	rec.Anchor = -777
	rec.EndTime = -888
	rec.Target = CorrectionTarget{X: -1e6, Y: 2e6, Heading: 2.5, Variance: 12345}
	for i := range rec.Poses {
		rec.Poses[i].Before = Pose{Time: -1, X: -1000.5, Y: -2000.5, Heading: 1.5, Variance: 9999}
		rec.Poses[i].After = Pose{Time: -2, X: 3000.5, Y: 4000.5, Heading: -1.5, Variance: 8888}
	}
	if n := len(rec.Poses); n > 1 {
		rec.Poses[0], rec.Poses[n-1] = rec.Poses[n-1], rec.Poses[0]
	}
	for i := range rec.Landmarks {
		rec.Landmarks[i].ID = "tampered-landmark"
		rec.Landmarks[i].Occurrence = 42 + i
		rec.Landmarks[i].Before = Landmark{ID: "tampered-landmark", X: -7.5, Y: -8.5, Count: 777}
		rec.Landmarks[i].After = Landmark{ID: "tampered-landmark", X: 9.5, Y: 10.5, Count: 888}
	}
	if n := len(rec.Landmarks); n > 1 {
		rec.Landmarks[0], rec.Landmarks[n-1] = rec.Landmarks[n-1], rec.Landmarks[0]
	}
}

func approxPose(t *testing.T, name string, got, want Pose) {
	t.Helper()
	if got.Time != want.Time {
		t.Fatalf("%s time = %d, want %d", name, got.Time, want.Time)
	}
	approxEq(t, name+" x", got.X, want.X)
	approxEq(t, name+" y", got.Y, want.Y)
	approxEq(t, name+" heading", got.Heading, want.Heading)
	approxEq(t, name+" variance", got.Variance, want.Variance)
}

func approxLandmark(t *testing.T, name string, got, want Landmark) {
	t.Helper()
	if got.ID != want.ID || got.Count != want.Count {
		t.Fatalf("%s = %+v, want %+v", name, got, want)
	}
	approxEq(t, name+" x", got.X, want.X)
	approxEq(t, name+" y", got.Y, want.Y)
}

// checkC1Record 校验 c1 记录本身的内容：四帧位姿前后值，以及 L 两次出现
// （旧出现已失效、新出现仍有效）与 M 各自的前后值。
func checkC1Record(t *testing.T, rec CorrectionRecord) {
	t.Helper()
	if rec.ID != "c1" || rec.Anchor != 100 || rec.EndTime != 400 {
		t.Fatalf("record header = %+v", rec)
	}
	if rec.Target != (CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 2}) {
		t.Fatalf("record target = %+v", rec.Target)
	}
	wantPoses := []PoseChange{
		{Before: Pose{Time: 100, X: 2, Y: 2, Variance: 0.6}, After: Pose{Time: 100, X: 10, Y: 20, Variance: 2}},
		{Before: Pose{Time: 200, X: 3, Y: 2, Variance: 0.7}, After: Pose{Time: 200, X: 11, Y: 20, Variance: 2.1}},
		{Before: Pose{Time: 300, X: 4, Y: 2, Variance: 0.8}, After: Pose{Time: 300, X: 12, Y: 20, Variance: 2.2}},
		{Before: Pose{Time: 400, X: 5, Y: 2, Variance: 0.9}, After: Pose{Time: 400, X: 13, Y: 20, Variance: 2.3}},
	}
	if len(rec.Poses) != len(wantPoses) {
		t.Fatalf("record poses = %+v", rec.Poses)
	}
	for i, w := range wantPoses {
		approxPose(t, "pose before", rec.Poses[i].Before, w.Before)
		approxPose(t, "pose after", rec.Poses[i].After, w.After)
	}
	type wantLM struct {
		id     string
		num    int
		before Landmark
		after  Landmark
	}
	wantLMs := []wantLM{
		{"L", 1, Landmark{ID: "L", X: 3, Y: 2, Count: 2}, Landmark{ID: "L", X: 11, Y: 20, Count: 2}},
		{"L", 2, Landmark{ID: "L", X: 5, Y: 2, Count: 2}, Landmark{ID: "L", X: 13, Y: 20, Count: 2}},
		{"M", 1, Landmark{ID: "M", X: 5, Y: 3, Count: 1}, Landmark{ID: "M", X: 13, Y: 21, Count: 1}},
	}
	if len(rec.Landmarks) != len(wantLMs) {
		t.Fatalf("record landmarks = %+v", rec.Landmarks)
	}
	for i, w := range wantLMs {
		lc := rec.Landmarks[i]
		if lc.ID != w.id || lc.Occurrence != w.num {
			t.Fatalf("landmark change %d = %+v, want %s occurrence %d", i, lc, w.id, w.num)
		}
		approxLandmark(t, "landmark before", lc.Before, w.before)
		approxLandmark(t, "landmark after", lc.After, w.after)
	}
}

// checkC1State 校验 c1 成功后的真实地图状态：当前位姿、各帧历史位姿、
// 区域查询只含有效出现、L 的两次出现记录（位置、计数、失效信息）。
func checkC1State(t *testing.T, m *Map) {
	t.Helper()
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	approxPose(t, "current", cur, Pose{Time: 400, X: 13, Y: 20, Variance: 2.3})

	wantPoses := map[int64]Pose{
		100: {Time: 100, X: 10, Y: 20, Variance: 2},
		200: {Time: 200, X: 11, Y: 20, Variance: 2.1},
		300: {Time: 300, X: 12, Y: 20, Variance: 2.2},
		400: {Time: 400, X: 13, Y: 20, Variance: 2.3},
	}
	for tm, w := range wantPoses {
		p, err := m.PoseAt(tm)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", tm, err)
		}
		approxPose(t, "pose at", p, w)
	}

	// 区域查询只包含有效出现：L 取第 2 次出现，已失效的第 1 次不出现。
	lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 2 || lms[0].ID != "L" || lms[1].ID != "M" {
		t.Fatalf("landmarks in rect = %v", lms)
	}
	approxLandmark(t, "active L", lms[0], Landmark{ID: "L", X: 13, Y: 20, Count: 2})
	approxLandmark(t, "active M", lms[1], Landmark{ID: "M", X: 13, Y: 21, Count: 1})

	// 两次出现的位置、计数与失效信息都保持真实结果。
	hist, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(hist.Appearances) != 2 {
		t.Fatalf("appearances = %+v", hist.Appearances)
	}
	ap1 := hist.Appearances[0]
	if ap1.Number != 1 || ap1.Active || !ap1.HasFirstSeen || ap1.FirstSeenTime != 100 ||
		ap1.InvalidTime != 200 || ap1.InvalidReason != "expire" || ap1.InvalidOpID != "inv1" {
		t.Fatalf("appearance 1 = %+v", ap1)
	}
	approxLandmark(t, "appearance 1", ap1.Landmark, Landmark{ID: "L", X: 11, Y: 20, Count: 2})
	ap2 := hist.Appearances[1]
	if ap2.Number != 2 || !ap2.Active || !ap2.HasFirstSeen || ap2.FirstSeenTime != 300 {
		t.Fatalf("appearance 2 = %+v", ap2)
	}
	approxLandmark(t, "appearance 2", ap2.Landmark, Landmark{ID: "L", X: 13, Y: 20, Count: 2})
}

// 成功校正（多帧位姿 + 同标识两次出现的路标变化）返回的记录被调用方改写
// 后，再次查询仍取得原始记录，地图与历史状态保持真实结果。
func TestCorrectionReturnedRecordIsolation(t *testing.T) {
	m, rec := isolationFixture(t)
	checkC1Record(t, rec)
	snap := snapshotRecord(rec)

	// 调用方改写标识、目标、时间信息、位姿与路标前后值、出现编号。
	tamperRecord(&rec)

	// 再次查询仍取得原始记录。
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 || !recordsEqual(recs[0], snap) {
		t.Fatalf("records after tampering = %+v, want %+v", recs, snap)
	}

	// 当前位姿、历史位姿、路标位置/次数/失效信息都不采用改写值。
	checkC1State(t, m)

	// 再取一份记录仍是原始内容。
	recs2, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs2) != 1 || !recordsEqual(recs2[0], snap) {
		t.Fatalf("records on refetch = %+v, want %+v", recs2, snap)
	}
}

// Corrections 返回的列表同样是独立副本：改写其中记录、调换或替换元素都不
// 影响再次查询的内容与次序；两份先后取得的列表互不影响。随后提交的合法
// 校正覆盖部分轨迹时，先前取得的列表保留获取时内容，已保存的第一条记录
// 仍说明第一次操作的前后变化，新记录反映第二次操作实际发生时的状态。
func TestCorrectionsListIsolation(t *testing.T) {
	m, rec := isolationFixture(t)
	snap := snapshotRecord(rec)

	// 改写列表中的记录并替换元素。
	listA, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(listA) != 1 {
		t.Fatalf("listA = %+v", listA)
	}
	tamperRecord(&listA[0])
	listA[0] = CorrectionRecord{ID: "forged"}

	// 再次查询不受 listA 影响。
	listB, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(listB) != 1 || !recordsEqual(listB[0], snap) {
		t.Fatalf("listB = %+v, want %+v", listB, snap)
	}

	// 先后取得的两份列表互不影响：改写 listB 后再查仍是原始内容。
	tamperRecord(&listB[0])
	listC, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(listC) != 1 || !recordsEqual(listC[0], snap) {
		t.Fatalf("listC = %+v, want %+v", listC, snap)
	}

	// 第二次合法校正：锚点 t=300，覆盖 c1 涉及轨迹的后半部分。
	r2, err := m.Correct(Correction{ID: "c2", Anchor: 300, Target: CorrectionTarget{X: 100, Y: 200, Heading: 0, Variance: 5}})
	if err != nil {
		t.Fatalf("correct c2: %v", err)
	}
	snap2 := snapshotRecord(r2)
	// 新记录反映第二次操作实际发生时的状态（以 c1 校正后的轨迹为起点）。
	if r2.Anchor != 300 || r2.EndTime != 400 || len(r2.Poses) != 2 {
		t.Fatalf("second record = %+v", r2)
	}
	approxPose(t, "c2 pose0 before", r2.Poses[0].Before, Pose{Time: 300, X: 12, Y: 20, Variance: 2.2})
	approxPose(t, "c2 pose0 after", r2.Poses[0].After, Pose{Time: 300, X: 100, Y: 200, Variance: 5})
	approxPose(t, "c2 pose1 before", r2.Poses[1].Before, Pose{Time: 400, X: 13, Y: 20, Variance: 2.3})
	approxPose(t, "c2 pose1 after", r2.Poses[1].After, Pose{Time: 400, X: 101, Y: 200, Variance: 5.1})
	if len(r2.Landmarks) != 2 || r2.Landmarks[0].ID != "L" || r2.Landmarks[0].Occurrence != 2 ||
		r2.Landmarks[1].ID != "M" || r2.Landmarks[1].Occurrence != 1 {
		t.Fatalf("c2 landmarks = %+v", r2.Landmarks)
	}
	approxLandmark(t, "c2 L before", r2.Landmarks[0].Before, Landmark{ID: "L", X: 13, Y: 20, Count: 2})
	approxLandmark(t, "c2 L after", r2.Landmarks[0].After, Landmark{ID: "L", X: 101, Y: 200, Count: 2})
	approxLandmark(t, "c2 M before", r2.Landmarks[1].Before, Landmark{ID: "M", X: 13, Y: 21, Count: 1})
	approxLandmark(t, "c2 M after", r2.Landmarks[1].After, Landmark{ID: "M", X: 101, Y: 201, Count: 1})

	// 第二次校正前取得的 listC 保留获取时的内容（仍只有 c1 一条）。
	if len(listC) != 1 || !recordsEqual(listC[0], snap) {
		t.Fatalf("previously fetched list changed = %+v, want %+v", listC, snap)
	}

	// 保存的记录：第一条仍说明第一次操作的前后变化，第二条为新记录。
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 2 || !recordsEqual(recs[0], snap) || !recordsEqual(recs[1], snap2) {
		t.Fatalf("records after c2 = %+v, want [%+v %+v]", recs, snap, snap2)
	}

	// 调换、替换、改写列表中的两条记录后，再次查询仍保留提交内容与次序。
	recs[0], recs[1] = recs[1], recs[0]
	tamperRecord(&recs[0])
	recs[1] = CorrectionRecord{ID: "forged", Poses: []PoseChange{{After: Pose{X: 1}}}}
	again, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(again) != 2 || !recordsEqual(again[0], snap) || !recordsEqual(again[1], snap2) {
		t.Fatalf("records after list tampering = %+v", again)
	}

	// 地图状态以第二次校正为准；c1 锚点帧不在 c2 范围内，保持 c1 结果。
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	approxPose(t, "current after c2", cur, Pose{Time: 400, X: 101, Y: 200, Variance: 5.1})
	p, err := m.PoseAt(100)
	if err != nil {
		t.Fatal(err)
	}
	approxPose(t, "c1 anchor after c2", p, Pose{Time: 100, X: 10, Y: 20, Variance: 2})
	lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 2 || lms[0].ID != "L" || lms[1].ID != "M" {
		t.Fatalf("landmarks after c2 = %v", lms)
	}
	approxLandmark(t, "L after c2", lms[0], Landmark{ID: "L", X: 101, Y: 200, Count: 2})
	approxLandmark(t, "M after c2", lms[1], Landmark{ID: "M", X: 101, Y: 201, Count: 1})
}

// 同一标识、同一原始请求再次提交：即使地图已接受后续校正，仍返回首次记录
// （不是最新轨迹），也不新增记录；该返回结果同样可被调用方独立改写而不污
// 染之后的查询与再次取得的首次结果。
func TestCorrectionDuplicateReturnIsolation(t *testing.T) {
	m, rec := isolationFixture(t)
	snap := snapshotRecord(rec)
	req := Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 2}}

	// 地图接受后续校正，覆盖 c1 涉及的部分轨迹。
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 300, Target: CorrectionTarget{X: 100, Y: 200, Heading: 0, Variance: 5}}); err != nil {
		t.Fatalf("correct c2: %v", err)
	}

	// 重复提交仍返回首次记录，不返回最新轨迹，也不新增记录。
	dup, err := m.Correct(req)
	if err != nil {
		t.Fatalf("duplicate correct: %v", err)
	}
	if !recordsEqual(dup, snap) {
		t.Fatalf("duplicate returned %+v, want first record %+v", dup, snap)
	}
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 2 || recs[0].ID != "c1" || recs[1].ID != "c2" {
		t.Fatalf("duplicate added a record: %+v", recs)
	}

	// 改写重复提交返回的记录。
	tamperRecord(&dup)

	// 之后的查询不被污染。
	recs, err = m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 2 || !recordsEqual(recs[0], snap) {
		t.Fatalf("records after tampering duplicate = %+v", recs)
	}

	// 再次取得的首次结果仍是原始内容。
	dup2, err := m.Correct(req)
	if err != nil {
		t.Fatalf("second duplicate correct: %v", err)
	}
	if !recordsEqual(dup2, snap) {
		t.Fatalf("second duplicate returned %+v, want %+v", dup2, snap)
	}

	// 当前状态仍是 c2 的真实结果。
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	approxPose(t, "current after duplicates", cur, Pose{Time: 400, X: 101, Y: 200, Variance: 5.1})
}

// 没有路标观测的合法校正同样保留位姿记录的隔离行为。
func TestCorrectionRecordIsolationNoLandmarks(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.1},
		{Time: 200, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Heading: 0, Variance: 1}})
	if err != nil {
		t.Fatal(err)
	}
	if len(rec.Poses) != 2 || len(rec.Landmarks) != 0 {
		t.Fatalf("record = %+v", rec)
	}
	snap := snapshotRecord(rec)

	tamperRecord(&rec)

	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 || !recordsEqual(recs[0], snap) {
		t.Fatalf("records after tampering = %+v, want %+v", recs, snap)
	}
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	approxPose(t, "current", cur, Pose{Time: 200, X: 6, Y: 6, Variance: 1.1})
	p, err := m.PoseAt(100)
	if err != nil {
		t.Fatal(err)
	}
	approxPose(t, "anchor", p, Pose{Time: 100, X: 5, Y: 6, Variance: 1})

	// 再取一份仍是原始内容。
	recs2, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs2) != 1 || !recordsEqual(recs2[0], snap) {
		t.Fatalf("records on refetch = %+v", recs2)
	}
}
