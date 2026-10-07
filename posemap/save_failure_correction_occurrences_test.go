package posemap

import (
	"path/filepath"
	"testing"
)

// 本文件为“已确认回环校正在落盘失败时整体回到提交前状态”补充一项回归
// 保障，专门覆盖校正跨越同一路标两次出现的情形：同一标识先在若干帧中被
// 观测并合并，失效后又在更晚的轨迹里建立新的一次出现；第一条出现已失效、
// 第二条仍有效，两条记录各自保留自己的观测。校正锚点落在第一次出现的观
// 测帧范围内，受影响帧同时包含两次出现的观测，两次出现的平均位置都会实
// 际改变，且重放结果满足既有合并距离规则——因此失败只能来自保存，不能
// 被误报成锚点不存在或路标冲突。锚点之前还有一份可查询的更早位姿，它在
// 整个过程中始终保持原值。
//
// 保存失败后：调用返回保存错误而非成功校正记录；当前位姿与受影响历史位
// 姿的位置、朝向、方差全部回到提交前；两次出现的位置、观测次数、编号、
// 首次观测时间、有效状态以及旧出现的失效时间、原因、操作标识都不变；区
// 域查询仍只返回提交前的有效出现（不能用失败校正后的坐标），已失效旧出
// 现仍被排除；此前已成功的校正记录按原顺序原样保留，既不改写也不追加本
// 次失败记录。
//
// 保存条件恢复后，同一地图对象以同一标识、同一内容再次提交，必须作为首
// 次成功提交处理：校正前后值取自失败前状态（而不是失败尝试曾暂存过的位
// 移再叠加一次），失败尝试不占用校正标识；成功记录分别对应路标的两次出
// 现、各自反映正确的位置变化，旧出现仍失效、新出现仍有效，观测次数不因
// 校正增加。
//
// 统一几何（初始位姿 t=0 (0,0)、朝向 0、初始方差 0.5、合并上限 5），全程
// 沿世界 X 轴直行、朝向始终为 0：
//
//	第一段 seg-1：
//	  t=100 DX=0 → (0,0)，方差 0.6：D 本地 (0,0) → (0,0)
//	  t=200 DX=1 → (1,0)，方差 0.7：D 本地 (0,0) → (1,0)
//	D 的第 1 次出现均值 (0.5,0)、计数 2、首次观测时间 100，随后在 t=200
//	被失效（原因 sign removed，操作标识 inv-d），旧出现冻结在 (0.5,0)。
//
//	第二段 seg-2：
//	  t=300 DX=1 → (2,0)，方差 0.9：D 本地 (0,0) → (2,0)（第 2 次出现首见）
//	  t=400 DX=1 → (3,0)，方差 1.1：D 本地 (0,0) → (3,0)
//	D 的第 2 次出现均值 (2.5,0)、计数 2、首次观测时间 300，仍有效。
//
//	先成功保存一条更早的校正 corr-earlier（锚点 t=100 平移到 (0,10)、
//	目标方差 0.3），它把两次出现整体平移到 (0.5,10) 与 (2.5,10)，使本次
//	回归所校验的失败校正之前确有一条按序保留的既有成功记录。
//
//	待校验校正 corr-loop：锚点仍取 t=100（第一次出现的观测范围内），目标
//	(20,30)、朝向 0、方差 1.0，相对提交前状态再整体平移 (+20,+20)。两次
//	出现分别重放为 (20.5,30) 与 (22.5,30)，同一次出现内相邻观测世界距离
//	恰为 1 ≤ 合并上限 5，不存在观测冲突；失败只可能来自落盘。

// saveFailureOccConfig 是这组场景的地图配置。
func saveFailureOccConfig() Config {
	return Config{InitialTime: 0, InitialVariance: 0.5, MaxInterval: 1000, MergeDistance: 5.0}
}

// seedSaveFailureOccMap 在 path 处创建地图并保存好“两段轨迹 + D 的第 1 次
// 出现已失效 + 一条更早的成功校正”的前置状态。返回时：
//
//	t=100 (0,10) v0.3，t=200 (1,10) v0.4，t=300 (2,10) v0.6，t=400 (3,10) v0.8；
//	D 第 1 次出现 (0.5,10) 计数 2 已失效（t=200, sign removed, inv-d），
//	D 第 2 次出现 (2.5,10) 计数 2 仍有效；
//	已保存校正记录恰为 corr-earlier 一条。
func seedSaveFailureOccMap(t *testing.T, path string) *Map {
	t.Helper()
	m, err := Create(path, saveFailureOccConfig())
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	if _, err := m.ImportSegment(Segment{ID: "seg-1", Frames: []Frame{
		{Time: 100, DX: 0, MoveVariance: 0.1, Observations: []Observation{{ID: "D", X: 0, Y: 0}}},
		{Time: 200, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "D", X: 0, Y: 0}}},
	}}); err != nil {
		t.Fatalf("seed import seg-1: %v", err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "inv-d", Reason: "sign removed", Landmarks: []string{"D"}}); err != nil {
		t.Fatalf("invalidate D: %v", err)
	}
	if _, err := m.ImportSegment(Segment{ID: "seg-2", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.2, Observations: []Observation{{ID: "D", X: 0, Y: 0}}},
		{Time: 400, DX: 1, MoveVariance: 0.2, Observations: []Observation{{ID: "D", X: 0, Y: 0}}},
	}}); err != nil {
		t.Fatalf("seed import seg-2: %v", err)
	}
	return m
}

// earlierOccurrenceCorrection 是失败校正之前已成功保存的那条校正。
func earlierOccurrenceCorrection() Correction {
	return Correction{ID: "corr-earlier", Anchor: 100,
		Target: CorrectionTarget{X: 0, Y: 10, Heading: 0, Variance: 0.3}}
}

// loopOccurrenceCorrection 是本场景待提交的跨两次出现校正：锚点 t=100
// 落在第一次出现的观测范围内，受影响帧覆盖两次出现的全部观测。
func loopOccurrenceCorrection() Correction {
	return Correction{ID: "corr-loop", Anchor: 100,
		Target: CorrectionTarget{X: 20, Y: 30, Heading: 0, Variance: 1.0}}
}

// assertPreLoopCorrectionState 断言地图完全处于“corr-earlier 已提交、
// corr-loop 尚未生效”的提交前状态；保存失败回滚后与参照地图都必须满足。
func assertPreLoopCorrectionState(t *testing.T, m *Map) {
	t.Helper()

	// 当前位姿与受影响历史位姿：位置、朝向、方差均为提交前值。
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatalf("CurrentPose: %v", err)
	}
	expectPose(t, "current pose", cur, Pose{Time: 400, X: 3, Y: 10, Heading: 0, Variance: 0.8})
	for _, want := range []Pose{
		{Time: 100, X: 0, Y: 10, Heading: 0, Variance: 0.3},
		{Time: 200, X: 1, Y: 10, Heading: 0, Variance: 0.4},
		{Time: 300, X: 2, Y: 10, Heading: 0, Variance: 0.6},
		{Time: 400, X: 3, Y: 10, Heading: 0, Variance: 0.8},
	} {
		p, err := m.PoseAt(want.Time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", want.Time, err)
		}
		expectPose(t, "affected historical pose", p, want)
	}

	// 锚点之前还可查询到初始位姿，这段更早历史始终保持原值（成功校正后
	// 同样不变）。
	p0, err := m.PoseAt(0)
	if err != nil {
		t.Fatalf("PoseAt(0): %v", err)
	}
	expectPose(t, "earlier pre-anchor pose", p0, Pose{Time: 0, X: 0, Y: 0, Heading: 0, Variance: 0.5})

	// 区域查询：只返回提交前仍有效的第 2 次出现，且是原坐标与原计数；
	// 失败校正后的坐标一处都查不到，已失效旧出现仍被排除。
	lms, err := m.LandmarksInRect(allRect())
	if err != nil {
		t.Fatalf("LandmarksInRect: %v", err)
	}
	if len(lms) != 1 || lms[0].ID != "D" {
		t.Fatalf("active landmarks = %v, want only D occurrence 2", lms)
	}
	if lms[0].Count != 2 {
		t.Fatalf("D count = %d, want unchanged 2", lms[0].Count)
	}
	approxEq(t, "active D x", lms[0].X, 2.5)
	approxEq(t, "active D y", lms[0].Y, 10)
	failedArea, err := m.LandmarksInRect(Rect{MinX: 20, MinY: 29, MaxX: 23, MaxY: 31})
	if err != nil {
		t.Fatalf("LandmarksInRect failed-correction area: %v", err)
	}
	if len(failedArea) != 0 {
		t.Fatalf("failed-correction coordinates queryable: %v", failedArea)
	}

	// 两次出现各自保留位置、次数、编号、首次观测时间与有效状态；旧出现的
	// 失效时间、原因与操作标识也原样保留。
	h, err := m.LandmarkAppearances("D")
	if err != nil {
		t.Fatalf("LandmarkAppearances(D): %v", err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("D appearances = %+v, want exactly 2", h.Appearances)
	}
	expectAppearance(t, "D occ1", h.Appearances[0], Appearance{
		Number: 1, Active: false, FirstSeenTime: 100, HasFirstSeen: true,
		Landmark:      Landmark{ID: "D", X: 0.5, Y: 10, Count: 2},
		InvalidTime:   200,
		InvalidReason: "sign removed",
		InvalidOpID:   "inv-d",
	})
	expectAppearance(t, "D occ2", h.Appearances[1], Appearance{
		Number: 2, Active: true, FirstSeenTime: 300, HasFirstSeen: true,
		Landmark: Landmark{ID: "D", X: 2.5, Y: 10, Count: 2},
	})
}

// TestCorrectSaveFailureRollsBackBothOccurrences 验证：一次计算本可成功、
// 跨越同一路标已失效旧出现与仍有效新出现的回环校正，仅因地图文件无法更新
// 而保存失败时，不返回成功记录、不报锚点/冲突类拒绝，位姿、两次出现、
// 区域查询与既有校正记录全部回到提交前；保存恢复后同一标识同一内容作为
// 首次成功提交，前后值取自失败前状态，结果与该内容从未经历失败直接成功
// 完全一致。
func TestCorrectSaveFailureRollsBackBothOccurrences(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m := seedSaveFailureOccMap(t, path)
	t.Cleanup(func() { _ = m.Close() })

	// 先提交一条更早的成功校正，作为“按序保留、不被改写”的既有记录。
	recEarlier, err := m.Correct(earlierOccurrenceCorrection())
	if err != nil {
		t.Fatalf("earlier correction: %v", err)
	}
	wantEarlier := snapshotRecord(recEarlier)
	if len(recEarlier.Poses) != 4 || len(recEarlier.Landmarks) != 2 {
		t.Fatalf("earlier record = %+v, want 4 poses and 2 occurrence changes", recEarlier)
	}

	// ---- 制造保存失败并提交本应成功的跨出现校正 ----
	breakSavingAtPath(t, path)
	req := loopOccurrenceCorrection()
	rec, err := m.Correct(req)
	if err == nil {
		t.Fatal("correction with unupdatable map file unexpectedly succeeded")
	}
	// 必须是保存错误：不能是 *RejectError（不能把合法校正误报成
	// anchor_not_found 或 landmark_conflict），也不能返回成功校正记录。
	if r, ok := AsRejectError(err); ok {
		t.Fatalf("valid correction rejected as %s: %v", r.Kind, err)
	}
	if rec.ID != "" || len(rec.Poses) != 0 || len(rec.Landmarks) != 0 {
		t.Fatalf("failed correction returned record %+v, want zero record", rec)
	}

	// ---- 失败后：地图完整保持提交前状态 ----
	assertPreLoopCorrectionState(t, m)

	// 既有成功校正记录按原顺序原样保留，本次失败不追加任何记录。
	recs, err := m.Corrections()
	if err != nil {
		t.Fatalf("Corrections after failure: %v", err)
	}
	checkRecords(t, recs, wantEarlier)

	// ---- 恢复保存条件，同一地图对象以同一标识、同一内容再次提交 ----
	restoreSavingAtPath(t, m, path)
	rec, err = m.Correct(req)
	if err != nil {
		t.Fatalf("retry same correction id after save restored: %v", err)
	}

	// 作为首次成功提交：前后值取自失败前状态（不是失败暂存位移的叠加）。
	if rec.ID != "corr-loop" || rec.Anchor != 100 || rec.EndTime != 400 {
		t.Fatalf("retried record header = %+v", rec)
	}
	wantPosesBefore := []Pose{
		{Time: 100, X: 0, Y: 10, Heading: 0, Variance: 0.3},
		{Time: 200, X: 1, Y: 10, Heading: 0, Variance: 0.4},
		{Time: 300, X: 2, Y: 10, Heading: 0, Variance: 0.6},
		{Time: 400, X: 3, Y: 10, Heading: 0, Variance: 0.8},
	}
	wantPosesAfter := []Pose{
		{Time: 100, X: 20, Y: 30, Heading: 0, Variance: 1.0},
		{Time: 200, X: 21, Y: 30, Heading: 0, Variance: 1.1},
		{Time: 300, X: 22, Y: 30, Heading: 0, Variance: 1.3},
		{Time: 400, X: 23, Y: 30, Heading: 0, Variance: 1.5},
	}
	if len(rec.Poses) != len(wantPosesBefore) {
		t.Fatalf("retried pose changes = %+v, want %d", rec.Poses, len(wantPosesBefore))
	}
	for i := range wantPosesBefore {
		expectPoseChange(t, i, rec.Poses[i], wantPosesBefore[i], wantPosesAfter[i])
	}
	// 成功记录分别对应路标的两次出现：观测次数不变，旧出现仍以提交前坐标
	// 为 Before，失败尝试暂存的位移没有被二次叠加。
	wantLandmarkChanges := []LandmarkChange{
		{ID: "D", Occurrence: 1,
			Before: Landmark{ID: "D", X: 0.5, Y: 10, Count: 2},
			After:  Landmark{ID: "D", X: 20.5, Y: 30, Count: 2}},
		{ID: "D", Occurrence: 2,
			Before: Landmark{ID: "D", X: 2.5, Y: 10, Count: 2},
			After:  Landmark{ID: "D", X: 22.5, Y: 30, Count: 2}},
	}
	if len(rec.Landmarks) != len(wantLandmarkChanges) {
		t.Fatalf("retried landmark changes = %+v, want %d", rec.Landmarks, len(wantLandmarkChanges))
	}
	for i, want := range wantLandmarkChanges {
		if got := rec.Landmarks[i]; got != want {
			t.Fatalf("retried landmark change %d = %+v, want %+v", i, got, want)
		}
	}

	// 两条成功记录按提交次序保留：既有记录内容未被改写，本次记录追加其后。
	recs, _ = m.Corrections()
	checkRecords(t, recs, wantEarlier, snapshotRecord(rec))

	// 成功后的地图状态：旧出现仍失效且失效信息不变，新出现仍有效；两次
	// 出现的观测计数都不因校正增加；更早位姿仍保持原值。
	cur, _ := m.CurrentPose()
	expectPose(t, "current after retry", cur, Pose{Time: 400, X: 23, Y: 30, Heading: 0, Variance: 1.5})
	p0, _ := m.PoseAt(0)
	expectPose(t, "pre-anchor pose after retry", p0, Pose{Time: 0, X: 0, Y: 0, Heading: 0, Variance: 0.5})
	h, _ := m.LandmarkAppearances("D")
	if len(h.Appearances) != 2 {
		t.Fatalf("D appearances after retry = %+v, want 2", h.Appearances)
	}
	expectAppearance(t, "D occ1 after retry", h.Appearances[0], Appearance{
		Number: 1, Active: false, FirstSeenTime: 100, HasFirstSeen: true,
		Landmark:      Landmark{ID: "D", X: 20.5, Y: 30, Count: 2},
		InvalidTime:   200,
		InvalidReason: "sign removed",
		InvalidOpID:   "inv-d",
	})
	expectAppearance(t, "D occ2 after retry", h.Appearances[1], Appearance{
		Number: 2, Active: true, FirstSeenTime: 300, HasFirstSeen: true,
		Landmark: Landmark{ID: "D", X: 22.5, Y: 30, Count: 2},
	})
	lms, _ := m.LandmarksInRect(allRect())
	if len(lms) != 1 || lms[0].ID != "D" || lms[0].Count != 2 {
		t.Fatalf("active landmarks after retry = %v, want only D occ2 count 2", lms)
	}
	approxEq(t, "active D x after retry", lms[0].X, 22.5)
	approxEq(t, "active D y after retry", lms[0].Y, 30)

	// 同一标识同一内容再次提交，按已保存的首次成功记录返回，不重复追加、
	// 不再叠加位移。
	dup, err := m.Correct(req)
	if err != nil {
		t.Fatalf("duplicate correction after retry: %v", err)
	}
	if !recordsEqual(dup, rec) {
		t.Fatalf("duplicate correction = %+v, want first success %+v", dup, rec)
	}
	if recs, _ := m.Corrections(); len(recs) != 2 {
		t.Fatalf("duplicate correction appended a record: %d", len(recs))
	}

	// ---- 与“同内容直接成功、从未经历失败”的参照地图逐值比对 ----
	refPath := filepath.Join(t.TempDir(), "ref.pose")
	ref := seedSaveFailureOccMap(t, refPath)
	t.Cleanup(func() { _ = ref.Close() })
	refEarlier, err := ref.Correct(earlierOccurrenceCorrection())
	if err != nil {
		t.Fatalf("reference earlier correction: %v", err)
	}
	refRec, err := ref.Correct(loopOccurrenceCorrection())
	if err != nil {
		t.Fatalf("reference loop correction: %v", err)
	}
	if !recordsEqual(refEarlier, wantEarlier) {
		t.Fatalf("earlier record parity mismatch:\n got %+v\nwant %+v", refEarlier, wantEarlier)
	}
	if !recordsEqual(refRec, rec) {
		t.Fatalf("loop record parity mismatch:\n got %+v\nwant %+v", refRec, rec)
	}
	for _, at := range []int64{0, 100, 200, 300, 400} {
		got, gerr := m.PoseAt(at)
		want, werr := ref.PoseAt(at)
		if gerr != nil || werr != nil {
			t.Fatalf("PoseAt(%d) errors: %v %v", at, gerr, werr)
		}
		expectPose(t, "parity pose vs reference", got, want)
	}
	refH, _ := ref.LandmarkAppearances("D")
	if len(refH.Appearances) != len(h.Appearances) {
		t.Fatalf("appearance count parity = %d, want %d", len(h.Appearances), len(refH.Appearances))
	}
	for i := range h.Appearances {
		expectAppearance(t, "parity D appearance", h.Appearances[i], refH.Appearances[i])
	}
	refLms, _ := ref.LandmarksInRect(allRect())
	if len(refLms) != len(lms) {
		t.Fatalf("rect counts differ: %v vs %v", lms, refLms)
	}
	for _, want := range refLms {
		got, ok := findLandmark(lms, want.ID)
		if !ok || got.Count != want.Count {
			t.Fatalf("rect %s = %+v ok=%t, want count %d", want.ID, got, ok, want.Count)
		}
		approxEq(t, "parity rect x", got.X, want.X)
		approxEq(t, "parity rect y", got.Y, want.Y)
	}

	// ---- 关闭重开：落盘内容只有两次成功校正，失败尝试不留痕迹 ----
	if err := m.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}
	reopened, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	defer reopened.Close()
	reRecs, err := reopened.Corrections()
	if err != nil {
		t.Fatalf("Corrections after reopen: %v", err)
	}
	if len(reRecs) != 2 || reRecs[0].ID != "corr-earlier" || reRecs[1].ID != "corr-loop" {
		t.Fatalf("corrections after reopen = %+v", reRecs)
	}
	if !recordsEqual(reRecs[1], rec) {
		t.Fatalf("reopened loop record = %+v, want %+v", reRecs[1], rec)
	}
	reH, _ := reopened.LandmarkAppearances("D")
	if len(reH.Appearances) != 2 ||
		reH.Appearances[0].Active || reH.Appearances[0].InvalidReason != "sign removed" ||
		!reH.Appearances[1].Active || reH.Appearances[1].Landmark.Count != 2 {
		t.Fatalf("D appearances after reopen = %+v", reH.Appearances)
	}
	approxEq(t, "reopened occ1 x", reH.Appearances[0].Landmark.X, 20.5)
	approxEq(t, "reopened occ2 x", reH.Appearances[1].Landmark.X, 22.5)
	reCur, _ := reopened.CurrentPose()
	expectPose(t, "reopened current pose", reCur, Pose{Time: 400, X: 23, Y: 30, Heading: 0, Variance: 1.5})
}
