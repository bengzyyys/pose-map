package posemap

// 本文件围绕 LandmarksInRect 的既有公开行为补充回归保障，重点是
// “提交回环校正后，区域查询是否落在矩形内必须按校正后的当前有效记录
// 最新位置判断”，避免出现坐标已经更新、区域命中却仍按旧位置判断的
// 偏差。只使用现有公开入口（Create/ImportSegment/Correct/Invalidate/
// LandmarksInRect/LandmarkAppearances/Open/Close），不改变任何公开
// 接口、校正规则与查询含义。
//
// 判定依据始终是该次出现全部已接受观测（含锚点之前保持原位的固定
// 观测与锚点及之后随校正移动的观测）的等权平均位置，而不是某一帧的
// 位姿、最后一条观测位置或校正目标位置。

import (
	"math"
	"path/filepath"
	"testing"
)

// crossAnchorSegment 构造一个同一次出现横跨校正锚点的轨迹：
//
//	初始位姿 (0,0)，朝向恒为 0，全部沿世界 X 轴。
//	t=50  位姿 (0,0)，观测 L 本地 (0,0) → 地图 (0,0)   （锚点之前，校正后固定）
//	t=100 位姿 (1,0)，观测 L 本地 (0,0) → 地图 (1,0)   （锚点帧）
//	t=200 位姿 (2,0)，观测 L 本地 (0,0) → 地图 (2,0)   （锚点之后）
//
// L 第 1 次出现为 3 条观测，均值 (1,0)，次数 3。校正锚点取 t=100、
// 只做沿 X 轴的平移（朝向不变）时：t=50 观测固定在 (0,0)，t=100、
// t=200 两条观测随轨迹整体平移 d，校正后均值为 ((0)+(1+d)+(2+d))/3
// = 1 + 2d/3，而不是锚点/末帧位置或校正目标位置。
func crossAnchorSegment() Segment {
	return Segment{
		ID: "s1",
		Frames: []Frame{
			{Time: 50, DX: 0, MoveVariance: 0.1, Observations: []Observation{{ID: "L"}}},
			{Time: 100, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "L"}}},
			{Time: 200, DX: 1, MoveVariance: 0.1, Observations: []Observation{{ID: "L"}}},
		},
	}
}

// mustQueryRect 执行矩形查询并在出错时终止测试。
func mustQueryRect(t *testing.T, m *Map, r Rect) []Landmark {
	t.Helper()
	lms, err := m.LandmarksInRect(r)
	if err != nil {
		t.Fatalf("LandmarksInRect(%+v): %v", r, err)
	}
	return lms
}

// findLandmarkOrFail 在查询结果中找出指定标识；不存在即失败。
func findLandmarkOrFail(t *testing.T, lms []Landmark, id string) Landmark {
	t.Helper()
	lm, ok := findLandmark(lms, id)
	if !ok {
		t.Fatalf("landmark %q missing from results %v", id, lms)
	}
	return lm
}

// assertAbsent 断言指定标识不在查询结果中。
func assertAbsent(t *testing.T, lms []Landmark, id string) {
	t.Helper()
	if _, ok := findLandmark(lms, id); ok {
		t.Fatalf("landmark %q unexpectedly present in results %v", id, lms)
	}
}

// assertEmptyNonNull 断言查询返回非 nil 的空切片（无命中时的既有约定）。
func assertEmptyNonNull(t *testing.T, lms []Landmark) {
	t.Helper()
	if lms == nil {
		t.Fatal("result is nil, want non-nil empty slice")
	}
	if len(lms) != 0 {
		t.Fatalf("result = %v, want empty", lms)
	}
}

// assertSortedByID 断言结果严格按标识升序。
func assertSortedByID(t *testing.T, lms []Landmark) {
	t.Helper()
	for i := 1; i < len(lms); i++ {
		if lms[i-1].ID >= lms[i].ID {
			t.Fatalf("results not sorted by id at %d: %v", i, lms)
		}
	}
}

// assertAppearanceMatches 断言区域查询返回的某路标与其当前有效出现
// 记录中的最新位置、次数完全一致。
func assertAppearanceMatches(t *testing.T, m *Map, lm Landmark) {
	t.Helper()
	h, err := m.LandmarkAppearances(lm.ID)
	if err != nil {
		t.Fatalf("LandmarkAppearances(%q): %v", lm.ID, err)
	}
	var active *Appearance
	for i := range h.Appearances {
		if h.Appearances[i].Active {
			active = &h.Appearances[i]
		}
	}
	if active == nil {
		t.Fatalf("landmark %q has no active appearance", lm.ID)
	}
	approxEq(t, lm.ID+" rect x vs active x", lm.X, active.Landmark.X)
	approxEq(t, lm.ID+" rect y vs active y", lm.Y, active.Landmark.Y)
	if lm.Count != active.Landmark.Count {
		t.Fatalf("%s rect count = %d, active occurrence count = %d", lm.ID, lm.Count, active.Landmark.Count)
	}
}

// TestRectAfterCorrectionMoveOut 是核心回归：路标校正前落在查询矩形内、
// 校正后到了矩形外，原矩形必须立即查不到它；包含新位置的矩形必须返回
// 它的标识、新位置与原观测次数。判定位置是全部已接受观测的平均位置
// （锚点之前的固定观测与锚点及之后随校正移动的观测共同决定），不能把
// 整个路标挪到校正目标附近。
func TestRectAfterCorrectionMoveOut(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 5.0}
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(crossAnchorSegment()); err != nil {
		t.Fatal(err)
	}

	// 校正前：均值 (1,0)、次数 3，落在以 (1,0) 为中心的原矩形内。
	oldRect := Rect{MinX: 0.5, MinY: -0.5, MaxX: 1.5, MaxY: 0.5}
	before := mustQueryRect(t, m, oldRect)
	lm := findLandmarkOrFail(t, before, "L")
	approxEq(t, "before x", lm.X, 1)
	approxEq(t, "before y", lm.Y, 0)
	if lm.Count != 3 {
		t.Fatalf("before count = %d, want 3", lm.Count)
	}

	// 锚点 t=100 从 (1,0) 平移到 (4,0)（平移 d=3，朝向不变）。
	// 三条观测重放为 (0,0)、(4,0)、(5,0)（相邻间距 4、1 均不超过
	// 合并距离 5），均值 3 = 1 + 2*3/3，次数仍为 3。
	// 注意均值既不是锚点目标 (4,0)，也不是末条观测 (5,0)：锚点之前
	// 的固定观测把均值拉在 (3,0)。
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 4, Y: 0, Variance: 0}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	lc := rec.Landmarks[0]
	if lc.ID != "L" || lc.Occurrence != 1 {
		t.Fatalf("landmark change = %+v", lc)
	}
	approxEq(t, "record after x", lc.After.X, 3)
	if lc.After.Count != 3 {
		t.Fatalf("record after count = %d, want 3", lc.After.Count)
	}

	// 原矩形立即查不到 L（坐标已到 x=3，不能仍按旧位置 x=1 命中）。
	assertEmptyNonNull(t, mustQueryRect(t, m, oldRect))

	// 包含新位置但不包含旧位置的矩形：返回标识、新位置与原观测次数。
	newRect := Rect{MinX: 2.5, MinY: -0.5, MaxX: 3.5, MaxY: 0.5}
	after := mustQueryRect(t, m, newRect)
	lm = findLandmarkOrFail(t, after, "L")
	approxEq(t, "after x", lm.X, 3)
	approxEq(t, "after y", lm.Y, 0)
	if lm.Count != 3 {
		t.Fatalf("after count = %d, want 3", lm.Count)
	}
	assertAppearanceMatches(t, m, lm)

	// 覆盖两个位置的大矩形仍只有这一个路标、同一次数。
	big := mustQueryRect(t, m, Rect{MinX: -1, MinY: -1, MaxX: 6, MaxY: 1})
	if len(big) != 1 || big[0].ID != "L" || big[0].Count != 3 {
		t.Fatalf("big rect = %v", big)
	}
}

// TestRectAfterCorrectionMoveIn 是相反方向的回归：路标校正前在矩形外、
// 校正后移入矩形，原矩形从空结果变为命中，新位置同样由固定观测与移动
// 观测共同决定。
func TestRectAfterCorrectionMoveIn(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 5.0}
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(crossAnchorSegment()); err != nil {
		t.Fatal(err)
	}

	// 新位置附近的矩形：校正前均值 (1,0) 在其外。
	rect := Rect{MinX: 2.5, MinY: -0.5, MaxX: 3.5, MaxY: 0.5}
	assertEmptyNonNull(t, mustQueryRect(t, m, rect))

	// 同一平移 d=3：均值移到 (3,0)。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 4, Y: 0}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}
	lms := mustQueryRect(t, m, rect)
	lm := findLandmarkOrFail(t, lms, "L")
	approxEq(t, "moved-in x", lm.X, 3)
	if lm.Count != 3 {
		t.Fatalf("moved-in count = %d, want 3", lm.Count)
	}
	assertAppearanceMatches(t, m, lm)
}

// TestRectMeanUsesAllAcceptedObservations 明确防住三种“用错位置”的
// 退化：区域命中既不能按某一帧（锚点/末帧）的位姿判断，也不能按最后
// 一条观测位置或校正目标位置判断；必须按全部已接受观测的平均位置。
// 平移 d=3 后：锚点位姿 (4,0)、末帧位姿与末条观测 (5,0)、校正目标
// (4,0)，而平均位置是 (3,0)。
func TestRectMeanUsesAllAcceptedObservations(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 5.0}
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(crossAnchorSegment()); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 4, Y: 0}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}

	// 只覆盖平均位置 (3,0) 的窄矩形：必须命中。
	atMean := Rect{MinX: 3, MinY: -0.5, MaxX: 3, MaxY: 0.5}
	lms := mustQueryRect(t, m, atMean)
	if len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("rect at mean = %v, want L", lms)
	}
	approxEq(t, "mean x", lms[0].X, 3)

	// 覆盖锚点/目标 (4,0) 但不含均值 (3,0) 的矩形：按均值必须为空，
	// 即使锚点帧位姿与校正目标都落在里面。
	atAnchor := Rect{MinX: 3.6, MinY: -0.5, MaxX: 4.4, MaxY: 0.5}
	assertEmptyNonNull(t, mustQueryRect(t, m, atAnchor))

	// 覆盖末帧位姿/末条观测 (5,0) 但不含均值的矩形：同样必须为空。
	atLast := Rect{MinX: 4.6, MinY: -0.5, MaxX: 5.4, MaxY: 0.5}
	assertEmptyNonNull(t, mustQueryRect(t, m, atLast))
}

// TestRectBoundaryInclusiveAfterCorrection 回归边界语义：校正后的平均
// 位置恰好落在矩形边或角上时仍应返回（含边界）；刚越过边界则排除。
func TestRectBoundaryInclusiveAfterCorrection(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 5.0}
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(crossAnchorSegment()); err != nil {
		t.Fatal(err)
	}
	// d=3 → 均值 (3,0)。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 4, Y: 0}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}

	// 均值恰在矩形中心时命中（对照）。
	center := Rect{MinX: 2, MinY: -1, MaxX: 4, MaxY: 1}
	if len(mustQueryRect(t, m, center)) != 1 {
		t.Fatal("center rect should contain L")
	}
	// 均值恰落在左边 (MinX=3) 与下边 (MinY=0)，即矩形左下角：仍返回。
	corner := Rect{MinX: 3, MinY: 0, MaxX: 4, MaxY: 1}
	lms := mustQueryRect(t, m, corner)
	if len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("corner rect = %v, want L on corner", lms)
	}
	// 均值恰落在右边 (MaxX=3) 与上边 (MaxY=0)，即矩形右上角：仍返回。
	corner2 := Rect{MinX: 2, MinY: -1, MaxX: 3, MaxY: 0}
	if len(mustQueryRect(t, m, corner2)) != 1 {
		t.Fatal("top-right corner rect should contain L")
	}
	// 刚越过左边界一个最小可表示量：排除。
	justOut := corner
	justOut.MinX = math.Nextafter(3, 4)
	assertEmptyNonNull(t, mustQueryRect(t, m, justOut))
	// 刚越过上边界：排除。
	justOut2 := corner2
	justOut2.MaxY = math.Nextafter(0, -1)
	assertEmptyNonNull(t, mustQueryRect(t, m, justOut2))
}

// TestRectOnlyActiveOccurrenceAfterCorrection 回归“失效旧出现 + 有效新
// 出现”与校正、区域查询三者的组合：区域查询只考虑当前有效的新出现；
// 即使旧出现的位置落在矩形里，或者校正范围同时涉及旧出现与新出现，
// 查询也不能把旧记录重新显示出来；两次出现都落在同一矩形内时，同一
// 标识仍只返回一次，次数取新出现自己的计数。
func TestRectOnlyActiveOccurrenceAfterCorrection(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 5.0}
	m := newMap(t, cfg)

	// occ1：t=50 观测 → (0,0)，t=100 观测 → (1,0)，均值 (0.5,0) 次数 2。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "L"}}},
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "iv1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	// occ2（当前有效）：t=200 位姿 (2,0) 单条观测 → (2,0)，次数 1。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 校正锚点 t=100、平移 d=3：范围 [100,200] 同时涉及 occ1（t=100
	// 一条，t=50 固定）与 occ2（t=200 一条）。
	//   occ1：(0,0) 固定 + (4,0) 移动 → 均值 (2,0) 次数 2（仍失效）；
	//   occ2：(5,0) 移动 → 均值 (5,0) 次数 1（仍有效）。
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 4, Y: 0}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if len(rec.Landmarks) != 2 ||
		rec.Landmarks[0].Occurrence != 1 || rec.Landmarks[1].Occurrence != 2 {
		t.Fatalf("landmark changes = %+v", rec.Landmarks)
	}

	// 覆盖 occ1 校正后位置 (2,0) 但不覆盖 occ2 位置 (5,0) 的矩形：
	// 旧出现已失效，必须为空——校正不能把旧记录重新显示出来。
	oldRect := Rect{MinX: 1.5, MinY: -0.5, MaxX: 2.5, MaxY: 0.5}
	assertEmptyNonNull(t, mustQueryRect(t, m, oldRect))

	// 覆盖 occ2 新位置 (5,0) 的矩形：返回一次 L，次数取新出现自己的 1。
	newRect := Rect{MinX: 4.5, MinY: -0.5, MaxX: 5.5, MaxY: 0.5}
	lms := mustQueryRect(t, m, newRect)
	if len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("new rect = %v, want only active occurrence", lms)
	}
	approxEq(t, "active x", lms[0].X, 5)
	if lms[0].Count != 1 {
		t.Fatalf("active count = %d, want new occurrence's own count 1", lms[0].Count)
	}
	assertAppearanceMatches(t, m, lms[0])

	// 两次出现的位置都落在大矩形内：同一标识仍只返回一次。
	big := Rect{MinX: -1, MinY: -1, MaxX: 6, MaxY: 1}
	both := mustQueryRect(t, m, big)
	if len(both) != 1 || both[0].ID != "L" {
		t.Fatalf("big rect = %v, want L exactly once", both)
	}
	if both[0].Count != 1 {
		t.Fatalf("big rect count = %d, want active occurrence count 1", both[0].Count)
	}
	approxEq(t, "big rect x is active", both[0].X, 5)

	// 旧出现历史仍可查、保持失效，位置随校正更新但不参与区域查询。
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	if h.Appearances[0].Active {
		t.Fatal("old occurrence reactivated by correction")
	}
	approxEq(t, "old occurrence x", h.Appearances[0].Landmark.X, 2)
	if h.Appearances[0].Landmark.Count != 2 {
		t.Fatalf("old occurrence count = %d, want 2", h.Appearances[0].Landmark.Count)
	}
	if !h.Appearances[1].Active || h.Appearances[1].Number != 2 {
		t.Fatalf("new occurrence = %+v", h.Appearances[1])
	}
}

// TestRectConflictRejectionKeepsPreCommitResults 回归原子性：某次校正
// 使同一次出现中的观测超过合并距离时，按现有路标冲突原因整次拒绝；
// 拒绝后查询原区域与拟移动到的区域，都必须得到提交前的结果，不能留下
// 部分路标已经移入或移出的状态。
func TestRectConflictRejectionKeepsPreCommitResults(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 5.0}
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(crossAnchorSegment()); err != nil {
		t.Fatal(err)
	}
	// 均值 (1,0) 次数 3；再加一个始终不受失败校正影响的参照路标 R：
	// t=300 位姿 (2,0)，本地 (1,1) → 地图 (3,1)，用来确认整次拒绝不
	// 影响其他路标。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 0, Observations: []Observation{{ID: "R", X: 1, Y: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}

	oldRect := Rect{MinX: 0.5, MinY: -0.5, MaxX: 1.5, MaxY: 0.5}
	newRect := Rect{MinX: 8.5, MinY: -0.5, MaxX: 9.5, MaxY: 0.5}
	beforeOld := mustQueryRect(t, m, oldRect)
	if len(beforeOld) != 1 || beforeOld[0].ID != "L" || beforeOld[0].Count != 3 {
		t.Fatalf("pre-commit old rect = %v", beforeOld)
	}
	assertEmptyNonNull(t, mustQueryRect(t, m, newRect))

	// 锚点 t=100 平移 d=9：t=50 观测固定 (0,0)，t=100 观测移到
	// (10,0)，二者相距 10 > 合并距离 5 → occ1 冲突，整次拒绝。
	_, err := m.Correct(Correction{ID: "cbad", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 0}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectLandmarkConflict || r.Landmark != "L" ||
		!r.HasOccurrence || r.Occurrence != 1 {
		t.Fatalf("err = %v, want landmark_conflict for L occurrence 1", err)
	}

	// 原区域仍是提交前结果：L 还在、位置 (1,0)、次数 3。
	afterOld := mustQueryRect(t, m, oldRect)
	if len(afterOld) != 1 {
		t.Fatalf("old rect after rejected correction = %v, want L still inside", afterOld)
	}
	lm := findLandmarkOrFail(t, afterOld, "L")
	approxEq(t, "L stays at old x", lm.X, 1)
	if lm.Count != 3 {
		t.Fatalf("L count = %d, want 3", lm.Count)
	}
	// 拟移动到的区域仍为空：没有部分移入。
	assertEmptyNonNull(t, mustQueryRect(t, m, newRect))
	// 参照路标不受影响（地图位置 (3,1)）。
	rRect := Rect{MinX: 2.5, MinY: 0.5, MaxX: 3.5, MaxY: 1.5}
	rLM := mustQueryRect(t, m, rRect)
	if len(rLM) != 1 || rLM[0].ID != "R" {
		t.Fatalf("reference landmark changed: %v", rLM)
	}
	// 没有留下校正记录。
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("correction record leaked: %v", recs)
	}
	// 全部有效路标仍是提交前的两个、各自计数不变。
	all := mustQueryRect(t, m, allRect())
	if len(all) != 2 {
		t.Fatalf("all landmarks = %v, want L and R", all)
	}
	assertSortedByID(t, all)

	// 拒绝后再提交一个合法校正（d=3，均值到 3）仍正常生效，区域命中
	// 立即按新位置切换，证明地图未被失败的提交污染。
	if _, err := m.Correct(Correction{ID: "cgood", Anchor: 100, Target: CorrectionTarget{X: 4, Y: 0}}); err != nil {
		t.Fatalf("valid correction after rejected one: %v", err)
	}
	assertEmptyNonNull(t, mustQueryRect(t, m, oldRect))
	moved := mustQueryRect(t, m, Rect{MinX: 2.5, MinY: -0.5, MaxX: 3.5, MaxY: 0.5})
	if len(moved) != 1 || moved[0].ID != "L" {
		t.Fatalf("L not found at new mean after valid correction: %v", moved)
	}
	approxEq(t, "moved x", moved[0].X, 3)
}

// TestRectMultipleHitsSortedAndNonNullEmpty 回归多命中与空结果约定：
// 校正把多个路标移动后，命中多个时继续按标识排序；没有有效路标命中时
// 继续返回非 nil 的空结果。
func TestRectMultipleHitsSortedAndNonNullEmpty(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 5.0}
	m := newMap(t, cfg)
	// 三个路标在不同帧、不同本地位置被观测一次；朝向恒 0。
	//   b: t=50  位姿 (0,0)，本地 (0,0) → (0,0)
	//   a: t=100 位姿 (1,0)，本地 (0,0) → (1,0)
	//   c: t=200 位姿 (2,0)，本地 (0,0) → (2,0)
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "b"}}},
		{Time: 100, DX: 1, Observations: []Observation{{ID: "a"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "c"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// 锚点 t=100 平移 d=3：b 固定 (0,0)，a → (4,0)，c → (5,0)。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 4, Y: 0}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}

	// 只覆盖移动后的 a、c（不含固定的 b）：按标识排序返回。
	rect := Rect{MinX: 3.5, MinY: -0.5, MaxX: 5.5, MaxY: 0.5}
	lms := mustQueryRect(t, m, rect)
	if len(lms) != 2 || lms[0].ID != "a" || lms[1].ID != "c" {
		t.Fatalf("moved hits = %v, want [a c] sorted", lms)
	}
	if lms[0].Count != 1 || lms[1].Count != 1 {
		t.Fatalf("counts = %d %d, want 1 1", lms[0].Count, lms[1].Count)
	}
	approxEq(t, "a x", lms[0].X, 4)
	approxEq(t, "c x", lms[1].X, 5)
	assertAppearanceMatches(t, m, lms[0])
	assertAppearanceMatches(t, m, lms[1])

	// 覆盖固定的 b：只有 b。
	bRect := Rect{MinX: -0.5, MinY: -0.5, MaxX: 0.5, MaxY: 0.5}
	bOnly := mustQueryRect(t, m, bRect)
	if len(bOnly) != 1 || bOnly[0].ID != "b" {
		t.Fatalf("fixed b rect = %v", bOnly)
	}

	// 三个都不命中：非 nil 空结果。
	empty := Rect{MinX: 10, MinY: 10, MaxX: 20, MaxY: 20}
	assertEmptyNonNull(t, mustQueryRect(t, m, empty))

	// 全部命中仍按标识排序：a,b,c。
	all := mustQueryRect(t, m, Rect{MinX: -1, MinY: -1, MaxX: 6, MaxY: 1})
	if len(all) != 3 {
		t.Fatalf("all hits = %v", all)
	}
	assertSortedByID(t, all)
	if all[0].ID != "a" || all[1].ID != "b" || all[2].ID != "c" {
		t.Fatalf("all hits order = %v", all)
	}
}

// TestRectAfterCorrectionPersistRoundTrip 回归持久化：校正后的区域命中
// 结果在关闭重开后保持一致（不会退回旧位置判断），区域返回位置与重开
// 后当前有效出现记录的最新位置一致。
func TestRectAfterCorrectionPersistRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 5.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(crossAnchorSegment()); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 0, Observations: []Observation{{ID: "R", X: 1, Y: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// 失效 R 后让它在远处再现，确保重开后区域查询仍只认有效新出现。
	if _, err := m.Invalidate(Invalidation{ID: "iv1", Reason: "gone", Landmarks: []string{"R"}}); err != nil {
		t.Fatal(err)
	}
	// R/1 在 t=300 为本地 (1,1) → 地图 (3,1)；R/2 在 t=400 位姿仍为
	// (2,0)，本地 (7,1) → 地图 (9,1)，次数 1。
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 400, DX: 0, Observations: []Observation{{ID: "R", X: 7, Y: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// L：d=3 → 均值 (3,0) 次数 3；校正范围 [100,400] 同时重放两次 R：
	// R/1（已失效）t=300 观测 → (6,1)，R/2（当前有效）t=400 观测 → (12,1)。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 4, Y: 0}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}

	oldRect := Rect{MinX: 0.5, MinY: -0.5, MaxX: 1.5, MaxY: 0.5}
	lRect := Rect{MinX: 2.5, MinY: -0.5, MaxX: 3.5, MaxY: 0.5}
	rOldRect := Rect{MinX: 5.5, MinY: 0.5, MaxX: 6.5, MaxY: 1.5}   // R/1 校正后 (6,1)，仍失效
	rNewRect := Rect{MinX: 11.5, MinY: 0.5, MaxX: 12.5, MaxY: 1.5} // R/2 校正后 (12,1)
	want := func(mm *Map) {
		t.Helper()
		assertEmptyNonNull(t, mustQueryRect(t, mm, oldRect))
		lms := mustQueryRect(t, mm, lRect)
		if len(lms) != 1 || lms[0].ID != "L" {
			t.Fatalf("L rect = %v, want L", lms)
		}
		approxEq(t, "L x", lms[0].X, 3)
		if lms[0].Count != 3 {
			t.Fatalf("L count = %d, want 3", lms[0].Count)
		}
		assertAppearanceMatches(t, mm, lms[0])
		assertEmptyNonNull(t, mustQueryRect(t, mm, rOldRect))
		rms := mustQueryRect(t, mm, rNewRect)
		if len(rms) != 1 || rms[0].ID != "R" {
			t.Fatalf("R rect = %v, want active R/2", rms)
		}
		approxEq(t, "R x", rms[0].X, 12)
		approxEq(t, "R y", rms[0].Y, 1)
		if rms[0].Count != 1 {
			t.Fatalf("R count = %d, want 1", rms[0].Count)
		}
		assertAppearanceMatches(t, mm, rms[0])
	}
	want(m)
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	defer m2.Close()
	want(m2)

	// 重开后全部命中仍按标识排序（L、R），且只含有效出现。
	all := mustQueryRect(t, m2, Rect{MinX: -1, MinY: -1, MaxX: 13, MaxY: 9})
	if len(all) != 2 {
		t.Fatalf("all after reopen = %v, want L and R active only", all)
	}
	assertSortedByID(t, all)
	if all[0].ID != "L" || all[1].ID != "R" {
		t.Fatalf("order after reopen = %v", all)
	}
}

// TestRectAfterCorrectionNoActiveLandmarks 校验空地图及“路标全部失效”
// 两种情形下，校正前后区域查询都返回非 nil 空结果，不因为校正涉及已
// 失效出现而把旧记录带回查询。
func TestRectAfterCorrectionNoActiveLandmarks(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 5.0}

	// 情形一：根本没有路标观测的轨迹，校正后查询仍为空且非 nil。
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 9, Y: 0}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}
	assertEmptyNonNull(t, mustQueryRect(t, m, Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9}))

	// 情形二：仅有的路标在锚点之前被观测、随后失效；校正范围涉及这条
	// 已失效旧出现，区域查询仍为空，旧记录不能复活。
	m2 := newMap(t, cfg)
	if _, err := m2.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "L"}}},
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m2.Invalidate(Invalidation{ID: "iv", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	rec, err := m2.Correct(Correction{ID: "c2", Anchor: 100, Target: CorrectionTarget{X: 4, Y: 0}})
	if err != nil {
		t.Fatalf("Correct across invalidated occurrence: %v", err)
	}
	if len(rec.Landmarks) != 1 || rec.Landmarks[0].Occurrence != 1 {
		t.Fatalf("expected change for invalidated occurrence 1: %+v", rec.Landmarks)
	}
	assertEmptyNonNull(t, mustQueryRect(t, m2, Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9}))
	// 旧出现的位置确实随校正改变（历史可查），但查询不返回它。
	h, err := m2.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 || h.Appearances[0].Active {
		t.Fatalf("old occurrence = %+v", h.Appearances)
	}
	approxEq(t, "inactive occurrence moved x", h.Appearances[0].Landmark.X, 2)
}
