package posemap

import "testing"

// 本文件为“路标失效的返回结果与地图保存的失效事实相互隔离”这一已有行为
// 提供回归保障。Invalidate 成功时会保存首次操作结果，此后同一操作标识、
// 相同原因、相同路标集合的请求一律返回首次结果。调用方可以自由改写手上
// 的那份返回结果（失效时间、路标标识、出现编号、条目次序，乃至增删条目），
// 但这些改写只能影响调用方自己的副本：既不能改写地图保存的失效事实与首次
// 结果，也不能影响先后取得的其他副本，更不能让一份合法的重复请求被误判
// 为内容冲突。只使用现有公开入口（ImportSegment/Invalidate/
// LandmarksInRect/LandmarkAppearances），沿用既有错误含义，不改变任何
// 公开使用方式。
//
// 统一场景（初始位姿 (0,0)、朝向 0、合并上限 1）：
//
//	t=100：观测 L 本地 (1,0) → 世界 (1,0)（L 第 1 次出现，计数 1）
//	       观测 M 本地 (2,0) → 世界 (2,0)（M 第 1 次出现，计数 1）
//	       观测 N 本地 (3,0) → 世界 (3,0)（N 第 1 次出现，计数 1）
//	t=200：再次观测 L（仍 (1,0)，L 计数 2）
//	t=200 提交 op0={L}：撤下 L 的第 1 次出现，失效时间 200
//	t=300：DX=10 → 位姿 (10,0)，观测 L 本地 (0,0) → (10,0)（L 第 2 次出现）
//	t=300 提交 op1={L,M,N}（乱序给出）：同时撤下 L 的第 2 次出现与 M、N
//	       的第 1 次出现，失效时间 300。结果按标识排列为 L→2、M→1、N→1，
//	       同一次失效里包含不同的出现编号。
//	t=400：DX=10 → 位姿 (20,0)，L 再现为第 3 次出现（世界 (20,0)）
//	t=500：DX=10 → 位姿 (30,0)，M 再现为第 2 次出现（世界 (30,0)）
//	       N 从此不再出现。
//
// op1 之后重复提交 op1 必须始终返回首次事实（时间 300，L→2、M→1、N→1）：
// 这些编号永远指向已失效的旧出现，不会被替换成当前的 L→3、M→2，重复提交
// 也不会撤下任何新出现。

// isolationSeedSegment 是统一场景的初始轨迹：t=100 同时观测 L、M、N，
// t=200 再次观测 L。
func isolationSeedSegment() Segment {
	return Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{
			{ID: "L", X: 1, Y: 0},
			{ID: "M", X: 2, Y: 0},
			{ID: "N", X: 3, Y: 0},
		}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
	}}
}

// newIsolationMapBeforeBatch 构造统一场景中“op1 提交之前”的地图：
// L 的第 1 次出现已被 op0 撤下，L 的第 2 次出现当前有效，M、N 的第 1 次
// 出现仍有效。各用例据此提交 op1，并按需继续导入 t=400/500 的再现轨迹。
func newIsolationMapBeforeBatch(t *testing.T) *Map {
	t.Helper()
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0})
	if _, err := m.ImportSegment(isolationSeedSegment()); err != nil {
		t.Fatalf("seed import: %v", err)
	}
	if _, err := m.Invalidate(Invalidation{
		ID: "op0", Reason: "first gone", Landmarks: []string{"L"},
	}); err != nil {
		t.Fatalf("preliminary invalidation op0: %v", err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatalf("L reappearance as occurrence 2: %v", err)
	}
	return m
}

// isolationBatchRequest 是统一场景里的批量失效请求：集合故意以乱序给出，
// 成功结果仍应按标识排列为 L、M、N。
func isolationBatchRequest() Invalidation {
	return Invalidation{
		ID:        "op1",
		Reason:    "batch removed",
		Landmarks: []string{"M", "N", "L"},
	}
}

// isolationBatchResult 是 op1 首次成功时保存的事实：失效时间 300，L 被
// 撤下的是第 2 次出现，M、N 被撤下的是第 1 次出现，按标识排列。
func isolationBatchResult() InvalidationResult {
	return InvalidationResult{
		Time: 300,
		Landmarks: []InvalidatedLandmark{
			{ID: "L", Occurrence: 2},
			{ID: "M", Occurrence: 1},
			{ID: "N", Occurrence: 1},
		},
	}
}

// snapshotInvalidationResult 在测试侧深拷贝一份失效结果作为期望快照。
// 拷贝逻辑独立于被测实现的 cloneInvalidationResult，避免被测实现退化为
// 浅拷贝时期望值与内部状态共享底层切片、被一并改写而失去对照意义。
func snapshotInvalidationResult(r InvalidationResult) InvalidationResult {
	cp := r
	cp.Landmarks = append([]InvalidatedLandmark(nil), r.Landmarks...)
	return cp
}

// invalidationResultsEqual 判断两份失效结果的时间与逐项内容、次序一致。
func invalidationResultsEqual(a, b InvalidationResult) bool {
	if a.Time != b.Time || len(a.Landmarks) != len(b.Landmarks) {
		return false
	}
	for i := range a.Landmarks {
		if a.Landmarks[i] != b.Landmarks[i] {
			return false
		}
	}
	return true
}

// expectBatchResult 断言一份结果等于 op1 首次保存的事实。
func expectBatchResult(t *testing.T, what string, got InvalidationResult) {
	t.Helper()
	if want := isolationBatchResult(); !invalidationResultsEqual(got, want) {
		t.Fatalf("%s = %+v, want %+v", what, got, want)
	}
}

// expectInvalidAppearance 断言某次出现已失效，且保留着期望的失效时间、
// 原因与操作标识。
func expectInvalidAppearance(t *testing.T, id string, ap Appearance, wantTime int64, wantReason, wantOp string) {
	t.Helper()
	if ap.Number <= 0 || ap.Active {
		t.Fatalf("landmark %s occurrence %d unexpectedly active", id, ap.Number)
	}
	if ap.InvalidTime != wantTime || ap.InvalidReason != wantReason || ap.InvalidOpID != wantOp {
		t.Fatalf("landmark %s occurrence %d invalid info = %d %q %q, want %d %q %q",
			id, ap.Number, ap.InvalidTime, ap.InvalidReason, ap.InvalidOpID,
			wantTime, wantReason, wantOp)
	}
}

// importReappearances 在 op1 之后导入 L 的第 3 次出现（t=400，世界
// (20,0)）与 M 的第 2 次出现（t=500，世界 (30,0)）；N 不再现。
func importReappearances(t *testing.T, m *Map) {
	t.Helper()
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 400, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatalf("L reappearance as occurrence 3: %v", err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s4", Frames: []Frame{
		{Time: 500, DX: 10, Observations: []Observation{{ID: "M"}}},
	}}); err != nil {
		t.Fatalf("M reappearance as occurrence 2: %v", err)
	}
}

// expectReappearedActive 断言再现后区域查询只含 L 的第 3 次与 M 的第 2 次
// 当前有效出现，未再现的 N 继续被排除。
func expectReappearedActive(t *testing.T, m *Map) {
	t.Helper()
	lms := activeLandmarks(t, m)
	if len(lms) != 2 {
		t.Fatalf("active landmarks = %v, want only reappeared L and N-excluded set {L,M}", lms)
	}
	lL, ok := lms["L"]
	if !ok || lL.Count != 1 {
		t.Fatalf("active L = %+v ok=%v, want occurrence 3 count 1", lL, ok)
	}
	approxEq(t, "active L x", lL.X, 20)
	lM, ok := lms["M"]
	if !ok || lM.Count != 1 {
		t.Fatalf("active M = %+v ok=%v, want occurrence 2 count 1", lM, ok)
	}
	approxEq(t, "active M x", lM.X, 30)
	if _, ok := lms["N"]; ok {
		t.Fatal("non-reappeared N back in region query")
	}
}

// 首次成功返回：一次失效涉及多个路标时结果按标识排列，每项指出实际被
// 撤下的出现编号，其中包含不同编号（L 是第 2 次，M、N 是第 1 次）。
func TestInvalidateFirstResultOrderAndOccurrences(t *testing.T) {
	m := newIsolationMapBeforeBatch(t)
	first, err := m.Invalidate(isolationBatchRequest())
	if err != nil {
		t.Fatalf("first batch Invalidate: %v", err)
	}
	expectBatchResult(t, "first result", first)

	// 区域查询立即排除三个被撤下的路标。
	if lms, _ := m.LandmarksInRect(allRect()); len(lms) != 0 {
		t.Fatalf("invalidated landmarks still in region query: %v", lms)
	}

	// 历次出现记录：L 两次出现分别保留各自的失效事实；M、N 的第 1 次出现
	// 记录本次失效。
	hL, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(hL.Appearances) != 2 {
		t.Fatalf("L history = %+v, want two occurrences", hL.Appearances)
	}
	expectInvalidAppearance(t, "L", hL.Appearances[0], 200, "first gone", "op0")
	expectInvalidAppearance(t, "L", hL.Appearances[1], 300, "batch removed", "op1")
	if hL.Appearances[1].Number != 2 {
		t.Fatalf("L second occurrence number = %d, want 2", hL.Appearances[1].Number)
	}
	for _, id := range []string{"M", "N"} {
		h, err := m.LandmarkAppearances(id)
		if err != nil {
			t.Fatalf("history %s: %v", id, err)
		}
		if len(h.Appearances) != 1 {
			t.Fatalf("history %s = %+v, want single occurrence 1", id, h.Appearances)
		}
		expectInvalidAppearance(t, id, h.Appearances[0], 300, "batch removed", "op1")
	}
}

// 调用方改写首次返回的失效时间、路标标识、出现编号并调换条目次序后，
// 用原来的操作内容重复提交：得到的仍是原始时间、完整集合与正确编号，
// 既不带入被修改的内容，也不漏掉其他条目；地图保存的失效事实同样不变。
func TestInvalidateDuplicateResultIsolatedFromFirstMutation(t *testing.T) {
	m := newIsolationMapBeforeBatch(t)
	req := isolationBatchRequest()
	first, err := m.Invalidate(req)
	if err != nil {
		t.Fatalf("first batch Invalidate: %v", err)
	}
	want := snapshotInvalidationResult(first)

	// ---- 就地改写调用方手上的首次结果：时间、标识、编号，再调换次序 ----
	first.Time = -77
	first.Landmarks[0].ID = "ZZ"
	first.Landmarks[0].Occurrence = 99
	first.Landmarks[1].ID = ""
	first.Landmarks[1].Occurrence = -3
	first.Landmarks[2].ID = "tampered"
	first.Landmarks[2].Occurrence = 12345
	first.Landmarks[0], first.Landmarks[2] = first.Landmarks[2], first.Landmarks[0]

	// 原操作内容重复提交：返回首次保存的事实，改写内容一个都不能出现，
	// L、M、N 三项都不能少，次序仍按标识排列。
	again, err := m.Invalidate(req)
	if err != nil {
		t.Fatalf("duplicate Invalidate after mutating first result: %v", err)
	}
	if !invalidationResultsEqual(again, want) {
		t.Fatalf("duplicate result = %+v, want saved first %+v", again, want)
	}
	expectBatchResult(t, "duplicate result", again)

	// 地图保存的失效事实没被改写：L 两次出现与 M、N 一次出现的失效时间、
	// 原因、编号保持原样。
	hL, _ := m.LandmarkAppearances("L")
	if len(hL.Appearances) != 2 {
		t.Fatalf("L history = %+v", hL.Appearances)
	}
	expectInvalidAppearance(t, "L", hL.Appearances[0], 200, "first gone", "op0")
	expectInvalidAppearance(t, "L", hL.Appearances[1], 300, "batch removed", "op1")
	for _, id := range []string{"M", "N"} {
		h, _ := m.LandmarkAppearances(id)
		if len(h.Appearances) != 1 {
			t.Fatalf("history %s = %+v", id, h.Appearances)
		}
		expectInvalidAppearance(t, id, h.Appearances[0], 300, "batch removed", "op1")
	}
}

// 重复提交得到的结果也是独立副本：改写其中一份，先前保留的另一份返回
// 结果不跟着变化；后续再次取得的结果仍与首次保存的事实一致。同一请求
// 的路标列表换一种先后顺序仍是相同集合，返回次序继续按标识排列；不能
// 因为调用方修改过上一次的输出，就把合法的重复请求误判为内容冲突。
func TestInvalidateDuplicateCopiesIndependent(t *testing.T) {
	m := newIsolationMapBeforeBatch(t)
	first, err := m.Invalidate(isolationBatchRequest())
	if err != nil {
		t.Fatalf("first batch Invalidate: %v", err)
	}
	want := snapshotInvalidationResult(first)

	// 第一份重复结果：集合以另一种次序给出（集合语义，与次序无关）。
	copyA, err := m.Invalidate(Invalidation{
		ID: "op1", Reason: "batch removed", Landmarks: []string{"L", "M", "N"},
	})
	if err != nil {
		t.Fatalf("duplicate with reordered set rejected as mismatch: %v", err)
	}
	if !invalidationResultsEqual(copyA, want) {
		t.Fatalf("copyA = %+v, want first result %+v", copyA, want)
	}

	// 调用方只改写 copyA 中的一项。
	copyA.Time = -1
	copyA.Landmarks[0].ID = "tampered"
	copyA.Landmarks[0].Occurrence = 424

	// 先前保留的首次结果不跟着变化。
	if !invalidationResultsEqual(first, want) {
		t.Fatalf("earlier result changed after editing another copy: %+v", first)
	}

	// 再次取得的结果仍是首次事实：即便上次输出被改成陌生标识/编号，
	// 合法的重复请求也不会被误判为 invalidation_mismatch。
	copyB, err := m.Invalidate(isolationBatchRequest())
	if err != nil {
		t.Fatalf("second duplicate after mutating copyA: %v", err)
	}
	if !invalidationResultsEqual(copyB, want) {
		t.Fatalf("copyB = %+v, want first result %+v", copyB, want)
	}

	// 再改写 copyB 的另一项并追加一个伪造条目，先前两份结果互不影响。
	copyB.Landmarks[1].ID = "also-tampered"
	copyB.Landmarks[1].Occurrence = -9
	copyB.Landmarks = append(copyB.Landmarks, InvalidatedLandmark{ID: "EXTRA", Occurrence: 7})
	if !invalidationResultsEqual(first, want) {
		t.Fatalf("first result changed after editing copyB: %+v", first)
	}
	if copyA.Landmarks[1].ID != "M" || copyA.Landmarks[1].Occurrence != 1 ||
		copyA.Landmarks[2].ID != "N" || copyA.Landmarks[2].Occurrence != 1 || len(copyA.Landmarks) != 3 {
		t.Fatalf("copyA changed after editing copyB: %+v", copyA)
	}

	// 第三次重复（再换一种集合次序）仍返回首次事实，伪造条目不渗入。
	copyC, err := m.Invalidate(Invalidation{
		ID: "op1", Reason: "batch removed", Landmarks: []string{"N", "L", "M"},
	})
	if err != nil {
		t.Fatalf("third duplicate: %v", err)
	}
	if !invalidationResultsEqual(copyC, want) {
		t.Fatalf("copyC = %+v, want first result %+v (saved fact polluted)", copyC, want)
	}
}

// 原操作涉及的路标后来再次出现：首次结果中的编号仍指向已经失效的旧
// 出现，不会被替换成当前有效出现的编号；即使调用方把拿到的旧结果改成
// 新编号（并改掉时间、标识、次序），再提交原操作也不能撤下新出现。
// 区域查询仍包含新出现，历次出现查询保留旧记录的失效时间和原因；
// 未再次出现的路标继续被区域查询排除。
func TestInvalidateDuplicateKeepsStaleOccurrenceAfterReappearance(t *testing.T) {
	m := newIsolationMapBeforeBatch(t)
	first, err := m.Invalidate(isolationBatchRequest())
	if err != nil {
		t.Fatalf("first batch Invalidate: %v", err)
	}
	want := snapshotInvalidationResult(first)

	// L 再现为第 3 次出现、M 再现为第 2 次出现，N 不再出现。
	importReappearances(t, m)
	expectReappearedActive(t, m)

	// 历次出现记录：旧出现保留各自失效时间与原因，新出现当前有效。
	hL, _ := m.LandmarkAppearances("L")
	if len(hL.Appearances) != 3 {
		t.Fatalf("L history = %+v, want three occurrences", hL.Appearances)
	}
	expectInvalidAppearance(t, "L", hL.Appearances[0], 200, "first gone", "op0")
	expectInvalidAppearance(t, "L", hL.Appearances[1], 300, "batch removed", "op1")
	if !hL.Appearances[2].Active || hL.Appearances[2].Number != 3 ||
		hL.Appearances[2].Landmark.Count != 1 || !hL.Appearances[2].HasFirstSeen ||
		hL.Appearances[2].FirstSeenTime != 400 {
		t.Fatalf("L occurrence 3 = %+v, want active occurrence first seen at 400", hL.Appearances[2])
	}
	hM, _ := m.LandmarkAppearances("M")
	if len(hM.Appearances) != 2 {
		t.Fatalf("M history = %+v, want two occurrences", hM.Appearances)
	}
	expectInvalidAppearance(t, "M", hM.Appearances[0], 300, "batch removed", "op1")
	if !hM.Appearances[1].Active || hM.Appearances[1].Number != 2 ||
		hM.Appearances[1].FirstSeenTime != 500 {
		t.Fatalf("M occurrence 2 = %+v, want active occurrence first seen at 500", hM.Appearances[1])
	}
	hN, _ := m.LandmarkAppearances("N")
	if len(hN.Appearances) != 1 {
		t.Fatalf("N history = %+v, want only invalidated occurrence 1", hN.Appearances)
	}
	expectInvalidAppearance(t, "N", hN.Appearances[0], 300, "batch removed", "op1")

	// 调用方把拿到的旧结果改成“当前有效编号”（L 改 3、M 改 2，N 改成
	// 根本不存在的 2），并改掉失效时间与标识、调换次序——试图让重复提交
	// 撤下新出现。
	first.Time = 500
	first.Landmarks[0].Occurrence = 3
	first.Landmarks[1].Occurrence = 2
	first.Landmarks[2].Occurrence = 2
	first.Landmarks[0].ID = "L-current"
	first.Landmarks[1].ID = "M-current"
	first.Landmarks[2].ID = "N-ghost"
	first.Landmarks[0], first.Landmarks[1] = first.Landmarks[1], first.Landmarks[0]

	// 用原来的操作内容重复提交：结果仍是旧事实（时间 300，L→2、M→1、
	// N→1），不因调用方手上的篡改撤下任何新出现。
	again, err := m.Invalidate(isolationBatchRequest())
	if err != nil {
		t.Fatalf("duplicate after reappearance: %v", err)
	}
	if !invalidationResultsEqual(again, want) {
		t.Fatalf("duplicate after reappearance = %+v, want stale first result %+v", again, want)
	}

	// L 第 3 次、M 第 2 次出现仍然有效，区域查询结果不变。
	expectReappearedActive(t, m)

	// 旧出现的失效时间与原因不变。
	hL2, _ := m.LandmarkAppearances("L")
	if len(hL2.Appearances) != 3 {
		t.Fatalf("L occurrences after duplicate = %+v", hL2.Appearances)
	}
	expectInvalidAppearance(t, "L", hL2.Appearances[0], 200, "first gone", "op0")
	expectInvalidAppearance(t, "L", hL2.Appearances[1], 300, "batch removed", "op1")
	if !hL2.Appearances[2].Active {
		t.Fatal("L new occurrence was taken down by duplicate submission")
	}
	hM2, _ := m.LandmarkAppearances("M")
	if len(hM2.Appearances) != 2 || hM2.Appearances[0].Active || !hM2.Appearances[1].Active {
		t.Fatalf("M occurrences changed by duplicate: %+v", hM2.Appearances)
	}
	expectInvalidAppearance(t, "M", hM2.Appearances[0], 300, "batch removed", "op1")
	hN2, _ := m.LandmarkAppearances("N")
	if len(hN2.Appearances) != 1 || hN2.Appearances[0].Active {
		t.Fatalf("N occurrences changed by duplicate: %+v", hN2.Appearances)
	}
}

// 同一操作标识换了原因或换了路标集合，仍以 invalidation_mismatch 拒绝，
// 且拒绝不改变此前的结果隔离关系与有效/失效状态：先前取得的结果保持
// 原样，原操作随后重复提交仍返回首次事实，再现的新出现继续有效。
func TestInvalidateMismatchAfterMutatedDuplicatesPreservesState(t *testing.T) {
	m := newIsolationMapBeforeBatch(t)
	first, err := m.Invalidate(isolationBatchRequest())
	if err != nil {
		t.Fatalf("first batch Invalidate: %v", err)
	}
	want := snapshotInvalidationResult(first)

	// L 再现为第 3 次出现、M 再现为第 2 次出现，N 不再出现。
	importReappearances(t, m)

	// 先取一份重复结果并彻底改写，制造“合法重复请求可能被上次输出污染”
	// 的前提。
	dup, err := m.Invalidate(isolationBatchRequest())
	if err != nil {
		t.Fatalf("duplicate before mismatch: %v", err)
	}
	dup.Time = -5
	dup.Landmarks[0].ID = "ZZ"
	dup.Landmarks[0].Occurrence = 333
	dup.Landmarks[1].ID = "QQ"
	dup.Landmarks[1].Occurrence = 666
	dup.Landmarks[2].ID = "XX"
	dup.Landmarks[2].Occurrence = 999
	dup.Landmarks = append(dup.Landmarks, InvalidatedLandmark{ID: "EXTRA", Occurrence: 1})

	expectMismatch := func(req Invalidation) {
		t.Helper()
		_, err := m.Invalidate(req)
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectInvalidationMismatch ||
			!r.HasLandmark || r.Landmark != "op1" {
			t.Fatalf("mismatch request %+v: err = %v, want invalidation_mismatch for op1", req, err)
		}
	}

	// 同标识换原因。
	expectMismatch(Invalidation{ID: "op1", Reason: "other reason", Landmarks: []string{"M", "N", "L"}})
	// 同标识换路标集合：减少、只留一个、扩大到陌生标识。
	expectMismatch(Invalidation{ID: "op1", Reason: "batch removed", Landmarks: []string{"M", "N"}})
	expectMismatch(Invalidation{ID: "op1", Reason: "batch removed", Landmarks: []string{"L"}})
	expectMismatch(Invalidation{ID: "op1", Reason: "batch removed", Landmarks: []string{"L", "M", "N", "ZZ"}})

	// 先前保留的首次结果未被任何拒绝改变。
	if !invalidationResultsEqual(first, want) {
		t.Fatalf("first result changed after mismatch rejections: %+v", first)
	}
	// 原操作内容重复提交仍返回首次事实（时间 300，L→2、M→1、N→1）。
	again, err := m.Invalidate(isolationBatchRequest())
	if err != nil {
		t.Fatalf("original request after mismatches: %v", err)
	}
	if !invalidationResultsEqual(again, want) {
		t.Fatalf("result after mismatches = %+v, want first result %+v", again, want)
	}

	// 有效/失效状态不变：L 第 3 次、M 第 2 次出现仍有效并在区域查询中，
	// N 仍只有失效的第 1 次出现、继续被排除；旧出现失效信息原样保留。
	expectReappearedActive(t, m)
	hL, _ := m.LandmarkAppearances("L")
	if len(hL.Appearances) != 3 || hL.Appearances[0].Active || hL.Appearances[1].Active ||
		!hL.Appearances[2].Active {
		t.Fatalf("L occurrences after mismatches = %+v", hL.Appearances)
	}
	expectInvalidAppearance(t, "L", hL.Appearances[0], 200, "first gone", "op0")
	expectInvalidAppearance(t, "L", hL.Appearances[1], 300, "batch removed", "op1")
	hM, _ := m.LandmarkAppearances("M")
	if len(hM.Appearances) != 2 || hM.Appearances[0].Active || !hM.Appearances[1].Active {
		t.Fatalf("M occurrences after mismatches = %+v", hM.Appearances)
	}
	expectInvalidAppearance(t, "M", hM.Appearances[0], 300, "batch removed", "op1")
	hN, _ := m.LandmarkAppearances("N")
	if len(hN.Appearances) != 1 || hN.Appearances[0].Active {
		t.Fatalf("N occurrences after mismatches = %+v, want single invalid occurrence", hN.Appearances)
	}
	expectInvalidAppearance(t, "N", hN.Appearances[0], 300, "batch removed", "op1")
}
