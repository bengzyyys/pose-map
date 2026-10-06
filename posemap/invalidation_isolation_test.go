package posemap

import "testing"

// 本文件为“失效操作的返回结果是独立副本”这一已有行为提供回归保障：
// Invalidate 成功时保存首次结果，之后同一操作标识、相同原因与相同
// 路标集合的提交都返回首次结果。调用方拿到结果后可以任意改写其中的
// 失效时间、路标标识、出现编号，或调换、截断条目，这些改写只能影响
// 手上的那份副本，不能改写地图保存的失效事实，也不能让合法的重复
// 提交被误判为内容冲突。只使用现有公开入口（ImportSegment/Invalidate/
// LandmarkAppearances/LandmarksInRect），不改变任何公开使用方式与
// 错误含义。

// snapshotInvalidation 在测试侧深拷贝一份失效结果作为期望快照。不复用
// 被测实现里的 cloneInvalidationResult，避免被测实现退化为浅拷贝时
// 期望值与内部状态共享底层切片、被一并改写而失去对照意义。
func snapshotInvalidation(r InvalidationResult) InvalidationResult {
	cp := r
	cp.Landmarks = append([]InvalidatedLandmark(nil), r.Landmarks...)
	return cp
}

func checkInvalidationResult(t *testing.T, got, want InvalidationResult) {
	t.Helper()
	if got.Time != want.Time {
		t.Fatalf("invalidation time = %d, want %d", got.Time, want.Time)
	}
	if len(got.Landmarks) != len(want.Landmarks) {
		t.Fatalf("invalidation landmarks = %+v, want %+v", got.Landmarks, want.Landmarks)
	}
	for i := range want.Landmarks {
		if got.Landmarks[i] != want.Landmarks[i] {
			t.Fatalf("invalidation landmark %d = %+v, want %+v", i, got.Landmarks[i], want.Landmarks[i])
		}
	}
}

// scrambleInvalidation 模拟调用方就地改写一份失效结果的全部可写字段：
// 失效时间、每个条目的路标标识与出现编号，并调换条目次序、截断列表。
// 若返回值不是独立副本，这些改写会污染地图内部保存的失效事实。
func scrambleInvalidation(r *InvalidationResult) {
	r.Time = -1
	for i := range r.Landmarks {
		r.Landmarks[i].ID = "tampered"
		r.Landmarks[i].Occurrence = 999
	}
	if len(r.Landmarks) > 1 {
		r.Landmarks[0], r.Landmarks[1] = r.Landmarks[1], r.Landmarks[0]
		r.Landmarks = r.Landmarks[:1]
	}
}

// seedInvalidationIsolationMap 构造一次失效涉及多个路标、且各条目出现
// 编号不同的场景：
//   - s1：t=100 观测 L(1,0)、M(2,0)，t=200 再观测 L → 当前位姿时间 200；
//   - op0：失效 {L} → L 第 1 次出现失效于 200；
//   - s2：t=300 位姿 (10,0) 观测 L → L 第 2 次出现有效，当前位姿时间 300。
//
// 之后 op1 = {ID: "op1", Reason: "retired", Landmarks: ["M", "L"]} 首次
// 成功时应返回 Time=300、按标识排序的 [{L,2},{M,1}]：同一次操作中不同
// 路标被撤下的是不同编号的出现。
func seedInvalidationIsolationMap(t *testing.T) *Map {
	t.Helper()
	m := newMap(t, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0})
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1, Y: 0}, {ID: "M", X: 2, Y: 0}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "op0", Reason: "stale", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	return m
}

// op1Request 返回场景中的多路标失效操作内容（列表次序 M、L）。
func op1Request() Invalidation {
	return Invalidation{ID: "op1", Reason: "retired", Landmarks: []string{"M", "L"}}
}

// op1Want 是 op1 首次成功应保存的事实：失效时间 300，按标识排序，
// L 被撤下的是第 2 次出现、M 是第 1 次出现。
func op1Want() InvalidationResult {
	return InvalidationResult{
		Time:      300,
		Landmarks: []InvalidatedLandmark{{ID: "L", Occurrence: 2}, {ID: "M", Occurrence: 1}},
	}
}

// 首次成功返回的结果是独立副本：一次失效涉及多个路标时结果按标识排列、
// 各条目指出实际被撤下的出现编号（此处 L 为 2、M 为 1，编号不同）。
// 调用方改写失效时间、路标标识、出现编号，调换条目次序并截断列表后，
// 再提交原来的操作内容，得到的仍应是原始时间、完整的路标集合和正确
// 编号；地图中的失效事实与有效状态也不受影响。
func TestInvalidateFirstResultIsolation(t *testing.T) {
	m := seedInvalidationIsolationMap(t)

	res, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("Invalidate: %v", err)
	}
	want := op1Want()
	checkInvalidationResult(t, res, want)
	if res.Landmarks[0].Occurrence == res.Landmarks[1].Occurrence {
		t.Fatalf("expected distinct occurrence numbers: %+v", res.Landmarks)
	}

	// 调用方改写时间、标识、编号，调换条目并截断列表。
	scrambleInvalidation(&res)

	// 再提交原来的操作内容：仍返回首次保存的事实，不带入被修改的
	// 内容，也不漏掉被截断的条目。
	again, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("resubmit after tampering: %v", err)
	}
	checkInvalidationResult(t, again, want)

	// 地图保存的失效事实不受影响：L 第 2 次出现与 M 第 1 次出现
	// 失效于 300、原因与操作标识为首次提交的内容。
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("L appearances = %+v", h.Appearances)
	}
	if a := h.Appearances[0]; a.Active || a.InvalidTime != 200 || a.InvalidReason != "stale" || a.InvalidOpID != "op0" {
		t.Fatalf("L occ1 = %+v", a)
	}
	if a := h.Appearances[1]; a.Active || a.InvalidTime != 300 || a.InvalidReason != "retired" || a.InvalidOpID != "op1" {
		t.Fatalf("L occ2 = %+v", a)
	}
	hm, err := m.LandmarkAppearances("M")
	if err != nil {
		t.Fatal(err)
	}
	if len(hm.Appearances) != 1 {
		t.Fatalf("M appearances = %+v", hm.Appearances)
	}
	if a := hm.Appearances[0]; a.Active || a.InvalidTime != 300 || a.InvalidReason != "retired" || a.InvalidOpID != "op1" {
		t.Fatalf("M occ1 = %+v", a)
	}
	if lms, _ := m.LandmarksInRect(allRect()); len(lms) != 0 {
		t.Fatalf("invalidated landmarks still queried: %v", lms)
	}
}

// 重复提交得到的结果同样是独立副本：先后取得的两份重复结果互不影响，
// 改写其中一份后，先前保留的另一份不变，后续再次取得的结果仍与首次
// 保存的事实一致。
func TestInvalidateDuplicateResultIsolation(t *testing.T) {
	m := seedInvalidationIsolationMap(t)
	first, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("Invalidate: %v", err)
	}
	want := snapshotInvalidation(first)

	// 重复提交取得两份结果，保留其中一份作为对照。
	kept, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("duplicate: %v", err)
	}
	dup, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("duplicate: %v", err)
	}
	checkInvalidationResult(t, kept, want)
	checkInvalidationResult(t, dup, want)

	// 改写 dup：kept 不应跟着变化，再次取得的结果仍是首次事实。
	scrambleInvalidation(&dup)
	checkInvalidationResult(t, kept, want)
	onceMore, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("resubmit after scrambling duplicate: %v", err)
	}
	checkInvalidationResult(t, onceMore, want)
}

// 同一请求的路标列表换一种顺序仍属于相同集合：即使调用方修改过上次
// 的输出，换序提交也不会被误判为内容冲突，返回次序继续遵守按标识
// 排列的约定。
func TestInvalidateResubmitSetOrderAfterTampering(t *testing.T) {
	m := seedInvalidationIsolationMap(t)
	res, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("Invalidate: %v", err)
	}
	want := snapshotInvalidation(res)

	// 调用方修改上次的输出（含调换条目次序）。
	scrambleInvalidation(&res)

	// 换序提交相同集合：仍是合法重复，返回首次结果而非冲突拒绝。
	reordered := Invalidation{ID: "op1", Reason: "retired", Landmarks: []string{"L", "M"}}
	again, err := m.Invalidate(reordered)
	if err != nil {
		t.Fatalf("reordered resubmit rejected: %v", err)
	}
	checkInvalidationResult(t, again, want)
	if again.Landmarks[0].ID != "L" || again.Landmarks[1].ID != "M" {
		t.Fatalf("result not sorted by id: %+v", again.Landmarks)
	}
}

// 原操作对应的路标后来再次出现：原操作返回的编号仍指向已经失效的旧
// 出现，不被替换成当前有效出现的编号；即使调用方把拿到的旧结果改成
// 新编号，再提交原操作也不能撤下新出现。区域查询仍包含新出现，历次
// 出现查询保留旧记录的失效时间与原因，未再次出现的路标继续被区域
// 查询排除。
func TestInvalidateResultIsolationAfterReappearance(t *testing.T) {
	m := seedInvalidationIsolationMap(t)
	res, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("Invalidate: %v", err)
	}
	want := snapshotInvalidation(res)

	// L 在新轨迹中再次出现：第 3 次出现，当前有效。
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 400, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 重复提交仍返回首次结果：编号 2 指向已失效的旧出现，不被替换
	// 成当前有效出现的编号 3。
	again, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("resubmit after reappearance: %v", err)
	}
	checkInvalidationResult(t, again, want)

	// 调用方把拿到的旧结果改成新编号，再提交原操作：新出现不受影响。
	again.Landmarks[0].Occurrence = 3
	again.Landmarks[0].ID = "tampered"
	again.Time = -1
	if _, err := m.Invalidate(op1Request()); err != nil {
		t.Fatalf("resubmit after tampering to new occurrence: %v", err)
	}

	// L 第 3 次出现仍有效并出现在区域查询中；未再次出现的 M 继续
	// 被排除。
	lms := activeLandmarks(t, m)
	if len(lms) != 1 {
		t.Fatalf("active landmarks = %v, want only L occ3", lms)
	}
	l, ok := lms["L"]
	if !ok {
		t.Fatalf("active L missing: %v", lms)
	}
	approxEq(t, "occ3 x", l.X, 20)
	if l.Count != 1 {
		t.Fatalf("occ3 count = %d, want 1", l.Count)
	}
	if _, ok := lms["M"]; ok {
		t.Fatalf("M reappeared in rect query: %v", lms)
	}

	// 历次出现查询保留旧记录的失效时间与原因。
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 3 {
		t.Fatalf("L appearances = %+v", h.Appearances)
	}
	if a := h.Appearances[0]; a.Active || a.InvalidTime != 200 || a.InvalidReason != "stale" || a.InvalidOpID != "op0" {
		t.Fatalf("L occ1 = %+v", a)
	}
	if a := h.Appearances[1]; a.Active || a.InvalidTime != 300 || a.InvalidReason != "retired" || a.InvalidOpID != "op1" {
		t.Fatalf("L occ2 = %+v", a)
	}
	if a := h.Appearances[2]; !a.Active || a.Number != 3 || !a.HasFirstSeen || a.FirstSeenTime != 400 {
		t.Fatalf("L occ3 = %+v", a)
	}
	hm, err := m.LandmarkAppearances("M")
	if err != nil {
		t.Fatal(err)
	}
	if len(hm.Appearances) != 1 || hm.Appearances[0].Active ||
		hm.Appearances[0].InvalidTime != 300 || hm.Appearances[0].InvalidReason != "retired" {
		t.Fatalf("M appearances = %+v", hm.Appearances)
	}
}

// 同一操作标识换了原因或路标集合，仍以 invalidation_mismatch 拒绝，
// 且不改变已保存的首次结果与各路标的有效状态。
func TestInvalidateMismatchKeepsSavedResult(t *testing.T) {
	m := seedInvalidationIsolationMap(t)
	res, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("Invalidate: %v", err)
	}
	want := snapshotInvalidation(res)

	// L 再现（第 3 次出现，当前有效），使“不改变有效状态”可检验。
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 400, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 同标识不同原因：拒绝。
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "other", Landmarks: []string{"M", "L"}}); err == nil {
		t.Fatal("different reason accepted")
	} else if r, ok := AsRejectError(err); !ok || r.Kind != RejectInvalidationMismatch || r.Landmark != "op1" {
		t.Fatalf("reason mismatch err = %v", err)
	}
	// 同标识不同集合（子集与超集）：拒绝。
	for _, set := range [][]string{{"L"}, {"L", "M", "N"}} {
		if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "retired", Landmarks: set}); err == nil {
			t.Fatalf("different set %v accepted", set)
		} else if r, ok := AsRejectError(err); !ok || r.Kind != RejectInvalidationMismatch {
			t.Fatalf("set %v mismatch err = %v", set, err)
		}
	}

	// 拒绝不改变已保存的首次结果：原内容重复提交仍返回首次事实。
	again, err := m.Invalidate(op1Request())
	if err != nil {
		t.Fatalf("resubmit after mismatches: %v", err)
	}
	checkInvalidationResult(t, again, want)

	// 有效状态不变：L 第 3 次出现仍有效，M 仍被排除。
	lms := activeLandmarks(t, m)
	if len(lms) != 1 {
		t.Fatalf("active landmarks = %v, want only L occ3", lms)
	}
	if _, ok := lms["L"]; !ok {
		t.Fatalf("L occ3 deactivated by mismatch: %v", lms)
	}
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 3 || !h.Appearances[2].Active ||
		h.Appearances[0].InvalidTime != 200 || h.Appearances[1].InvalidTime != 300 {
		t.Fatalf("L appearances changed by mismatch: %+v", h.Appearances)
	}
}
