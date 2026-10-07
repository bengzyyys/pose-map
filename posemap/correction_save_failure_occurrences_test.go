package posemap

import (
	"path/filepath"
	"testing"
)

// 本文件为“回环校正在落盘失败时整体失败”补充一项回归保障，专门覆盖校正
// 跨越同一路标两次出现的情形：同一标识先在种子段中被观测合并（第 1 次
// 出现），失效后又在更晚的轨迹段里建立新出现（第 2 次出现，仍有效），两条
// 记录各自保留自己的观测。校正锚点位于第 1 次出现的观测范围内，受影响帧
// 同时包含两次出现的观测；校正目标使两次出现的平均位置都实际改变，且都
// 符合合并距离规则——即校正本身完全合法，仅因本地地图文件无法更新而保存
// 失败。此时提交前的全部状态必须精确恢复：当前位姿与各历史位姿的位置、
// 朝向、方差，两次出现的位置、计数、编号、首次观测时间、有效状态以及旧
// 出现的失效时间、原因、操作标识，还有此前已成功的校正记录（按原顺序、
// 内容不改写、不追加失败记录）。区域查询仍只返回提交前的有效出现（提交前
// 坐标），已失效的旧出现仍被排除。保存条件恢复后，同一地图对象以同一标识、
// 同一内容再次提交必须按首次成功处理：失败尝试不占用校正标识，曾暂存的
// 位移也不叠加第二次。
//
// 统一几何（初始位姿 (0,0)、朝向 0、初始方差 0.5、合并上限 5）：
//
// 种子段 seg-a（第 1 次出现的观测范围）：
//
//	t=100 位姿 (0,0)，方差 0.6：L 本地 (1,0) → 世界 (1,0)
//	t=200 位姿 (1,0)，方差 0.7：L 本地 (0,0) → 世界 (1,0)
//
// L 第 1 次出现：均值 (1,0)、计数 2、首次观测时间 100。
//
// 既有成功校正 c0-var（恒等几何、只调方差）：锚点 t=100、目标方差 0.8，
// 位姿方差变为 t=100 → 0.8、t=200 → 0.9，位置与路标不变。它提供一条
// “已成功的校正记录”，用于验证失败校正不改写、不追加。
//
// 失效 inv-l（原因 sign removed）：失效时间取提交时当前位姿时间 200，
// 第 1 次出现被撤下，保留 (1,0)、计数 2 与失效信息。
//
// 再现段 seg-b（第 2 次出现，锚点之后的更晚轨迹）：
//
//	t=300 位姿 (2,0)，方差 1.0：L 本地 (8,0) → 世界 (10,0)
//	             （旧出现已失效不参与合并，新出现首条观测不做距离判定）
//	t=400 位姿 (3,0)，方差 1.1：L 本地 (7,0) → 世界 (10,0)
//
// L 第 2 次出现：均值 (10,0)、计数 2、首次观测时间 300，仍有效。
//
// 待提交校正 c-span：锚点 t=200（位于第 1 次出现的观测范围内，锚点之前
// 还有 t=0 初始位姿与 t=100 帧这段更早历史），目标位置 (2,0)、朝向 0、
// 方差 1.0——纯平移 +1。受影响帧 t=200..400 同时包含两次出现的观测：
//
//	位姿：t=200 → (2,0) 方差 1.0；t=300 → (3,0) 方差 1.1；
//	      t=400 → (4,0) 方差 1.2；更早的 t=0、t=100 不变。
//	第 1 次出现重放：t=100 观测固定为 (1,0)，t=200 观测随新位姿 → (2,0)，
//	      二者相距 1 ≤ 5 可合并，均值 (1,0) → (1.5,0)，计数不变。
//	第 2 次出现重放：(11,0)、(11,0)，均值 (10,0) → (11,0)，计数不变。
//
// 两次出现的平均位置都实际改变，且都符合合并距离规则，不会提前拒绝。

// spanningCorrectionConfig 是这组场景的地图配置。
func spanningCorrectionConfig() Config {
	return Config{InitialTime: 0, InitialVariance: 0.5, MaxInterval: 1000, MergeDistance: 5.0}
}

// spanningCorrectionRequest 构造待提交的跨两次出现的校正。
func spanningCorrectionRequest() Correction {
	return Correction{ID: "c-span", Anchor: 200, Target: CorrectionTarget{X: 2, Y: 0, Heading: 0, Variance: 1.0}}
}

// seedSpanningCorrectionMap 在 path 处创建地图并保存好“种子段 + 既有成功
// 校正 + L 失效 + 再现段”的前置状态，返回的地图当前位姿停在 t=400 (3,0)、
// 方差 1.1：L 的第 1 次出现已失效（(1,0)、计数 2、失效时间 200），第 2 次
// 出现仍有效（(10,0)、计数 2、首次观测时间 300）。同时返回既有成功校正
// c0-var 的记录，供后续比对记录不被改写。
func seedSpanningCorrectionMap(t *testing.T, path string) (*Map, CorrectionRecord) {
	t.Helper()
	m, err := Create(path, spanningCorrectionConfig())
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	if _, err := m.ImportSegment(Segment{ID: "seg-a", Frames: []Frame{
		{Time: 100, DX: 0, MoveVariance: 0.1, Observations: []Observation{
			{ID: "L", X: 1, Y: 0},
		}},
		{Time: 200, DX: 1, MoveVariance: 0.1, Observations: []Observation{
			{ID: "L", X: 0, Y: 0},
		}},
	}}); err != nil {
		t.Fatalf("seed import seg-a: %v", err)
	}
	c0, err := m.Correct(Correction{ID: "c0-var", Anchor: 100,
		Target: CorrectionTarget{X: 0, Y: 0, Heading: 0, Variance: 0.8}})
	if err != nil {
		t.Fatalf("seed correction c0-var: %v", err)
	}
	inv, err := m.Invalidate(Invalidation{ID: "inv-l", Reason: "sign removed", Landmarks: []string{"L"}})
	if err != nil {
		t.Fatalf("seed invalidate L: %v", err)
	}
	if inv.Time != 200 || len(inv.Landmarks) != 1 ||
		inv.Landmarks[0].ID != "L" || inv.Landmarks[0].Occurrence != 1 {
		t.Fatalf("invalidation result = %+v, want L occurrence 1 at t=200", inv)
	}
	if _, err := m.ImportSegment(Segment{ID: "seg-b", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.1, Observations: []Observation{
			{ID: "L", X: 8, Y: 0},
		}},
		{Time: 400, DX: 1, MoveVariance: 0.1, Observations: []Observation{
			{ID: "L", X: 7, Y: 0},
		}},
	}}); err != nil {
		t.Fatalf("seed import seg-b: %v", err)
	}
	return m, c0
}

// wantPreSpanningPoses 是提交 c-span 之前（也是保存失败回滚之后）的全部
// 位姿，按时间升序。
func wantPreSpanningPoses() []Pose {
	return []Pose{
		{Time: 0, X: 0, Y: 0, Heading: 0, Variance: 0.5},
		{Time: 100, X: 0, Y: 0, Heading: 0, Variance: 0.8},
		{Time: 200, X: 1, Y: 0, Heading: 0, Variance: 0.9},
		{Time: 300, X: 2, Y: 0, Heading: 0, Variance: 1.0},
		{Time: 400, X: 3, Y: 0, Heading: 0, Variance: 1.1},
	}
}

// expectLandmarkChange 断言校正记录中某次出现的校正前后值与期望一致。
func expectLandmarkChange(t *testing.T, what string, got, want LandmarkChange) {
	t.Helper()
	if got.ID != want.ID || got.Occurrence != want.Occurrence {
		t.Fatalf("%s change header = (%q,%d), want (%q,%d)",
			what, got.ID, got.Occurrence, want.ID, want.Occurrence)
	}
	if got.Before != want.Before {
		t.Fatalf("%s before = %+v, want %+v", what, got.Before, want.Before)
	}
	approxEq(t, what+" after x", got.After.X, want.After.X)
	approxEq(t, what+" after y", got.After.Y, want.After.Y)
	if got.After.ID != want.After.ID || got.After.Count != want.After.Count {
		t.Fatalf("%s after = %+v, want id=%q count=%d",
			what, got.After, want.After.ID, want.After.Count)
	}
}

// expectPreSpanningAppearances 断言 L 的两次出现都保持提交 c-span 之前的
// 记录：旧出现已失效并保留失效信息，新出现仍有效，位置、计数、编号、首次
// 观测时间均为原值。
func expectPreSpanningAppearances(t *testing.T, m *Map) {
	t.Helper()
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatalf("LandmarkAppearances(L): %v", err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("L appearances = %+v, want exactly 2", h.Appearances)
	}
	expectAppearance(t, "L occ1", h.Appearances[0], Appearance{
		Number: 1, Active: false, FirstSeenTime: 100, HasFirstSeen: true,
		Landmark:      Landmark{ID: "L", X: 1, Y: 0, Count: 2},
		InvalidTime:   200,
		InvalidReason: "sign removed",
		InvalidOpID:   "inv-l",
	})
	expectAppearance(t, "L occ2", h.Appearances[1], Appearance{
		Number: 2, Active: true, FirstSeenTime: 300, HasFirstSeen: true,
		Landmark: Landmark{ID: "L", X: 10, Y: 0, Count: 2},
	})
}

// assertPreSpanningCorrectionState 断言地图完全处于提交 c-span 之前的状态，
// 失败回滚后与从未提交过 c-span 的参照地图都必须满足它。
func assertPreSpanningCorrectionState(t *testing.T, m *Map, c0 CorrectionRecord) {
	t.Helper()
	pre := wantPreSpanningPoses()

	// 当前位姿：提交前末帧位姿。
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatalf("CurrentPose: %v", err)
	}
	expectPose(t, "current pose", cur, pre[len(pre)-1])

	// 各帧时间的位姿（含锚点之前的更早历史）都保持原值；帧间查询借用
	// 最近一帧，也只能取得提交前的位姿。
	for _, want := range pre {
		p, err := m.PoseAt(want.Time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", want.Time, err)
		}
		expectPose(t, "pose at frame time", p, want)
	}
	for _, at := range []int64{50, 250, 350, 1 << 40} {
		p, err := m.PoseAt(at)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", at, err)
		}
		var want Pose
		for _, cand := range pre {
			if cand.Time <= at {
				want = cand
			}
		}
		expectPose(t, "borrowed pose", p, want)
	}

	// 区域查询：只有仍有效的第 2 次出现，且是提交前的位置与计数；已失效
	// 的旧出现仍被排除，失败校正后的坐标 (11,0) 一处也查不到。
	lms, err := m.LandmarksInRect(allRect())
	if err != nil {
		t.Fatalf("LandmarksInRect: %v", err)
	}
	if len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("active landmarks = %v, want only L (active occurrence)", lms)
	}
	if lms[0].Count != 2 {
		t.Fatalf("L count = %d, want unchanged 2", lms[0].Count)
	}
	approxEq(t, "L rect x", lms[0].X, 10)
	approxEq(t, "L rect y", lms[0].Y, 0)
	failedRect, err := m.LandmarksInRect(Rect{MinX: 10.5, MinY: -1, MaxX: 11.5, MaxY: 1})
	if err != nil {
		t.Fatalf("LandmarksInRect failed-correction area: %v", err)
	}
	if len(failedRect) != 0 {
		t.Fatalf("failed correction coordinates queryable after failed save: %v", failedRect)
	}

	// 两次出现的记录都保持提交前原值。
	expectPreSpanningAppearances(t, m)

	// 既有成功校正记录按原顺序保留、内容不被改写，失败记录没有被追加。
	recs, err := m.Corrections()
	if err != nil {
		t.Fatalf("Corrections: %v", err)
	}
	if len(recs) != 1 || recs[0].ID != "c0-var" {
		t.Fatalf("corrections = %v, want only [c0-var]", recs)
	}
	if !recordsEqual(recs[0], c0) {
		t.Fatalf("existing record rewritten: %+v want %+v", recs[0], c0)
	}
}

// assertSpanningCorrectionRecord 断言 c-span 首次成功提交生成的记录内容：
// 位姿与两次出现的校正前后值都与“以提交前状态为起点”的推演一致。
func assertSpanningCorrectionRecord(t *testing.T, rec CorrectionRecord) {
	t.Helper()
	if rec.ID != "c-span" || rec.Anchor != 200 || rec.EndTime != 400 {
		t.Fatalf("record header = %+v, want c-span anchor 200 end 400", rec)
	}
	if rec.Target != (CorrectionTarget{X: 2, Y: 0, Heading: 0, Variance: 1.0}) {
		t.Fatalf("record target = %+v", rec.Target)
	}
	if len(rec.Poses) != 3 {
		t.Fatalf("record poses = %+v, want 3", rec.Poses)
	}
	expectPoseChange(t, 0, rec.Poses[0],
		Pose{Time: 200, X: 1, Y: 0, Heading: 0, Variance: 0.9},
		Pose{Time: 200, X: 2, Y: 0, Heading: 0, Variance: 1.0})
	expectPoseChange(t, 1, rec.Poses[1],
		Pose{Time: 300, X: 2, Y: 0, Heading: 0, Variance: 1.0},
		Pose{Time: 300, X: 3, Y: 0, Heading: 0, Variance: 1.1})
	expectPoseChange(t, 2, rec.Poses[2],
		Pose{Time: 400, X: 3, Y: 0, Heading: 0, Variance: 1.1},
		Pose{Time: 400, X: 4, Y: 0, Heading: 0, Variance: 1.2})
	if len(rec.Landmarks) != 2 {
		t.Fatalf("record landmarks = %+v, want 2 occurrences of L", rec.Landmarks)
	}
	expectLandmarkChange(t, "L occ1", rec.Landmarks[0], LandmarkChange{
		ID: "L", Occurrence: 1,
		Before: Landmark{ID: "L", X: 1, Y: 0, Count: 2},
		After:  Landmark{ID: "L", X: 1.5, Y: 0, Count: 2},
	})
	expectLandmarkChange(t, "L occ2", rec.Landmarks[1], LandmarkChange{
		ID: "L", Occurrence: 2,
		Before: Landmark{ID: "L", X: 10, Y: 0, Count: 2},
		After:  Landmark{ID: "L", X: 11, Y: 0, Count: 2},
	})
}

// expectPostSpanningPoses 断言 c-span 成功后的全部位姿：锚点至末帧平移
// +1、方差按目标方差加原运动方差累计；锚点之前的更早历史保持原值。
func expectPostSpanningPoses(t *testing.T, m *Map) {
	t.Helper()
	for _, want := range []Pose{
		{Time: 0, X: 0, Y: 0, Heading: 0, Variance: 0.5},
		{Time: 100, X: 0, Y: 0, Heading: 0, Variance: 0.8},
		{Time: 200, X: 2, Y: 0, Heading: 0, Variance: 1.0},
		{Time: 300, X: 3, Y: 0, Heading: 0, Variance: 1.1},
		{Time: 400, X: 4, Y: 0, Heading: 0, Variance: 1.2},
	} {
		p, err := m.PoseAt(want.Time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", want.Time, err)
		}
		expectPose(t, "post-correction pose", p, want)
	}
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatalf("CurrentPose: %v", err)
	}
	expectPose(t, "post-correction current", cur,
		Pose{Time: 400, X: 4, Y: 0, Heading: 0, Variance: 1.2})
}

// expectPostSpanningAppearances 断言 c-span 成功后 L 的两次出现：旧出现
// 位置随校正改变但仍失效、失效信息原样保留，新出现位置随校正改变且仍
// 有效；两次出现的观测计数都不因校正增加。
func expectPostSpanningAppearances(t *testing.T, m *Map) {
	t.Helper()
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatalf("LandmarkAppearances(L): %v", err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("L appearances = %+v, want exactly 2", h.Appearances)
	}
	expectAppearance(t, "L occ1 post", h.Appearances[0], Appearance{
		Number: 1, Active: false, FirstSeenTime: 100, HasFirstSeen: true,
		Landmark:      Landmark{ID: "L", X: 1.5, Y: 0, Count: 2},
		InvalidTime:   200,
		InvalidReason: "sign removed",
		InvalidOpID:   "inv-l",
	})
	expectAppearance(t, "L occ2 post", h.Appearances[1], Appearance{
		Number: 2, Active: true, FirstSeenTime: 300, HasFirstSeen: true,
		Landmark: Landmark{ID: "L", X: 11, Y: 0, Count: 2},
	})
}

// TestCorrectionSaveFailureRollsBackBothOccurrences 验证：锚点位于旧出现
// 观测范围内、受影响帧同时包含失效旧出现与有效新出现观测的合法校正，仅因
// 本地地图文件无法更新而保存失败时，整次校正失败、无任何可查询的部分结果；
// 恢复后以同一标识同一内容重试按首次成功处理，结果与该校正从未经历失败、
// 直接成功提交完全一致。
func TestCorrectionSaveFailureRollsBackBothOccurrences(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m, c0 := seedSpanningCorrectionMap(t, path)
	t.Cleanup(func() { _ = m.Close() })

	// ---- 制造保存失败并提交合法校正 ----
	breakSavingAtPath(t, path)
	req := spanningCorrectionRequest()
	rec, err := m.Correct(req)
	if err == nil {
		t.Fatal("correction with unupdatable map file unexpectedly succeeded")
	}
	// 必须是保存错误：不能是 *RejectError（不能把合法校正误报成锚点不
	// 存在、路标冲突等），也不能返回成功校正记录。
	if r, ok := AsRejectError(err); ok {
		t.Fatalf("valid correction rejected as %s: %v", r.Kind, err)
	}
	if rec.ID != "" || rec.Anchor != 0 || rec.Poses != nil || rec.Landmarks != nil {
		t.Fatalf("failed correction returned record %+v, want zero record", rec)
	}

	// ---- 失败后：位姿、两次出现、既有校正记录都完整保持提交前状态 ----
	assertPreSpanningCorrectionState(t, m, c0)

	// ---- 恢复保存条件，以同一标识同一内容再次提交 ----
	restoreSavingAtPath(t, m, path)
	rec, err = m.Correct(req)
	if err != nil {
		t.Fatalf("retry same correction id after save restored: %v", err)
	}
	// 按首次成功提交处理：记录以失败前的状态为起点生成，曾暂存的位移
	// 不会叠加第二次（锚点精确落在目标 (2,0)，而不是 (3,0)）。
	assertSpanningCorrectionRecord(t, rec)

	// 校正后的实际状态与记录一致：位姿只平移一次，锚点之前的历史不变。
	expectPostSpanningPoses(t, m)
	expectPostSpanningAppearances(t, m)

	// 区域查询现在给出新出现的校正后位置；旧出现仍被排除。
	lms, err := m.LandmarksInRect(allRect())
	if err != nil {
		t.Fatalf("LandmarksInRect after retry: %v", err)
	}
	if len(lms) != 1 || lms[0].ID != "L" || lms[0].Count != 2 {
		t.Fatalf("active landmarks after retry = %v, want only L count 2", lms)
	}
	approxEq(t, "L rect x after retry", lms[0].X, 11)
	approxEq(t, "L rect y after retry", lms[0].Y, 0)

	// 校正记录：既有记录仍在首位且内容不改写，本次成功记录追加在后。
	recs, err := m.Corrections()
	if err != nil {
		t.Fatalf("Corrections after retry: %v", err)
	}
	if len(recs) != 2 || recs[0].ID != "c0-var" || recs[1].ID != "c-span" {
		t.Fatalf("corrections after retry = %v, want [c0-var c-span]", recs)
	}
	if !recordsEqual(recs[0], c0) {
		t.Fatalf("existing record rewritten after retry: %+v want %+v", recs[0], c0)
	}
	if !recordsEqual(recs[1], rec) {
		t.Fatalf("saved record = %+v, want returned %+v", recs[1], rec)
	}

	// ---- 与“同内容直接成功提交、从未经历失败”的参照地图逐值比对 ----
	refPath := filepath.Join(t.TempDir(), "ref.pose")
	ref, c0Ref := seedSpanningCorrectionMap(t, refPath)
	t.Cleanup(func() { _ = ref.Close() })
	if !recordsEqual(c0Ref, c0) {
		t.Fatalf("reference c0 record = %+v, want %+v", c0Ref, c0)
	}
	refRec, err := ref.Correct(spanningCorrectionRequest())
	if err != nil {
		t.Fatalf("reference correction: %v", err)
	}
	if !recordsEqual(refRec, rec) {
		t.Fatalf("reference record = %+v, want %+v", refRec, rec)
	}
	for _, at := range []int64{0, 100, 200, 300, 400} {
		got, gerr := m.PoseAt(at)
		want, werr := ref.PoseAt(at)
		if gerr != nil || werr != nil {
			t.Fatalf("PoseAt(%d) errors: %v %v", at, gerr, werr)
		}
		expectPose(t, "parity pose vs reference", got, want)
	}
	gotH, _ := m.LandmarkAppearances("L")
	wantH, _ := ref.LandmarkAppearances("L")
	if len(gotH.Appearances) != len(wantH.Appearances) {
		t.Fatalf("L occurrence count = %d, want %d", len(gotH.Appearances), len(wantH.Appearances))
	}
	for i := range gotH.Appearances {
		expectAppearance(t, "parity L appearance", gotH.Appearances[i], wantH.Appearances[i])
	}
	refRecs, _ := ref.Corrections()
	if len(refRecs) != 2 {
		t.Fatalf("reference corrections = %v, want 2", refRecs)
	}
	for i := range refRecs {
		if !recordsEqual(recs[i], refRecs[i]) {
			t.Fatalf("correction %d = %+v, want reference %+v", i, recs[i], refRecs[i])
		}
	}

	// 正常校正行为保持不变：同标识同内容重复提交直接返回首次记录。
	again, err := m.Correct(req)
	if err != nil {
		t.Fatalf("duplicate correction after retry: %v", err)
	}
	if !recordsEqual(again, rec) {
		t.Fatalf("duplicate returned %+v, want %+v", again, rec)
	}
	if recs, _ := m.Corrections(); len(recs) != 2 {
		t.Fatalf("duplicate added a record: %v", recs)
	}

	// ---- 关闭重开：落盘内容与内存一致，失败的那次没落任何痕迹 ----
	if err := m.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}
	reopened, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	defer reopened.Close()
	cur, err := reopened.CurrentPose()
	if err != nil {
		t.Fatalf("reopened CurrentPose: %v", err)
	}
	expectPose(t, "reopened current pose", cur,
		Pose{Time: 400, X: 4, Y: 0, Heading: 0, Variance: 1.2})
	expectPostSpanningAppearances(t, reopened)
	reRecs, _ := reopened.Corrections()
	if len(reRecs) != 2 || reRecs[0].ID != "c0-var" || reRecs[1].ID != "c-span" {
		t.Fatalf("reopened corrections = %v, want [c0-var c-span]", reRecs)
	}
	if !recordsEqual(reRecs[0], c0) || !recordsEqual(reRecs[1], rec) {
		t.Fatalf("reopened records = %v", reRecs)
	}
	// 重开后用同一标识同一内容再提交，仍按已保存校正返回首次记录而非
	// 被误判成冲突——证明失败尝试没有占用校正标识。
	dup, err := reopened.Correct(spanningCorrectionRequest())
	if err != nil {
		t.Fatalf("duplicate correction after reopen: %v", err)
	}
	if !recordsEqual(dup, rec) {
		t.Fatalf("reopened duplicate record = %+v, want %+v", dup, rec)
	}
}
