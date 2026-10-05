package posemap

import (
	"math"
	"os"
	"path/filepath"
	"testing"
)

// 本文件为“整段轨迹在保存失败时整段失败、已确认数据保持导入前状态”这一
// 已有行为提供回归保障。段内同时包含两类被现有规则接受的路标变化：
//
//   - 一个仍有效的路标在本段继续合并观测，其平均位置与计数会被改变；
//   - 一个此前已失效的路标在与旧位置明显不同的地点再次出现，按“失效后
//     再现即新的一次出现”规则接受，会新增一条出现记录并消耗下一个编号。
//
// 这两种变化都先发生在内存暂存/提交阶段，随后才落盘。当本地地图文件
// 无法更新（保存路径不可写，rename 失败）时，导入必须返回保存错误而不是
// 成功结果，也不能把这段完全合法的内容误报成时间倒退或路标冲突；上述
// 两种变化必须连同新位姿一起精确回滚，不留任何能被查询到的部分结果。
// 保存条件恢复后，用同一段标识再次提交同一内容应正常成功：前一次失败
// 不消耗段标识、不消耗出现编号，最终状态与直接成功导入这段内容一致。
//
// 只使用现有公开入口（Create/ImportSegment/Invalidate/CurrentPose/
// PoseAt/LandmarksInRect/LandmarkAppearances），配合与既有
// TestSaveFailureRollback/TestInvalidateSaveFailureRollback 相同的
// 包内保存故障注入（把地图路径换成目录使 saveReplace 的 rename 失败、
// 再用 saveReplace 恢复），不改变任何公开使用方式、错误约定与正常导入
// 行为。
//
// 统一几何（初始位姿 (0,0)、朝向 0、初始方差 1、合并上限 10）：
//
//	p0   t=0     (0,0)   朝向 0，方差 1
//	s1（此前成功保存的轨迹）：
//	f100 t=100   原地     A 本地 (1,0) → (1,0)；B 本地 (2,0) → (2,0)
//	f200 t=200   原地     A 本地 (1,0) → (1,0)
//	  → A 两次观测均在 (1,0)：均值 (1,0)、计数 2、首次观测 t=100；
//	    B 一次观测 (2,0)、计数 1、首次观测 t=100。
//	t=200 提交失效操作 op-gone（原因 sign-removed）：B 的第 1 次出现
//	  失效，失效时间 200；A 仍有效。
//
//	s3（待导入段，连续三帧非平凡运动；两种变化同段发生）：
//	f300 t=300   沿自身 X 走 3，方差 +0.3 → (3,0)，朝向 0，方差 1.5
//	      A 本地 (1,0) → (4,0)；B 本地 (0,0) → (3,0)
//	f400 t=400   沿自身 X 走 3，方差 +0.4 → (6,0)，朝向 0，方差 1.9
//	      A 本地 (1,0) → (7,0)；B 本地 (0,0) → (6,0)
//	f500 t=500   沿自身 X 走 3，方差 +0.5 → (9,0)，朝向 0，方差 2.4
//	      A 本地 (1,0) → (10,0)
//
// A 仍有效：以已提交的两次 (1,0) 为固定起点，本段三条 (4,0)/(7,0)/
// (10,0) 与当时均值的距离依次为 3、4、4.5，均不超过合并上限 10，全部
// 接受；合并后均值 ((1+1+4+7+10)/5, 0) = (4.6,0)，计数 5。
// B 已失效：旧出现 (2,0) 不参与合并，本段在 (3,0)/(6,0) 的观测属于全新
// 的第 2 次出现（与旧位置相距至少 1，明显不同但因新出现不做距离判定而
// 被接受），首条在 t=300、计数 2，均值 (4.5,0)。
//
// 保存失败回滚后：A 仍是 (1,0)、计数 2；B 仍只有第 1 次失效出现，
// 位置 (2,0)、计数 1、失效时间 200 与原因不变；当前位姿仍停在 t=200。
// 恢复后以同一标识 "s3" 成功导入即得到上述 A=(4.6,0)/5、B 第 2 次出现
// (4.5,0)/2/首次 t=300，末位姿 t=500。

// importRollbackConfig 是这组场景的地图配置：原点初始位姿、非零初始
// 方差、足够大的合并上限。
func importRollbackConfig() Config {
	return Config{InitialTime: 0, InitialVariance: 1.0, MaxInterval: 1000, MergeDistance: 10.0}
}

// importRollbackHistory 构造此前已成功保存的轨迹与失效状态：A 为仍有
// 两次观测的有效路标，B 为仅有一次观测、在 t=200 失效的路标。返回末
// 位姿（即导入前状态）供断言对照。
func importRollbackHistory(t *testing.T, m *Map) Pose {
	t.Helper()
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, MoveVariance: 0.1, Observations: []Observation{
			{ID: "A", X: 1, Y: 0},
			{ID: "B", X: 2, Y: 0},
		}},
		{Time: 200, MoveVariance: 0.1, Observations: []Observation{
			{ID: "A", X: 1, Y: 0},
		}},
	}}); err != nil {
		t.Fatalf("seed ImportSegment: %v", err)
	}
	if _, err := m.Invalidate(Invalidation{
		ID: "op-gone", Reason: "sign-removed", Landmarks: []string{"B"},
	}); err != nil {
		t.Fatalf("seed Invalidate: %v", err)
	}
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	return cur
}

// importRollbackSegment 构造待导入段：连续三帧沿 X 正向的非平凡运动，
// 有效路标 A 在每帧继续被观测（改变其均值与计数），已失效路标 B 在新
// 地点于前两帧再次出现（新增一次出现）。这些观测按现有规则均可接受。
func importRollbackSegment() Segment {
	return Segment{ID: "s3", Frames: []Frame{
		{Time: 300, DX: 3, DY: 0, MoveVariance: 0.3, Observations: []Observation{
			{ID: "A", X: 1, Y: 0},
			{ID: "B", X: 0, Y: 0},
		}},
		{Time: 400, DX: 3, DY: 0, MoveVariance: 0.4, Observations: []Observation{
			{ID: "A", X: 1, Y: 0},
			{ID: "B", X: 0, Y: 0},
		}},
		{Time: 500, DX: 3, DY: 0, MoveVariance: 0.5, Observations: []Observation{
			{ID: "A", X: 1, Y: 0},
		}},
	}}
}

// makeSaveFail 让后续 saveReplace 必然失败：把地图数据文件路径替换成
// 同名目录，rename 到一个目录会返回 EISDIR。与既有
// TestSaveFailureRollback/TestInvalidateSaveFailureRollback 的注入方式
// 相同。
func makeSaveFail(t *testing.T, m *Map) {
	t.Helper()
	if err := os.Remove(m.path); err != nil {
		t.Fatalf("remove map file before fault: %v", err)
	}
	if err := os.Mkdir(m.path, 0o755); err != nil {
		t.Fatalf("replace map file with directory: %v", err)
	}
}

// restoreSave 移除占据地图路径的目录，并把当前内存状态落盘，使后续保存
// 条件恢复正常。
func restoreSave(t *testing.T, m *Map) {
	t.Helper()
	if err := os.Remove(m.path); err != nil {
		t.Fatalf("remove directory to restore save: %v", err)
	}
	if err := m.saveReplace(); err != nil {
		t.Fatalf("saveReplace after restore: %v", err)
	}
}

// assertPreImportState 按任务要求逐项断言地图仍是“导入前”状态：
// 当前/历史位姿、区域查询结果、两个路标的历次出现与失效信息。
func assertPreImportState(t *testing.T, m *Map, pre Pose) {
	t.Helper()

	// 当前位姿仍是导入前末位姿：时间、位置、朝向、方差均未变化。
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	if cur != pre {
		t.Fatalf("current pose after failed save = %+v, want unchanged %+v", cur, pre)
	}

	// 按失败段内的帧时间（300/400/500）查询历史位姿：这些帧不存在，
	// “不晚于 t 的最后一份位姿”仍是导入前最后一份位姿（t=200）。
	for _, ft := range []int64{300, 400, 500} {
		p, err := m.PoseAt(ft)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", ft, err)
		}
		if p != pre {
			t.Fatalf("PoseAt(%d) = %+v, want pre-import last pose %+v", ft, p, pre)
		}
	}
	// 导入前各历史时间也仍可查且不变（方差累加含浮点误差，按容差比较）。
	for _, want := range []Pose{
		{Time: 0, X: 0, Y: 0, Heading: 0, Variance: 1.0},
		{Time: 100, X: 0, Y: 0, Heading: 0, Variance: 1.1},
		{Time: 200, X: 0, Y: 0, Heading: 0, Variance: 1.2},
	} {
		p, err := m.PoseAt(want.Time)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", want.Time, err)
		}
		expectPose(t, "pre-import history", p, want)
	}

	// 区域查询只显示仍有效的 A，且为原来的位置与次数；不显示失效的 B，
	// 也不显示 B 在失败段中的新出现（若 B 被错误地新建为有效出现，这里
	// 会一并暴露）。
	lms, err := m.LandmarksInRect(allRect())
	if err != nil {
		t.Fatal(err)
	}
	a, ok := findLandmark(lms, "A")
	if !ok {
		t.Fatalf("active landmark A missing after failed save: %v", lms)
	}
	if a.X != 1 || a.Y != 0 || a.Count != 2 {
		t.Fatalf("A after failed save = %+v, want (1,0) count 2", a)
	}
	if _, ok := findLandmark(lms, "B"); ok {
		t.Fatalf("invalidated B still appears in region query after failed save: %v", lms)
	}

	// A 的历次出现：仍只有第 1 次且有效，位置/次数/首次观测时间不变，
	// 没有混入本段观测。
	hA, err := m.LandmarkAppearances("A")
	if err != nil {
		t.Fatal(err)
	}
	if len(hA.Appearances) != 1 {
		t.Fatalf("A appearances after failed save = %+v, want exactly 1", hA.Appearances)
	}
	a1 := hA.Appearances[0]
	if a1.Number != 1 || !a1.Active || !a1.HasFirstSeen || a1.FirstSeenTime != 100 {
		t.Fatalf("A occurrence 1 after failed save = %+v", a1)
	}
	if a1.Landmark.X != 1 || a1.Landmark.Y != 0 || a1.Landmark.Count != 2 {
		t.Fatalf("A occurrence 1 landmark = %+v, want (1,0) count 2", a1.Landmark)
	}

	// B 的历次出现：仍只有第 1 次，保持失效，位置、次数、失效时间、原因、
	// 失效操作标识完整保留；没有增加第 2 次出现，也没有被重新标成有效。
	hB, err := m.LandmarkAppearances("B")
	if err != nil {
		t.Fatal(err)
	}
	if len(hB.Appearances) != 1 {
		t.Fatalf("B appearances after failed save = %+v, want exactly 1 (no new occurrence)", hB.Appearances)
	}
	b1 := hB.Appearances[0]
	if b1.Number != 1 || b1.Active {
		t.Fatalf("B occurrence 1 must remain inactive: %+v", b1)
	}
	if b1.Landmark.X != 2 || b1.Landmark.Y != 0 || b1.Landmark.Count != 1 {
		t.Fatalf("B occurrence 1 landmark = %+v, want (2,0) count 1", b1.Landmark)
	}
	if !b1.HasFirstSeen || b1.FirstSeenTime != 100 {
		t.Fatalf("B occurrence 1 first seen = %+v, want known at 100", b1)
	}
	if b1.InvalidTime != 200 || b1.InvalidReason != "sign-removed" || b1.InvalidOpID != "op-gone" {
		t.Fatalf("B occurrence 1 invalid info = time %d reason %q op %q, want 200/sign-removed/op-gone",
			b1.InvalidTime, b1.InvalidReason, b1.InvalidOpID)
	}
}

// TestImportSegmentSaveFailureRollsBackBothOccurrenceKinds 是核心回归：
// 同一段里既有有效路标的继续合并，又有失效路标的异地再现，保存失败时
// 必须整段失败并把两类变化都回滚干净；恢复后同段标识同内容可成功，且
// 结果与直接成功导入完全一致。
func TestImportSegmentSaveFailureRollsBackBothOccurrenceKinds(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m, err := Create(path, importRollbackConfig())
	if err != nil {
		t.Fatal(err)
	}
	defer m.Close()

	pre := importRollbackHistory(t, m)
	expectPose(t, "seed end pose", pre, Pose{Time: 200, X: 0, Y: 0, Heading: 0, Variance: 1.2})
	seg := importRollbackSegment()

	// ---- 注入保存故障：路径被同名目录占据，saveReplace 的 rename 失败 ----
	makeSaveFail(t, m)

	// 段内容完全合法（时间、数值、观测距离校验都通过），失败只应来自
	// 保存：返回普通错误而不是成功结果，也绝不能是 *RejectError
	// （time_order / landmark_conflict 等语义拒绝）。
	res, err := m.ImportSegment(seg)
	if err == nil {
		t.Fatalf("ImportSegment unexpectedly succeeded with save fault; result = %+v", res)
	}
	if r, ok := AsRejectError(err); ok {
		t.Fatalf("valid segment rejected with *RejectError %+v: %v", r, err)
	}

	// 失败后地图仍是导入前状态，两类变化都没有留下可查询的部分结果。
	assertPreImportState(t, m, pre)

	// ---- 恢复保存条件，用同一段标识再次提交同一内容 ----
	restoreSave(t, m)

	res2, err := m.ImportSegment(seg)
	if err != nil {
		t.Fatalf("ImportSegment retry with same id after save restored: %v", err)
	}
	// 返回成功结果：末位姿即本段末帧，路标集合为段内触及的 A 与 B（排序）。
	expectPose(t, "retry end pose", res2.EndPose, Pose{Time: 500, X: 9, Y: 0, Heading: 0, Variance: 2.4})
	if len(res2.LandmarkIDs) != 2 || res2.LandmarkIDs[0] != "A" || res2.LandmarkIDs[1] != "B" {
		t.Fatalf("retry landmark ids = %v, want [A B]", res2.LandmarkIDs)
	}

	// 成功后当前位姿推进到本段末帧。
	cur, _ := m.CurrentPose()
	if cur != res2.EndPose {
		t.Fatalf("current pose = %+v, want result end pose %+v", cur, res2.EndPose)
	}

	// 区域查询：A 仍是有效路标，只增加本段实际的 3 次观测（计数 5），
	// 平均位置与直接成功导入一致；B 以新的第 2 次有效出现回到区域查询，
	// 计数只含本段实际观测（2），位置为 (3,0)/(6,0) 的均值 (4.5,0)。
	lms, _ := m.LandmarksInRect(allRect())
	a, ok := findLandmark(lms, "A")
	if !ok {
		t.Fatalf("A missing from region query after retry: %v", lms)
	}
	approxEq(t, "A x after retry", a.X, 4.6)
	approxEq(t, "A y after retry", a.Y, 0)
	if a.Count != 5 {
		t.Fatalf("A count after retry = %d, want 5 (2 prior + 3 in segment)", a.Count)
	}
	b, ok := findLandmark(lms, "B")
	if !ok {
		t.Fatalf("B new occurrence missing from region query after retry: %v", lms)
	}
	approxEq(t, "B x after retry", b.X, 4.5)
	approxEq(t, "B y after retry", b.Y, 0)
	if b.Count != 2 {
		t.Fatalf("B count after retry = %d, want 2 (only this segment's observations)", b.Count)
	}

	// A 仍只有一次出现，位置/次数更新为本段合并结果，首次观测时间不变。
	hA, _ := m.LandmarkAppearances("A")
	if len(hA.Appearances) != 1 {
		t.Fatalf("A appearances after retry = %+v, want exactly 1", hA.Appearances)
	}
	a1 := hA.Appearances[0]
	if !a1.Active || a1.FirstSeenTime != 100 {
		t.Fatalf("A occurrence 1 after retry = %+v", a1)
	}
	approxEq(t, "A occurrence x after retry", a1.Landmark.X, 4.6)
	approxEq(t, "A occurrence y after retry", a1.Landmark.Y, 0)
	if a1.Landmark.Count != 5 {
		t.Fatalf("A occurrence count after retry = %d, want 5", a1.Landmark.Count)
	}

	// B 现在有两次出现：第 1 次仍保持失效、位置/次数/失效信息不变，且不
	// 参与新位置合并；第 2 次为本次新出现，编号接在旧记录之后，计数从
	// 本次实际观测重新开始（2），首次观测时间取本段中它首次被观测的
	// 帧时间 300。
	hB, _ := m.LandmarkAppearances("B")
	if len(hB.Appearances) != 2 {
		t.Fatalf("B appearances after retry = %+v, want 2", hB.Appearances)
	}
	old := hB.Appearances[0]
	if old.Number != 1 || old.Active {
		t.Fatalf("B old occurrence must remain inactive: %+v", old)
	}
	if old.Landmark.X != 2 || old.Landmark.Y != 0 || old.Landmark.Count != 1 {
		t.Fatalf("B old occurrence landmark = %+v, want unchanged (2,0) count 1", old.Landmark)
	}
	if old.InvalidTime != 200 || old.InvalidReason != "sign-removed" || old.InvalidOpID != "op-gone" {
		t.Fatalf("B old occurrence invalid info = %d %q %q, want unchanged 200/sign-removed/op-gone",
			old.InvalidTime, old.InvalidReason, old.InvalidOpID)
	}
	nu := hB.Appearances[1]
	if nu.Number != 2 || !nu.Active {
		t.Fatalf("B new occurrence = %+v, want active number 2", nu)
	}
	if !nu.HasFirstSeen || nu.FirstSeenTime != 300 {
		t.Fatalf("B new occurrence first seen = %+v, want known at 300", nu)
	}
	approxEq(t, "B new occurrence x", nu.Landmark.X, 4.5)
	approxEq(t, "B new occurrence y", nu.Landmark.Y, 0)
	if nu.Landmark.Count != 2 {
		t.Fatalf("B new occurrence count = %d, want 2 (restarted from this segment)", nu.Landmark.Count)
	}
}

// TestImportSegmentSaveFailureDoesNotConsumeSegmentID 单独钉住“失败不消耗
// 段标识”：保存失败后立即以同一标识提交一段不同内容，也应被当作首次
// 提交正常校验（这里用合法内容验证标识未被占用）；随后再次重复提交则按
// 既有重复导入规则返回首次结果。
func TestImportSegmentSaveFailureDoesNotConsumeSegmentID(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m, err := Create(path, importRollbackConfig())
	if err != nil {
		t.Fatal(err)
	}
	defer m.Close()

	pre := importRollbackHistory(t, m)
	seg := importRollbackSegment()

	makeSaveFail(t, m)
	if _, err := m.ImportSegment(seg); err == nil {
		t.Fatal("ImportSegment unexpectedly succeeded with save fault")
	}
	assertPreImportState(t, m, pre)
	restoreSave(t, m)

	// 同标识、不同内容也能提交，说明失败没有把 "s3" 记成已保存段。
	alt := Segment{ID: "s3", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.2},
	}}
	if _, err := m.ImportSegment(alt); err != nil {
		t.Fatalf("same id with different content after failed save should be a fresh commit: %v", err)
	}
	// 再次以相同内容提交：走既有“相同内容直接返回首次结果”路径，成功且
	// 不再次改变状态。
	r1, err := m.ImportSegment(alt)
	if err != nil {
		t.Fatalf("identical re-import: %v", err)
	}
	r2, err := m.ImportSegment(alt)
	if err != nil {
		t.Fatalf("identical re-import again: %v", err)
	}
	if r1.EndPose != r2.EndPose {
		t.Fatalf("duplicate import end pose = %+v, want %+v", r2.EndPose, r1.EndPose)
	}
	cur, _ := m.CurrentPose()
	if cur.Time != 300 || math.Abs(cur.X-1) > 1e-12 {
		t.Fatalf("current pose after alt commit = %+v, want t=300 x=1", cur)
	}
}
