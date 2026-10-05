package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// 本文件为“整段导入在落盘失败时整体失败”补充一项回归保障，专门覆盖两种
// 路标变化发生在同一段内时的情形：既有仍有效的路标在本段继续合并观测
// （原地更新当前出现），同时一个此前已失效的路标在与旧位置明显不同的
// 地点再次被观测（追加一次全新出现）。本段内容本身完全合法——时间严格
// 递增、数值有限、观测距离满足合并上限——仅因本地地图文件无法更新而
// 保存失败。此时已确认数据必须保持导入前的状态：位姿、有效路标的均值
// 与计数、失效路标的旧出现（位置、次数、失效时间与原因）都不变，查不到
// 任何部分结果；失败返回的是保存错误而不是成功结果，也不能把合法内容
// 误报成时间倒退或路标冲突。保存条件恢复后，以同一段标识再次提交同一
// 内容必须正常成功（前一次失败不占用段标识），且结果与这段内容从未经历
// 失败、直接成功导入完全一致。
//
// 统一几何（初始位姿 (0,0)、朝向 0、初始方差 0.5、合并上限 5）：
//
// 已保存的种子段（段标识 seg-seed）：
//
//	t=100 位姿 (0,0)，方差 0.6：A 本地 (1,0) → 世界 (1,0)；B 本地 (2,0) → (2,0)
//	t=200 位姿 (0,0)，方差 0.7：A 再观测一次 (1,0)
//
// 因而 A 为仍有效的第 1 次出现，均值 (1,0)、计数 2、首次观测时间 100；
// B 同为第 1 次出现、计数 1，随后在 t=200 被失效（原因 sign removed），
// 旧出现保留在 (2,0)。
//
// 待导入段（段标识 seg-next）为连续四帧沿世界 X 轴的运动，朝向始终为 0：
//
//	t=300 DX=1 → (1,0)，方差 0.9：A 本地 (0,0) → (1,0)
//	t=400 DX=1 → (2,0)，方差 1.1：A 本地 (0,0) → (2,0)；
//	             B 本地 (0,10) → (2,10)（与旧位置 (2,0) 相距 10，明显不同）
//	t=500 DX=1 → (3,0)，方差 1.3：B 本地 (0,10) → (3,10)
//	t=600 DX=1 → (4,0)，方差 1.7：无观测
//
// 全部观测按现有规则均可接受：
//
//	A 以已提交的 (1,0)、计数 2 为固定起点：(1,0) 距离 0 并入后计数 3、
//	均值仍为 (1,0)；(2,0) 与均值相距 1 ≤ 5，并入后计数 4、均值 (1.25,0)。
//	B 的最近一次出现已失效，(2,10) 是第 2 次出现的首条观测（不做距离
//	判定），首次观测时间 400、计数 1；(3,10) 与 (2,10) 相距 1 ≤ 5，
//	并入后计数 2、均值 (2.5,10)。旧出现 (2,0) 不参与合并。
//
// 直接成功导入本段时末位姿为 t=600 (4,0)、朝向 0、方差 1.7，涉及路标
// [A B]。

// saveFailureMixedConfig 是这组场景的地图配置。
func saveFailureMixedConfig() Config {
	return Config{InitialTime: 0, InitialVariance: 0.5, MaxInterval: 1000, MergeDistance: 5.0}
}

// seedMixedLandmarkMap 在 path 处创建地图并保存好“种子段 + B 失效”的
// 前置状态，返回的地图当前位姿停在 t=200 (0,0)、方差 0.7：A 仍有效
// （(1,0)、计数 2），B 的第 1 次出现已失效（(2,0)、计数 1、失效时间 200）。
func seedMixedLandmarkMap(t *testing.T, path string) *Map {
	t.Helper()
	m, err := Create(path, saveFailureMixedConfig())
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	if _, err := m.ImportSegment(Segment{ID: "seg-seed", Frames: []Frame{
		{Time: 100, DX: 0, MoveVariance: 0.1, Observations: []Observation{
			{ID: "A", X: 1, Y: 0},
			{ID: "B", X: 2, Y: 0},
		}},
		{Time: 200, DX: 0, MoveVariance: 0.1, Observations: []Observation{
			{ID: "A", X: 1, Y: 0},
		}},
	}}); err != nil {
		t.Fatalf("seed import: %v", err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "inv-b", Reason: "sign removed", Landmarks: []string{"B"}}); err != nil {
		t.Fatalf("invalidate B: %v", err)
	}
	return m
}

// mixedLandmarkNextSegment 构造待导入的四帧合法轨迹段。
func mixedLandmarkNextSegment() Segment {
	return Segment{ID: "seg-next", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.2, Observations: []Observation{
			{ID: "A", X: 0, Y: 0},
		}},
		{Time: 400, DX: 1, MoveVariance: 0.2, Observations: []Observation{
			{ID: "A", X: 0, Y: 0},
			{ID: "B", X: 0, Y: 10},
		}},
		{Time: 500, DX: 1, MoveVariance: 0.2, Observations: []Observation{
			{ID: "B", X: 0, Y: 10},
		}},
		{Time: 600, DX: 1, MoveVariance: 0.4},
	}}
}

// breakSavingAtPath 让目标路径无法被原子替换：删掉地图文件并在原路径建
// 一个同名目录，saveReplace 的 rename 因此失败（EISDIR），而此前的地图
// 内容也不再能通过该路径更新。
func breakSavingAtPath(t *testing.T, path string) {
	t.Helper()
	if err := os.Remove(path); err != nil {
		t.Fatalf("remove map file: %v", err)
	}
	if err := os.Mkdir(path, 0o755); err != nil {
		t.Fatalf("mkdir over map path: %v", err)
	}
}

// restoreSavingAtPath 恢复保存条件：移除占位目录，并把当前内存状态重新
// 落盘为正常地图文件。
func restoreSavingAtPath(t *testing.T, m *Map, path string) {
	t.Helper()
	if err := os.Remove(path); err != nil {
		t.Fatalf("remove directory at map path: %v", err)
	}
	if err := m.saveReplace(); err != nil {
		t.Fatalf("restore map file: %v", err)
	}
}

// expectAppearance 断言某次出现的编号、有效状态、首次观测时间、位置、
// 计数与失效信息与期望一致。
func expectAppearance(t *testing.T, what string, got Appearance, want Appearance) {
	t.Helper()
	if got.Number != want.Number || got.Active != want.Active ||
		got.HasFirstSeen != want.HasFirstSeen || got.FirstSeenTime != want.FirstSeenTime {
		t.Fatalf("%s header = %+v, want number=%d active=%t firstSeen(%t,%d)",
			what, got, want.Number, want.Active, want.HasFirstSeen, want.FirstSeenTime)
	}
	if got.Landmark.ID != want.Landmark.ID || got.Landmark.Count != want.Landmark.Count {
		t.Fatalf("%s landmark = %+v, want id=%q count=%d",
			what, got.Landmark, want.Landmark.ID, want.Landmark.Count)
	}
	approxEq(t, what+" x", got.Landmark.X, want.Landmark.X)
	approxEq(t, what+" y", got.Landmark.Y, want.Landmark.Y)
	if got.InvalidTime != want.InvalidTime || got.InvalidReason != want.InvalidReason ||
		got.InvalidOpID != want.InvalidOpID {
		t.Fatalf("%s invalid info = (%d,%q,%q), want (%d,%q,%q)",
			what, got.InvalidTime, got.InvalidReason, got.InvalidOpID,
			want.InvalidTime, want.InvalidReason, want.InvalidOpID)
	}
}

// wantPreImportPose 是导入前段末位姿（也是失败后所有失败段帧时间的
// PoseAt 查询应取得的位姿）。
func wantPreImportPose() Pose {
	return Pose{Time: 200, X: 0, Y: 0, Heading: 0, Variance: 0.7}
}

// assertPreImportState 断言地图完全处于“种子段 + B 失效”的导入前状态，
// 失败回滚后与从未导入过 seg-next 的参照地图都必须满足它。
func assertPreImportState(t *testing.T, m *Map) {
	t.Helper()
	pre := wantPreImportPose()

	// 当前位姿：时间、位置、朝向、方差均为导入前末位姿。
	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatalf("CurrentPose: %v", err)
	}
	expectPose(t, "current pose", cur, pre)

	// 按失败段内各帧时间（以及段前缝隙）查询，都只能取得导入前最后一份
	// 位姿；更早的历史位姿也保持原样。
	for _, at := range []int64{250, 300, 400, 500, 600, 1 << 40} {
		p, err := m.PoseAt(at)
		if err != nil {
			t.Fatalf("PoseAt(%d): %v", at, err)
		}
		expectPose(t, "pose after failed segment time", p, pre)
	}
	p100, err := m.PoseAt(100)
	if err != nil {
		t.Fatalf("PoseAt(100): %v", err)
	}
	expectPose(t, "earlier pose", p100, Pose{Time: 100, X: 0, Y: 0, Heading: 0, Variance: 0.6})

	// 区域查询：只有仍有效的 A，且是原来的位置与计数；B 已失效不显示，
	// B 的新出现位置 (2..3, 9..11) 一处也查不到。
	lms, err := m.LandmarksInRect(allRect())
	if err != nil {
		t.Fatalf("LandmarksInRect: %v", err)
	}
	if len(lms) != 1 || lms[0].ID != "A" {
		t.Fatalf("active landmarks = %v, want only A", lms)
	}
	if lms[0].Count != 2 {
		t.Fatalf("A count = %d, want unchanged 2", lms[0].Count)
	}
	approxEq(t, "A x", lms[0].X, 1)
	approxEq(t, "A y", lms[0].Y, 0)
	newBRect, err := m.LandmarksInRect(Rect{MinX: 2, MinY: 9, MaxX: 3, MaxY: 11})
	if err != nil {
		t.Fatalf("LandmarksInRect new B area: %v", err)
	}
	if len(newBRect) != 0 {
		t.Fatalf("new B occurrence queryable after failed save: %v", newBRect)
	}

	// A：仍是唯一一次有效出现，位置与计数不变。
	hA, err := m.LandmarkAppearances("A")
	if err != nil {
		t.Fatalf("LandmarkAppearances(A): %v", err)
	}
	if len(hA.Appearances) != 1 {
		t.Fatalf("A appearances = %+v, want exactly 1", hA.Appearances)
	}
	expectAppearance(t, "A occ1", hA.Appearances[0], Appearance{
		Number: 1, Active: true, FirstSeenTime: 100, HasFirstSeen: true,
		Landmark: Landmark{ID: "A", X: 1, Y: 0, Count: 2},
	})

	// B：仍只有旧的第 1 次出现，保留自己的位置、次数、失效时间与原因，
	// 没有新增出现编号，也没有被重新标成有效。
	hB, err := m.LandmarkAppearances("B")
	if err != nil {
		t.Fatalf("LandmarkAppearances(B): %v", err)
	}
	if len(hB.Appearances) != 1 {
		t.Fatalf("B appearances = %+v, want exactly 1 (old, invalidated)", hB.Appearances)
	}
	expectAppearance(t, "B occ1", hB.Appearances[0], Appearance{
		Number: 1, Active: false, FirstSeenTime: 100, HasFirstSeen: true,
		Landmark:      Landmark{ID: "B", X: 2, Y: 0, Count: 1},
		InvalidTime:   200,
		InvalidReason: "sign removed",
		InvalidOpID:   "inv-b",
	})
}

// TestImportSegmentSaveFailureRollsBackMergedAndReappearingLandmarks 验证：
// 一段同时“给有效路标合并新观测”和“让失效路标重新出现”的合法轨迹，仅因
// 本地地图文件无法更新而保存失败时，整段失败、无任何可查询的部分结果；
// 恢复后以同一标识重试成功，结果与该内容直接成功导入完全一致。
func TestImportSegmentSaveFailureRollsBackMergedAndReappearingLandmarks(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m := seedMixedLandmarkMap(t, path)
	t.Cleanup(func() { _ = m.Close() })

	// ---- 制造保存失败并提交合法段 ----
	breakSavingAtPath(t, path)
	seg := mixedLandmarkNextSegment()
	res, err := m.ImportSegment(seg)
	if err == nil {
		t.Fatal("import with unupdatable map file unexpectedly succeeded")
	}
	// 必须是保存错误：不能是 *RejectError（不能把合法内容误报成时间倒退、
	// 路标冲突等），也不能返回成功导入结果。
	if r, ok := AsRejectError(err); ok {
		t.Fatalf("valid segment rejected as %s: %v", r.Kind, err)
	}
	if res.EndPose.Time != 0 || res.LandmarkIDs != nil {
		t.Fatalf("failed import returned result %+v, want zero result", res)
	}

	// ---- 失败后：已确认数据完整保持导入前状态 ----
	assertPreImportState(t, m)

	// ---- 恢复保存条件，以同一段标识提交同一内容 ----
	restoreSavingAtPath(t, m, path)
	res, err = m.ImportSegment(seg)
	if err != nil {
		t.Fatalf("retry same segment id after save restored: %v", err)
	}
	// 有效路标只增加本段实际观测的次数；失效路标只新增下一次出现。
	expectPose(t, "retry end pose", res.EndPose,
		Pose{Time: 600, X: 4, Y: 0, Heading: 0, Variance: 1.7})
	if len(res.LandmarkIDs) != 2 || res.LandmarkIDs[0] != "A" || res.LandmarkIDs[1] != "B" {
		t.Fatalf("retry landmark ids = %v, want [A B]", res.LandmarkIDs)
	}

	// 末位姿与段内各帧均已生效，数值与直接推演一致。
	cur, _ := m.CurrentPose()
	expectPose(t, "current after retry", cur, res.EndPose)
	for _, want := range []Pose{
		{Time: 300, X: 1, Y: 0, Heading: 0, Variance: 0.9},
		{Time: 400, X: 2, Y: 0, Heading: 0, Variance: 1.1},
		{Time: 500, X: 3, Y: 0, Heading: 0, Variance: 1.3},
		{Time: 600, X: 4, Y: 0, Heading: 0, Variance: 1.7},
	} {
		p, err := m.PoseAt(want.Time)
		if err != nil {
			t.Fatalf("PoseAt(%d) after retry: %v", want.Time, err)
		}
		expectPose(t, "pose after retry", p, want)
	}

	// A：计数只增加本段实际观测的 2 次（2 → 4），均值并入两条新观测。
	hA, _ := m.LandmarkAppearances("A")
	if len(hA.Appearances) != 1 {
		t.Fatalf("A appearances after retry = %+v, want 1", hA.Appearances)
	}
	expectAppearance(t, "A occ1 after retry", hA.Appearances[0], Appearance{
		Number: 1, Active: true, FirstSeenTime: 100, HasFirstSeen: true,
		Landmark: Landmark{ID: "A", X: 1.25, Y: 0, Count: 4},
	})

	// B：旧出现仍失效、不参与新位置合并；只新增第 2 次出现，编号接在旧
	// 记录之后，计数从本次实际观测重新开始，首次观测时间取本段首次看到
	// 它的帧时间 400。
	hB, _ := m.LandmarkAppearances("B")
	if len(hB.Appearances) != 2 {
		t.Fatalf("B appearances after retry = %+v, want 2", hB.Appearances)
	}
	expectAppearance(t, "B occ1 after retry", hB.Appearances[0], Appearance{
		Number: 1, Active: false, FirstSeenTime: 100, HasFirstSeen: true,
		Landmark:      Landmark{ID: "B", X: 2, Y: 0, Count: 1},
		InvalidTime:   200,
		InvalidReason: "sign removed",
		InvalidOpID:   "inv-b",
	})
	expectAppearance(t, "B occ2 after retry", hB.Appearances[1], Appearance{
		Number: 2, Active: true, FirstSeenTime: 400, HasFirstSeen: true,
		Landmark: Landmark{ID: "B", X: 2.5, Y: 10, Count: 2},
	})

	// 区域查询现在同时给出 A 的合并结果与 B 的新出现。
	lms, _ := m.LandmarksInRect(allRect())
	if len(lms) != 2 {
		t.Fatalf("active landmarks after retry = %v, want A and B", lms)
	}
	aLM, ok := findLandmark(lms, "A")
	if !ok || aLM.Count != 4 {
		t.Fatalf("A in rect = %+v ok=%t", aLM, ok)
	}
	approxEq(t, "A rect x", aLM.X, 1.25)
	bLM, ok := findLandmark(lms, "B")
	if !ok || bLM.Count != 2 {
		t.Fatalf("B in rect = %+v ok=%t", bLM, ok)
	}
	approxEq(t, "B rect x", bLM.X, 2.5)
	approxEq(t, "B rect y", bLM.Y, 10)

	// ---- 与“同内容直接成功导入、从未经历失败”的参照地图逐值比对 ----
	refPath := filepath.Join(t.TempDir(), "ref.pose")
	ref := seedMixedLandmarkMap(t, refPath)
	t.Cleanup(func() { _ = ref.Close() })
	refRes, err := ref.ImportSegment(mixedLandmarkNextSegment())
	if err != nil {
		t.Fatalf("reference import: %v", err)
	}
	expectPose(t, "reference end pose", refRes.EndPose, res.EndPose)
	if len(refRes.LandmarkIDs) != len(res.LandmarkIDs) {
		t.Fatalf("reference ids = %v, want %v", refRes.LandmarkIDs, res.LandmarkIDs)
	}
	for i, id := range refRes.LandmarkIDs {
		if res.LandmarkIDs[i] != id {
			t.Fatalf("ids differ at %d: %q vs %q", i, res.LandmarkIDs[i], id)
		}
	}
	for _, at := range []int64{0, 100, 200, 300, 400, 500, 600} {
		got, gerr := m.PoseAt(at)
		want, werr := ref.PoseAt(at)
		if gerr != nil || werr != nil {
			t.Fatalf("PoseAt(%d) errors: %v %v", at, gerr, werr)
		}
		expectPose(t, "parity pose vs reference", got, want)
	}
	for _, id := range []string{"A", "B"} {
		got, _ := m.LandmarkAppearances(id)
		want, _ := ref.LandmarkAppearances(id)
		if len(got.Appearances) != len(want.Appearances) {
			t.Fatalf("%s occurrence count = %d, want %d", id, len(got.Appearances), len(want.Appearances))
		}
		for i := range got.Appearances {
			expectAppearance(t, "parity "+id+" appearance", got.Appearances[i], want.Appearances[i])
		}
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

	// 正常导入行为保持不变：同段同内容重复提交直接返回首次结果。
	again, err := m.ImportSegment(mixedLandmarkNextSegment())
	if err != nil {
		t.Fatalf("duplicate import after retry: %v", err)
	}
	expectPose(t, "duplicate end pose", again.EndPose, res.EndPose)

	// ---- 关闭重开：落盘内容与内存一致，失败的那次没落任何痕迹 ----
	if err := m.Close(); err != nil {
		t.Fatalf("close: %v", err)
	}
	reopened, err := Open(path)
	if err != nil {
		t.Fatalf("reopen: %v", err)
	}
	defer reopened.Close()
	cur, _ = reopened.CurrentPose()
	expectPose(t, "reopened current pose", cur, res.EndPose)
	hB, _ = reopened.LandmarkAppearances("B")
	if len(hB.Appearances) != 2 || hB.Appearances[0].Active ||
		hB.Appearances[0].InvalidReason != "sign removed" ||
		!hB.Appearances[1].Active || hB.Appearances[1].FirstSeenTime != 400 ||
		hB.Appearances[1].Landmark.Count != 2 {
		t.Fatalf("B after reopen = %+v", hB.Appearances)
	}
	approxEq(t, "reopened B occ2 x", hB.Appearances[1].Landmark.X, 2.5)
	approxEq(t, "reopened B occ2 y", hB.Appearances[1].Landmark.Y, 10)
	hA, _ = reopened.LandmarkAppearances("A")
	if len(hA.Appearances) != 1 || !hA.Appearances[0].Active ||
		hA.Appearances[0].Landmark.Count != 4 {
		t.Fatalf("A after reopen = %+v", hA.Appearances)
	}
	approxEq(t, "reopened A x", hA.Appearances[0].Landmark.X, 1.25)

	// 重开后用同段标识再提交同一内容，仍按已保存段返回首次结果而非被
	// 误判成冲突——证明失败尝试没有污染段标识。
	dup, err := reopened.ImportSegment(mixedLandmarkNextSegment())
	if err != nil {
		t.Fatalf("duplicate import after reopen: %v", err)
	}
	if dup.EndPose.Time != 600 || len(dup.LandmarkIDs) != 2 {
		t.Fatalf("reopened duplicate result = %+v", dup)
	}
	// 陌生标识仍然不存在。
	if _, err := reopened.LandmarkAppearances("ZZ"); !errors.Is(err, ErrLandmarkNotFound) {
		t.Fatalf("unexpected landmark ZZ: %v", err)
	}
}
