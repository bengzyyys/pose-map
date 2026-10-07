package posemap

import (
	"path/filepath"
	"testing"
)

// 本文件为 LandmarksInRect 与已确认回环校正 Correct 组合使用补充回归保障：
// 校正提交后，区域查询判断“是否落在矩形内”所依据的必须是该次出现全部
// 已接受观测按校正后位姿重放得到的等权平均位置（不是某一帧位姿、最后一
// 条观测位置或校正目标位置），且查询只考虑当前仍有效的出现。全部用例只
// 经公开接口（ImportSegment/Correct/Invalidate/LandmarksInRect/
// LandmarkAppearances/Open）构造与核对，不改变既有接口、校正规则与查询
// 含义。
//
// 统一配置：初始位姿 (0,0)、朝向 0、合并距离 10——足以接纳设计内同一次
// 出现的观测，又能被刻意放大的校正触发冲突。观测本地坐标取 (0,±5)，
// 朝向始终为 0，因此地图坐标下的观测位置就是“当时位姿 + (0,±5)”，
// 便于直接核对平均位置。

func rectRegConfig() Config {
	return Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 10}
}

// mustRect 查询区域，查询本身（含空结果）必须无错误。
func mustRect(t *testing.T, m *Map, r Rect) []Landmark {
	t.Helper()
	lms, err := m.LandmarksInRect(r)
	if err != nil {
		t.Fatalf("LandmarksInRect %+v: %v", r, err)
	}
	return lms
}

// assertSortedByID 断言返回结果按标识升序（LandmarksInRect 的既有契约）。
func assertSortedByID(t *testing.T, lms []Landmark) {
	t.Helper()
	for i := 1; i < len(lms); i++ {
		if lms[i-1].ID >= lms[i].ID {
			t.Fatalf("landmarks not sorted by id at %d: %v", i, lms)
		}
	}
}

// rectAround 返回以 (cx,cy) 为中心、半边长 pad 的小矩形。
func rectAround(cx, cy, pad float64) Rect {
	return Rect{MinX: cx - pad, MinY: cy - pad, MaxX: cx + pad, MaxY: cy + pad}
}

// landmarksEqual 比较两次区域查询结果（标识、位置、次数、顺序）是否一致。
func landmarksEqual(a, b []Landmark) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}

// findLandmarkChange 在校正记录的路标变化列表中按标识查找。
func findLandmarkChange(changes []LandmarkChange, id string) (LandmarkChange, bool) {
	for _, c := range changes {
		if c.ID == id {
			return c, true
		}
	}
	return LandmarkChange{}, false
}

// TestRectAfterCorrectionMovesOutAndIn 保障：路标校正前在矩形内、校正后
// 平均位置移到矩形外时，原矩形立即查不到它；覆盖新平均位置的矩形能查到
// 它的标识、新位置与原观测次数。判定依据是全部已接受观测的平均位置，而
// 非锚点帧位姿、最后一条观测或校正目标位置。
func TestRectAfterCorrectionMovesOutAndIn(t *testing.T) {
	m := newMap(t, rectRegConfig())

	//	t=50  位姿 (0,0)  观测 R 本地 (0,0)  → 地图 (0,0)    （锚点之前，校正后固定）
	//	t=100 位姿 (10,0) 观测 L 本地 (0,5)  → 地图 (10,5)
	//	t=200 位姿 (20,0) 观测 L 本地 (0,5)  → 地图 (20,5)
	// L 校正前平均 (15,5)、次数 2；两次观测相距 10 恰为合并上限，可接纳。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 50, DX: 0, Observations: []Observation{{ID: "R", X: 0, Y: 0}}},
		{Time: 100, DX: 10, Observations: []Observation{{ID: "L", X: 0, Y: 5}}},
		{Time: 200, DX: 10, Observations: []Observation{{ID: "L", X: 0, Y: 5}}},
	}}); err != nil {
		t.Fatal(err)
	}

	oldArea := rectAround(15, 5, 1.0)                       // 只包住旧均值 (15,5)
	newArea := rectAround(30, 5, 1.0)                       // 只包住新均值 (30,5)
	wideArea := Rect{MinX: -1, MinY: -1, MaxX: 40, MaxY: 6} // 同时覆盖固定的 R 与移动后的 L

	// 校正前：L 在 oldArea、不在 newArea。
	if lms := mustRect(t, m, oldArea); len(lms) != 1 || lms[0].ID != "L" {
		t.Fatalf("before correction oldArea = %v, want [L]", lms)
	}
	if lms := mustRect(t, m, newArea); len(lms) != 0 || lms == nil {
		t.Fatalf("before correction newArea = %v, want non-nil empty", lms)
	}

	// 锚点 t=100 平移到 (25,0)（+15，朝向不变）：t=100/200 两帧作为
	// 刚体一并 +15，L 两条观测随之到 (25,5)、(35,5)，相距仍恰为合并
	// 上限，平均 (30,5)，次数不变；t=50 的 R 在锚点之前，保持 (0,0)。
	rec, err := m.Correct(Correction{ID: "c-move", Anchor: 100, Target: CorrectionTarget{X: 25, Y: 0}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	lc, ok := findLandmarkChange(rec.Landmarks, "L")
	if !ok {
		t.Fatalf("L change missing: %+v", rec.Landmarks)
	}
	if lc.Before.Count != 2 || lc.After.Count != 2 {
		t.Fatalf("L count changed: %d -> %d", lc.Before.Count, lc.After.Count)
	}
	approxEq(t, "L corrected mean x", lc.After.X, 30)
	approxEq(t, "L corrected mean y", lc.After.Y, 5)

	// 原矩形立即查不到 L（坐标已更新，不能再按旧位置 (15,5) 判断）。
	if lms := mustRect(t, m, oldArea); len(lms) != 0 || lms == nil {
		t.Fatalf("after correction oldArea = %v, want non-nil empty (L moved out)", lms)
	}
	// 覆盖新平均位置的矩形得到 L 的标识、新位置与原观测次数。
	got := mustRect(t, m, newArea)
	if len(got) != 1 {
		t.Fatalf("after correction newArea = %v, want [L]", got)
	}
	if got[0].ID != "L" || got[0].Count != 2 {
		t.Fatalf("newArea hit = %+v, want L count 2", got[0])
	}
	approxEq(t, "newArea hit x", got[0].X, lc.After.X)
	approxEq(t, "newArea hit y", got[0].Y, lc.After.Y)

	// 区域查询返回的位置必须与该次出现记录中的最新位置一致。
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 || !h.Appearances[0].Active {
		t.Fatalf("L appearances = %+v", h.Appearances)
	}
	if hx, hy := h.Appearances[0].Landmark.X, h.Appearances[0].Landmark.Y; hx != got[0].X || hy != got[0].Y {
		t.Fatalf("rect pos (%v,%v) != occurrence pos (%v,%v)", got[0].X, got[0].Y, hx, hy)
	}

	// 宽矩形同时包含固定的 R 与移动后的 L，按标识排序返回。
	both := mustRect(t, m, wideArea)
	assertSortedByID(t, both)
	if lm, ok := findLandmark(both, "L"); !ok || lm.Count != 2 || lm.X != 30 || lm.Y != 5 {
		t.Fatalf("wideArea L = %+v ok=%v (full list %v)", lm, ok, both)
	}
	if lm, ok := findLandmark(both, "R"); !ok || lm.Count != 1 || lm.X != 0 || lm.Y != 0 {
		t.Fatalf("wideArea R = %+v ok=%v (full list %v)", lm, ok, both)
	}
}

// TestRectAfterCorrectionMovesIn 保障从矩形外移入矩形内的对称情形：校正
// 把平均位置搬进矩形的路标立即能被查到（带新位置与原次数），不会因为它
// “校正前在外面”而被漏掉。
func TestRectAfterCorrectionMovesIn(t *testing.T) {
	m := newMap(t, rectRegConfig())

	//	t=100 位姿 (30,0) 观测 L 本地 (0,5) → 地图 (30,5)
	//	t=200 位姿 (40,0) 观测 L 本地 (0,5) → 地图 (40,5)
	// 均值 (35,5)、次数 2；两观测相距 10 恰为合并上限。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 30, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 200, DX: 10, Observations: []Observation{{ID: "L", Y: 5}}},
	}}); err != nil {
		t.Fatal(err)
	}

	home := rectAround(5, 5, 1.0) // 目标归属区，中心 (5,5)
	if lms := mustRect(t, m, home); len(lms) != 0 || lms == nil {
		t.Fatalf("before correction home = %v, want empty", lms)
	}

	// 锚点 t=100 平移 -30（→(0,0)）：观测 → (0,5)、(10,5)，均值 (5,5)，
	// 进入 home；两次观测相距仍恰为合并上限，可接纳。
	if _, err := m.Correct(Correction{ID: "c-in", Anchor: 100, Target: CorrectionTarget{X: 0, Y: 0}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}
	got := mustRect(t, m, home)
	if len(got) != 1 || got[0].ID != "L" || got[0].Count != 2 {
		t.Fatalf("after moving in home = %v, want [L count 2]", got)
	}
	approxEq(t, "moved-in x", got[0].X, 5)
	approxEq(t, "moved-in y", got[0].Y, 5)
}

// TestRectUsesMeanAcrossAnchorBoundary 保障：同一次出现既有锚点之前的
// 观测（保持原位），也有锚点及之后的观测（随校正移动）时，区域查询位置
// 由“固定的更早观测”和“随校正变化的后续观测”共同平均决定，不能把整个
// 路标挪到校正目标附近，也不能用锚点帧位姿、最后一条观测或目标位置代替。
func TestRectUsesMeanAcrossAnchorBoundary(t *testing.T) {
	m := newMap(t, rectRegConfig())

	// 三条观测，全部本地 (0,5)，朝向始终 0：
	//	t=50  位姿 (0,0) → 观测 (0,5)   （锚点之前，校正后固定）
	//	t=100 位姿 (8,0) → 观测 (8,5)   （锚点帧）
	//	t=200 位姿 (8,0) → 观测 (8,5)   （锚点之后，与上一帧同位姿）
	// 导入时对“当时平均”的距离依次为 8、4，均在上限内；均值 (16/3,5)。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 50, DX: 0, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 100, DX: 8, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 200, DX: 0, Observations: []Observation{{ID: "L", Y: 5}}},
	}}); err != nil {
		t.Fatal(err)
	}
	oldMean := 16.0 / 3.0
	before := rectAround(oldMean, 5, 0.5)
	if lms := mustRect(t, m, before); len(lms) != 1 || lms[0].ID != "L" || lms[0].Count != 3 {
		t.Fatalf("before = %v, want [L at (16/3,5) count 3]", lms)
	}

	// 锚点 t=100 平移 +2（→(10,0)）：固定观测 (0,5)，t=100/200 观测随
	// 刚体到 (10,5)、(10,5)。重放接纳距离依次为 10（恰为上限）、5；
	// 三条等权平均 = 20/3 ≈ 6.667，计数仍为 3。
	rec, err := m.Correct(Correction{ID: "c-mix", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 0}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	lc, ok := findLandmarkChange(rec.Landmarks, "L")
	if !ok {
		t.Fatalf("L change missing: %+v", rec.Landmarks)
	}
	wantMean := 20.0 / 3.0
	approxEq(t, "mixed mean x", lc.After.X, wantMean)
	approxEq(t, "mixed mean y", lc.After.Y, 5)
	if lc.After.Count != 3 {
		t.Fatalf("L count = %d, want 3", lc.After.Count)
	}

	// 正确混合均值区：必须命中。
	meanArea := rectAround(wantMean, 5, 0.5)
	got := mustRect(t, m, meanArea)
	if len(got) != 1 || got[0].ID != "L" {
		t.Fatalf("meanArea = %v, want [L at mixed mean]", got)
	}

	// 若错误地只取某个单点位置，会落进下列矩形；正确的混合均值都在它们
	// 之外，这些查询必须为空：
	// 锚点帧观测（也是最后一条观测、校正目标的 x 坐标）位置 (10,5)：
	if lms := mustRect(t, m, rectAround(10, 5, 0.4)); len(lms) != 0 {
		t.Fatalf("anchor/last-obs area = %v, must be empty (query uses blended mean)", lms)
	}
	// 校正目标位置 (10,0)：
	if lms := mustRect(t, m, rectAround(10, 0, 0.4)); len(lms) != 0 {
		t.Fatalf("target area = %v, must be empty (mean is not correction target)", lms)
	}
	// 更早固定观测单独位置 (0,5)：
	if lms := mustRect(t, m, rectAround(0, 5, 0.4)); len(lms) != 0 {
		t.Fatalf("fixed-obs area = %v, must be empty (mean blends fixed and moved)", lms)
	}
	// 旧均值区也不再命中。
	if lms := mustRect(t, m, before); len(lms) != 0 {
		t.Fatalf("old mean area = %v, must be empty after correction", lms)
	}

	// 区域位置与该次出现记录的最新位置一致。
	h, _ := m.LandmarkAppearances("L")
	if h.Appearances[0].Landmark.X != got[0].X || h.Appearances[0].Landmark.Y != got[0].Y {
		t.Fatalf("rect (%v,%v) != occurrence (%v,%v)",
			got[0].X, got[0].Y, h.Appearances[0].Landmark.X, h.Appearances[0].Landmark.Y)
	}
}

// TestRectBoundaryAfterCorrection 保障：校正后的平均位置恰好落在矩形边
// 或角上时仍返回（含边界），刚越过边界则排除。
func TestRectBoundaryAfterCorrection(t *testing.T) {
	m := newMap(t, rectRegConfig())
	// t=100 (10,5)、t=200 (20,5)，均值 (15,5)。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 10, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 200, DX: 10, Observations: []Observation{{ID: "L", Y: 5}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// 平移 +10：观测 (20,5)、(30,5)，均值 (25,5)。
	rec, err := m.Correct(Correction{ID: "c-edge", Anchor: 100, Target: CorrectionTarget{X: 20, Y: 0}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	lc, _ := findLandmarkChange(rec.Landmarks, "L")
	approxEq(t, "edge mean x", lc.After.X, 25)
	approxEq(t, "edge mean y", lc.After.Y, 5)

	eps := 1e-9
	// 均值 (25,5) 恰好落在矩形各条边与角上 → 均命中。
	if lms := mustRect(t, m, Rect{MinX: 0, MinY: 0, MaxX: 25, MaxY: 10}); len(lms) != 1 {
		t.Fatalf("on right edge excluded: %v", lms)
	}
	if lms := mustRect(t, m, Rect{MinX: 25, MinY: 0, MaxX: 30, MaxY: 10}); len(lms) != 1 {
		t.Fatalf("on left edge excluded: %v", lms)
	}
	if lms := mustRect(t, m, Rect{MinX: 20, MinY: 5, MaxX: 30, MaxY: 6}); len(lms) != 1 {
		t.Fatalf("on bottom edge excluded: %v", lms)
	}
	if lms := mustRect(t, m, Rect{MinX: 20, MinY: 0, MaxX: 30, MaxY: 5}); len(lms) != 1 {
		t.Fatalf("on top edge excluded: %v", lms)
	}
	if lms := mustRect(t, m, Rect{MinX: 25, MinY: 5, MaxX: 30, MaxY: 10}); len(lms) != 1 {
		t.Fatalf("on corner excluded: %v", lms)
	}
	// 刚越过右边界一点点 → 排除。
	if lms := mustRect(t, m, Rect{MinX: 0, MinY: 0, MaxX: 25 - eps, MaxY: 10}); len(lms) != 0 || lms == nil {
		t.Fatalf("just past right edge included: %v", lms)
	}
	// 刚越过左边界（左界比均值大一丁点）→ 排除。
	if lms := mustRect(t, m, Rect{MinX: 25 + eps, MinY: 0, MaxX: 30, MaxY: 10}); len(lms) != 0 || lms == nil {
		t.Fatalf("just past left edge included: %v", lms)
	}
}

// TestRectEmptyResultAfterCorrection 保障：校正后没有任何有效路标落在
// 区域内时，继续返回非 nil 的空结果。
func TestRectEmptyResultAfterCorrection(t *testing.T) {
	m := newMap(t, rectRegConfig())
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 10, Observations: []Observation{{ID: "L", Y: 5}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// 单条观测随锚点平移到 (100,5)，不存在同出现冲突。
	if _, err := m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 100, Y: 0}}); err != nil {
		t.Fatal(err)
	}
	lms, err := m.LandmarksInRect(Rect{MinX: 0, MinY: 0, MaxX: 1, MaxY: 1})
	if err != nil {
		t.Fatalf("LandmarksInRect: %v", err)
	}
	if lms == nil || len(lms) != 0 {
		t.Fatalf("empty result = %v, want non-nil empty", lms)
	}
}

// TestRectOnlyConsidersActiveOccurrenceAfterCorrection 保障：同一标识既
// 有已失效的旧出现、又有当前有效的新出现时，区域查询只考虑新出现——
//   - 旧出现位置即使落在矩形里也不返回；
//   - 校正范围同时跨越旧出现与新出现时，旧记录不被重新显示；
//   - 两次出现都落在区域内时，同一标识仍只返回一次，次数取新出现自己的
//     计数（不是两次之和）。
func TestRectOnlyConsidersActiveOccurrenceAfterCorrection(t *testing.T) {
	m := newMap(t, rectRegConfig())

	// occ1（随后失效）：t=100/200 位姿均 (0,0)，观测本地 (0,5) → (0,5)，
	// 均值 (0,5)、计数 2。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 200, Observations: []Observation{{ID: "L", Y: 5}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "off1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}

	// occ2（当前有效）：t=300 位姿 (100,0)、t=400 位姿 (101,0)，观测本地
	// (0,5) → (100,5)、(101,5)，均值 (100.5,5)、计数 2；旧出现不参与合并。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 100, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 400, DX: 1, Observations: []Observation{{ID: "L", Y: 5}}},
	}}); err != nil {
		t.Fatal(err)
	}

	oldArea := rectAround(0, 5, 1.0)                        // 只包 occ1 位置 (0,5)
	newArea := rectAround(100.5, 5, 1.0)                    // 只包 occ2 校正前位置
	bothSpan := Rect{MinX: -2, MinY: 4, MaxX: 102, MaxY: 6} // 同时覆盖两次出现位置

	// 校正前：旧出现不返回；新出现按自己的位置与计数返回。
	if lms := mustRect(t, m, oldArea); len(lms) != 0 {
		t.Fatalf("inactive occ1 in oldArea before correction: %v", lms)
	}
	if lms := mustRect(t, m, newArea); len(lms) != 1 || lms[0].ID != "L" || lms[0].Count != 2 {
		t.Fatalf("active occ2 newArea = %v", lms)
	}
	// 跨度覆盖两次出现：同一标识仍只返回一次，次数取新出现的 2。
	if lms := mustRect(t, m, bothSpan); len(lms) != 1 || lms[0].ID != "L" || lms[0].Count != 2 {
		t.Fatalf("both occurrences in span -> %v, want single active L count 2", lms)
	}

	// 一次校正同时跨越旧出现（t=100/200）与新出现（t=300/400）：锚点
	// t=100 平移 +5。occ1 观测 → (5,5)（仍失效）；occ2 两帧作为锚点之后
	// 的刚体一并 +5，观测 → (105,5)、(106,5)，均值 (105.5,5)、计数 2。
	rec, err := m.Correct(Correction{ID: "c-span", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 0}})
	if err != nil {
		t.Fatalf("Correct across occurrences: %v", err)
	}
	var sawOcc1, sawOcc2 bool
	for _, lc := range rec.Landmarks {
		if lc.ID != "L" {
			continue
		}
		switch lc.Occurrence {
		case 1:
			sawOcc1 = true
			if lc.After.Count != 2 || lc.After.X != 5 {
				t.Fatalf("occ1 change = %+v, want (5,5) count 2", lc)
			}
		case 2:
			sawOcc2 = true
			if lc.After.Count != 2 || lc.After.X != 105.5 {
				t.Fatalf("occ2 change = %+v, want x 105.5 count 2", lc)
			}
		}
	}
	if !sawOcc1 || !sawOcc2 {
		t.Fatalf("expected both occurrence changes, got %+v", rec.Landmarks)
	}

	// 校正后：旧出现即使位置移动了也依旧不返回（不被重新显示）。
	if lms := mustRect(t, m, rectAround(5, 5, 1.0)); len(lms) != 0 {
		t.Fatalf("inactive occ1 resurfaced after correction: %v", lms)
	}
	if lms := mustRect(t, m, oldArea); len(lms) != 0 {
		t.Fatalf("oldArea after correction = %v, want empty", lms)
	}
	// occ2 校正前位置的矩形已空；新位置矩形返回 occ2，次数仍是它自己的 2。
	if lms := mustRect(t, m, newArea); len(lms) != 0 {
		t.Fatalf("newArea at stale occ2 pos = %v, want empty", lms)
	}
	movedNew := rectAround(105.5, 5, 1.0)
	got := mustRect(t, m, movedNew)
	if len(got) != 1 || got[0].ID != "L" || got[0].Count != 2 {
		t.Fatalf("moved active occ2 = %v", got)
	}
	// 跨度同时覆盖被移动后的两次出现：仍只返回一次，次数取新出现。
	if lms := mustRect(t, m, Rect{MinX: -2, MinY: 4, MaxX: 107, MaxY: 6}); len(lms) != 1 ||
		lms[0].ID != "L" || lms[0].Count != 2 || lms[0].X != 105.5 {
		t.Fatalf("span after correction = %v, want single active L at 105.5 count 2", lms)
	}
}

// TestRectAfterConflictingCorrectionUnchanged 保障：某次校正会让同一次
// 出现中的观测超过合并距离时，按既有 landmark_conflict 原因整次拒绝；
// 拒绝后查询原区域与拟移动到的区域，都必须得到提交前的结果，不能留下
// 部分路标已经移入或移出的状态。
func TestRectAfterConflictingCorrectionUnchanged(t *testing.T) {
	m := newMap(t, rectRegConfig())

	// 一次出现含 4 条观测，全部本地 (0,5)，分布在锚点前后：
	//	t=50  位姿 (0,0)  → (0,5)   锚点之前，固定
	//	t=100 位姿 (10,0) → (10,5)  锚点帧
	//	t=150 位姿 (11,0) → (11,5)  锚点之后
	//	t=200 位姿 (12,0) → (12,5)  锚点之后；同帧另观测 S 本地 (0,-5) → (12,-5)
	// L 各观测对当时平均的距离依次为 10、6、5，均在上限内；均值
	// (8.25,5)、次数 4。S 只有一条观测，本身平移多远都不冲突。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 50, DX: 0, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 100, DX: 10, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 150, DX: 1, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L", Y: 5}, {ID: "S", Y: -5}}},
	}}); err != nil {
		t.Fatal(err)
	}

	oldArea := rectAround(8.25, 5, 1.0) // L 校正前均值区
	homeS := rectAround(12, -5, 1.0)    // S 校正前位置
	if lms := mustRect(t, m, homeS); len(lms) != 1 || lms[0].ID != "S" || lms[0].Count != 1 {
		t.Fatalf("S before = %v", lms)
	}

	// 拟把锚点 t=100 平移 +30：t=50 的观测固定在 (0,5)，重放到 t=100 时
	// 该观测移到 (40,5)，与固定均值相距 40 > 合并上限 10 → 冲突，整次
	// 拒绝。intendedNewArea 包住“若被半提交”锚点观测会落到的位置。
	intendedNewArea := rectAround(40, 5, 1.0)

	beforeOld := mustRect(t, m, oldArea)
	beforeNew := mustRect(t, m, intendedNewArea)
	beforeS := mustRect(t, m, homeS)
	if len(beforeOld) != 1 || beforeOld[0].ID != "L" || beforeOld[0].Count != 4 {
		t.Fatalf("precondition oldArea = %v", beforeOld)
	}
	if len(beforeNew) != 0 {
		t.Fatalf("precondition intendedNewArea = %v, want empty", beforeNew)
	}

	_, err := m.Correct(Correction{ID: "c-conflict", Anchor: 100, Target: CorrectionTarget{X: 40, Y: 0}})
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("err = %v, want *RejectError", err)
	}
	if r.Kind != RejectLandmarkConflict || r.Landmark != "L" || r.Time != 100 ||
		!r.HasOccurrence || r.Occurrence != 1 {
		t.Fatalf("reject = %+v, want landmark_conflict L occ1 at t=100", r)
	}

	// 拒绝后原区域结果与提交前完全一致（L 仍在，计数 4）。
	if after := mustRect(t, m, oldArea); !landmarksEqual(after, beforeOld) {
		t.Fatalf("oldArea changed after rejected correction:\n before %v\n after  %v", beforeOld, after)
	}
	// 拟移动到的区域仍是提交前的空结果：L 没有被半移入。
	if after := mustRect(t, m, intendedNewArea); !landmarksEqual(after, beforeNew) {
		t.Fatalf("intendedNewArea changed after rejected correction:\n before %v\n after  %v", beforeNew, after)
	}
	// 同次校正里本可移动的 S 也必须留在原处（没有部分提交）。
	if after := mustRect(t, m, homeS); !landmarksEqual(after, beforeS) {
		t.Fatalf("S partially moved after rejected correction:\n before %v\n after  %v", beforeS, after)
	}
	// 校正记录不增加，锚点位姿不变。
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("correction record leaked: %v", recs)
	}
	if p, _ := m.PoseAt(100); p.X != 10 || p.Y != 0 {
		t.Fatalf("anchor pose changed after rejected correction: %+v", p)
	}
}

// TestRectCorrectionPersistsAcrossReopen 保障：校正后区域查询基于新平均
// 位置的结果在关闭重开后保持一致（移出原矩形、移入新矩形、计数与位置）。
func TestRectCorrectionPersistsAcrossReopen(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, rectRegConfig())
	if err != nil {
		t.Fatal(err)
	}
	// t=100 (10,5)、t=200 (20,5)，均值 (15,5)、计数 2。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 10, Observations: []Observation{{ID: "L", Y: 5}}},
		{Time: 200, DX: 10, Observations: []Observation{{ID: "L", Y: 5}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// +15：观测 (25,5)、(35,5)，均值 (30,5)。
	if _, err := m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 25, Y: 0}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}
	oldArea := rectAround(15, 5, 0.5)
	newArea := rectAround(30, 5, 0.5)
	want := mustRect(t, m, newArea)
	if len(want) != 1 || want[0].ID != "L" || want[0].Count != 2 || want[0].X != 30 {
		t.Fatalf("pre-close newArea = %v", want)
	}
	if lms := mustRect(t, m, oldArea); len(lms) != 0 {
		t.Fatalf("pre-close oldArea = %v, want empty", lms)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer m2.Close()

	if lms, err := m2.LandmarksInRect(oldArea); err != nil || len(lms) != 0 || lms == nil {
		t.Fatalf("after reopen oldArea = %v, %v, want non-nil empty", lms, err)
	}
	got, err := m2.LandmarksInRect(newArea)
	if err != nil || len(got) != 1 {
		t.Fatalf("after reopen newArea = %v, %v", got, err)
	}
	if got[0].ID != "L" || got[0].Count != 2 {
		t.Fatalf("after reopen hit = %+v, want L count 2", got[0])
	}
	approxEq(t, "after reopen x", got[0].X, 30)
	approxEq(t, "after reopen y", got[0].Y, 5)
	h, err := m2.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if h.Appearances[0].Landmark.X != got[0].X || h.Appearances[0].Landmark.Y != got[0].Y {
		t.Fatalf("after reopen occurrence %+v != rect %+v", h.Appearances[0].Landmark, got[0])
	}
}

// TestRectMultiHitSortedAfterCorrection 保障：多个路标在校正后命中同一
// 矩形时，继续按标识排序返回，各自带新位置与原次数。
func TestRectMultiHitSortedAfterCorrection(t *testing.T) {
	m := newMap(t, rectRegConfig())
	// t=100 位姿 (10,0)：同帧先观测 b、再观测 a，本地均为 (0,5) → 地图
	// (10,5)；t=200 位姿 (11,0) 再观测 b → (11,5)。
	// a：(10,5) 计数 1；b：均值 (10.5,5) 计数 2。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 10, Observations: []Observation{{ID: "b", Y: 5}, {ID: "a", Y: 5}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "b", Y: 5}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// 锚点 t=100 平移 +40：a 的单条观测 → (50,5)；b 的两条 → (50,5)、
	// (51,5)，均值 (50.5,5)，计数不变。
	if _, err := m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 50, Y: 0}}); err != nil {
		t.Fatalf("Correct: %v", err)
	}
	got := mustRect(t, m, Rect{MinX: 49, MinY: 4, MaxX: 52, MaxY: 6})
	if len(got) != 2 {
		t.Fatalf("multi hit = %v, want a and b", got)
	}
	assertSortedByID(t, got)
	if got[0].ID != "a" || got[0].Count != 1 || got[0].X != 50 || got[0].Y != 5 {
		t.Fatalf("a = %+v", got[0])
	}
	if got[1].ID != "b" || got[1].Count != 2 || got[1].X != 50.5 || got[1].Y != 5 {
		t.Fatalf("b = %+v", got[1])
	}
}
