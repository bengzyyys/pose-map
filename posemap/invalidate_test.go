package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// 辅助：创建一个含单个路标的地图并导入一段。
func setupMapWithLandmark(t *testing.T, id string) *Map {
	t.Helper()
	m := newMap(t, baseConfig())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: id}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	return m
}

func TestInvalidateBasic(t *testing.T) {
	m := setupMapWithLandmark(t, "L1")

	// 失效前：区域查询包含 L1。
	lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].ID != "L1" {
		t.Fatalf("landmarks before invalidate = %v", lms)
	}

	// 失效 L1。
	res, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "removed", LandmarkIDs: []string{"L1"}})
	if err != nil {
		t.Fatalf("Invalidate: %v", err)
	}
	if res.InvalidTime != 100 {
		t.Fatalf("invalid time = %d, want 100", res.InvalidTime)
	}
	if res.Appearances["L1"] != 1 {
		t.Fatalf("appearance number = %d, want 1", res.Appearances["L1"])
	}

	// 失效后：区域查询立即排除 L1。
	lms, err = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 0 {
		t.Fatalf("landmarks after invalidate = %v, want empty", lms)
	}

	// 位姿与观测不被删除。
	cur, _ := m.CurrentPose()
	if cur.Time != 100 {
		t.Fatalf("current time = %d, want 100", cur.Time)
	}

	// 历史查询：L1 有一次失效记录。
	hist, err := m.LandmarkHistory("L1")
	if err != nil {
		t.Fatal(err)
	}
	if len(hist.Appearances) != 1 {
		t.Fatalf("appearances = %d, want 1", len(hist.Appearances))
	}
	a := hist.Appearances[0]
	if a.Number != 1 || a.Valid || a.Count != 1 || a.InvalidTime != 100 || a.InvalidReason != "removed" {
		t.Fatalf("appearance = %+v", a)
	}
}

func TestInvalidateValidation(t *testing.T) {
	m := setupMapWithLandmark(t, "L1")
	// 再导入一个路标。
	_, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L2"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}

	type tc struct {
		name string
		req  InvalidateRequest
		kind string
	}
	cases := []tc{
		{"empty id", InvalidateRequest{Reason: "r", LandmarkIDs: []string{"L1"}}, RejectEmptyID},
		{"empty reason", InvalidateRequest{ID: "i", LandmarkIDs: []string{"L1"}}, RejectEmptyReason},
		{"empty list", InvalidateRequest{ID: "i", Reason: "r"}, RejectEmptyLandmarkList},
		{"empty landmark id", InvalidateRequest{ID: "i", Reason: "r", LandmarkIDs: []string{""}}, RejectEmptyID},
		{"duplicate landmark", InvalidateRequest{ID: "i", Reason: "r", LandmarkIDs: []string{"L1", "L1"}}, RejectDuplicateLandmark},
		{"landmark not found", InvalidateRequest{ID: "i", Reason: "r", LandmarkIDs: []string{"L99"}}, RejectLandmarkNotFound},
		{"already invalid", InvalidateRequest{ID: "i", Reason: "r", LandmarkIDs: []string{"L1"}}, ""}, // 先失效一次
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			if c.kind == "" {
				// 先失效 L1。
				if _, err := m.Invalidate(InvalidateRequest{ID: "first", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
					t.Fatal(err)
				}
				_, err := m.Invalidate(InvalidateRequest{ID: "i", Reason: "r", LandmarkIDs: []string{"L1"}})
				r, ok := AsRejectError(err)
				if !ok || r.Kind != RejectLandmarkAlreadyInvalid {
					t.Fatalf("err = %v, want %s", err, RejectLandmarkAlreadyInvalid)
				}
				return
			}
			_, err := m.Invalidate(c.req)
			r, ok := AsRejectError(err)
			if !ok || r.Kind != c.kind {
				t.Fatalf("err = %v, want %s", err, c.kind)
			}
		})
	}

	// 全部拒绝后状态不变：L1 仍有效。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	for _, lm := range lms {
		if lm.ID == "L1" {
			t.Fatalf("L1 still valid after rejections")
		}
	}
}

func TestInvalidateIdempotent(t *testing.T) {
	m := setupMapWithLandmark(t, "L1")
	req := InvalidateRequest{ID: "inv1", Reason: "removed", LandmarkIDs: []string{"L1"}}
	first, err := m.Invalidate(req)
	if err != nil {
		t.Fatal(err)
	}

	// 相同内容重复提交：返回首次结果，不重复失效。
	again, err := m.Invalidate(req)
	if err != nil {
		t.Fatalf("duplicate invalidate: %v", err)
	}
	if again.InvalidTime != first.InvalidTime || again.Appearances["L1"] != first.Appearances["L1"] {
		t.Fatalf("duplicate returned %+v, want %+v", again, first)
	}

	// 不同内容：明确拒绝。
	diff := req
	diff.Reason = "different"
	_, err = m.Invalidate(diff)
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectInvalidationMismatch {
		t.Fatalf("mismatch err = %v, want %s", err, RejectInvalidationMismatch)
	}

	// 即使路标后来再次出现，同标识同内容也不再撤下它。
	// 先让 L1 再次出现。
	_, err = m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	// L1 再次有效。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	found := false
	for _, lm := range lms {
		if lm.ID == "L1" {
			found = true
		}
	}
	if !found {
		t.Fatalf("L1 not valid after reappearance")
	}
	// 重复提交同标识同内容：返回首次结果，不撤下新出现的 L1。
	again, err = m.Invalidate(req)
	if err != nil {
		t.Fatalf("duplicate after reappearance: %v", err)
	}
	if again.InvalidTime != first.InvalidTime {
		t.Fatalf("duplicate changed invalid time")
	}
	// L1 仍有效。
	lms, _ = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	found = false
	for _, lm := range lms {
		if lm.ID == "L1" {
			found = true
		}
	}
	if !found {
		t.Fatalf("L1 invalidated by duplicate after reappearance")
	}
}

func TestReappearance(t *testing.T) {
	m := setupMapWithLandmark(t, "L1")
	// 第一次观测后 L1 位置 (2,2)，count 1。
	// 失效。
	if _, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatal(err)
	}

	// 在新轨迹中再次观测 L1，位置远离旧记录。
	_, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 5, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}

	// 新记录 count 从 1 开始，位置来自首次新观测。
	lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].ID != "L1" || lms[0].Count != 1 {
		t.Fatalf("landmark = %v, want L1 count 1", lms)
	}
	// 新位置：位姿 (7,2)，观测本地 (0,0) → 地图 (7,2)。
	approxEq(t, "new x", lms[0].X, 7)
	approxEq(t, "new y", lms[0].Y, 2)

	// 历史：两次出现。
	hist, err := m.LandmarkHistory("L1")
	if err != nil {
		t.Fatal(err)
	}
	if len(hist.Appearances) != 2 {
		t.Fatalf("appearances = %d, want 2", len(hist.Appearances))
	}
	first := hist.Appearances[0]
	second := hist.Appearances[1]
	if first.Number != 1 || first.Valid || first.Count != 1 {
		t.Fatalf("first appearance = %+v", first)
	}
	if second.Number != 2 || !second.Valid || second.Count != 1 || second.FirstTime != 200 {
		t.Fatalf("second appearance = %+v", second)
	}

	// 同帧后续观测合并到新记录。
	_, err = m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 300, DX: 0, Observations: []Observation{{ID: "L1", X: 0, Y: 0.5}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	lms, _ = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if lms[0].Count != 2 {
		t.Fatalf("count = %d, want 2", lms[0].Count)
	}
}

func TestReappearanceMergeDistance(t *testing.T) {
	m := setupMapWithLandmark(t, "L1")
	if _, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatal(err)
	}

	// 新观测距旧记录很远，但旧记录不参加合并，所以不冲突。
	_, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 100, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatalf("reappearance conflicted with old record: %v", err)
	}

	// 新记录的同帧后续观测仍受合并距离限制。
	_, err = m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 300, DX: 0, Observations: []Observation{{ID: "L1", X: 50, Y: 0}}},
	}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectLandmarkConflict {
		t.Fatalf("err = %v, want %s", err, RejectLandmarkConflict)
	}
}

func TestLandmarkHistoryUnknown(t *testing.T) {
	m := newMap(t, baseConfig())
	_, err := m.LandmarkHistory("unknown")
	if !errors.Is(err, ErrNotFound) {
		t.Fatalf("err = %v, want ErrNotFound", err)
	}
}

func TestInvalidateMultiple(t *testing.T) {
	m := newMap(t, baseConfig())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L1"}, {ID: "L2"}, {ID: "L3"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}

	// 同时失效两个路标。
	res, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1", "L3"}})
	if err != nil {
		t.Fatal(err)
	}
	if res.InvalidTime != 100 {
		t.Fatalf("invalid time = %d", res.InvalidTime)
	}
	if res.Appearances["L1"] != 1 || res.Appearances["L3"] != 1 {
		t.Fatalf("appearances = %v", res.Appearances)
	}

	// 区域查询只剩 L2。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "L2" {
		t.Fatalf("landmarks = %v", lms)
	}
}

func TestCorrectionAcrossAppearances(t *testing.T) {
	m := newMap(t, baseConfig())
	// 第一次出现：t=100 观测 L1。
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	// 失效 L1。
	if _, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatal(err)
	}
	// 第二次出现：t=200 观测 L1。
	_, err = m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}

	// 校正锚点 t=200，平移 10 米。
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 200, Target: CorrectionTarget{X: 10, Y: 2}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}

	// 校正记录区分各次出现。
	if len(rec.Landmarks) != 1 {
		t.Fatalf("landmark changes = %d", len(rec.Landmarks))
	}
	lc := rec.Landmarks[0]
	if lc.ID != "L1" {
		t.Fatalf("landmark id = %s", lc.ID)
	}
	// 第二次出现受影响（校正范围从 t=200 开始）。
	if len(lc.Appearances) != 1 || lc.Appearances[0].Number != 2 {
		t.Fatalf("appearances = %+v", lc.Appearances)
	}
	ac := lc.Appearances[0]
	// 第二次出现的观测从 (3,2) 移到 (10,2)（锚点目标位置）。
	approxEq(t, "appearance 2 after x", ac.After.X, 10)
	approxEq(t, "appearance 2 after y", ac.After.Y, 2)
	if ac.After.Count != 1 {
		t.Fatalf("count = %d, want 1", ac.After.Count)
	}

	// 第一次出现不受影响（校正范围不包含 t=100）。
	hist, _ := m.LandmarkHistory("L1")
	if len(hist.Appearances) != 2 {
		t.Fatalf("appearances = %d", len(hist.Appearances))
	}
	first := hist.Appearances[0]
	if first.Valid || first.InvalidTime != 100 {
		t.Fatalf("first appearance changed: %+v", first)
	}
}

func TestCorrectionConflictWithAppearanceNumber(t *testing.T) {
	m := newMap(t, baseConfig())
	// 第一次出现：t=100 和 t=150 各观测一次 L1。
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L1"}}},
		{Time: 150, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	// 失效。
	if _, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatal(err)
	}
	// 第二次出现：t=200 观测 L1。
	_, err = m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}

	// 校正锚点 t=150，平移 10 米 → 第一次出现的观测冲突。
	_, err = m.Correct(Correction{ID: "c1", Anchor: 150, Target: CorrectionTarget{X: 10, Y: 2}})
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("err = %v, want *RejectError", err)
	}
	if r.Kind != RejectLandmarkConflict || r.Landmark != "L1" || !r.HasLandmark {
		t.Fatalf("reject = %+v", r)
	}
	if !r.HasAppearance || r.Appearance != 1 {
		t.Fatalf("appearance = %d(ok=%v), want 1", r.Appearance, r.HasAppearance)
	}
	if !r.HasTime || r.Time != 150 {
		t.Fatalf("time = %d(ok=%v), want 150", r.Time, r.HasTime)
	}
}

func TestInvalidatePersistRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	_, err = m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L1"}, {ID: "L2"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "removed", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer m2.Close()

	// 区域查询仍排除 L1。
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "L2" {
		t.Fatalf("landmarks after reopen = %v", lms)
	}

	// 历史查询一致。
	hist, err := m2.LandmarkHistory("L1")
	if err != nil {
		t.Fatal(err)
	}
	if len(hist.Appearances) != 1 || hist.Appearances[0].Valid || hist.Appearances[0].InvalidReason != "removed" {
		t.Fatalf("history after reopen = %+v", hist.Appearances[0])
	}

	// 重复提交结果一致。
	req := InvalidateRequest{ID: "inv1", Reason: "removed", LandmarkIDs: []string{"L1"}}
	again, err := m2.Invalidate(req)
	if err != nil {
		t.Fatalf("duplicate after reopen: %v", err)
	}
	if again.InvalidTime != 100 {
		t.Fatalf("duplicate invalid time = %d", again.InvalidTime)
	}
}

func TestReappearanceAfterReopen(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	_, err = m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer m2.Close()

	// 重开后 L1 仍失效。
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 0 {
		t.Fatalf("landmarks = %v, want empty", lms)
	}

	// 新轨迹中 L1 再次出现。
	_, err = m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].Count != 1 {
		t.Fatalf("landmark after reappearance = %v", lms)
	}

	// 历史：两次出现。
	hist, _ := m2.LandmarkHistory("L1")
	if len(hist.Appearances) != 2 {
		t.Fatalf("appearances = %d, want 2", len(hist.Appearances))
	}
	if hist.Appearances[1].Number != 2 || !hist.Appearances[1].Valid {
		t.Fatalf("second appearance = %+v", hist.Appearances[1])
	}
}

func TestInvalidateSaveFailureRollback(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	_, err = m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	// 把目标路径变成目录，使 saveReplace 失败。
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(path, 0o755); err != nil {
		t.Fatal(err)
	}
	_, err = m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}})
	if err == nil {
		t.Fatal("invalidate across directory target unexpectedly succeeded")
	}
	// 内存状态精确回滚：L1 仍有效。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 {
		t.Fatalf("landmark leaked after failed save: %v", lms)
	}
	// 恢复文件后可正常失效。
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := m.saveReplace(); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatalf("retry after failed save: %v", err)
	}
}

func TestInvalidateClosedMap(t *testing.T) {
	m := newMap(t, baseConfig())
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	_, err := m.Invalidate(InvalidateRequest{ID: "i", Reason: "r", LandmarkIDs: []string{"L1"}})
	if !errors.Is(err, ErrClosed) {
		t.Fatalf("invalidate on closed: %v", err)
	}
	_, err = m.LandmarkHistory("L1")
	if !errors.Is(err, ErrClosed) {
		t.Fatalf("history on closed: %v", err)
	}
}

func TestRejectedSegmentNoNewAppearance(t *testing.T) {
	m := setupMapWithLandmark(t, "L1")
	var err error
	if _, err = m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatal(err)
	}

	// 导入一段会被拒绝的轨迹（时间顺序错误）。
	_, err = m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "L1"}}},
	}})
	if err == nil {
		t.Fatal("expected rejection")
	}

	// L1 仍失效，没有新记录，编号未消耗。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 0 {
		t.Fatalf("landmark leaked: %v", lms)
	}
	hist, _ := m.LandmarkHistory("L1")
	if len(hist.Appearances) != 1 {
		t.Fatalf("appearances = %d, want 1 (no new record on rejection)", len(hist.Appearances))
	}
}

func TestDuplicateImportAfterInvalidate(t *testing.T) {
	m := setupMapWithLandmark(t, "L1")
	// 第一次导入的段。
	seg := Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L1"}}},
	}}
	if _, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatal(err)
	}

	// 重复导入原轨迹段：返回首次结果，不重新激活路标或增加观测。
	res, err := m.ImportSegment(seg)
	if err != nil {
		t.Fatalf("duplicate import: %v", err)
	}
	if res.EndPose.Time != 100 {
		t.Fatalf("duplicate result = %+v", res.EndPose)
	}
	// L1 仍失效。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 0 {
		t.Fatalf("landmark reactivated by duplicate import: %v", lms)
	}
}

func TestLegacyFileAppearance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "old.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	// t=100 位姿 (1,0)，K(1,0)；t=200 位姿 (2,0)，K(2,0)；合并后 K(1.5,0) 次数 2。
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "K"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open legacy: %v", err)
	}
	defer m2.Close()

	// 旧文件中的路标视为第 1 次有效记录，首次观测时间未知。
	hist, err := m2.LandmarkHistory("K")
	if err != nil {
		t.Fatal(err)
	}
	if len(hist.Appearances) != 1 {
		t.Fatalf("appearances = %d, want 1", len(hist.Appearances))
	}
	a := hist.Appearances[0]
	if a.Number != 1 || !a.Valid || a.Count != 2 || a.FirstTime != -1 {
		t.Fatalf("legacy appearance = %+v", a)
	}

	// 旧文件路标可失效。
	if _, err := m2.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"K"}}); err != nil {
		t.Fatalf("invalidate legacy: %v", err)
	}
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 0 {
		t.Fatalf("landmarks after invalidate = %v", lms)
	}

	// 失效后可再次出现，编号为 2。
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 1, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	hist, _ = m2.LandmarkHistory("K")
	if len(hist.Appearances) != 2 {
		t.Fatalf("appearances = %d, want 2", len(hist.Appearances))
	}
	if hist.Appearances[1].Number != 2 || !hist.Appearances[1].Valid || hist.Appearances[1].Count != 1 {
		t.Fatalf("second appearance = %+v", hist.Appearances[1])
	}
}

func TestInvalidateMultipleRejectAtomic(t *testing.T) {
	m := newMap(t, baseConfig())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L1"}, {ID: "L2"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}

	// 列表中有一个不存在的路标：整次拒绝，不失效任何一个。
	_, err = m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1", "L99"}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectLandmarkNotFound {
		t.Fatalf("err = %v, want %s", err, RejectLandmarkNotFound)
	}
	// L1 仍有效（区域查询中仍能找到）。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	found := false
	for _, lm := range lms {
		if lm.ID == "L1" {
			found = true
		}
	}
	if !found {
		t.Fatalf("L1 invalidated despite rejection")
	}
}

func TestInvalidateReturnsAppearanceNumbers(t *testing.T) {
	m := newMap(t, baseConfig())
	// L1 第一次出现后失效，再次出现。
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(InvalidateRequest{ID: "inv1", Reason: "r", LandmarkIDs: []string{"L1"}}); err != nil {
		t.Fatal(err)
	}
	_, err = m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, Observations: []Observation{{ID: "L1"}}},
	}})
	if err != nil {
		t.Fatal(err)
	}

	// 再次失效：返回的出现编号应为 2。
	res, err := m.Invalidate(InvalidateRequest{ID: "inv2", Reason: "r2", LandmarkIDs: []string{"L1"}})
	if err != nil {
		t.Fatal(err)
	}
	if res.Appearances["L1"] != 2 {
		t.Fatalf("appearance number = %d, want 2", res.Appearances["L1"])
	}
	if res.InvalidTime != 200 {
		t.Fatalf("invalid time = %d, want 200", res.InvalidTime)
	}
}
