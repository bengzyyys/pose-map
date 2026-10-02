package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// allRect 覆盖整个（有限）平面，便于取全部当前有效路标。
func allRect() Rect { return Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9} }

func activeLandmarks(t *testing.T, m *Map) map[string]Landmark {
	t.Helper()
	lms, err := m.LandmarksInRect(allRect())
	if err != nil {
		t.Fatal(err)
	}
	out := make(map[string]Landmark, len(lms))
	for _, lm := range lms {
		out[lm.ID] = lm
	}
	return out
}

// 构造：t=100/200 两帧各观测 L（本地 (1,0)）与一帧 M；初始位姿 (0,0)。
func seedOccurrenceMap(t *testing.T) *Map {
	t.Helper()
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1, Y: 0}, {ID: "M", X: 2, Y: 0}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	return m
}

func TestInvalidateSuccessAndHistory(t *testing.T) {
	m := seedOccurrenceMap(t)

	res, err := m.Invalidate(Invalidation{ID: "op1", Reason: "sign removed", Landmarks: []string{"L", "M"}})
	if err != nil {
		t.Fatalf("Invalidate: %v", err)
	}
	// 失效时间为提交时当前位姿时间。
	if res.Time != 200 {
		t.Fatalf("invalidate time = %d, want 200", res.Time)
	}
	// 结果按标识排序，编号为 1。
	if len(res.Landmarks) != 2 ||
		res.Landmarks[0] != (InvalidatedLandmark{ID: "L", Occurrence: 1}) ||
		res.Landmarks[1] != (InvalidatedLandmark{ID: "M", Occurrence: 1}) {
		t.Fatalf("invalidate result = %+v", res.Landmarks)
	}

	// 区域查询立即排除，位姿与观测不删除。
	if lms, _ := m.LandmarksInRect(allRect()); len(lms) != 0 {
		t.Fatalf("invalidated landmarks still queried: %v", lms)
	}
	cur, _ := m.CurrentPose()
	if cur.Time != 200 {
		t.Fatalf("pose changed: %+v", cur)
	}

	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if h.ID != "L" || len(h.Appearances) != 1 {
		t.Fatalf("history = %+v", h)
	}
	a := h.Appearances[0]
	if a.Number != 1 || !a.HasFirstSeen || a.FirstSeenTime != 100 || a.Active {
		t.Fatalf("appearance = %+v", a)
	}
	if a.Landmark.Count != 2 || a.Landmark.X != 1 || a.Landmark.Y != 0 {
		t.Fatalf("appearance landmark = %+v", a.Landmark)
	}
	if a.InvalidTime != 200 || a.InvalidReason != "sign removed" || a.InvalidOpID != "op1" {
		t.Fatalf("invalid info = %d %q %q", a.InvalidTime, a.InvalidReason, a.InvalidOpID)
	}

	// 未知标识返回可区分的未找到结果。
	if _, err := m.LandmarkAppearances("nope"); !errors.Is(err, ErrLandmarkNotFound) {
		t.Fatalf("unknown id: %v", err)
	}
}

func TestInvalidateRejections(t *testing.T) {
	m := seedOccurrenceMap(t)
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}

	type tc struct {
		name string
		req  Invalidation
		kind string
		lm   string
	}
	cases := []tc{
		{"empty id", Invalidation{Reason: "x", Landmarks: []string{"M"}}, RejectEmptyID, ""},
		{"empty reason", Invalidation{ID: "x", Landmarks: []string{"M"}}, RejectEmptyReason, ""},
		{"empty list", Invalidation{ID: "x", Reason: "r"}, RejectEmptyLandmarkList, ""},
		{"empty landmark id", Invalidation{ID: "x", Reason: "r", Landmarks: []string{""}}, RejectEmptyID, ""},
		{"duplicate entry", Invalidation{ID: "x", Reason: "r", Landmarks: []string{"M", "M"}}, RejectDuplicateLandmarkEntry, "M"},
		{"unknown landmark", Invalidation{ID: "x", Reason: "r", Landmarks: []string{"ZZ"}}, RejectLandmarkNotFound, "ZZ"},
		{"already invalidated", Invalidation{ID: "x", Reason: "r", Landmarks: []string{"L"}}, RejectLandmarkInactive, "L"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			_, err := m.Invalidate(c.req)
			r, ok := AsRejectError(err)
			if !ok || r.Kind != c.kind {
				t.Fatalf("err = %v, want %s", err, c.kind)
			}
			if c.lm != "" && (!r.HasLandmark || r.Landmark != c.lm) {
				t.Fatalf("landmark = %q, want %q", r.Landmark, c.lm)
			}
		})
	}

	// 一次操作中单个路标不合法，整次拒绝：M 仍有效（未被连带撤下）。
	_, err := m.Invalidate(Invalidation{ID: "batch", Reason: "r", Landmarks: []string{"M", "L"}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectLandmarkInactive || r.Landmark != "L" {
		t.Fatalf("batch err = %v", err)
	}
	if _, err := m.LandmarkAppearances("M"); err != nil {
		t.Fatal(err)
	}
	if hm, _ := m.LandmarkAppearances("M"); !hm.Appearances[0].Active {
		t.Fatal("M was partially invalidated")
	}
}

func TestReappearanceNumberingAndMerge(t *testing.T) {
	m := seedOccurrenceMap(t)
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}

	// 新轨迹在远处重新看到 L：旧记录不参与合并，距离再远也不冲突。
	// t=300 位姿 (10,0)，本地 (0,0) → 地图 (10,0)；同帧第二次观测
	// (0,0.5) 在合并距离内，编号 2、计数从 1 起递增。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 10, Observations: []Observation{{ID: "L"}, {ID: "L", X: 0, Y: 0.5}}},
	}}); err != nil {
		t.Fatalf("reappearance: %v", err)
	}
	lms := activeLandmarks(t, m)
	lm, ok := lms["L"]
	if !ok {
		t.Fatal("active L missing")
	}
	if lm.Count != 2 {
		t.Fatalf("count = %d, want 2", lm.Count)
	}
	approxEq(t, "new y", lm.Y, 0.25)
	approxEq(t, "new x", lm.X, 10)

	h, _ := m.LandmarkAppearances("L")
	if len(h.Appearances) != 2 {
		t.Fatalf("history len = %d", len(h.Appearances))
	}
	if h.Appearances[0].Active || h.Appearances[0].Number != 1 {
		t.Fatalf("occ1 = %+v", h.Appearances[0])
	}
	a2 := h.Appearances[1]
	if !a2.Active || a2.Number != 2 || !a2.HasFirstSeen || a2.FirstSeenTime != 300 {
		t.Fatalf("occ2 = %+v", a2)
	}

	// 再次失效撤下的是第 2 次出现。
	res, err := m.Invalidate(Invalidation{ID: "op2", Reason: "gone again", Landmarks: []string{"L"}})
	if err != nil {
		t.Fatal(err)
	}
	if res.Time != 300 || res.Landmarks[0].Occurrence != 2 {
		t.Fatalf("second invalidation = %+v", res)
	}
	h, _ = m.LandmarkAppearances("L")
	for _, ap := range h.Appearances {
		if ap.Active {
			t.Fatalf("occurrence %d still active", ap.Number)
		}
	}

	// 第三次出现编号为 3。
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 400, DX: 0, Observations: []Observation{{ID: "L", X: -5, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	h, _ = m.LandmarkAppearances("L")
	if len(h.Appearances) != 3 || !h.Appearances[2].Active || h.Appearances[2].Number != 3 {
		t.Fatalf("occ3 = %+v", h.Appearances)
	}
	if c := h.Appearances[2].Landmark.Count; c != 1 {
		t.Fatalf("occ3 count = %d, want 1", c)
	}
}

func TestReappearanceRejectedSegmentConsumesNothing(t *testing.T) {
	m := seedOccurrenceMap(t)
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	// 同帧第二次观测距首次新观测超距：整段拒绝，不留新记录、不消耗编号。
	_, err := m.ImportSegment(Segment{ID: "bad", Frames: []Frame{
		{Time: 300, DX: 10, Observations: []Observation{{ID: "L"}, {ID: "L", X: 5, Y: 0}}},
	}})
	rejectKind(t, err, RejectLandmarkConflict, 0, "L")
	h, _ := m.LandmarkAppearances("L")
	if len(h.Appearances) != 1 {
		t.Fatalf("occurrence leaked: %+v", h.Appearances)
	}
	// 随后合法再现仍是编号 2。
	if _, err := m.ImportSegment(Segment{ID: "good", Frames: []Frame{
		{Time: 300, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	h, _ = m.LandmarkAppearances("L")
	if len(h.Appearances) != 2 || !h.Appearances[1].Active {
		t.Fatalf("expected occ 2 active: %+v", h.Appearances)
	}
}

func TestInvalidateIdempotent(t *testing.T) {
	m := seedOccurrenceMap(t)
	req := Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}
	first, err := m.Invalidate(req)
	if err != nil {
		t.Fatal(err)
	}

	// L 在别处再次出现（编号 2，当前有效）。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 相同操作标识 + 相同原因 + 相同集合：返回首次结果，不再撤下 occ2。
	again, err := m.Invalidate(req)
	if err != nil {
		t.Fatalf("duplicate invalidation: %v", err)
	}
	if again.Time != first.Time || len(again.Landmarks) != 1 ||
		again.Landmarks[0] != (InvalidatedLandmark{ID: "L", Occurrence: 1}) {
		t.Fatalf("duplicate result = %+v, want %+v", again, first)
	}
	h, _ := m.LandmarkAppearances("L")
	if !h.Appearances[1].Active {
		t.Fatal("duplicate invalidation deactivated the new occurrence")
	}

	// 同标识不同原因：明确拒绝。
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "other", Landmarks: []string{"L"}}); err == nil {
		t.Fatal("different reason accepted")
	} else if r, ok := AsRejectError(err); !ok || r.Kind != RejectInvalidationMismatch || r.Landmark != "op1" {
		t.Fatalf("reason mismatch err = %v", err)
	}
	// 同标识不同集合：明确拒绝。
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"M"}}); err == nil {
		t.Fatal("different set accepted")
	} else if r, ok := AsRejectError(err); !ok || r.Kind != RejectInvalidationMismatch {
		t.Fatalf("set mismatch err = %v", err)
	}
	// 次序不同但集合相同：仍视为重复。
	oneMore, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}})
	if err != nil || oneMore.Time != first.Time {
		t.Fatalf("same set resubmit: %v %+v", err, oneMore)
	}
}

func TestDuplicateImportDoesNotReactivate(t *testing.T) {
	m := seedOccurrenceMap(t)
	orig := Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1, Y: 0}, {ID: "M", X: 2, Y: 0}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
	}}
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	// 原轨迹重复导入：返回首次结果，occ1 不复活、观测不增加。
	res, err := m.ImportSegment(orig)
	if err != nil {
		t.Fatal(err)
	}
	if res.EndPose.Time != 200 {
		t.Fatalf("duplicate result = %+v", res.EndPose)
	}
	h, _ := m.LandmarkAppearances("L")
	if len(h.Appearances) != 1 || h.Appearances[0].Active || h.Appearances[0].Landmark.Count != 2 {
		t.Fatalf("occ1 changed on duplicate import: %+v", h.Appearances)
	}
}

func TestInvalidatePersistRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	first, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 5, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
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

	h, err := m2.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("history after reopen = %+v", h.Appearances)
	}
	if h.Appearances[0].Active || h.Appearances[0].InvalidReason != "gone" ||
		h.Appearances[0].InvalidTime != 100 || h.Appearances[0].InvalidOpID != "op1" {
		t.Fatalf("occ1 after reopen = %+v", h.Appearances[0])
	}
	if !h.Appearances[1].Active || h.Appearances[1].Number != 2 ||
		h.Appearances[1].FirstSeenTime != 200 || h.Appearances[1].Landmark.Count != 1 {
		t.Fatalf("occ2 after reopen = %+v", h.Appearances[1])
	}
	lms, _ := m2.LandmarksInRect(allRect())
	if len(lms) != 1 || lms[0].X != 5 || lms[0].Count != 1 {
		t.Fatalf("active after reopen = %v", lms)
	}

	// 失效重复提交与冲突提交在重开后一致。
	again, err := m2.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}})
	if err != nil || again.Time != first.Time || again.Landmarks[0].Occurrence != 1 {
		t.Fatalf("duplicate invalidation after reopen: %v %+v", err, again)
	}
	h, _ = m2.LandmarkAppearances("L")
	if !h.Appearances[1].Active {
		t.Fatal("duplicate op re-invalidated occ2 after reopen")
	}
	if _, err := m2.Invalidate(Invalidation{ID: "op1", Reason: "x", Landmarks: []string{"L"}}); err == nil {
		t.Fatal("mismatch invalidation accepted after reopen")
	}
}

func TestInvalidateSaveFailureRollback(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(path, 0o755); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err == nil {
		t.Fatal("invalidate across directory target unexpectedly succeeded")
	}
	h, _ := m.LandmarkAppearances("L")
	if len(h.Appearances) != 1 || !h.Appearances[0].Active {
		t.Fatalf("state leaked after failed save: %+v", h.Appearances)
	}
	if lms, _ := m.LandmarksInRect(allRect()); len(lms) != 1 {
		t.Fatalf("region query leaked: %v", lms)
	}
	// 恢复文件后同一操作标识仍可成功提交。
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := m.saveReplace(); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatalf("retry invalidation: %v", err)
	}
}

func TestLegacyFileAppearance(t *testing.T) {
	path := filepath.Join(t.TempDir(), "old.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "K"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer m2.Close()

	// 旧文件路标视为第 1 次有效记录，首次观测时间未知。
	h, err := m2.LandmarkAppearances("K")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 {
		t.Fatalf("legacy history = %+v", h.Appearances)
	}
	a := h.Appearances[0]
	if a.Number != 1 || !a.Active || a.HasFirstSeen {
		t.Fatalf("legacy appearance = %+v", a)
	}
	if a.Landmark.Count != 2 || a.Landmark.X != 1.5 {
		t.Fatalf("legacy landmark = %+v", a.Landmark)
	}

	// 旧路标可被失效；之后新导入产生编号 2，首次观测时间已知。
	if _, err := m2.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"K"}}); err != nil {
		t.Fatalf("invalidate legacy: %v", err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, Observations: []Observation{{ID: "K", X: 2}}},
	}}); err != nil {
		t.Fatal(err)
	}
	h, _ = m2.LandmarkAppearances("K")
	if len(h.Appearances) != 2 {
		t.Fatalf("history = %+v", h.Appearances)
	}
	if h.Appearances[0].HasFirstSeen || h.Appearances[0].Active {
		t.Fatalf("legacy occ1 = %+v", h.Appearances[0])
	}
	if !h.Appearances[1].Active || !h.Appearances[1].HasFirstSeen || h.Appearances[1].FirstSeenTime != 300 {
		t.Fatalf("occ2 = %+v", h.Appearances[1])
	}
}

func TestCorrectionAcrossOccurrences(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m := newMap(t, cfg)
	// occ1：t=100 位姿 (0,0)，L 本地 (1,0) → (1,0)；t=200 再观测一次。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	// occ2：t=300 位姿 (10,0)，L 本地 (0,0) → (10,0)。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 锚点 t=100 平移 +2（朝向不变）：occ1 观测 → (3,0)，occ2 帧位姿
	// (12,0)、观测 → (12,0)；各次出现内部无冲突。
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 2, Y: 0}})
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}
	if len(rec.Landmarks) != 2 {
		t.Fatalf("landmark changes = %+v", rec.Landmarks)
	}
	lc1, lc2 := rec.Landmarks[0], rec.Landmarks[1]
	if lc1.ID != "L" || lc1.Occurrence != 1 ||
		lc1.Before.X != 1 || lc1.After.X != 3 || lc1.Before.Count != 2 || lc1.After.Count != 2 {
		t.Fatalf("occ1 change = %+v", lc1)
	}
	if lc2.ID != "L" || lc2.Occurrence != 2 ||
		lc2.Before.X != 10 || lc2.After.X != 12 || lc2.After.Count != 1 {
		t.Fatalf("occ2 change = %+v", lc2)
	}

	// 失效状态、编号、观测计数不变；位置随校正更新。
	h, _ := m.LandmarkAppearances("L")
	if h.Appearances[0].Active || h.Appearances[0].InvalidReason != "gone" ||
		h.Appearances[0].Landmark.Count != 2 || h.Appearances[0].Landmark.X != 3 {
		t.Fatalf("occ1 after correction = %+v", h.Appearances[0])
	}
	if !h.Appearances[1].Active || h.Appearances[1].Landmark.X != 12 || h.Appearances[1].Landmark.Count != 1 {
		t.Fatalf("occ2 after correction = %+v", h.Appearances[1])
	}
	lms := activeLandmarks(t, m)
	if len(lms) != 1 || lms["L"].X != 12 {
		t.Fatalf("active landmarks = %v", lms)
	}

	// 第二次校正不改写既有记录，且新记录仍区分出现编号。
	rec2, err := m.Correct(Correction{ID: "c2", Anchor: 300, Target: CorrectionTarget{X: 13, Y: 0}})
	if err != nil {
		t.Fatal(err)
	}
	if len(rec2.Landmarks) != 1 || rec2.Landmarks[0].Occurrence != 2 {
		t.Fatalf("second record = %+v", rec2.Landmarks)
	}
	recs, _ := m.Corrections()
	if len(recs) != 2 {
		t.Fatalf("records = %d", len(recs))
	}
	if recs[0].Landmarks[0].After.X != 3 {
		t.Fatalf("first record rewritten: %+v", recs[0].Landmarks[0])
	}
}

func TestCorrectionOccurrence1ConflictRejectsAll(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m := newMap(t, cfg)
	// occ1：t=50 与 t=100 各观测一次（均在 (1,0)）。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "L", X: 1}}},
		{Time: 100, Observations: []Observation{{ID: "L", X: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	// occ2：t=200 位姿 (10,0)，单个观测。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 锚点 t=100 平移到 (10,0)：t=50 的 occ1 观测固定在 (1,0)，
	// t=100 的观测随锚点移到 (11,0) → occ1 内部冲突；occ2 不受影响，
	// 但任一次出现冲突整次校正拒绝。
	_, err := m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 0}})
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("err = %v", err)
	}
	if r.Kind != RejectLandmarkConflict || r.Landmark != "L" ||
		!r.HasOccurrence || r.Occurrence != 1 || r.Time != 100 {
		t.Fatalf("reject = %+v", r)
	}

	// 无部分更新。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 10 {
		t.Fatalf("pose changed after conflict: %+v", cur)
	}
	h, _ := m.LandmarkAppearances("L")
	if h.Appearances[0].Landmark.X != 1 || !h.Appearances[1].Active || h.Appearances[1].Landmark.X != 10 {
		t.Fatalf("landmarks changed after conflict: %+v", h.Appearances)
	}
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("record leaked: %v", recs)
	}
}

func TestCorrectionOccurrence2Conflict(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m := newMap(t, cfg)
	// occ1 在 (10,0)，t=100 失效。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	// occ2：t=200、t=300 均在 (10,0) 观测，计数 2。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, Observations: []Observation{{ID: "L"}}},
		{Time: 300, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 锚点 t=300 平移 +10：t=200 的 occ2 观测固定 (10,0)，t=300 观测
	// 移到 (20,0) → occ2 内部冲突，错误指出编号 2 与帧时间 300。
	_, err := m.Correct(Correction{ID: "c", Anchor: 300, Target: CorrectionTarget{X: 20, Y: 0}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectLandmarkConflict || r.Landmark != "L" ||
		r.Occurrence != 2 || r.Time != 300 {
		t.Fatalf("reject = %v", err)
	}
}

func TestNewAPIsClosedMap(t *testing.T) {
	m := newMap(t, baseConfig())
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "x", Reason: "r", Landmarks: []string{"L"}}); !errors.Is(err, ErrClosed) {
		t.Fatalf("invalidate closed: %v", err)
	}
	if _, err := m.LandmarkAppearances("L"); !errors.Is(err, ErrClosed) {
		t.Fatalf("appearances closed: %v", err)
	}
}

func TestInvalidateAtCurrentPoseTime(t *testing.T) {
	// 失效时间取提交时的当前位姿时间：首帧之后即失效时取该帧时间。
	m := newMap(t, Config{InitialTime: 7, MaxInterval: 1000, MergeDistance: 1.0})
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	res, err := m.Invalidate(Invalidation{ID: "op", Reason: "r", Landmarks: []string{"L"}})
	if err != nil {
		t.Fatal(err)
	}
	if res.Time != 50 {
		t.Fatalf("time = %d, want current pose time 50", res.Time)
	}
}

func TestInvalidatedOnlyLandmarkRoundTrip(t *testing.T) {
	// 路标只有一次已失效记录：重开后仍可查询历史、区域查询持续排除，
	// 且同标识失效操作幂等。
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	first, err := m.Invalidate(Invalidation{ID: "op", Reason: "r", Landmarks: []string{"L"}})
	if err != nil {
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
	if lms, _ := m2.LandmarksInRect(allRect()); len(lms) != 0 {
		t.Fatalf("invalidated-only landmark in rect: %v", lms)
	}
	h, _ := m2.LandmarkAppearances("L")
	if len(h.Appearances) != 1 || h.Appearances[0].Active {
		t.Fatalf("history = %+v", h.Appearances)
	}
	again, err := m2.Invalidate(Invalidation{ID: "op", Reason: "r", Landmarks: []string{"L"}})
	if err != nil || again.Time != first.Time {
		t.Fatalf("idempotent after reopen: %v %+v", err, again)
	}
}

func TestInvalidateSetOrderIndependent(t *testing.T) {
	m := seedOccurrenceMap(t)
	req := Invalidation{ID: "op", Reason: "r", Landmarks: []string{"M", "L"}}
	first, err := m.Invalidate(req)
	if err != nil {
		t.Fatal(err)
	}
	if len(first.Landmarks) != 2 || first.Landmarks[0].ID != "L" || first.Landmarks[1].ID != "M" {
		t.Fatalf("result order = %+v", first.Landmarks)
	}
	// 同标识、同原因、同集合（次序相反）：返回首次结果。
	again, err := m.Invalidate(Invalidation{ID: "op", Reason: "r", Landmarks: []string{"L", "M"}})
	if err != nil || len(again.Landmarks) != 2 {
		t.Fatalf("resubmit: %v %+v", err, again)
	}
	if again.Landmarks[0].ID != "L" || again.Time != first.Time {
		t.Fatalf("repeated result not first result: %+v", again.Landmarks)
	}
}
