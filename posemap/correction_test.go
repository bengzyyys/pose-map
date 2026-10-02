package posemap

import (
	"encoding/binary"
	"encoding/json"
	"errors"
	"hash/crc32"
	"math"
	"os"
	"path/filepath"
	"testing"
)

// 三帧轨迹（含初始位姿）：
//
//	p0 (t=0)  (1,2)   h=0     v=0.5
//	p1 (t=100)(4,2)   h=π/2   v=0.75   观测 L1 本地(1,0) → 地图(4,3)
//	p2 (t=200)(4,4)   h=π/2   v=1.0    观测 L1 本地(0,0) → 地图(4,4)
//
// L1 为两次观测平均 (4,3.5)，次数 2。
func corrSegment() Segment {
	return Segment{
		ID: "s1",
		Frames: []Frame{
			{
				Time: 100, DX: 3, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.25,
				Observations: []Observation{{ID: "L1", X: 1, Y: 0}},
			},
			{
				Time: 200, DX: 2, DY: 0, DHeading: 0, MoveVariance: 0.25,
				Observations: []Observation{{ID: "L1", X: 0, Y: 0}, {ID: "M", X: 0, Y: 1}},
			},
		},
	}
}

func TestCorrectionRigidTransform(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(corrSegment()); err != nil {
		t.Fatal(err)
	}

	req := Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Heading: 0, Variance: 2}}
	rec, err := m.Correct(req)
	if err != nil {
		t.Fatalf("Correct: %v", err)
	}

	// 锚点 p1：精确采用目标位姿；δ = 0 - π/2 = -π/2。
	// p2 相对锚点 (0,2) 旋转 -π/2 → (2,0)，新位置 (12,20)，朝向 0，
	// 方差 = 2 + 原运动方差 0.25 = 2.25。
	if rec.Anchor != 100 || rec.EndTime != 200 || len(rec.Poses) != 2 {
		t.Fatalf("record header = %+v", rec)
	}
	got := rec.Poses
	if got[0].Before.X != 4 || got[0].After.X != 10 || got[0].After.Y != 20 {
		t.Fatalf("anchor pose change = %+v", got[0])
	}
	approxEq(t, "p2 after x", got[1].After.X, 12)
	approxEq(t, "p2 after y", got[1].After.Y, 20)
	approxEq(t, "p2 after heading", got[1].After.Heading, 0)
	approxEq(t, "p2 after variance", got[1].After.Variance, 2.25)
	approxEq(t, "anchor variance", got[0].After.Variance, 2)

	// 路标：p1 观测 → (11,20)，p2 观测 L1 → (12,20)，两次观测距离恰为
	// 合并距离 1，可接受；平均 (11.5,20)，次数不变。
	if len(rec.Landmarks) != 2 || rec.Landmarks[0].ID != "L1" || rec.Landmarks[1].ID != "M" {
		t.Fatalf("landmark changes order = %+v", rec.Landmarks)
	}
	lc := rec.Landmarks[0]
	approxEq(t, "L1 before x", lc.Before.X, 4)
	approxEq(t, "L1 before y", lc.Before.Y, 3.5)
	approxEq(t, "L1 after x", lc.After.X, 11.5)
	approxEq(t, "L1 after y", lc.After.Y, 20)
	if lc.Before.Count != 2 || lc.After.Count != 2 {
		t.Fatalf("L1 count changed: %d -> %d", lc.Before.Count, lc.After.Count)
	}

	// 更早的初始位姿不变。
	p0, err := m.PoseAt(0)
	if err != nil || p0.X != 1 || p0.Y != 2 || p0.Variance != 0.5 {
		t.Fatalf("initial pose changed: %+v %v", p0, err)
	}
	// 当前位姿与历史查询立即返回校正结果（PoseAt 借用最近帧也一样）。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 12 || cur.Y != 20 {
		t.Fatalf("current = %+v", cur)
	}
	p, _ := m.PoseAt(150)
	if p.Time != 100 || p.X != 10 || p.Y != 20 {
		t.Fatalf("pose at 150 = %+v, want corrected anchor", p)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 2 {
		t.Fatalf("landmarks = %v", lms)
	}
	for _, lm := range lms {
		if lm.ID == "L1" {
			approxEq(t, "queried L1 x", lm.X, 11.5)
			approxEq(t, "queried L1 y", lm.Y, 20)
			if lm.Count != 2 {
				t.Fatalf("queried L1 count = %d", lm.Count)
			}
		}
	}

	// 后续导入从校正后的末位姿接续：DX=1, 朝向 0 → (13,20)，方差 2.75。
	res, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{{Time: 300, DX: 1, MoveVariance: 0.5}}})
	if err != nil {
		t.Fatalf("import after correction: %v", err)
	}
	approxEq(t, "continued x", res.EndPose.X, 13)
	approxEq(t, "continued variance", res.EndPose.Variance, 2.75)
}

func TestCorrectionHeadingWraps(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{{Time: 100, DX: 1}}}); err != nil {
		t.Fatal(err)
	}
	// 目标朝向 3π 归一到 -π；结果朝向必须落在既有 [-π,π) 范围。
	rec, err := m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 0, Y: 0, Heading: 3 * math.Pi}})
	if err != nil {
		t.Fatal(err)
	}
	approxEq(t, "heading", rec.Poses[0].After.Heading, -math.Pi)
}

func TestCorrectionRejections(t *testing.T) {
	m := newMap(t, baseConfig())
	first, err := m.ImportSegment(corrSegment())
	if err != nil {
		t.Fatal(err)
	}

	type tc struct {
		name string
		req  Correction
		kind string
		lm   string
	}
	cases := []tc{
		{"empty id", Correction{Anchor: 100, Target: CorrectionTarget{X: 1}}, RejectEmptyID, ""},
		{"anchor is initial pose", Correction{ID: "x", Anchor: 0, Target: CorrectionTarget{X: 1}}, RejectAnchorNotFound, ""},
		{"anchor between frames must not borrow latest", Correction{ID: "x", Anchor: 150, Target: CorrectionTarget{X: 1}}, RejectAnchorNotFound, ""},
		{"anchor before initial", Correction{ID: "x", Anchor: -1, Target: CorrectionTarget{X: 1}}, RejectAnchorNotFound, ""},
		{"anchor missing", Correction{ID: "x", Anchor: 999, Target: CorrectionTarget{X: 1}}, RejectAnchorNotFound, ""},
		{"negative variance", Correction{ID: "x", Anchor: 100, Target: CorrectionTarget{X: 1, Variance: -0.01}}, RejectNegativeVariance, ""},
		{"non-finite x", Correction{ID: "x", Anchor: 100, Target: CorrectionTarget{X: math.NaN()}}, RejectNonFinite, ""},
		{"non-finite heading", Correction{ID: "x", Anchor: 100, Target: CorrectionTarget{Heading: math.Inf(1)}}, RejectNonFinite, ""},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			_, err := m.Correct(c.req)
			r, ok := AsRejectError(err)
			if !ok || r.Kind != c.kind {
				t.Fatalf("err = %v, want kind %s", err, c.kind)
			}
			if c.kind != RejectEmptyID {
				if !r.HasTime || r.Time != c.req.Anchor {
					t.Fatalf("time = %d(ok=%v), want %d", r.Time, r.HasTime, c.req.Anchor)
				}
			}
			if c.lm != "" && (!r.HasLandmark || r.Landmark != c.lm) {
				t.Fatalf("landmark = %q, want %q", r.Landmark, c.lm)
			}
		})
	}

	// 全部拒绝后状态不变：位姿、方差、路标、次数。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != first.EndPose.X || cur.Variance != 1.0 {
		t.Fatalf("state leaked after rejected corrections: %+v", cur)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 2 {
		t.Fatalf("landmarks leaked: %v", lms)
	}
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("correction records leaked: %v", recs)
	}
}

func TestCorrectionLandmarkConflict(t *testing.T) {
	cfg := baseConfig()
	cfg.MergeDistance = 1.0
	m := newMap(t, cfg)
	// K 在锚点之前的 t=50 与锚点帧 t=100 各观测一次，原位置相同。
	_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "K", X: 0, Y: 0}}},
		{Time: 100, Observations: []Observation{{ID: "K", X: 0, Y: 0}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	// 平移 9 米后，锚点帧观测远离更早且固定的观测 → 冲突，整次拒绝。
	_, err = m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 2}})
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("err = %v, want *RejectError", err)
	}
	if r.Kind != RejectLandmarkConflict || r.Landmark != "K" || r.Time != 100 || !r.HasLandmark || !r.HasTime {
		t.Fatalf("reject = %+v", r)
	}

	// 无部分更新：位姿、路标、记录都保持原样。
	cur, _ := m.CurrentPose()
	if cur.X != 1 || cur.Y != 2 {
		t.Fatalf("pose changed after conflict: %+v", cur)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].X != 1 || lms[0].Count != 2 {
		t.Fatalf("landmark changed after conflict: %v", lms)
	}
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("record leaked after conflict: %v", recs)
	}
}

func TestCorrectionIdempotentAndMismatch(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(corrSegment()); err != nil {
		t.Fatal(err)
	}
	req := Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Variance: 2}}
	first, err := m.Correct(req)
	if err != nil {
		t.Fatal(err)
	}

	// 同标识同内容：返回首次结果，不再修改数据，也不新增记录。
	again, err := m.Correct(req)
	if err != nil {
		t.Fatalf("duplicate correction: %v", err)
	}
	if !recordsEqual(again, first) {
		t.Fatalf("duplicate returned %+v, want %+v", again, first)
	}
	if recs, _ := m.Corrections(); len(recs) != 1 {
		t.Fatalf("duplicate added a record: %v", recs)
	}
	cur, _ := m.CurrentPose()
	if cur.X != 12 || cur.Variance != 2.25 {
		t.Fatalf("duplicate changed state: %+v", cur)
	}

	// 同标识不同内容：明确拒绝。
	diff := req
	diff.Target.Variance = 3
	_, err = m.Correct(diff)
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectCorrectionMismatch || r.Landmark != "c1" {
		t.Fatalf("mismatch err = %v", err)
	}
	if recs, _ := m.Corrections(); len(recs) != 1 {
		t.Fatalf("mismatch added a record: %v", recs)
	}

	// 校正后重复导入原轨迹段：返回首次导入结果，不撤销校正、不增加次数。
	res, err := m.ImportSegment(corrSegment())
	if err != nil {
		t.Fatalf("duplicate import after correction: %v", err)
	}
	if res.EndPose.Time != 200 || res.EndPose.X != 4 || res.EndPose.Variance != 1.0 {
		t.Fatalf("duplicate import result = %+v, want first import result", res.EndPose)
	}
	cur, _ = m.CurrentPose()
	if cur.Time != 200 || cur.X != 12 || cur.Variance != 2.25 {
		t.Fatalf("duplicate import undid correction: %+v", cur)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 2 {
		t.Fatalf("landmark count = %d after duplicate import", len(lms))
	}
	for _, lm := range lms {
		if (lm.ID == "L1" && lm.Count != 2) || (lm.ID == "M" && lm.Count != 1) {
			t.Fatalf("%s count = %d after duplicate import", lm.ID, lm.Count)
		}
	}
}

func TestCorrectionRecordsAppendOnly(t *testing.T) {
	m := newMap(t, baseConfig())
	// 无观测轨迹：p1 (1,2)，p2 (2,2)。
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, DX: 0, MoveVariance: 0.1},
		{Time: 200, DX: 1, MoveVariance: 0.1},
	}}); err != nil {
		t.Fatal(err)
	}
	c1 := Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Variance: 2}}
	r1, err := m.Correct(c1)
	if err != nil {
		t.Fatal(err)
	}
	// 第二次校正以当时最新状态为起点，把末帧 p2 放到 (50,60)。
	c2 := Correction{ID: "c2", Anchor: 200, Target: CorrectionTarget{X: 50, Y: 60, Variance: 7}}
	r2, err := m.Correct(c2)
	if err != nil {
		t.Fatal(err)
	}
	if len(r2.Poses) != 1 || r2.Poses[0].Before.X != 11 || r2.Poses[0].After.X != 50 {
		t.Fatalf("second record = %+v", r2)
	}

	recs, _ := m.Corrections()
	if len(recs) != 2 || recs[0].ID != "c1" || recs[1].ID != "c2" {
		t.Fatalf("records order = %v", recs)
	}
	// 先前记录不被改写：r1 的快照明旧。
	if !recordsEqual(recs[0], r1) {
		t.Fatalf("first record was rewritten: %+v want %+v", recs[0], r1)
	}
	// 当前位姿以第二次校正为准。
	cur, _ := m.CurrentPose()
	if cur.X != 50 || cur.Y != 60 || cur.Variance != 7 {
		t.Fatalf("current after second correction = %+v", cur)
	}
	// 第一次锚点帧 p1 不在第二次影响范围内，保持 c1 结果。
	p, _ := m.PoseAt(100)
	if p.X != 10 || p.Y != 20 {
		t.Fatalf("anchor of c1 changed = %+v", p)
	}
}

func TestCorrectionPersistRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(corrSegment()); err != nil {
		t.Fatal(err)
	}
	req := Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Variance: 2}}
	rec, err := m.Correct(req)
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

	cur, _ := m2.CurrentPose()
	if cur.X != 12 || cur.Y != 20 || cur.Variance != 2.25 {
		t.Fatalf("current after reopen = %+v", cur)
	}
	p, _ := m2.PoseAt(100)
	if p.X != 10 || p.Y != 20 || p.Variance != 2 {
		t.Fatalf("history after reopen = %+v", p)
	}
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 2 {
		t.Fatalf("landmarks after reopen: %v", lms)
	}
	for _, lm := range lms {
		if lm.ID == "L1" && (lm.X != 11.5 || lm.Y != 20 || lm.Count != 2) {
			t.Fatalf("L1 after reopen = %+v", lm)
		}
	}

	recs, _ := m2.Corrections()
	if len(recs) != 1 || !recordsEqual(recs[0], rec) {
		t.Fatalf("records after reopen = %v, want %+v", recs, rec)
	}

	// 两类重复提交在重开后行为一致。
	again, err := m2.Correct(req)
	if err != nil || !recordsEqual(again, rec) {
		t.Fatalf("duplicate correction after reopen: %v %+v", err, again)
	}
	diff := req
	diff.Target.X = 11
	if _, err := m2.Correct(diff); err == nil {
		t.Fatal("mismatch correction accepted after reopen")
	}
	// 原轨迹段重复导入仍返回首次结果，不撤销校正。
	res, err := m2.ImportSegment(corrSegment())
	if err != nil {
		t.Fatalf("duplicate import after reopen: %v", err)
	}
	if res.EndPose.X != 4 || res.EndPose.Variance != 1.0 {
		t.Fatalf("duplicate import result after reopen = %+v", res.EndPose)
	}
	cur, _ = m2.CurrentPose()
	if cur.X != 12 {
		t.Fatalf("duplicate import undid correction after reopen: %+v", cur)
	}
}

// writeLegacyV1File 用当前状态构造一个去掉 sources/corrections 字段的
// 旧版文件（二进制版本号仍为 1），模拟现网旧地图。
func writeLegacyV1File(t *testing.T, path string, st *mapState) {
	t.Helper()
	data, err := encodeFile(st)
	if err != nil {
		t.Fatal(err)
	}
	var raw map[string]any
	if err := json.Unmarshal(data[headerLen:len(data)-4], &raw); err != nil {
		t.Fatal(err)
	}
	delete(raw, "sources")
	delete(raw, "corrections")
	payload, err := json.Marshal(raw)
	if err != nil {
		t.Fatal(err)
	}
	out := make([]byte, headerLen+len(payload)+4)
	copy(out[:headerLen], data[:headerLen])
	binary.BigEndian.PutUint32(out[10:14], uint32(len(payload)))
	copy(out[headerLen:], payload)
	binary.BigEndian.PutUint32(out[len(out)-4:], crc32.ChecksumIEEE(out[:len(out)-4]))
	if err := os.WriteFile(path, out, 0o644); err != nil {
		t.Fatal(err)
	}
}

func TestLegacyFileBasis(t *testing.T) {
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

	// 旧文件可正常打开、查询。
	p, err := m2.PoseAt(100)
	if err != nil || p.X != 1 {
		t.Fatalf("legacy pose: %+v %v", p, err)
	}
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].X != 1.5 || lms[0].Count != 2 {
		t.Fatalf("legacy landmark = %v", lms)
	}

	// 涉及旧范围的校正被拒绝并说明原因。
	for _, anchor := range []int64{100, 200} {
		_, err = m2.Correct(Correction{ID: "c", Anchor: anchor, Target: CorrectionTarget{X: 1}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNoBasis {
			t.Fatalf("anchor %d: err = %v, want no_basis", anchor, err)
		}
	}

	// 旧地图上可继续导入：t=300 位姿 (2.5,0)，K 观测 (2.5,0) 距旧均值
	// (1.5,0) 恰为合并距离，接受；K 均值变为 5.5/3，次数 3。
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatalf("append to legacy: %v", err)
	}
	lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].Count != 3 {
		t.Fatalf("K after append = %v", lms)
	}
	approxEq(t, "K mean after append", lms[0].X, 5.5/3.0)

	// 仅覆盖依据完整新范围的校正可成功；旧观测贡献固定不变。
	rec, err := m2.Correct(Correction{ID: "c2", Anchor: 300, Target: CorrectionTarget{X: 2.0, Y: 0}})
	if err != nil {
		t.Fatalf("correct new range: %v", err)
	}
	if rec.EndTime != 300 || len(rec.Poses) != 1 {
		t.Fatalf("record = %+v", rec)
	}
	// 新观测从 (2.5,0) 移到 (2.0,0)，与固定旧均值 (1.5,0) 距离 0.5 可接受。
	lc := rec.Landmarks[0]
	if lc.ID != "K" {
		t.Fatalf("change id = %q", lc.ID)
	}
	approxEq(t, "K after correct", lc.After.X, (1.5*2+2.0)/3.0)
	if lc.After.Count != 3 {
		t.Fatalf("K count = %d", lc.After.Count)
	}

	// 新范围校正若让新观测远离固定旧贡献，仍冲突拒绝。
	_, err = m2.Correct(Correction{ID: "c3", Anchor: 300, Target: CorrectionTarget{X: 10, Y: 0}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectLandmarkConflict || r.Landmark != "K" || r.Time != 300 {
		t.Fatalf("legacy conflict err = %v", err)
	}

	// 重开后：null 来源（旧帧）与校正记录都保持。
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}
	m3, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer m3.Close()
	if _, err := m3.Correct(Correction{ID: "x", Anchor: 100, Target: CorrectionTarget{X: 1}}); err == nil {
		t.Fatal("old range became correctable after reopen")
	}
	recs, _ := m3.Corrections()
	if len(recs) != 1 || recs[0].ID != "c2" {
		t.Fatalf("records after legacy reopen = %v", recs)
	}
}

func TestCorrectionSaveFailureRollback(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(corrSegment()); err != nil {
		t.Fatal(err)
	}
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(path, 0o755); err != nil {
		t.Fatal(err)
	}
	req := Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 10, Y: 20, Variance: 2}}
	if _, err := m.Correct(req); err == nil {
		t.Fatal("correction across directory target unexpectedly succeeded")
	}
	// 内存状态精确回滚。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 4 || cur.Variance != 1.0 {
		t.Fatalf("current after failed save = %+v", cur)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	for _, lm := range lms {
		if lm.ID == "L1" && (lm.X != 4 || lm.Y != 3.5) {
			t.Fatalf("landmark after failed save = %+v", lm)
		}
	}
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("record leaked after failed save: %v", recs)
	}
	// 恢复文件后同标识校正可正常成功（记录此前未落盘）。
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := m.saveReplace(); err != nil {
		t.Fatalf("restore save: %v", err)
	}
	if _, err := m.Correct(req); err != nil {
		t.Fatalf("retry correction after failed save: %v", err)
	}
}

func TestCorrectionClosedMap(t *testing.T) {
	m := newMap(t, baseConfig())
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "x", Anchor: 1, Target: CorrectionTarget{X: 1}}); !errors.Is(err, ErrClosed) {
		t.Fatalf("correct on closed: %v", err)
	}
	if _, err := m.Corrections(); !errors.Is(err, ErrClosed) {
		t.Fatalf("corrections on closed: %v", err)
	}
}

func recordsEqual(a, b CorrectionRecord) bool {
	if a.ID != b.ID || a.Anchor != b.Anchor || a.EndTime != b.EndTime ||
		a.Target != b.Target || len(a.Poses) != len(b.Poses) || len(a.Landmarks) != len(b.Landmarks) {
		return false
	}
	for i := range a.Poses {
		if a.Poses[i] != b.Poses[i] {
			return false
		}
	}
	for i := range a.Landmarks {
		la, lb := a.Landmarks[i], b.Landmarks[i]
		if la.ID != lb.ID || la.Before != lb.Before || la.After != lb.After ||
			len(la.Appearances) != len(lb.Appearances) {
			return false
		}
		for j := range la.Appearances {
			if la.Appearances[j] != lb.Appearances[j] {
				return false
			}
		}
	}
	return true
}
