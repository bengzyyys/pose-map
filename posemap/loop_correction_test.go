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

// loopCfg 返回一个用于回环校正测试的配置：初始位姿 (0,0,0)，方差 0，
// 最大帧间隔 1000，合并距离较大以避免路标冲突。
func loopCfg() Config {
	return Config{
		InitialTime:     0,
		InitialX:        0,
		InitialY:        0,
		InitialHeading:  0,
		InitialVariance: 0,
		MaxInterval:     1000,
		MergeDistance:   20,
	}
}

// approxEqPose 比较位姿的位置与朝向（方差按需比较）。
func approxEqPose(t *testing.T, name string, got, want Pose) {
	t.Helper()
	approxEq(t, name+".x", got.X, want.X)
	approxEq(t, name+".y", got.Y, want.Y)
	approxEq(t, name+".heading", got.Heading, want.Heading)
	approxEq(t, name+".variance", got.Variance, want.Variance)
}

// TestCorrectLoopRigidTransform 验证校正对轨迹的整体平移加旋转：
// 锚点采用目标位姿，锚点之后各帧保持与锚点校正前的相对位置与朝向，
// 时间不变，朝向归一，方差按累计叠加。
func TestCorrectLoopRigidTransform(t *testing.T) {
	m := newMap(t, loopCfg())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
		{Time: 300, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
	}})
	if err != nil {
		t.Fatal(err)
	}

	// 锚点 t=200（原位姿 (2,0,0)）→ 目标 (5,5,π/2)，方差 0.5。
	rec, err := m.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 200, X: 5, Y: 5, Heading: math.Pi / 2, Variance: 0.5,
	})
	if err != nil {
		t.Fatal(err)
	}

	// 锚点帧采用目标位姿。
	approxEqPose(t, "anchor", rec.Poses[0].After, Pose{
		Time: 200, X: 5, Y: 5, Heading: math.Pi / 2, Variance: 0.5,
	})
	// t=300：相对锚点校正前为 (1,0)，旋转 π/2 后为 (0,1)，平移 (5,5) → (5,6)；
	// 朝向 π/2；方差 0.5 + (0.3-0.2) = 0.6。
	approxEqPose(t, "after", rec.Poses[1].After, Pose{
		Time: 300, X: 5, Y: 6, Heading: math.Pi / 2, Variance: 0.6,
	})
	// 末帧时间为 300。
	if rec.EndTime != 300 {
		t.Fatalf("end time = %d, want 300", rec.EndTime)
	}

	// 当前位姿立即反映校正结果。
	cur, _ := m.CurrentPose()
	approxEqPose(t, "current", cur, rec.Poses[1].After)

	// 历史查询同样返回校正后的结果。
	p200, _ := m.PoseAt(200)
	approxEqPose(t, "poseAt200", p200, rec.Poses[0].After)
	p150, _ := m.PoseAt(150) // 早于锚点，位姿不变
	approxEqPose(t, "poseAt150", p150, Pose{Time: 100, X: 1, Y: 0, Heading: 0, Variance: 0.1})

	// 锚点之前的位姿不变。
	if rec.Poses[0].Before.X != 2 || rec.Poses[0].Before.Y != 0 {
		t.Fatalf("anchor before = %+v", rec.Poses[0].Before)
	}
}

// TestCorrectLoopLandmarkReplay 验证校正后路标观测随位姿改变地图位置，
// 更早观测不变，同一路标按全部已接受观测取平均，标识与次数不变。
func TestCorrectLoopLandmarkReplay(t *testing.T) {
	m := newMap(t, loopCfg())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L1", X: 0, Y: 0}}}, // (1,0)
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L1", X: 0, Y: 0}}}, // (2,0)
		{Time: 300, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L2", X: 0, Y: 0}}}, // (3,0)
	}})
	if err != nil {
		t.Fatal(err)
	}

	// 校正锚点 t=200 → (5,5,π/2)。L1 受影响（t=200 观测），L2 受影响（t=300 观测）。
	rec, err := m.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 200, X: 5, Y: 5, Heading: math.Pi / 2, Variance: 0.5,
	})
	if err != nil {
		t.Fatal(err)
	}

	// L1：锚点前观测 (1,0) 不变；锚点观测校正后 (5,5)；平均 (3,2.5)，次数 2。
	// L2：仅 t=300 观测，校正后位姿 (5,6,π/2)，自身 (0,0) → 地图 (5,6)，次数 1。
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 2 {
		t.Fatalf("landmarks = %v", lms)
	}
	byID := map[string]Landmark{}
	for _, lm := range lms {
		byID[lm.ID] = lm
	}
	approxEq(t, "L1 x", byID["L1"].X, 3)
	approxEq(t, "L1 y", byID["L1"].Y, 2.5)
	if byID["L1"].Count != 2 {
		t.Fatalf("L1 count = %d, want 2", byID["L1"].Count)
	}
	approxEq(t, "L2 x", byID["L2"].X, 5)
	approxEq(t, "L2 y", byID["L2"].Y, 6)
	if byID["L2"].Count != 1 {
		t.Fatalf("L2 count = %d, want 1", byID["L2"].Count)
	}

	// 记录中包含受影响路标的前后值。
	if len(rec.Landmarks) != 2 {
		t.Fatalf("record landmarks = %v", rec.Landmarks)
	}
	for _, cl := range rec.Landmarks {
		switch cl.ID {
		case "L1":
			approxEq(t, "rec L1 before x", cl.Before.X, 1.5)
			approxEq(t, "rec L1 after x", cl.After.X, 3)
			approxEq(t, "rec L1 after y", cl.After.Y, 2.5)
		case "L2":
			approxEq(t, "rec L2 before x", cl.Before.X, 3)
			approxEq(t, "rec L2 after x", cl.After.X, 5)
			approxEq(t, "rec L2 after y", cl.After.Y, 6)
		}
	}
}

// TestCorrectLoopLandmarkConflict 验证校正后若观测与当前聚合超出合并距离，
// 整次校正被拒绝，并说明冲突帧时间与路标，且数据不变。
func TestCorrectLoopLandmarkConflict(t *testing.T) {
	cfg := loopCfg()
	cfg.MergeDistance = 1
	m := newMap(t, cfg)
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L", X: 0, Y: 0}}}, // (1,0)
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L", X: 0, Y: 0}}}, // (2,0)，距 (1,0)=1 合并
	}})
	if err != nil {
		t.Fatal(err)
	}

	// 校正锚点 t=200 → (10,0,0)。锚点观测校正后为 (10,0)，距锚点前聚合 (1,0) 为 9 > 1，冲突。
	_, err = m.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 200, X: 10, Y: 0, Heading: 0, Variance: 0.5,
	})
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("err = %v, want *RejectError", err)
	}
	if r.Kind != RejectLandmarkConflict {
		t.Fatalf("kind = %q, want %q", r.Kind, RejectLandmarkConflict)
	}
	if !r.HasFrameTime || r.FrameTime != 200 {
		t.Fatalf("frame time = %d(ok=%v), want 200", r.FrameTime, r.HasFrameTime)
	}
	if !r.HasLandmark || r.Landmark != "L" {
		t.Fatalf("landmark = %q(ok=%v), want L", r.Landmark, r.HasLandmark)
	}

	// 数据不变：当前位姿、路标、校正记录均保持拒绝前状态。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 2 {
		t.Fatalf("current = %+v, state leaked after rejection", cur)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].X != 1.5 || lms[0].Count != 2 {
		t.Fatalf("landmarks leaked: %v", lms)
	}
	recs, _ := m.Corrections()
	if len(recs) != 0 {
		t.Fatalf("correction records leaked: %v", recs)
	}
}

// TestCorrectLoopDuplicate 验证校正标识的重复提交语义：
// 同标识同内容返回首次结果且不修改数据；同标识不同内容明确拒绝。
func TestCorrectLoopDuplicate(t *testing.T) {
	m := newMap(t, loopCfg())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
	}})
	if err != nil {
		t.Fatal(err)
	}

	lc := LoopCorrection{ID: "c1", AnchorTime: 100, X: 5, Y: 5, Heading: 0, Variance: 0.5}
	rec1, err := m.CorrectLoop(lc)
	if err != nil {
		t.Fatal(err)
	}

	// 同标识同内容：返回首次结果，数据不变。
	rec2, err := m.CorrectLoop(lc)
	if err != nil {
		t.Fatal(err)
	}
	if rec2.AnchorTime != rec1.AnchorTime || rec2.Target.X != rec1.Target.X {
		t.Fatalf("duplicate returned %+v, want first %+v", rec2, rec1)
	}
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 6 { // 校正后 t=200 应为 (6,5)
		t.Fatalf("current = %+v, duplicate changed data", cur)
	}
	recs, _ := m.Corrections()
	if len(recs) != 1 {
		t.Fatalf("records = %d, want 1", len(recs))
	}

	// 同标识不同内容：明确拒绝。
	diff := lc
	diff.X = 99
	_, err = m.CorrectLoop(diff)
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectCorrectionDuplicateMismatch {
		t.Fatalf("err = %v, want duplicate mismatch", err)
	}

	// 原内容仍可重复提交。
	if _, err := m.CorrectLoop(lc); err != nil {
		t.Fatalf("original correction no longer accepted: %v", err)
	}
}

// TestCorrectLoopRejections 验证各类可区分的拒绝原因。
func TestCorrectLoopRejections(t *testing.T) {
	m := newMap(t, loopCfg())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
	}})
	if err != nil {
		t.Fatal(err)
	}

	cases := []struct {
		name string
		lc   LoopCorrection
		kind string
	}{
		{"empty id", LoopCorrection{ID: "", AnchorTime: 100, X: 1, Y: 1, Heading: 0, Variance: 0}, RejectEmptyCorrectionID},
		{"anchor not found", LoopCorrection{ID: "c", AnchorTime: 999, X: 1, Y: 1, Heading: 0, Variance: 0}, RejectAnchorNotFound},
		{"anchor is initial pose", LoopCorrection{ID: "c", AnchorTime: 0, X: 1, Y: 1, Heading: 0, Variance: 0}, RejectAnchorNotFound},
		{"negative variance", LoopCorrection{ID: "c", AnchorTime: 100, X: 1, Y: 1, Heading: 0, Variance: -0.1}, RejectNegativeTargetVariance},
		{"non-finite x", LoopCorrection{ID: "c", AnchorTime: 100, X: math.NaN(), Y: 1, Heading: 0, Variance: 0}, RejectCorrectionNonFinite},
		{"non-finite heading", LoopCorrection{ID: "c", AnchorTime: 100, X: 1, Y: 1, Heading: math.Inf(1), Variance: 0}, RejectCorrectionNonFinite},
		{"non-finite variance", LoopCorrection{ID: "c", AnchorTime: 100, X: 1, Y: 1, Heading: 0, Variance: math.NaN()}, RejectCorrectionNonFinite},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			_, err := m.CorrectLoop(c.lc)
			r, ok := AsRejectError(err)
			if !ok || r.Kind != c.kind {
				t.Fatalf("err = %v, want kind %q", err, c.kind)
			}
		})
	}

	// 全部失败后数据不变。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 2 {
		t.Fatalf("current = %+v, state leaked after rejection", cur)
	}
}

// TestCorrectLoopSubsequentImportContinues 验证校正后继续导入从新的末位姿接续。
func TestCorrectLoopSubsequentImportContinues(t *testing.T) {
	m := newMap(t, loopCfg())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
	}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 100, X: 5, Y: 5, Heading: 0, Variance: 0.5,
	}); err != nil {
		t.Fatal(err)
	}

	// 新段从校正后的末位姿 (6,5) 接续。
	res, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
	}})
	if err != nil {
		t.Fatal(err)
	}
	approxEq(t, "end x", res.EndPose.X, 7)
	approxEq(t, "end y", res.EndPose.Y, 5)
	approxEq(t, "end variance", res.EndPose.Variance, 0.7)
}

// TestCorrectLoopReimportOriginalSegment 验证校正后重复导入原轨迹段仍返回
// 首次导入结果，不撤销校正或增加路标次数。
func TestCorrectLoopReimportOriginalSegment(t *testing.T) {
	m := newMap(t, loopCfg())
	seg := Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
	}}
	res1, err := m.ImportSegment(seg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 100, X: 5, Y: 5, Heading: 0, Variance: 0.5,
	}); err != nil {
		t.Fatal(err)
	}

	// 重复导入原段：返回首次结果，不撤销校正。
	res2, err := m.ImportSegment(seg)
	if err != nil {
		t.Fatal(err)
	}
	if res2.EndPose.X != res1.EndPose.X || res2.EndPose.Time != res1.EndPose.Time {
		t.Fatalf("reimport returned %+v, want first %+v", res2, res1)
	}
	cur, _ := m.CurrentPose()
	if cur.X != 5 {
		t.Fatalf("current = %+v, correction undone by reimport", cur)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].Count != 1 {
		t.Fatalf("landmark count = %v, reimport added count", lms)
	}
}

// TestCorrectLoopMultipleCorrections 验证后续校正可覆盖相同时间范围，
// 以当时最新状态为起点，但不改写先前记录。
func TestCorrectLoopMultipleCorrections(t *testing.T) {
	m := newMap(t, loopCfg())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
	}})
	if err != nil {
		t.Fatal(err)
	}

	if _, err := m.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 100, X: 5, Y: 5, Heading: 0, Variance: 0.5,
	}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.CorrectLoop(LoopCorrection{
		ID: "c2", AnchorTime: 200, X: 10, Y: 10, Heading: 0, Variance: 1.0,
	}); err != nil {
		t.Fatal(err)
	}

	recs, _ := m.Corrections()
	if len(recs) != 2 {
		t.Fatalf("records = %d, want 2", len(recs))
	}
	// 记录按提交次序排列，且标识不同。
	if recs[0].ID != "c1" || recs[1].ID != "c2" {
		t.Fatalf("record ids = %q, %q", recs[0].ID, recs[1].ID)
	}
	// 第二次校正以第一次校正后的状态为起点：t=200 锚点采用目标 (10,10)。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 10 || cur.Y != 10 {
		t.Fatalf("current = %+v", cur)
	}
	// 第一次记录未被改写。
	if recs[0].Target.X != 5 {
		t.Fatalf("first record rewritten: %+v", recs[0])
	}
}

// TestCorrectLoopPersistence 验证关闭重开后校正结果、记录与两类重复提交
// 结果均保持一致。
func TestCorrectLoopPersistence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, loopCfg())
	if err != nil {
		t.Fatal(err)
	}
	_, err = m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
	}})
	if err != nil {
		t.Fatal(err)
	}
	seg := Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1},
	}}
	if _, err := m.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 100, X: 5, Y: 5, Heading: math.Pi / 2, Variance: 0.5,
	}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	// 当前位姿为校正后结果。
	cur, err := m2.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	approxEqPose(t, "reopen current", cur, Pose{
		Time: 200, X: 5, Y: 6, Heading: math.Pi / 2, Variance: 0.6,
	})
	// 历史查询一致。
	p100, _ := m2.PoseAt(100)
	approxEqPose(t, "reopen poseAt100", p100, Pose{
		Time: 100, X: 5, Y: 5, Heading: math.Pi / 2, Variance: 0.5,
	})
	// 路标位置一致。
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 {
		t.Fatalf("landmarks = %v", lms)
	}
	approxEq(t, "reopen lm x", lms[0].X, 5)
	approxEq(t, "reopen lm y", lms[0].Y, 5)
	if lms[0].Count != 1 {
		t.Fatalf("reopen lm count = %d", lms[0].Count)
	}
	// 校正记录一致。
	recs, err := m2.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 || recs[0].ID != "c1" {
		t.Fatalf("records = %v", recs)
	}
	// 两类重复提交结果一致。
	if _, err := m2.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 100, X: 5, Y: 5, Heading: math.Pi / 2, Variance: 0.5,
	}); err != nil {
		t.Fatalf("duplicate correction after reopen: %v", err)
	}
	if _, err := m2.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 100, X: 99, Y: 5, Heading: math.Pi / 2, Variance: 0.5,
	}); err == nil {
		t.Fatal("correction mismatch accepted after reopen")
	}
	if _, err := m2.ImportSegment(seg); err != nil {
		t.Fatalf("duplicate segment after reopen: %v", err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "s1", Frames: []Frame{{Time: 100, DX: 9}}}); err == nil {
		t.Fatal("segment mismatch accepted after reopen")
	}
}

// TestCorrectLoopClosed 验证关闭后校正相关操作返回 ErrClosed。
func TestCorrectLoopClosed(t *testing.T) {
	m := newMap(t, loopCfg())
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := m.CorrectLoop(LoopCorrection{ID: "c", AnchorTime: 100, X: 1, Y: 1, Heading: 0, Variance: 0}); !errors.Is(err, ErrClosed) {
		t.Fatalf("correct: %v", err)
	}
	if _, err := m.Corrections(); !errors.Is(err, ErrClosed) {
		t.Fatalf("corrections: %v", err)
	}
}

// downgradeToV1 把一份 v2 地图文件改写为 v1 格式（去掉逐帧观测与校正记录），
// 用于模拟旧版文件。
func downgradeToV1(t *testing.T, path string) {
	t.Helper()
	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	// 解析 payload。
	length := binary.BigEndian.Uint32(raw[10:14])
	var fd fileData
	if err := json.Unmarshal(raw[headerLen:headerLen+int(length)], &fd); err != nil {
		t.Fatal(err)
	}
	fd.Version = 1
	fd.FrameObservations = nil
	fd.Corrections = nil
	payload, err := json.Marshal(&fd)
	if err != nil {
		t.Fatal(err)
	}
	out := make([]byte, headerLen+len(payload)+4)
	copy(out[0:8], fileMagic)
	binary.BigEndian.PutUint16(out[8:10], 1)
	binary.BigEndian.PutUint32(out[10:14], uint32(len(payload)))
	copy(out[headerLen:], payload)
	sum := crc32.ChecksumIEEE(out[:headerLen+len(payload)])
	binary.BigEndian.PutUint32(out[headerLen+len(payload):], sum)
	if err := os.WriteFile(path, out, 0o644); err != nil {
		t.Fatal(err)
	}
}

// TestCorrectLoopLegacyFile 验证旧版文件（无逐帧观测来源）仍可正常打开、
// 查询和继续导入；对缺少完整依据的历史范围拒绝校正；新追加且依据完整
// 的范围仍可校正，旧观测的既有贡献保持不变。
func TestCorrectLoopLegacyFile(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, loopCfg())
	if err != nil {
		t.Fatal(err)
	}
	_, err = m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
		{Time: 200, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 改写为 v1 旧文件。
	downgradeToV1(t, path)

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	// 正常查询。
	cur, _ := m2.CurrentPose()
	if cur.Time != 200 || cur.X != 2 {
		t.Fatalf("legacy current = %+v", cur)
	}
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].X != 1.5 || lms[0].Count != 2 {
		t.Fatalf("legacy landmarks = %v", lms)
	}

	// 对缺少完整依据的历史范围（锚点 t=100，受影响帧均为旧帧）拒绝校正。
	_, err = m2.CorrectLoop(LoopCorrection{
		ID: "c1", AnchorTime: 100, X: 5, Y: 5, Heading: 0, Variance: 0.5,
	})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectIncompleteBasis {
		t.Fatalf("legacy correction err = %v, want incomplete basis", err)
	}
	// 数据不变。
	cur, _ = m2.CurrentPose()
	if cur.Time != 200 || cur.X != 2 {
		t.Fatalf("current = %+v, leaked after legacy rejection", cur)
	}

	// 继续导入新段（新帧具备完整观测来源）。
	if _, err := m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "L2", X: 0, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 对新追加且依据完整的范围（锚点 t=300，仅新帧）仍可校正。
	if _, err := m2.CorrectLoop(LoopCorrection{
		ID: "c2", AnchorTime: 300, X: 10, Y: 10, Heading: 0, Variance: 0.5,
	}); err != nil {
		t.Fatalf("new-range correction rejected: %v", err)
	}
	cur, _ = m2.CurrentPose()
	if cur.Time != 300 || cur.X != 10 || cur.Y != 10 {
		t.Fatalf("current after new-range correction = %+v", cur)
	}
	// 旧观测的既有贡献保持不变：L 仍为 (1.5,0)，次数 2。
	lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	byID := map[string]Landmark{}
	for _, lm := range lms {
		byID[lm.ID] = lm
	}
	if byID["L"].X != 1.5 || byID["L"].Count != 2 {
		t.Fatalf("L changed: %+v", byID["L"])
	}
	// L2 为新帧观测，随校正移动到 (10,10)。
	if byID["L2"].X != 10 || byID["L2"].Y != 10 || byID["L2"].Count != 1 {
		t.Fatalf("L2 = %+v", byID["L2"])
	}

	// 关闭重开后校正结果保持。
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}
	m3, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer m3.Close()
	cur, _ = m3.CurrentPose()
	if cur.X != 10 {
		t.Fatalf("reopen current = %+v", cur)
	}
	recs, _ := m3.Corrections()
	if len(recs) != 1 || recs[0].ID != "c2" {
		t.Fatalf("records = %v", recs)
	}
}
