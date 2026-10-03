package posemap

import (
	"math"
	"path/filepath"
	"testing"
)

// 大数值坐标下的等权平均：旧实现用 (mean*count+v)/(count+1)，在平均值
// 接近 float64 上限时 mean*count 溢出为 Inf，导致虽每次观测有限且满足
// 合并距离、正确平均值仍有限，导入却被拒绝（JSON 无法编码 Inf）。
func TestImportMergeHugeCoordinates(t *testing.T) {
	cfg := func() Config {
		return Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 3e307}
	}

	// 机器人在原点、朝向为零且不移动：同一路标连续两次 (1e308,0)，
	// 应保存为 (1e308,0)，次数 2，不得拒绝。
	m := newMap(t, cfg())
	_, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 1, Observations: []Observation{{ID: "L", X: 1e308, Y: 0}}},
		{Time: 2, Observations: []Observation{{ID: "L", X: 1e308, Y: 0}}},
	}})
	if err != nil {
		t.Fatalf("两次 1e308 观测被拒绝: %v", err)
	}
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	lm := h.Appearances[0].Landmark
	if lm.X != 1e308 || lm.Y != 0 || lm.Count != 2 {
		t.Fatalf("L = %+v, want (1e308,0) count 2", lm)
	}
	// 区域查询立即反映合并后的真实位置与次数。
	lms, err := m.LandmarksInRect(Rect{MinX: 1e308 - 1e293, MaxX: 1e308 + 1e293, MinY: -1, MaxY: 1})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].X != 1e308 || lms[0].Count != 2 {
		t.Fatalf("区域查询 = %+v", lms)
	}

	// 两次 (8e307,0) 后再来一次 (1e308,0)，合并距离 3e307：三次都应接受，
	// 均值约 8.666666666666667e307，次数 3；既有次数必须参与权重，且
	// 结果有限并落在参与平均的坐标范围内。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 3, Observations: []Observation{{ID: "K", X: 8e307, Y: 0}}},
		{Time: 4, Observations: []Observation{{ID: "K", X: 8e307, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// 第三次观测位于另一段：复制的既有出现（次数 2）必须正确参与权重。
	if _, err := m.ImportSegment(Segment{ID: "s3", Frames: []Frame{
		{Time: 5, Observations: []Observation{{ID: "K", X: 1e308, Y: 0}}},
	}}); err != nil {
		t.Fatalf("跨段并入 1e308 观测被拒绝: %v", err)
	}
	hk, err := m.LandmarkAppearances("K")
	if err != nil {
		t.Fatal(err)
	}
	km := hk.Appearances[0].Landmark
	wantK := (8e307 + 8e307 + 1e308) / 3.0
	if !isFinite(km.X) || math.Abs(km.X-wantK) > 4e292 {
		t.Fatalf("K.X = %v, want ~%v 且有限", km.X, wantK)
	}
	if km.X < 8e307 || km.X > 1e308 || km.Count != 3 {
		t.Fatalf("K = %+v，应落在 [8e307,1e308] 且次数 3", km)
	}

	// 纵坐标与负数坐标遵守同一规则。
	if _, err := m.ImportSegment(Segment{ID: "s4", Frames: []Frame{
		{Time: 6, Observations: []Observation{{ID: "N", X: -1e308, Y: -1e308}}},
		{Time: 7, Observations: []Observation{{ID: "N", X: -1e308, Y: -1e308}}},
	}}); err != nil {
		t.Fatalf("负坐标观测被拒绝: %v", err)
	}
	hn, _ := m.LandmarkAppearances("N")
	if nm := hn.Appearances[0].Landmark; nm.X != -1e308 || nm.Y != -1e308 || nm.Count != 2 {
		t.Fatalf("N = %+v", nm)
	}

	// 恰好等于合并距离仍可接受（浮点比较相等）。
	if _, err := m.ImportSegment(Segment{ID: "s5", Frames: []Frame{
		{Time: 8, Observations: []Observation{{ID: "E", X: 0, Y: 0}}},
		{Time: 9, Observations: []Observation{{ID: "E", X: 3e307, Y: 0}}},
	}}); err != nil {
		t.Fatalf("恰在合并距离上限被拒绝: %v", err)
	}

	// 超过上限仍按原路标冲突拒绝，并指出帧与路标；整段不留部分更新。
	_, err = m.ImportSegment(Segment{ID: "s6", Frames: []Frame{
		{Time: 10, Observations: []Observation{{ID: "C", X: 0, Y: 0}}},
		{Time: 11, Observations: []Observation{{ID: "C", X: 3e307 + 1e292, Y: 0}}},
	}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectLandmarkConflict || r.Frame != 1 || r.Landmark != "C" {
		t.Fatalf("超限观测 err = %v (%+v)", err, r)
	}
	if _, err := m.LandmarkAppearances("C"); err == nil {
		t.Fatalf("被拒整段留下了路标 C 的部分更新")
	}

	// 非有限输入继续按既有规则拒绝。
	_, err = m.ImportSegment(Segment{ID: "s7", Frames: []Frame{
		{Time: 12, Observations: []Observation{{ID: "Z", X: math.Inf(1), Y: 0}}},
	}})
	if r, ok := AsRejectError(err); !ok || r.Kind != RejectNonFinite {
		t.Fatalf("非有限输入 err = %v", err)
	}

	// 落盘与重新打开结果一致（Inf 曾使 JSON 编码失败）。
	path := m.Path()
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("Open: %v", err)
	}
	t.Cleanup(func() { _ = m2.Close() })
	h2, err := m2.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if lm := h2.Appearances[0].Landmark; lm.X != 1e308 || lm.Count != 2 {
		t.Fatalf("重开后 L = %+v", lm)
	}
	h2k, _ := m2.LandmarkAppearances("K")
	if km := h2k.Appearances[0].Landmark; km.Count != 3 || !isFinite(km.X) {
		t.Fatalf("重开后 K = %+v", km)
	}
}

// 回环校正按新位姿重放观测时，大数值坐标下平均位置仍必须有限；校正记录
// 的校正后位置与校正完成时的查询一致，观测次数不变。
func TestCorrectionReplayHugeCoordinates(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 3e307}
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1e308, Y: 0}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1e308, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 整体平移 1e293（该量级处可表示）：两次校正后观测仍相同，距离 0，
	// 合并距离满足；均值有限、次数 2。
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 1e293, Y: 0, Heading: 0}})
	if err != nil {
		t.Fatalf("大数值坐标校正被拒绝: %v", err)
	}
	if len(rec.Landmarks) != 1 {
		t.Fatalf("landmark changes = %+v", rec.Landmarks)
	}
	lc := rec.Landmarks[0]
	if !isFinite(lc.After.X) || !isFinite(lc.After.Y) || lc.After.Count != 2 {
		t.Fatalf("校正后 = %+v，应有限且次数 2", lc.After)
	}
	if lc.Before.Count != lc.After.Count {
		t.Fatalf("观测次数被改变: %d -> %d", lc.Before.Count, lc.After.Count)
	}
	// 校正记录与校正完成时的查询、区域查询一致。
	h, _ := m.LandmarkAppearances("L")
	if qm := h.Appearances[0].Landmark; qm.X != lc.After.X || qm.Y != lc.After.Y || qm.Count != 2 {
		t.Fatalf("记录 %+v != 查询 %+v", lc, qm)
	}
	lms, err := m.LandmarksInRect(Rect{
		MinX: lc.After.X - 1e293, MaxX: lc.After.X + 1e293,
		MinY: lc.After.Y - 1, MaxY: lc.After.Y + 1,
	})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 || lms[0].X != lc.After.X || lms[0].Count != 2 {
		t.Fatalf("区域查询 = %+v，应与记录 %+v 一致", lms, lc.After)
	}
}

// 大数值坐标下校正重放超过合并距离仍返回原路标冲突（含冲突帧时间、路标
// 与出现编号），整次校正不留部分更新。
func TestCorrectionReplayHugeConflict(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 3e307}
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = m.Close() })
	// 帧 50 的观测在校正范围之前（固定贡献，次数 1）；帧 100/200 受影响。
	if _, err := m.ImportSegment(Segment{ID: "s0", Frames: []Frame{
		{Time: 50, Observations: []Observation{{ID: "L", X: 1e308, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1e308, Y: 0}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1e308, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 平移 4e307：校正后首个受影响观测（帧 100）与处理它之前的固定均值
	// 距离 4e307 > 3e307，必须冲突拒绝。
	_, err = m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 4e307, Y: 0, Heading: 0}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectLandmarkConflict || r.Time != 100 ||
		r.Landmark != "L" || r.Occurrence != 1 || !r.HasOccurrence {
		t.Fatalf("err = %v (%+v)", err, r)
	}

	// 不留部分更新：位姿、路标均值/次数、校正记录均保持校正前状态。
	cur, _ := m.CurrentPose()
	if cur.Time != 200 || cur.X != 0 || cur.Y != 0 {
		t.Fatalf("失败校正改动了位姿: %+v", cur)
	}
	h, _ := m.LandmarkAppearances("L")
	if qm := h.Appearances[0].Landmark; qm.X != 1e308 || qm.Count != 3 {
		t.Fatalf("失败校正改动了路标: %+v", qm)
	}
	if recs, _ := m.Corrections(); len(recs) != 0 {
		t.Fatalf("失败校正留下了校正记录")
	}
	// 重新打开磁盘文件，确认失败时未落任何部分更新。
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = m2.Close() })
	h2, _ := m2.LandmarkAppearances("L")
	if qm := h2.Appearances[0].Landmark; qm.X != 1e308 || qm.Count != 3 {
		t.Fatalf("磁盘文件含部分更新: %+v", qm)
	}
}
