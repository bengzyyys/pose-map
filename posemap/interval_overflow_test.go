package posemap

import (
	"math"
	"testing"
)

func hugeRect() Rect {
	return Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9}
}

func TestIntervalOverflowAcrossSignFirstFrame(t *testing.T) {
	// 初始时间为 MinInt64、间隔上限 1000：唯一一帧时间为 0 时真实间隔
	// 约 9.2e18 毫秒，直接相减会回绕成负值。必须按 interval_exceeded
	// 拒绝在第 0 帧，且不留下任何位姿、路标或段记录。
	cfg := Config{InitialTime: math.MinInt64, MaxInterval: 1000, MergeDistance: 1.0}
	m := newMap(t, cfg)
	_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 0, Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
	}})
	rejectKind(t, err, RejectInterval, 0, "")

	cur, _ := m.CurrentPose()
	if cur.Time != math.MinInt64 {
		t.Fatalf("current time = %d, initial pose leaked after rejection", cur.Time)
	}
	if lms, _ := m.LandmarksInRect(hugeRect()); len(lms) != 0 {
		t.Fatalf("landmarks leaked after rejection: %v", lms)
	}

	// 失败不得占用段标识：同一标识以合法内容再次导入应当成功。
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: math.MinInt64 + 1, Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
	}}); err != nil {
		t.Fatalf("segment id blocked by failed import: %v", err)
	}
	if lms, _ := m.LandmarksInRect(hugeRect()); len(lms) != 1 || lms[0].ID != "L" || lms[0].Count != 1 {
		t.Fatalf("landmarks after retry = %v", lms)
	}
}

func TestIntervalOverflowAcrossSignLaterFrameAtomic(t *testing.T) {
	// 段内先有时间 MinInt64+1 的合法帧（相对初始时间间隔 1），再出现
	// 时间 0 的帧：必须因间隔超限指向第 1 帧，且第 0 帧的位姿变化、
	// 新路标与观测次数都不得留下。
	m := newMap(t, Config{InitialTime: math.MinInt64, MaxInterval: 1000, MergeDistance: 1.0})
	_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: math.MinInt64 + 1, DX: 1, DY: 0, MoveVariance: 0.25,
			Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
		{Time: 0},
	}})
	rejectKind(t, err, RejectInterval, 1, "")

	cur, _ := m.CurrentPose()
	if cur.Time != math.MinInt64 {
		t.Fatalf("current time = %d, earlier frame pose leaked", cur.Time)
	}
	if cur.X != 0 || cur.Variance != 0 {
		t.Fatalf("pose change leaked: %+v", cur)
	}
	if lms, _ := m.LandmarksInRect(hugeRect()); len(lms) != 0 {
		t.Fatalf("earlier frame landmark leaked: %v", lms)
	}
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: math.MinInt64 + 1},
	}}); err != nil {
		t.Fatalf("segment id blocked by failed import: %v", err)
	}
}

func TestIntervalOverflowBoundaryMaxInterval(t *testing.T) {
	// 初始时间 MinInt64、间隔上限 MaxInt64：时间 -1 的首帧间隔恰为
	// MaxInt64，必须接受；时间 0 的首帧间隔为 MaxInt64+1，必须拒绝。
	cfg := func() Config {
		return Config{InitialTime: math.MinInt64, MaxInterval: math.MaxInt64, MergeDistance: 1.0}
	}

	mAccept := newMap(t, cfg())
	if _, err := mAccept.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: -1},
	}}); err != nil {
		t.Fatalf("frame at -1 with interval == MaxInt64 rejected: %v", err)
	}
	if cur, _ := mAccept.CurrentPose(); cur.Time != -1 {
		t.Fatalf("current time = %d, want -1", cur.Time)
	}

	mReject := newMap(t, cfg())
	_, err := mReject.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 0},
	}})
	rejectKind(t, err, RejectInterval, 0, "")
}

func TestIntervalOverflowFromExistingMapToNewSegment(t *testing.T) {
	// 已有地图末位姿为负、新段首帧为正且跨整数范围：间隔限制同样适用。
	m := newMap(t, Config{InitialTime: math.MinInt64, MaxInterval: 1000, MergeDistance: 1.0})
	if _, err := m.ImportSegment(Segment{ID: "a", Frames: []Frame{
		{Time: math.MinInt64 + 1},
	}}); err != nil {
		t.Fatal(err)
	}
	_, err := m.ImportSegment(Segment{ID: "b", Frames: []Frame{{Time: 0}}})
	rejectKind(t, err, RejectInterval, 0, "")

	// 与上一段末帧时间相等仍是 time_order，不能混成间隔超限。
	_, err = m.ImportSegment(Segment{ID: "b", Frames: []Frame{{Time: math.MinInt64 + 1}}})
	rejectKind(t, err, RejectTimeOrder, 0, "")

	if cur, _ := m.CurrentPose(); cur.Time != math.MinInt64+1 {
		t.Fatalf("current time = %d, rejected segment leaked", cur.Time)
	}
}

func TestNegativeTimesNormalIntervalBehavior(t *testing.T) {
	// 普通负时间范围内的边界行为保持不变：间隔恰为上限接受，跨正负号
	// 但跨度有限时按真实毫秒间隔判断。
	m := newMap(t, Config{InitialTime: -2000, MaxInterval: 1000, MergeDistance: 1.0})
	if _, err := m.ImportSegment(Segment{ID: "a", Frames: []Frame{
		{Time: -1000}, // 间隔恰为 1000
		{Time: -999},  // 段内间隔 1
	}}); err != nil {
		t.Fatalf("negative-time frames within limit rejected: %v", err)
	}
	// -999 -> 1 跨正负号，间隔恰为 1000，可接受。
	if _, err := m.ImportSegment(Segment{ID: "b", Frames: []Frame{{Time: 1}}}); err != nil {
		t.Fatalf("cross-sign gap == max interval rejected: %v", err)
	}
	_, err := m.ImportSegment(Segment{ID: "c", Frames: []Frame{{Time: 1002}}}) // 间隔 1001
	rejectKind(t, err, RejectInterval, 0, "")
}

func TestNegativeTimesTimeOrderNotInterval(t *testing.T) {
	// 时间相等或倒退即使发生在跨正负边界，也仍按 time_order 拒绝，
	// 不能被误判为间隔超限。
	m := newMap(t, Config{InitialTime: math.MinInt64, MaxInterval: 1000, MergeDistance: 1.0})

	// 首帧与初始时间相等。
	_, err := m.ImportSegment(Segment{ID: "eq", Frames: []Frame{{Time: math.MinInt64}}})
	rejectKind(t, err, RejectTimeOrder, 0, "")

	// 段内先有合法帧 MinInt64+1，随后时间相等与倒退都指向第 1 帧。
	_, err = m.ImportSegment(Segment{ID: "seg", Frames: []Frame{
		{Time: math.MinInt64 + 1},
		{Time: math.MinInt64 + 1},
	}})
	rejectKind(t, err, RejectTimeOrder, 1, "")
	_, err = m.ImportSegment(Segment{ID: "seg", Frames: []Frame{
		{Time: math.MinInt64 + 1},
		{Time: math.MinInt64},
	}})
	rejectKind(t, err, RejectTimeOrder, 1, "")
}
