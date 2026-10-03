package posemap

import (
	"math"
	"testing"
)

func frameAt(t int64) Frame {
	return Frame{Time: t, DX: 0, DY: 0, DHeading: 0, MoveVariance: 0}
}

// 初始时间 MinInt64、MaxInterval 1000：首帧时间 0 必须因间隔超限被拒绝，
// 且不留下任何状态变化；先有一帧 MinInt64+1 时拒绝应指向第 1 帧。
func TestIntervalOverflowRejected(t *testing.T) {
	m, err := Create(t.TempDir()+"/m.bin", Config{
		InitialTime: math.MinInt64, MaxInterval: 1000, MergeDistance: 1,
	})
	if err != nil {
		t.Fatal(err)
	}
	defer m.Close()

	_, err = m.ImportSegment(Segment{ID: "s1", Frames: []Frame{frameAt(0)}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectInterval || !r.HasFrame || r.Frame != 0 {
		t.Fatalf("want interval_exceeded at frame 0, got %v", err)
	}
	// 状态未被改变：当前位姿仍是初始位姿，重复导入同样被拒绝（未保存成功结果）。
	p, _ := m.CurrentPose()
	if p.Time != math.MinInt64 {
		t.Fatalf("state changed: %v", p)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{frameAt(0)}}); err == nil {
		t.Fatal("failed segment must not be recorded as imported")
	}

	// 段内第二帧超限：序号为 1。
	_, err = m.ImportSegment(Segment{ID: "s2", Frames: []Frame{frameAt(math.MinInt64 + 1), frameAt(0)}})
	r, ok = AsRejectError(err)
	if !ok || r.Kind != RejectInterval || r.Frame != 1 {
		t.Fatalf("want interval_exceeded at frame 1, got %v", err)
	}
	p, _ = m.CurrentPose()
	if p.Time != math.MinInt64 {
		t.Fatalf("partial segment leaked: %v", p)
	}
}

// 初始时间 MinInt64、MaxInterval MaxInt64：首帧 -1 接受，首帧 0 拒绝。
func TestIntervalOverflowBoundary(t *testing.T) {
	m, err := Create(t.TempDir()+"/m.bin", Config{
		InitialTime: math.MinInt64, MaxInterval: math.MaxInt64, MergeDistance: 1,
	})
	if err != nil {
		t.Fatal(err)
	}
	defer m.Close()

	if _, err := m.ImportSegment(Segment{ID: "ok", Frames: []Frame{frameAt(-1)}}); err != nil {
		t.Fatalf("frame at -1 must be accepted: %v", err)
	}

	m2, err := Create(t.TempDir()+"/m2.bin", Config{
		InitialTime: math.MinInt64, MaxInterval: math.MaxInt64, MergeDistance: 1,
	})
	if err != nil {
		t.Fatal(err)
	}
	defer m2.Close()
	_, err = m2.ImportSegment(Segment{ID: "bad", Frames: []Frame{frameAt(0)}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectInterval || r.Frame != 0 {
		t.Fatalf("frame at 0 must be rejected with interval_exceeded, got %v", err)
	}
}

// 时间相等/倒退仍按 time_order 拒绝，不混入 interval_exceeded。
func TestTimeOrderUnchanged(t *testing.T) {
	m, err := Create(t.TempDir()+"/m.bin", Config{
		InitialTime: math.MinInt64, MaxInterval: 1000, MergeDistance: 1,
	})
	if err != nil {
		t.Fatal(err)
	}
	defer m.Close()
	_, err = m.ImportSegment(Segment{ID: "s", Frames: []Frame{frameAt(math.MinInt64)}})
	r, ok := AsRejectError(err)
	if !ok || r.Kind != RejectTimeOrder || r.Frame != 0 {
		t.Fatalf("want time_order, got %v", err)
	}
}
