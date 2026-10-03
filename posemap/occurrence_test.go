package posemap

import "testing"

// ownsTime 直接覆盖四类出现的边界规则：首见端点包含、失效端点包含、
// 有效出现无上界、首见未知（旧文件第 1 次出现）无下界。
func TestOccurrenceBoundsOwnsTime(t *testing.T) {
	cases := []struct {
		name   string
		b      occurrenceTimeBounds
		t      int64
		expect bool
	}{
		// 已知首次观测时间且仍有效：[firstSeen, +∞)。
		{"bounded active at first seen", occurrenceTimeBounds{hasFirst: true, firstSeen: 100, active: true}, 100, true},
		{"bounded active after first seen", occurrenceTimeBounds{hasFirst: true, firstSeen: 100, active: true}, 250, true},
		{"bounded active before first seen", occurrenceTimeBounds{hasFirst: true, firstSeen: 100, active: true}, 99, false},

		// 已知首次观测时间且已失效：[firstSeen, invalidTime]，两端点包含。
		{"closed at first seen", occurrenceTimeBounds{hasFirst: true, firstSeen: 100, invalidTime: 200}, 100, true},
		{"closed inside", occurrenceTimeBounds{hasFirst: true, firstSeen: 100, invalidTime: 200}, 150, true},
		{"closed at invalid time", occurrenceTimeBounds{hasFirst: true, firstSeen: 100, invalidTime: 200}, 200, true},
		{"closed after invalid time", occurrenceTimeBounds{hasFirst: true, firstSeen: 100, invalidTime: 200}, 201, false},
		{"closed before first seen", occurrenceTimeBounds{hasFirst: true, firstSeen: 100, invalidTime: 200}, 99, false},

		// 旧文件第 1 次出现仍有效：首见未知，任何时间都接纳（包括 0
		// 与负值）——零时间不是补造的开始时间。
		{"legacy active at zero", occurrenceTimeBounds{active: true}, 0, true},
		{"legacy active negative", occurrenceTimeBounds{active: true}, -100, true},
		{"legacy active positive", occurrenceTimeBounds{active: true}, 1 << 40, true},

		// 首见未知但已失效：(-∞, invalidTime]，只有失效之后不接纳。
		{"legacy closed at invalid time", occurrenceTimeBounds{invalidTime: 200}, 200, true},
		{"legacy closed at zero", occurrenceTimeBounds{invalidTime: 200}, 0, true},
		{"legacy closed negative", occurrenceTimeBounds{invalidTime: 200}, -100, true},
		{"legacy closed after invalid time", occurrenceTimeBounds{invalidTime: 200}, 201, false},
	}
	for _, c := range cases {
		if got := c.b.ownsTime(c.t); got != c.expect {
			t.Errorf("%s: ownsTime(%d) = %v, want %v", c.name, c.t, got, c.expect)
		}
	}
}

// occurrenceIndexAtTime 在互不重叠的出现序列上按“当时”归属；落在间隙
// （失效之后、下一次首见之前）的时间没有归属。
func TestOccurrenceIndexAtTime(t *testing.T) {
	// occ1: [100,200] 已失效；occ2: [300,+∞) 仍有效。
	bounds := []occurrenceTimeBounds{
		{hasFirst: true, firstSeen: 100, invalidTime: 200},
		{hasFirst: true, firstSeen: 300, active: true},
	}
	want := map[int64]int{
		100: 0, 150: 0, 200: 0,
		300: 1, 400: 1,
	}
	for at, idx := range want {
		got, ok := occurrenceIndexAtTime(bounds, at)
		if !ok || got != idx {
			t.Errorf("t=%d: index=%d ok=%v, want %d true", at, got, ok, idx)
		}
	}
	for _, at := range []int64{99, 201, 250, 299} {
		if _, ok := occurrenceIndexAtTime(bounds, at); ok {
			t.Errorf("t=%d falls between occurrences, must have no owner", at)
		}
	}
	// 旧文件第 1 次出现无下界：最早的历史时间也有归属。
	legacy := []occurrenceTimeBounds{{active: true}}
	if idx, ok := occurrenceIndexAtTime(legacy, -1<<62); !ok || idx != 0 {
		t.Errorf("legacy occurrence must own unbounded history: idx=%d ok=%v", idx, ok)
	}
}

// landmarkState.occurrenceNumberAt 给出 1 起编号；nil、空状态与间隙
// 返回 0。
func TestOccurrenceNumberAt(t *testing.T) {
	var nilLM *landmarkState
	if n := nilLM.occurrenceNumberAt(100); n != 0 {
		t.Errorf("nil landmark: number = %d, want 0", n)
	}
	empty := &landmarkState{}
	if n := empty.occurrenceNumberAt(100); n != 0 {
		t.Errorf("empty landmark: number = %d, want 0", n)
	}
	lm := &landmarkState{appearances: []*occurrenceState{
		{hasFirstSeen: true, firstSeenTime: 100, active: false, invalidTime: 200},
		{hasFirstSeen: true, firstSeenTime: 300, active: true},
	}}
	want := map[int64]int{100: 1, 200: 1, 250: 0, 300: 2, 500: 2}
	for at, num := range want {
		if got := lm.occurrenceNumberAt(at); got != num {
			t.Errorf("t=%d: number = %d, want %d", at, got, num)
		}
	}
}
