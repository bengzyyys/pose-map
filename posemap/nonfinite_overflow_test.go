package posemap

import (
	"math"
	"os"
	"path/filepath"
	"testing"
)

// 输入全部为有限数值时，导入推演仍可能产生非有限结果：位置叠加、朝向
// 累计、方差累计溢出。这些情况必须按 non_finite 拒绝并指出从零开始的
// 帧序号，而不是在保存时返回普通编码错误。
func TestImportComputedNonFinitePose(t *testing.T) {
	huge := func() Config {
		return Config{
			InitialTime: 0, InitialX: 1e308, InitialY: 0, InitialHeading: 0,
			InitialVariance: 0, MaxInterval: 100, MergeDistance: 10,
		}
	}
	type tc struct {
		name   string
		cfg    Config
		frames []Frame
		frame  int
	}
	cases := []tc{
		{
			name:   "position overflow on first frame",
			cfg:    huge(),
			frames: []Frame{{Time: 1, DX: 1e308}}, // 1e308 + 1e308 = +Inf
			frame:  0,
		},
		{
			name: "position overflow on later frame",
			cfg:  huge(),
			frames: []Frame{
				{Time: 1}, // 1e308+1 舍入为 1e308，仍有限
				{Time: 2},
				{Time: 3, DX: 1e308}, // 这一帧才溢出
			},
			frame: 2,
		},
		{
			name: "variance accumulation overflow",
			cfg: func() Config {
				c := huge()
				c.InitialX = 0
				c.InitialVariance = 1e308
				return c
			}(),
			frames: []Frame{{Time: 1, MoveVariance: 1e308}}, // 1e308+1e308 = +Inf
			frame:  0,
		},
		{
			name: "heading accumulation overflow",
			cfg: func() Config {
				c := huge()
				c.InitialX = 0
				c.InitialHeading = 1e308
				return c
			}(),
			frames: []Frame{{Time: 1, DHeading: 1e308}}, // 归一化后为 NaN
			frame:  0,
		},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			m := newMap(t, c.cfg)
			_, err := m.ImportSegment(Segment{ID: "s", Frames: c.frames})
			r, ok := AsRejectError(err)
			if !ok {
				t.Fatalf("err = %v, want *RejectError", err)
			}
			if r.Kind != RejectNonFinite || !r.HasFrame || r.Frame != c.frame {
				t.Fatalf("got kind=%q frame=%d(ok=%v), want non_finite frame %d", r.Kind, r.Frame, r.HasFrame, c.frame)
			}
			if r.HasLandmark {
				t.Fatalf("pose rejection unexpectedly carries landmark %q", r.Landmark)
			}
			// 拒绝后当前位姿仍是初始位姿。
			cur, _ := m.CurrentPose()
			if cur.Time != 0 {
				t.Fatalf("current time = %d, state leaked after rejection", cur.Time)
			}
		})
	}
}

// 观测转换到地图坐标后溢出时，无论该路标首次出现、失效后再次出现，还是
// 与有效记录（段内暂存或已提交记录）合并，都按 non_finite 拒绝并携带
// 路标标识；转换结果有限但超过合并距离时仍是原来的距离冲突。
func TestImportComputedNonFiniteObservation(t *testing.T) {
	hugeCfg := Config{
		InitialTime: 0, InitialX: 1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 100, MergeDistance: 10,
	}

	t.Run("first appearance overflow", func(t *testing.T) {
		m := newMap(t, hugeCfg)
		_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 1, Observations: []Observation{{ID: "L", X: 1e308}}}, // 地图 X = +Inf
		}})
		rejectKind(t, err, RejectNonFinite, 0, "L")
		lms, _ := m.LandmarksInRect(Rect{MinX: -math.MaxFloat64, MinY: -math.MaxFloat64, MaxX: math.MaxFloat64, MaxY: math.MaxFloat64})
		if len(lms) != 0 {
			t.Fatalf("landmark leaked: %+v", lms)
		}
	})

	t.Run("in-segment merge overflow not conflict", func(t *testing.T) {
		m := newMap(t, hugeCfg)
		_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 1, Observations: []Observation{{ID: "L", X: 0}}},     // 地图 (1e308,0)
			{Time: 2, Observations: []Observation{{ID: "L", X: 1e308}}}, // 转换溢出
		}})
		rejectKind(t, err, RejectNonFinite, 1, "L")
		lms, _ := m.LandmarksInRect(Rect{MinX: -math.MaxFloat64, MinY: -math.MaxFloat64, MaxX: math.MaxFloat64, MaxY: math.MaxFloat64})
		if len(lms) != 0 {
			t.Fatalf("staged landmark leaked after rejection: %+v", lms)
		}
		cur, _ := m.CurrentPose()
		if cur.Time != 0 {
			t.Fatalf("current time = %d, pose leaked", cur.Time)
		}
	})

	t.Run("merge with committed active overflow", func(t *testing.T) {
		m := newMap(t, hugeCfg)
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 1, Observations: []Observation{{ID: "L", X: 0}}},
		}}); err != nil {
			t.Fatal(err)
		}
		_, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 2, Observations: []Observation{{ID: "L", X: 1e308}}},
		}})
		rejectKind(t, err, RejectNonFinite, 0, "L")
		// 已提交记录的观测次数不被暂存合并改变。
		h, err := m.LandmarkAppearances("L")
		if err != nil || len(h.Appearances) != 1 || h.Appearances[0].Landmark.Count != 1 {
			t.Fatalf("committed count changed: %+v %v", h, err)
		}
	})

	t.Run("reappearance after invalidation overflow", func(t *testing.T) {
		m := newMap(t, hugeCfg)
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 1, Observations: []Observation{{ID: "L", X: 0}}},
		}}); err != nil {
			t.Fatal(err)
		}
		if _, err := m.Invalidate(Invalidation{ID: "iv", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
			t.Fatal(err)
		}
		// 失效后再次出现：转换溢出同样按数值异常拒绝。
		_, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 2, Observations: []Observation{{ID: "L", X: 1e308}}},
		}})
		rejectKind(t, err, RejectNonFinite, 0, "L")
		h, _ := m.LandmarkAppearances("L")
		if len(h.Appearances) != 1 || h.Appearances[0].Active {
			t.Fatalf("rejection consumed an occurrence number: %+v", h)
		}
	})

	t.Run("finite but beyond merge distance stays conflict", func(t *testing.T) {
		cfg := hugeCfg
		cfg.MergeDistance = 1
		m := newMap(t, cfg)
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 1, Observations: []Observation{{ID: "L", X: 0}}}, // 地图 1e308
		}}); err != nil {
			t.Fatal(err)
		}
		// 地图坐标 1.1e308，有限但与既有均值相距 1e307，超出合并距离。
		_, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 2, Observations: []Observation{{ID: "L", X: 1e307}}},
		}})
		rejectKind(t, err, RejectLandmarkConflict, 0, "L")
	})
}

// 拒绝保持原子性：即使异常出现在后面的帧，前面暂存的位姿、新路标和观测
// 次数都不可见；已提交地图和本地数据文件保持原样；失败不消耗段标识，
// 也不消耗失效后再次出现的出现编号。
func TestImportComputedNonFiniteAtomicity(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.bin")
	cfg := Config{
		InitialTime: 0, InitialX: 1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 100, MergeDistance: 10,
	}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	defer m.Close()

	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 1, Observations: []Observation{{ID: "L", X: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "iv", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}

	before, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}

	// 异常出现在第 2 帧（帧序号 1）：第 0 帧的暂存位姿与观测必须随拒绝丢弃。
	_, err = m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 2, Observations: []Observation{{ID: "N", X: 0}}},
		{Time: 3, Observations: []Observation{{ID: "L", X: 1e308}}},
	}})
	rejectKind(t, err, RejectNonFinite, 1, "L")

	// 当前位姿仍是上一段末帧；新路标 N 不可见；L 仍处于失效状态且只有 1 次出现。
	cur, _ := m.CurrentPose()
	if cur.Time != 1 {
		t.Fatalf("current time = %d, staged pose leaked", cur.Time)
	}
	if h, err := m.LandmarkAppearances("N"); err == nil {
		t.Fatalf("staged new landmark N leaked: %+v", h)
	}
	h, _ := m.LandmarkAppearances("L")
	if len(h.Appearances) != 1 || h.Appearances[0].Active {
		t.Fatalf("occurrence number consumed by failed import: %+v", h)
	}

	// 本地数据文件字节不变。
	after, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if len(after) != len(before) {
		t.Fatalf("data file changed size: %d -> %d", len(before), len(after))
	}
	for i := range before {
		if before[i] != after[i] {
			t.Fatalf("data file byte %d changed", i)
		}
	}

	// 段标识未被占用：同一 ID 修正内容后导入成功。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 2, Observations: []Observation{{ID: "L", X: 0}}},
	}}); err != nil {
		t.Fatalf("segment id consumed by failed import: %v", err)
	}
	// 失败没有消耗再现编号：L 的新出现仍是第 2 次，计数从 1 开始。
	h, _ = m.LandmarkAppearances("L")
	if len(h.Appearances) != 2 {
		t.Fatalf("want 2 appearances, got %d: %+v", len(h.Appearances), h)
	}
	if h.Appearances[0].Number != 1 || h.Appearances[0].Active {
		t.Fatalf("occurrence 1 changed: %+v", h.Appearances[0])
	}
	a2 := h.Appearances[1]
	if a2.Number != 2 || !a2.Active || a2.Landmark.Count != 1 || a2.Landmark.X != 1e308 || a2.Landmark.Y != 0 {
		t.Fatalf("reappearance = %+v, want #2 active count 1 at (1e308,0)", a2)
	}

	// 落盘结果一致：重开后状态相同，且 s2 已作为成功段记录。
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer m2.Close()
	cur, _ = m2.CurrentPose()
	if cur.Time != 2 {
		t.Fatalf("after reopen current time = %d", cur.Time)
	}
	h2, err := m2.LandmarkAppearances("L")
	if err != nil || len(h2.Appearances) != 2 || !h2.Appearances[1].Active || h2.Appearances[1].Landmark.Count != 1 {
		t.Fatalf("after reopen appearances: %+v %v", h2, err)
	}
	// 同 ID 同内容重复导入返回首次结果。
	if _, err := m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 2, Observations: []Observation{{ID: "L", X: 0}}},
	}}); err != nil {
		t.Fatalf("duplicate import after reopen: %v", err)
	}
}

// 大坐标本身不受限制：位姿在 1e308 处保持有限即接受，同一路标连续两次
// 转换到同一地图坐标 (1e308,0) 合并后仍是 (1e308,0)、次数 2。
func TestImportHugeButFiniteAccepted(t *testing.T) {
	m := newMap(t, Config{
		InitialTime: 0, InitialX: 1e308, InitialY: 0, InitialHeading: 0,
		InitialVariance: 1e308, MaxInterval: 100, MergeDistance: 10,
	})
	res, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 1, MoveVariance: 0, Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
		{Time: 2, MoveVariance: 0, Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
	}})
	if err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	if res.EndPose.X != 1e308 || res.EndPose.Variance != 1e308 {
		t.Fatalf("end pose = %+v, want X=1e308 variance 1e308", res.EndPose)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: 0, MinY: -1, MaxX: math.MaxFloat64, MaxY: 1})
	if len(lms) != 1 || lms[0].X != 1e308 || lms[0].Y != 0 || lms[0].Count != 2 {
		t.Fatalf("landmark = %+v, want (1e308,0) count 2", lms)
	}
}
