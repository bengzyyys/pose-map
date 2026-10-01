package posemap

import (
	"encoding/binary"
	"errors"
	"hash/crc32"
	"math"
	"os"
	"path/filepath"
	"testing"
)

func approxEq(t *testing.T, name string, got, want float64) {
	t.Helper()
	if math.IsNaN(want) {
		if !math.IsNaN(got) {
			t.Fatalf("%s = %v, want NaN", name, got)
		}
		return
	}
	if math.Abs(got-want) > 1e-10 {
		t.Fatalf("%s = %v, want %v", name, got, want)
	}
}

func baseConfig() Config {
	return Config{
		InitialTime:     0,
		InitialX:        1,
		InitialY:        2,
		InitialHeading:  0,
		InitialVariance: 0.5,
		MaxInterval:     1000,
		MergeDistance:   1.0,
	}
}

func newMap(t *testing.T, cfg Config) *Map {
	t.Helper()
	m, err := Create(filepath.Join(t.TempDir(), "map.pose"), cfg)
	if err != nil {
		t.Fatalf("Create: %v", err)
	}
	t.Cleanup(func() { _ = m.Close() })
	return m
}

func TestInvalidConfig(t *testing.T) {
	dir := t.TempDir()
	cases := []Config{
		baseConfig(),
	}
	{
		c := baseConfig()
		c.MaxInterval = 0
		cases = append(cases, c)
	}
	{
		c := baseConfig()
		c.MaxInterval = -1
		cases = append(cases, c)
	}
	{
		c := baseConfig()
		c.MergeDistance = 0
		cases = append(cases, c)
	}
	{
		c := baseConfig()
		c.MergeDistance = -2
		cases = append(cases, c)
	}
	{
		c := baseConfig()
		c.InitialVariance = -0.1
		cases = append(cases, c)
	}
	{
		c := baseConfig()
		c.MergeDistance = math.NaN()
		cases = append(cases, c)
	}
	{
		c := baseConfig()
		c.InitialX = math.Inf(1)
		cases = append(cases, c)
	}
	if err := validateConfig(cases[0]); err != nil {
		t.Fatalf("base config invalid: %v", err)
	}
	for i, c := range cases[1:] {
		if err := validateConfig(c); !errors.Is(err, ErrInvalidConfig) {
			t.Fatalf("case %d: got %v, want ErrInvalidConfig", i, err)
		}
		if _, err := Create(filepath.Join(dir, "x"), c); !errors.Is(err, ErrInvalidConfig) {
			t.Fatalf("case %d Create: got %v, want ErrInvalidConfig", i, err)
		}
	}
}

func TestKinematicsAndLandmarkMerge(t *testing.T) {
	m := newMap(t, baseConfig())

	seg := Segment{
		ID: "s1",
		Frames: []Frame{
			{
				Time: 100, DX: 3, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.25,
				Observations: []Observation{{ID: "L1", X: 1, Y: 0}}, // 运动后位姿：(4,3)
			},
			{
				Time: 200, DX: 2, DY: 0, DHeading: 0, MoveVariance: 0.25,
				Observations: []Observation{{ID: "L1", X: 0, Y: 0}}, // (4,4)，与 (4,3) 距离恰为 1
			},
		},
	}
	res, err := m.ImportSegment(seg)
	if err != nil {
		t.Fatalf("ImportSegment: %v", err)
	}
	// 末位姿：平移按运动前朝向 0 旋转 → (4,2)；第二帧朝向 π/2，自身 x 朝世界 y。
	approxEq(t, "end x", res.EndPose.X, 4)
	approxEq(t, "end y", res.EndPose.Y, 4)
	approxEq(t, "end heading", res.EndPose.Heading, math.Pi/2)
	approxEq(t, "end variance", res.EndPose.Variance, 1.0)
	if res.EndPose.Time != 200 {
		t.Fatalf("end time = %d", res.EndPose.Time)
	}
	if got, want := len(res.LandmarkIDs), 1; got != want || res.LandmarkIDs[0] != "L1" {
		t.Fatalf("landmark ids = %v", res.LandmarkIDs)
	}

	cur, err := m.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	approxEq(t, "cur x", cur.X, 4)
	approxEq(t, "cur variance", cur.Variance, 1.0)

	// 路标为两次观测的算术平均 (4, 3.5)，次数 2。
	lms, err := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 1 {
		t.Fatalf("landmarks = %v", lms)
	}
	approxEq(t, "lm x", lms[0].X, 4)
	approxEq(t, "lm y", lms[0].Y, 3.5)
	if lms[0].Count != 2 {
		t.Fatalf("count = %d", lms[0].Count)
	}
}

func TestHeadingNormalized(t *testing.T) {
	// π/2 + 3π = 7π/2，归一到 [-π,π) 为 -π/2。
	r := normalizeAngle(math.Pi/2 + 3*math.Pi)
	approxEq(t, "7pi/2", r, -math.Pi/2)
	approxEq(t, "pi wraps to -pi", normalizeAngle(math.Pi), -math.Pi)
	approxEq(t, "-pi stays", normalizeAngle(-math.Pi), -math.Pi)
	approxEq(t, "zero", normalizeAngle(0), 0)

	m := newMap(t, baseConfig())
	_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, DX: 0, DY: 0, DHeading: 3 * math.Pi, MoveVariance: 0},
	}})
	if err != nil {
		t.Fatal(err)
	}
	cur, _ := m.CurrentPose()
	approxEq(t, "stored heading", cur.Heading, -math.Pi)
}

func rejectKind(t *testing.T, err error, wantKind string, wantFrame int, wantLandmark string) {
	t.Helper()
	r, ok := AsRejectError(err)
	if !ok {
		t.Fatalf("err = %v, want *RejectError", err)
	}
	if r.Kind != wantKind {
		t.Fatalf("kind = %q, want %q", r.Kind, wantKind)
	}
	if wantFrame >= 0 && (!r.HasFrame || r.Frame != wantFrame) {
		t.Fatalf("frame = %d(ok=%v), want %d", r.Frame, r.HasFrame, wantFrame)
	}
	if wantLandmark != "" {
		if !r.HasLandmark || r.Landmark != wantLandmark {
			t.Fatalf("landmark = %q(ok=%v), want %q", r.Landmark, r.HasLandmark, wantLandmark)
		}
	}
}

func TestRejectionsAtomic(t *testing.T) {
	m := newMap(t, baseConfig())

	// 先成功导入一段，制造既有状态。
	good := Segment{ID: "g", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.1,
			Observations: []Observation{{ID: "K", X: 0, Y: 0}}},
	}}
	if _, err := m.ImportSegment(good); err != nil {
		t.Fatal(err)
	}

	type tc struct {
		name     string
		seg      Segment
		kind     string
		frame    int
		landmark string
	}
	cases := []tc{
		{"empty batch", Segment{ID: "x"}, RejectEmptySegment, -1, ""},
		{"empty segment id", Segment{Frames: []Frame{{Time: 1}}}, RejectEmptyID, -1, ""},
		{"empty landmark id", Segment{ID: "x", Frames: []Frame{
			{Time: 101, Observations: []Observation{{ID: ""}}},
		}}, RejectEmptyID, 0, ""},
		{"negative variance", Segment{ID: "x", Frames: []Frame{
			{Time: 101, MoveVariance: -0.01},
		}}, RejectNegativeVariance, 0, ""},
		{"non-finite dx", Segment{ID: "x", Frames: []Frame{
			{Time: 101, DX: math.NaN()},
		}}, RejectNonFinite, 0, ""},
		{"non-finite observation", Segment{ID: "x", Frames: []Frame{
			{Time: 101, Observations: []Observation{{ID: "P", X: 0, Y: math.Inf(1)}}},
		}}, RejectNonFinite, 0, "P"},
		{"first frame not later than initial time", Segment{ID: "x", Frames: []Frame{
			{Time: 0},
		}}, RejectTimeOrder, 0, ""},
		{"first frame not later than prev segment end", Segment{ID: "x", Frames: []Frame{
			{Time: 100},
		}}, RejectTimeOrder, 0, ""},
		{"non strictly increasing", Segment{ID: "x", Frames: []Frame{
			{Time: 101}, {Time: 101},
		}}, RejectTimeOrder, 1, ""},
		{"interval exceeded", Segment{ID: "x", Frames: []Frame{
			{Time: 1102}, // 1102-100 = 1002 > 1000
		}}, RejectInterval, 0, ""},
		{"interval exceeded later frame", Segment{ID: "x", Frames: []Frame{
			{Time: 101}, {Time: 1103},
		}}, RejectInterval, 1, ""},
		{"landmark conflict", Segment{ID: "x", Frames: []Frame{
			{Time: 101, Observations: []Observation{{ID: "K", X: 10, Y: 10}}},
		}}, RejectLandmarkConflict, 0, "K"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			_, err := m.ImportSegment(c.seg)
			if err == nil {
				t.Fatal("expected rejection")
			}
			if c.frame >= 0 {
				rejectKind(t, err, c.kind, c.frame, c.landmark)
			} else {
				r, ok := AsRejectError(err)
				if !ok || r.Kind != c.kind {
					t.Fatalf("kind = %v, want %s", err, c.kind)
				}
			}
		})
	}

	// 全部失败后状态仍是 good 段之后：位姿、路标、次数均不变。
	cur, _ := m.CurrentPose()
	if cur.Time != 100 {
		t.Fatalf("current time = %d, state leaked after rejection", cur.Time)
	}
	approxEq(t, "variance unchanged", cur.Variance, 0.6)
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "K" || lms[0].Count != 1 {
		t.Fatalf("landmarks leaked: %v", lms)
	}

	// 边界：间隔恰为上限可接受；路标距离恰为合并距离可接受。
	if _, err := m.ImportSegment(Segment{ID: "bound", Frames: []Frame{
		{Time: 1100, Observations: []Observation{{ID: "K", X: 1, Y: 0}}}, // 世界 (3,2)，距 K(2,2)=1
	}}); err != nil {
		t.Fatalf("boundary case rejected: %v", err)
	}
	lms, _ = m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	for _, lm := range lms {
		if lm.ID == "K" && lm.Count != 2 {
			t.Fatalf("K count = %d, want 2", lm.Count)
		}
	}
}

func TestSameFrameObservationsInOrder(t *testing.T) {
	m := newMap(t, baseConfig())
	// 两次同帧观测（初始位姿 (1,2)、朝向 0）：先在 (1,2) 建路标 Q，
	// 再在 (1,2.5) 合并 → (1,2.25) count2。
	_, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "Q", X: 0, Y: 0}, {ID: "Q", X: 0, Y: 0.5}}},
	}})
	if err != nil {
		t.Fatal(err)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].Count != 2 {
		t.Fatalf("landmarks = %v", lms)
	}
	approxEq(t, "q y", lms[0].Y, 2.25)

	// 同帧第二次观测距当前聚合位置超距也要拒绝整段，且不留第一次观测。
	m2 := newMap(t, baseConfig())
	_, err = m2.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "Q", X: 0, Y: 0}, {ID: "Q", X: 5, Y: 0}}},
	}})
	rejectKind(t, err, RejectLandmarkConflict, 0, "Q")
	lms, _ = m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 0 {
		t.Fatalf("landmark leaked: %v", lms)
	}
}

func TestDuplicateImport(t *testing.T) {
	m := newMap(t, baseConfig())
	a := Segment{ID: "A", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, MoveVariance: 0.1, Observations: []Observation{{ID: "L", X: 0, Y: 0}}},
	}}
	resA, err := m.ImportSegment(a)
	if err != nil {
		t.Fatal(err)
	}
	b := Segment{ID: "B", Frames: []Frame{
		{Time: 200, DX: 0, DY: 1, MoveVariance: 0.1},
	}}
	if _, err := m.ImportSegment(b); err != nil {
		t.Fatal(err)
	}

	// 再次导入 A（相同内容）：返回首次结果，当前数据不变。
	resA2, err := m.ImportSegment(a)
	if err != nil {
		t.Fatal(err)
	}
	if resA2.EndPose.Time != 100 || resA2.EndPose.X != resA.EndPose.X {
		t.Fatalf("duplicate returned %+v, want first result %+v", resA2, resA)
	}
	cur, _ := m.CurrentPose()
	if cur.Time != 200 {
		t.Fatalf("current time = %d, duplicate changed data", cur.Time)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].Count != 1 {
		t.Fatalf("landmark changed on duplicate: %v", lms)
	}
	p, err := m.PoseAt(100)
	if err != nil || p.X != 2 {
		t.Fatalf("pose at 100 leaked duplicate: %+v %v", p, err)
	}

	// 同一 ID 不同内容：明确拒绝，且不影响任何东西。
	aDiff := Segment{ID: "A", Frames: []Frame{{Time: 100, DX: 2, DY: 0}}}
	_, err = m.ImportSegment(aDiff)
	rejectKind(t, err, RejectDuplicateMismatch, -1, "A")

	// 原内容仍可重复导入。
	if _, err := m.ImportSegment(a); err != nil {
		t.Fatalf("original A no longer accepted: %v", err)
	}

	// 失败过的段修正后可用（该 ID 此前从未成功保存）。
	bad := Segment{ID: "C", Frames: []Frame{{Time: 200, MoveVariance: -1}}}
	if _, err := m.ImportSegment(bad); err == nil {
		t.Fatal("expected rejection")
	}
	fixed := Segment{ID: "C", Frames: []Frame{{Time: 250, DX: 1, DY: 0}}}
	if _, err := m.ImportSegment(fixed); err != nil {
		t.Fatalf("fixed segment rejected: %v", err)
	}
}

func TestPoseAtHistory(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 100, DX: 1, DY: 0, MoveVariance: 0},
		{Time: 200, DX: 1, DY: 0, MoveVariance: 0},
	}}); err != nil {
		t.Fatal(err)
	}

	if _, err := m.PoseAt(-1); !errors.Is(err, ErrNotFound) {
		t.Fatalf("before initial: %v", err)
	}
	p, err := m.PoseAt(0) // 恰为初始时间
	if err != nil || p.Time != 0 {
		t.Fatalf("at initial: %+v %v", p, err)
	}
	p, _ = m.PoseAt(50) // 早于首帧但不早于初始时间 → 初始位姿
	if p.Time != 0 {
		t.Fatalf("at 50 = %d", p.Time)
	}
	p, _ = m.PoseAt(100)
	if p.Time != 100 || p.X != 2 {
		t.Fatalf("at 100 = %+v", p)
	}
	p, _ = m.PoseAt(150)
	if p.Time != 100 {
		t.Fatalf("at 150 = %d, want last pose not later than t", p.Time)
	}
	p, _ = m.PoseAt(10_000_000)
	if p.Time != 200 {
		t.Fatalf("after end = %d", p.Time)
	}
}

func TestLandmarksRect(t *testing.T) {
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m := newMap(t, cfg)
	if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
		{Time: 10, Observations: []Observation{{ID: "b", X: 5, Y: 5}, {ID: "a", X: 1, Y: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 20, Observations: []Observation{{ID: "c", X: -1, Y: -1}}},
	}}); err != nil {
		t.Fatal(err)
	}

	// 边界包含；按标识排序。
	lms, err := m.LandmarksInRect(Rect{MinX: 1, MinY: 1, MaxX: 5, MaxY: 5})
	if err != nil {
		t.Fatal(err)
	}
	if len(lms) != 2 || lms[0].ID != "a" || lms[1].ID != "b" {
		t.Fatalf("lms = %v", lms)
	}

	// 空区域返回空（非 nil）结果。
	lms, err = m.LandmarksInRect(Rect{MinX: 10, MinY: 10, MaxX: 20, MaxY: 20})
	if err != nil || lms == nil || len(lms) != 0 {
		t.Fatalf("empty rect = %v, %v", lms, err)
	}

	// 上下界颠倒报错；非有限报错。
	if _, err := m.LandmarksInRect(Rect{MinX: 5, MaxX: 1, MinY: 0, MaxY: 1}); !errors.Is(err, ErrInvalidRect) {
		t.Fatalf("inverted x: %v", err)
	}
	if _, err := m.LandmarksInRect(Rect{MinX: 0, MaxX: 1, MinY: 9, MaxY: 1}); !errors.Is(err, ErrInvalidRect) {
		t.Fatalf("inverted y: %v", err)
	}
	if _, err := m.LandmarksInRect(Rect{MinX: 0, MaxX: math.NaN(), MinY: 0, MaxY: 1}); !errors.Is(err, ErrInvalidRect) {
		t.Fatalf("nan rect: %v", err)
	}
}

func TestCrossSegmentTimeGap(t *testing.T) {
	m := newMap(t, baseConfig())
	if _, err := m.ImportSegment(Segment{ID: "a", Frames: []Frame{{Time: 200}}}); err != nil {
		t.Fatal(err)
	}
	// 跨段间隔同样受限：200 -> 1201 超过 1000。
	_, err := m.ImportSegment(Segment{ID: "b", Frames: []Frame{{Time: 1201}}})
	rejectKind(t, err, RejectInterval, 0, "")
	// 恰为上限可接受。
	if _, err := m.ImportSegment(Segment{ID: "b", Frames: []Frame{{Time: 1200}}}); err != nil {
		t.Fatalf("gap == max interval rejected: %v", err)
	}
}

func TestPersistenceRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	cfg := baseConfig()
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	seg := Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 3, DY: 4, DHeading: 0.5, MoveVariance: 0.7,
			Observations: []Observation{{ID: "L", X: 1, Y: 1}}},
	}}
	res, err := m.ImportSegment(seg)
	if err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 文件在关闭后仍然存在且非空（每次导入即落盘）。
	if fi, err := os.Stat(path); err != nil || fi.Size() == 0 {
		t.Fatalf("file missing/empty: %v %v", fi, err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	cur, err := m2.CurrentPose()
	if err != nil {
		t.Fatal(err)
	}
	if cur.Time != res.EndPose.Time || cur.X != res.EndPose.X || cur.Heading != res.EndPose.Heading ||
		cur.Variance != res.EndPose.Variance {
		t.Fatalf("after reopen current = %+v, want %+v", cur, res.EndPose)
	}
	p, err := m2.PoseAt(100)
	if err != nil || p.Time != 100 {
		t.Fatalf("pose at after reopen: %+v %v", p, err)
	}
	lms, err := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if err != nil || len(lms) != 1 || lms[0].ID != "L" || lms[0].Count != 1 {
		t.Fatalf("landmarks after reopen: %v %v", lms, err)
	}

	// 重复导入信息同样持久化。
	if _, err := m2.ImportSegment(seg); err != nil {
		t.Fatalf("duplicate after reopen: %v", err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "s1", Frames: []Frame{{Time: 100, DX: 9}}}); err == nil {
		t.Fatal("mismatch accepted after reopen")
	}
	// 新段也能继续导入。
	if _, err := m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{{Time: 200}}}); err != nil {
		t.Fatalf("new segment after reopen: %v", err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// Create 不得覆盖已有文件。
	if _, err := Create(path, cfg); err == nil {
		t.Fatal("Create overwrote existing map")
	}
	// Open 不存在的文件返回 not-exist。
	if _, err := Open(filepath.Join(t.TempDir(), "nope")); !errors.Is(err, os.ErrNotExist) {
		t.Fatalf("missing file: %v", err)
	}
}

func TestCorruptAndVersion(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	good, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}

	writeBytes := func(name string, data []byte) string {
		p := filepath.Join(t.TempDir(), name)
		if err := os.WriteFile(p, data, 0o644); err != nil {
			t.Fatal(err)
		}
		return p
	}

	if _, err := Open(writeBytes("short", good[:5])); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("truncated: %v", err)
	}
	badMagic := append([]byte(nil), good...)
	copy(badMagic[:8], "XXXXXXXX")
	if _, err := Open(writeBytes("magic", badMagic)); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("bad magic: %v", err)
	}
	// 翻转 payload 中一个字节，CRC 不匹配。
	flip := append([]byte(nil), good...)
	flip[headerLen+1] ^= 0xFF
	if _, err := Open(writeBytes("flip", flip)); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("checksum: %v", err)
	}
	// 长度字段被改。
	badLen := append([]byte(nil), good...)
	badLen[10] ^= 0x01
	if _, err := Open(writeBytes("len", badLen)); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("length: %v", err)
	}

	// 支持的版本可以打开；构造一个 CRC 合法但版本号不受支持的文件。
	raw := append([]byte(nil), good...)
	raw[8] = 0x00
	raw[9] = 0x03 // version 3（本包仅支持 1 和 2）
	// 重算 CRC。
	sum := crc32.ChecksumIEEE(raw[:len(raw)-4])
	binary.BigEndian.PutUint32(raw[len(raw)-4:], sum)
	if _, err := Open(writeBytes("ver", raw)); !errors.Is(err, ErrUnsupportedVersion) {
		t.Fatalf("unsupported version: %v", err)
	}

	// 损坏文件不应被 Open/Create 静默当作新地图：Create 对已存在损坏文件同样拒绝。
	corruptPath := writeBytes("corrupt.pose", flip)
	if _, err := Open(corruptPath); !errors.Is(err, ErrCorrupt) {
		t.Fatal(err)
	}
	if _, err := Create(corruptPath, baseConfig()); err == nil {
		t.Fatal("Create overwrote corrupt file")
	}
}

func TestSaveFailureRollback(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "map.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	// 把目标路径变成目录，使 saveReplace 的 rename 失败（EISDIR）。
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := os.Mkdir(path, 0o755); err != nil {
		t.Fatal(err)
	}
	_, err = m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
	}})
	if err == nil {
		t.Fatal("import across directory target unexpectedly succeeded")
	}
	// 内存状态精确回滚。
	cur, _ := m.CurrentPose()
	if cur.Time != 0 {
		t.Fatalf("current time = %d after failed save", cur.Time)
	}
	lms, _ := m.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 0 {
		t.Fatalf("landmarks leaked after failed save: %v", lms)
	}
	// 段记录也回滚：恢复文件后同 ID 可正常导入。
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{{Time: 100, DX: 1}}}); err != nil {
		t.Fatalf("retry after failed save: %v", err)
	}
}

func TestClosedMap(t *testing.T) {
	m := newMap(t, baseConfig())
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	if _, err := m.CurrentPose(); !errors.Is(err, ErrClosed) {
		t.Fatalf("current: %v", err)
	}
	if _, err := m.PoseAt(0); !errors.Is(err, ErrClosed) {
		t.Fatalf("poseat: %v", err)
	}
	if _, err := m.LandmarksInRect(Rect{MinX: 0, MinY: 0, MaxX: 1, MaxY: 1}); !errors.Is(err, ErrClosed) {
		t.Fatalf("rect: %v", err)
	}
	if _, err := m.ImportSegment(Segment{ID: "x", Frames: []Frame{{Time: 1}}}); !errors.Is(err, ErrClosed) {
		t.Fatalf("import: %v", err)
	}
}

func TestCanonicalHash(t *testing.T) {
	f := []Frame{{Time: 1, DX: 2, Observations: []Observation{{ID: "a", X: 3}}}}
	if canonicalHash(f) != canonicalHash(f) {
		t.Fatal("hash not deterministic")
	}
	f2 := []Frame{{Time: 1, DX: 2.0001, Observations: []Observation{{ID: "a", X: 3}}}}
	if canonicalHash(f) == canonicalHash(f2) {
		t.Fatal("hash collided on different content")
	}
	f3 := []Frame{{Time: 1, DX: 2, Observations: []Observation{{ID: "a", X: 3, Y: math.Copysign(0, -1)}}}}
	f4 := []Frame{{Time: 1, DX: 2, Observations: []Observation{{ID: "a", X: 3, Y: 0.0}}}}
	if canonicalHash(f3) == canonicalHash(f4) {
		t.Fatal("+0/-0 should differ by bit hash")
	}
}
