package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"reflect"
	"testing"
)

// buildCorrectedRangeMap 构造初始位姿 t=0、已导入帧 t=100/200/300 的地图，
// 三帧都观测 L；随后以 t=100 为锚点校正一次（目标 X=5）。校正记录声明的
// 范围为 100..300，含三份逐帧前后位姿，时间分别为 100、200、300。
func buildCorrectedRangeMap(t *testing.T, path string) {
	t.Helper()
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 10.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 300, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Variance: 0}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 正常的三帧范围记录必须打开：每份前后位姿都对应范围内一帧，前后时间相同。
func TestOpenCorrectionRangeValid(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildCorrectedRangeMap(t, path)

	fd := readFileData(t, path)
	if len(fd.Corrections) != 1 || len(fd.Corrections[0].Poses) != 3 {
		t.Fatalf("test setup wrong: %+v", fd.Corrections)
	}

	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 {
		t.Fatalf("corrections = %d, want 1", len(recs))
	}
	rec := recs[0]
	if rec.Anchor != 100 || rec.EndTime != 300 || len(rec.Poses) != 3 {
		t.Fatalf("record = %+v", rec)
	}
	// 校正前位姿为原始轨迹 (1,0)/(2,0)/(3,0)，校正后锚点平移到 5：
	// (5,0)/(6,0)/(7,0)；快照保持保存时的内容。
	wantBefore := []Pose{
		{Time: 100, X: 1}, {Time: 200, X: 2}, {Time: 300, X: 3},
	}
	wantAfter := []Pose{
		{Time: 100, X: 5}, {Time: 200, X: 6}, {Time: 300, X: 7},
	}
	for i, pc := range rec.Poses {
		if pc.Before.Time != wantBefore[i].Time || pc.Before.X != wantBefore[i].X {
			t.Fatalf("poses[%d].Before = %+v, want time %d X %v", i, pc.Before, wantBefore[i].Time, wantBefore[i].X)
		}
		if pc.After.Time != wantAfter[i].Time || pc.After.X != wantAfter[i].X {
			t.Fatalf("poses[%d].After = %+v, want time %d X %v", i, pc.After, wantAfter[i].Time, wantAfter[i].X)
		}
	}
}

// 任务示例：范围 100..300、轨迹中还有 200 毫秒的帧，记录只含 100 和 300，
// 必须按 ErrCorrupt 拒绝。其它漏帧/重复/错序/越界/前后时间不一致等篡改同样
// 拒绝；任何情况下不返回地图对象，原文件保持不变。
func TestOpenRejectsIncompleteCorrectionRange(t *testing.T) {
	cases := []struct {
		name  string
		tweak func(*fileData)
	}{
		{
			name: "missing middle frame 100 and 300 only",
			tweak: func(fd *fileData) {
				cr := &fd.Corrections[0]
				cr.Poses[1] = cr.Poses[2]
				cr.Poses = cr.Poses[:2]
			},
		},
		{
			name: "missing anchor frame",
			tweak: func(fd *fileData) {
				cr := &fd.Corrections[0]
				cr.Poses = cr.Poses[1:]
			},
		},
		{
			name: "duplicate frame in range",
			tweak: func(fd *fileData) {
				cr := &fd.Corrections[0]
				cr.Poses[1].Before.Time = 100
				cr.Poses[1].After.Time = 100
			},
		},
		{
			name: "poses out of trajectory order",
			tweak: func(fd *fileData) {
				cr := &fd.Corrections[0]
				cr.Poses[1].Before.Time, cr.Poses[2].Before.Time = 300, 200
				cr.Poses[1].After.Time, cr.Poses[2].After.Time = 300, 200
			},
		},
		{
			name: "extra frame beyond declared end",
			tweak: func(fd *fileData) {
				cr := &fd.Corrections[0]
				extra := cr.Poses[2]
				extra.Before.Time, extra.After.Time = 400, 400
				cr.Poses = append(cr.Poses, extra)
			},
		},
		{
			name: "frame outside range replacing in-range frame",
			tweak: func(fd *fileData) {
				cr := &fd.Corrections[0]
				cr.Poses[2].Before.Time = 400
				cr.Poses[2].After.Time = 400
			},
		},
		{
			name: "adjacent frame substituted for real frame",
			tweak: func(fd *fileData) {
				cr := &fd.Corrections[0]
				cr.Poses[1].Before.Time = 150
				cr.Poses[1].After.Time = 150
			},
		},
		{
			name: "before and after times are different frames",
			tweak: func(fd *fileData) {
				// 条目含三份，但中间条目的前时间为 200、后时间为 300。
				fd.Corrections[0].Poses[1].After.Time = 300
			},
		},
		{
			name: "before time differs from after time at anchor",
			tweak: func(fd *fileData) {
				fd.Corrections[0].Poses[0].Before.Time = 200
			},
		},
		{
			name: "anchor is the initial pose",
			tweak: func(fd *fileData) {
				fd.Corrections[0].Anchor = 0
			},
		},
		{
			name: "end is the initial pose",
			tweak: func(fd *fileData) {
				fd.Corrections[0].EndTime = 0
			},
		},
		{
			name: "anchor does not hit any imported frame",
			tweak: func(fd *fileData) {
				fd.Corrections[0].Anchor = 150
			},
		},
		{
			name: "end does not hit any imported frame",
			tweak: func(fd *fileData) {
				fd.Corrections[0].EndTime = 250
			},
		},
		{
			name: "anchor after end",
			tweak: func(fd *fileData) {
				cr := &fd.Corrections[0]
				cr.Anchor, cr.EndTime = 300, 100
			},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildCorrectedRangeMap(t, path)
			fd := readFileData(t, path)
			tc.tweak(&fd)
			writeFileDataRaw(t, path, &fd)

			before, err := os.ReadFile(path)
			if err != nil {
				t.Fatal(err)
			}
			m, err := Open(path)
			if !errors.Is(err, ErrCorrupt) {
				t.Fatalf("err = %v, want ErrCorrupt", err)
			}
			if m != nil {
				t.Fatalf("Open returned usable map: %v", m)
			}
			after, err := os.ReadFile(path)
			if err != nil {
				t.Fatal(err)
			}
			if string(before) != string(after) {
				t.Fatal("Open modified the rejected file")
			}
		})
	}
}

// 只校正一帧：锚点与结束时间相同、只有一份前后位姿且前后时间相同，合法。
func TestOpenSingleFrameCorrectionValid(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 10.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
		{Time: 300, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 300, Target: CorrectionTarget{X: 9, Variance: 0}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open single-frame correction: %v", err)
	}
	defer m2.Close()
	recs, _ := m2.Corrections()
	if len(recs) != 1 || recs[0].Anchor != 300 || recs[0].EndTime != 300 {
		t.Fatalf("record = %+v", recs)
	}
	if len(recs[0].Poses) != 1 {
		t.Fatalf("poses = %+v, want exactly one entry", recs[0].Poses)
	}
	pc := recs[0].Poses[0]
	if pc.Before.Time != 300 || pc.After.Time != 300 {
		t.Fatalf("pose change times = %d/%d, want 300/300", pc.Before.Time, pc.After.Time)
	}
}

// 记录之后新导入的帧不属于此前的记录：校正时末帧为 200（记录含 100、200
// 两份），之后再导入 t=300，重新打开仍合法，记录范围保持 100..200。
func TestOpenCorrectionRangeBeforeLaterImport(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 10.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Variance: 0}}); err != nil {
		t.Fatal(err)
	}
	// 校正提交后再导入新帧；既有记录不扩展。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m2.Close()
	recs, _ := m2.Corrections()
	if len(recs) != 1 || recs[0].EndTime != 200 || len(recs[0].Poses) != 2 {
		t.Fatalf("record = %+v, want end 200 with 2 pose changes", recs)
	}
	for i, want := range []int64{100, 200} {
		if pc := recs[0].Poses[i]; pc.Before.Time != want || pc.After.Time != want {
			t.Fatalf("poses[%d] times = %d/%d, want %d", i, pc.Before.Time, pc.After.Time, want)
		}
	}
}

// 后来对相同范围再做校正是正常使用：重新打开后记录仍按提交次序返回，较早
// 记录保留当时的前后值，不被后续校正改写。
func TestOpenCorrectionsPreservedInCommitOrder(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 10.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	c1, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Variance: 0}})
	if err != nil {
		t.Fatal(err)
	}
	c2, err := m.Correct(Correction{ID: "c2", Anchor: 100, Target: CorrectionTarget{X: 9, Variance: 0}})
	if err != nil {
		t.Fatal(err)
	}
	before, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(before) != 2 || before[0].ID != "c1" || before[1].ID != "c2" {
		t.Fatalf("corrections before close = %+v", before)
	}
	// 较早记录的“后值”即第二次校正的“前值”，但较早记录本身保持原样。
	if !reflect.DeepEqual(before[0], c1) || !reflect.DeepEqual(before[1], c2) {
		t.Fatalf("returned records differ from commit results")
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m2.Close()
	after, err := m2.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if !reflect.DeepEqual(after, before) {
		t.Fatalf("corrections changed across reopen:\nbefore=%+v\nafter =%+v", before, after)
	}
}

// 没有校正记录的地图继续正常打开。
func TestOpenMapWithoutCorrectionsStillOpens(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m2.Close()
	recs, err := m2.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 0 {
		t.Fatalf("corrections = %+v, want none", recs)
	}
}
