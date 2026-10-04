package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// buildCorrectedMap 构造含三帧（t=100、200、300）并对 t=100 做过一次校正
// 的地图，校正记录覆盖全部三帧。
func buildCorrectedMap(t *testing.T, path string) {
	t.Helper()
	m, err := Create(path, baseConfig())
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
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// tamperFile 读取文件、按 fn 篡改后重算校验和写回。
func tamperFile(t *testing.T, path string, fn func(fd *fileData)) {
	t.Helper()
	fd := readFileData(t, path)
	fn(&fd)
	writeFileDataRaw(t, path, &fd)
}

// assertOpenCorrupt 断言 Open 以 ErrCorrupt 拒绝、不返回地图对象且文件
// 内容保持不变。
func assertOpenCorrupt(t *testing.T, path string) {
	t.Helper()
	before, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("map = %v, want nil", m)
	}
	after, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(before) != string(after) {
		t.Fatal("file modified by rejected Open")
	}
}

// 校正记录必须恰好覆盖声明范围内的每一帧：缺失、重复、错序、范围外的
// 额外条目、前后时间不一致，或锚点/结束时间不命中已导入帧，都按损坏
// 拒绝。
func TestOpenRejectsIncompleteCorrectionRecords(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"missing middle frame", func(fd *fileData) {
			p := fd.Corrections[0].Poses
			fd.Corrections[0].Poses = []poseChangeJSON{p[0], p[2]}
		}},
		{"missing last frame", func(fd *fileData) {
			fd.Corrections[0].Poses = fd.Corrections[0].Poses[:2]
		}},
		{"duplicate frame replaces another", func(fd *fileData) {
			fd.Corrections[0].Poses[1] = fd.Corrections[0].Poses[0]
		}},
		{"out of order", func(fd *fileData) {
			p := fd.Corrections[0].Poses
			p[0], p[1] = p[1], p[0]
		}},
		{"extra frame beyond range", func(fd *fileData) {
			p := fd.Corrections[0].Poses
			extra := p[2]
			extra.Before.Time, extra.After.Time = 400, 400
			fd.Corrections[0].Poses = append(p, extra)
		}},
		{"before and after times differ", func(fd *fileData) {
			fd.Corrections[0].Poses[1].After.Time = 250
		}},
		{"pose time not on any frame", func(fd *fileData) {
			fd.Corrections[0].Poses[1].Before.Time = 250
			fd.Corrections[0].Poses[1].After.Time = 250
		}},
		{"anchor is initial pose", func(fd *fileData) {
			fd.Corrections[0].Anchor = 0
		}},
		{"end time is initial pose", func(fd *fileData) {
			fd.Corrections[0].Anchor = 0
			fd.Corrections[0].EndTime = 0
			fd.Corrections[0].Poses = fd.Corrections[0].Poses[:1]
		}},
		{"anchor between frames", func(fd *fileData) {
			fd.Corrections[0].Anchor = 150
		}},
		{"end time between frames", func(fd *fileData) {
			fd.Corrections[0].EndTime = 250
		}},
		{"anchor shifted to later frame", func(fd *fileData) {
			fd.Corrections[0].Anchor = 200
		}},
		{"anchor after end time", func(fd *fileData) {
			fd.Corrections[0].Anchor = 300
			fd.Corrections[0].EndTime = 100
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildCorrectedMap(t, path)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 未篡改的校正记录正常打开，记录按原提交次序返回且内容保持保存时的
// 快照。
func TestOpenAcceptsCompleteCorrectionRecord(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildCorrectedMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 || recs[0].ID != "c1" || recs[0].Anchor != 100 || recs[0].EndTime != 300 {
		t.Fatalf("records = %+v", recs)
	}
	if len(recs[0].Poses) != 3 {
		t.Fatalf("poses = %+v", recs[0].Poses)
	}
	for i, pc := range recs[0].Poses {
		want := int64((i + 1) * 100)
		if pc.Before.Time != want || pc.After.Time != want {
			t.Fatalf("pose %d times = %d/%d, want %d", i, pc.Before.Time, pc.After.Time, want)
		}
	}
}

// 只校正一帧的合法记录（锚点与结束时间相同、仅一份前后位姿）正常打开。
func TestOpenAcceptsSingleFrameCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 200, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
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
	if len(recs) != 1 || recs[0].Anchor != 200 || recs[0].EndTime != 200 || len(recs[0].Poses) != 1 {
		t.Fatalf("records = %+v", recs)
	}
}

// 对相同范围再次校正、且之后导入新帧：较早记录仍只覆盖提交时的范围并
// 保留当时的前后值，新帧不属于之前的记录；两条记录都合法，重开后按
// 提交次序返回。
func TestOpenAcceptsRecorrectionAndLaterFrames(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
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
	rec1, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{{Time: 400, DX: 1}}}); err != nil {
		t.Fatal(err)
	}
	rec2, err := m.Correct(Correction{ID: "c2", Anchor: 100, Target: CorrectionTarget{X: 7, Y: 8, Variance: 2}})
	if err != nil {
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
	if len(recs) != 2 || recs[0].ID != "c1" || recs[1].ID != "c2" {
		t.Fatalf("records = %+v", recs)
	}
	if !recordsEqual(recs[0], rec1) {
		t.Fatalf("first record changed: got %+v want %+v", recs[0], rec1)
	}
	if !recordsEqual(recs[1], rec2) {
		t.Fatalf("second record changed: got %+v want %+v", recs[1], rec2)
	}
	if recs[0].EndTime != 300 || len(recs[0].Poses) != 3 {
		t.Fatalf("first record range = %+v", recs[0])
	}
	if recs[1].EndTime != 400 || len(recs[1].Poses) != 4 {
		t.Fatalf("second record range = %+v", recs[1])
	}
}
